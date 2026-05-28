#include "SuperPyramid.h"

#include <VSHelper4.h>
#include <cassert>

template<typename PixelType>
void PyramidPlane::CopyAndPadPlane(const VSFrame *src, int plane, int hPad, int vPad, int nBlkSizePadX, int nBlkSizePadY, VSCore *core, const VSAPI *vsapi) noexcept {
    const VSVideoFormat *format = vsapi->getVideoFrameFormat(src);
    VSVideoFormat dstFormat = {};
    vsapi->queryVideoFormat(&dstFormat, cfGray, format->sampleType, format->bitsPerSample, 0, 0, core);
    
    nRealWidth = vsapi->getFrameWidth(src, plane);
    nRealHeight = vsapi->getFrameHeight(src, plane);
    nWidth = nRealWidth + nBlkSizePadX;
    nHeight = nRealHeight + nBlkSizePadY;
    nPaddedWidth = nWidth + 2 * hPad;
    nPaddedHeight = nHeight + 2 * vPad;
    nHPadding = hPad;
    nVPadding = vPad;
    nHPaddingPel = nHPadding;
    nVPaddingPel = nVPadding;
    nPel = 1;

    VSFrame *dst = vsapi->newVideoFrame(&dstFormat, nPaddedWidth, nPaddedHeight, nullptr, core);
    storage[0] = dst;
    nPitch = vsapi->getStride(dst, 0);
    nOffsetPadding = nPitch * nVPadding + nHPadding * sizeof(PixelType);

    const PixelType *srcP  = reinterpret_cast<const PixelType *>(vsapi->getReadPtr(src, plane));
    PixelType *dstP = reinterpret_cast<PixelType *>(vsapi->getWritePtr(dst, 0));
    ptrdiff_t srcPitch = vsapi->getStride(src, plane) / sizeof(PixelType);
    ptrdiff_t dstPitch = nPitch / sizeof(PixelType);

    // Copy frame data and pad sides by extending the edges
    dstP += dstPitch * vPad;

    for (int h = 0; h < nRealHeight; h++) {
        PixelType padValueLeft = srcP[0];
        for (int w = 0; w < hPad; w++)
            dstP[w] = padValueLeft;
        memcpy(dstP + hPad, srcP, nRealWidth * sizeof(PixelType));
        PixelType padValueRight = srcP[nRealWidth - 1];
        for (int w = nRealWidth + hPad; w < nPaddedWidth; w++)
            dstP[w] = padValueRight;
        srcP += srcPitch;
        dstP += dstPitch;
    }

    // Top and bottom padding by copying the first and last actual image line that's already been extended horizontally
    const PixelType *dstPLastLine = dstP - dstPitch;
    for (int h = 0; h < vPad + nBlkSizePadY; h++) {
        memcpy(dstP, dstPLastLine, dstPitch);
        dstP += dstPitch;
    }

    dstP = reinterpret_cast<PixelType *>(vsapi->getWritePtr(dst, 0));
    const PixelType *dstPFirstLine = dstP + dstPitch * vPad;
    for (int h = 0; h < vPad; h++) {
        memcpy(dstP, dstPFirstLine, dstPitch);
        dstP += dstPitch;
    }

    pPlane[0] = vsapi->getReadPtr(storage[0], 0);
}

template <typename PixelType>
static void RB2F_C(uint8_t *VS_RESTRICT pDst8, const uint8_t *VS_RESTRICT pSrc8, ptrdiff_t nDstPitch,
    ptrdiff_t nSrcPitch, int nWidth, int nHeight) noexcept {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    for (int y = 0; y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x * 2] + pSrc[x * 2 + 1]
                + pSrc[x * 2 + nSrcPitch + 1] + pSrc[x * 2 + nSrcPitch] + 2) / 4;

        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }
}

// separable BilinearFiltered with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing
// interleaves vertical and horizontal downscaling row by row using tempBuffer as scratch
// tempBuffer must point to at least nWidth * 8 * sizeof(PixelType) bytes
template <typename PixelType>
static void RB2BilinearFiltered(uint8_t *pDst, const uint8_t * VS_RESTRICT pSrc, ptrdiff_t nDstPitch,
    ptrdiff_t nSrcPitch, int nWidth, int nHeight, uint8_t * VS_RESTRICT tempBuffer) noexcept {

    PixelType *dst = (PixelType *)pDst;
    const PixelType *src = (const PixelType *)pSrc;
    PixelType *tmp = (PixelType *)tempBuffer; // nWidth*2 entries used per row

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    const int srcWidth2 = nWidth * 2;

    for (int y = 0; y < nHeight; y++) {
        const PixelType *row = src + y * 2 * nSrcPitch;

        // Vertical filter: produce srcWidth2 intermediate samples into tmp
        if (y == 0 || y == nHeight - 1) {
            // Edge rows: simple average of the two source rows
            for (int x = 0; x < srcWidth2; x++)
                tmp[x] = (row[x] + row[x + nSrcPitch] + 1) / 2;
        } else {
            // Middle rows: 4-tap filter (1/8, 3/8, 3/8, 1/8) across rows 2y-1..2y+2
            for (int x = 0; x < srcWidth2; x++)
                tmp[x] = (row[x - nSrcPitch] + (row[x] + row[x + nSrcPitch]) * 3 + row[x + nSrcPitch * 2] + 4) / 8;
        }

        // Horizontal filter: reduce tmp from srcWidth2 to nWidth, write to dst row y
        PixelType *dstRow = dst + y * nDstPitch;

        dstRow[0] = (tmp[0] + tmp[1] + 1) / 2;

        for (int x = 1; x < nWidth - 1; x++)
            dstRow[x] = (tmp[x * 2 - 1] + (tmp[x * 2] + tmp[x * 2 + 1]) * 3 + tmp[x * 2 + 2] + 4) / 8;

        dstRow[nWidth - 1] = (tmp[(nWidth - 1) * 2] + tmp[(nWidth - 1) * 2 + 1] + 1) / 2;
    }
}

// separable filtered cubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing
// interleaves vertical and horizontal downscaling row by row using tempBuffer as scratch
// tempBuffer must point to at least nWidth * 8 * sizeof(PixelType) bytes
template <typename PixelType>
static void RB2Cubic(uint8_t *pDst, const uint8_t *pSrc, ptrdiff_t nDstPitch,
    ptrdiff_t nSrcPitch, int nWidth, int nHeight, uint8_t *VS_RESTRICT tempBuffer) noexcept {

    PixelType *dst = (PixelType *)pDst;
    const PixelType *src = (const PixelType *)pSrc;
    PixelType *tmp = (PixelType *)tempBuffer; // nWidth*2 entries used per row

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    const int srcWidth2 = nWidth * 2;

    for (int y = 0; y < nHeight; y++) {
        const PixelType *row = src + y * 2 * nSrcPitch;

        // Vertical filter: produce srcWidth2 intermediate samples into tmp
        if (y == 0 || y == nHeight - 1) {
            // Edge rows: simple average of the two source rows
            for (int x = 0; x < srcWidth2; x++)
                tmp[x] = (row[x] + row[x + nSrcPitch] + 1) / 2;
        } else {
            // Middle rows: 6-tap filter (1/32, 5/32, 10/32, 10/32, 5/32, 1/32) across rows 2y-2..2y+3
            for (int x = 0; x < srcWidth2; x++) {
                int m0 = row[x - nSrcPitch * 2];
                int m1 = row[x - nSrcPitch];
                int m2 = row[x];
                int m3 = row[x + nSrcPitch];
                int m4 = row[x + nSrcPitch * 2];
                int m5 = row[x + nSrcPitch * 3];

                m2 = (m2 + m3) * 10;
                m1 = (m1 + m4) * 5;
                m0 += m5 + m2 + m1 + 16;
                tmp[x] = m0 >> 5;
            }
        }

        // Horizontal filter: reduce tmp from srcWidth2 to nWidth, write to dst row y
        PixelType *dstRow = dst + y * nDstPitch;

        dstRow[0] = (tmp[0] + tmp[1] + 1) / 2;

        for (int x = 1; x < nWidth - 1; x++) {
            int m0 = tmp[x * 2 - 2];
            int m1 = tmp[x * 2 - 1];
            int m2 = tmp[x * 2];
            int m3 = tmp[x * 2 + 1];
            int m4 = tmp[x * 2 + 2];
            int m5 = tmp[x * 2 + 3];

            m2 = (m2 + m3) * 10;
            m1 = (m1 + m4) * 5;
            m0 += m5 + m2 + m1 + 16;
            dstRow[x] = m0 >> 5;
        }

        dstRow[nWidth - 1] = (tmp[(nWidth - 1) * 2] + tmp[(nWidth - 1) * 2 + 1] + 1) / 2;
    }
}

int PlaneDimensionLuma(int numPixels, int ratioUV, int pad) noexcept {
      return (pad >= ratioUV) ? ((numPixels / ratioUV + 1) / 2) * ratioUV : ((numPixels / ratioUV) / 2) * ratioUV;
}

template<typename PixelType>
void PyramidPlane::ReducePlane(const PyramidPlane &src, int xRatioUV, int yRatioUV, RFilterParam rFilter, uint8_t *tempBuffer, VSCore *core, const VSAPI *vsapi) noexcept {
    nVPadding = src.nVPadding;
    nHPadding = src.nHPadding;

    nHPaddingPel = nHPadding;
    nVPaddingPel = nVPadding;

    nWidth = PlaneDimensionLuma(src.nWidth, xRatioUV, nHPadding);
    nHeight = PlaneDimensionLuma(src.nHeight, yRatioUV, nVPadding);

    nRealWidth = nWidth;
    nRealHeight = nHeight;

    nPaddedWidth = nWidth + 2 * nHPadding;
    nPaddedHeight = nHeight + 2 * nVPadding;

    VSFrame *dst = vsapi->newVideoFrame(vsapi->getVideoFrameFormat(src.storage[0]), nPaddedWidth, nPaddedHeight, nullptr, core);
    storage[0] = dst;
    uint8_t *dstP = vsapi->getWritePtr(dst, 0);
    pPlane[0] = dstP;
    nPitch = vsapi->getStride(dst, 0);
    nOffsetPadding = nPitch * nVPadding + nHPadding * sizeof(PixelType);

    if (rFilter == RFilterParam::Simple) {
        RB2F_C<PixelType>(dstP + nOffsetPadding, src.pPlane[0] + src.nOffsetPadding, nPitch, src.nPitch, nWidth, nHeight);
    } else if (rFilter == RFilterParam::Bilinear) {
        RB2BilinearFiltered<PixelType>(dstP + nOffsetPadding, src.pPlane[0] + src.nOffsetPadding, nPitch, src.nPitch, nWidth, nHeight, tempBuffer);
    } else if (rFilter == RFilterParam::Cubic) {
        RB2Cubic<PixelType>(dstP + nOffsetPadding, src.pPlane[0] + src.nOffsetPadding, nPitch, src.nPitch, nWidth, nHeight, tempBuffer);
    }

    PadPlaneData<PixelType>(0);
}


template <typename PixelType>
static void VerticalBilinear(uint8_t *VS_RESTRICT pDst8, const uint8_t *VS_RESTRICT pSrc8,
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) noexcept {
    (void)bitsPerSample;

    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    for (int j = 0; j < nHeight - 1; j++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1;
        pDst += nPitch;
        pSrc += nPitch;
    }
    /* last row */
    for (int i = 0; i < nWidth; i++)
        pDst[i] = pSrc[i];
}


template <typename PixelType>
static void HorizontalBilinear(uint8_t *VS_RESTRICT pDst8, const uint8_t *VS_RESTRICT pSrc8,
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) noexcept {
    (void)bitsPerSample;

    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    for (int j = 0; j < nHeight; j++) {
        for (int i = 0; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];
        pDst += nPitch;
        pSrc += nPitch;
    }
}


template <typename PixelType>
static void DiagonalBilinear(uint8_t *VS_RESTRICT pDst8, const uint8_t *VS_RESTRICT pSrc8,
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) noexcept {
    (void)bitsPerSample;

    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    for (int j = 0; j < nHeight - 1; j++) {
        for (int i = 0; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nPitch] + pSrc[i + nPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nPitch - 1] + 1) >> 1;
        pDst += nPitch;
        pSrc += nPitch;
    }
    for (int i = 0; i < nWidth - 1; i++)
        pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;
    pDst[nWidth - 1] = pSrc[nWidth - 1];
}

// so called Wiener interpolation. (sharp, similar to Lanczos ?)
// invarint simplified, 6 taps. Weights: (1, -5, 20, 20, -5, 1)/32 - added by Fizick
template <typename PixelType>
static void VerticalWiener(uint8_t *VS_RESTRICT pDst8, const uint8_t *VS_RESTRICT pSrc8,
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) noexcept {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1;
        pDst += nPitch;
        pSrc += nPitch;
    }
    for (int j = 2; j < nHeight - 4; j++) {
        for (int i = 0; i < nWidth; i++) {
            int m0 = pSrc[i - nPitch * 2];
            int m1 = pSrc[i - nPitch];
            int m2 = pSrc[i];
            int m3 = pSrc[i + nPitch];
            int m4 = pSrc[i + nPitch * 2];
            int m5 = pSrc[i + nPitch * 3];

            m2 = (m2 + m3) * 4;

            m2 -= m1 + m4;
            m2 *= 5;

            m0 += m5 + m2 + 16;
            m0 >>= 5;

            pDst[i] = std::max(0, std::min(m0, pixelMax));
        }
        pDst += nPitch;
        pSrc += nPitch;
    }
    for (intptr_t j = nHeight - 4; j < nHeight - 1; j++) {
        for (intptr_t i = 0; i < nWidth; i++) {
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1;
        }

        pDst += nPitch;
        pSrc += nPitch;
    }
    /* last row */
    for (int i = 0; i < nWidth; i++)
        pDst[i] = pSrc[i];
}


template <typename PixelType>
static void HorizontalWiener(uint8_t *VS_RESTRICT pDst8, const uint8_t *VS_RESTRICT pSrc8,
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) noexcept {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < nHeight; j++) {
        pDst[0] = (pSrc[0] + pSrc[1] + 1) >> 1;
        pDst[1] = (pSrc[1] + pSrc[2] + 1) >> 1;

        for (int i = 2; i < nWidth - 4; i++) {
            int m0 = pSrc[i - 2];
            int m1 = pSrc[i - 1];
            int m2 = pSrc[i];
            int m3 = pSrc[i + 1];
            int m4 = pSrc[i + 2];
            int m5 = pSrc[i + 3];

            m2 = (m2 + m3) * 4;

            m2 -= m1 + m4;
            m2 *= 5;

            m0 += m5 + m2 + 16;
            m0 >>= 5;

            pDst[i] = std::max(0, std::min(m0, pixelMax));
        }

        for (intptr_t i = nWidth - 4; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];
        pDst += nPitch;
        pSrc += nPitch;
    }
}


// bicubic (Catmull-Rom 4 taps interpolation)
template <typename PixelType>
static void VerticalBicubic(uint8_t *VS_RESTRICT pDst8, const uint8_t *VS_RESTRICT pSrc8,
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) noexcept {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < 1; j++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1;
        pDst += nPitch;
        pSrc += nPitch;
    }
    for (int j = 1; j < nHeight - 3; j++) {
        for (int i = 0; i < nWidth; i++) {
            pDst[i] = std::min(pixelMax, std::max(0,
                (-pSrc[i - nPitch] - pSrc[i + nPitch * 2] + (pSrc[i] + pSrc[i + nPitch]) * 9 + 8) >> 4));
        }
        pDst += nPitch;
        pSrc += nPitch;
    }
    for (intptr_t j = nHeight - 3; j < nHeight - 1; j++) {
        for (int i = 0; i < nWidth; i++) {
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1;
        }

        pDst += nPitch;
        pSrc += nPitch;
    }
    /* last row */
    for (int i = 0; i < nWidth; i++)
        pDst[i] = pSrc[i];
}


template <typename PixelType>
static void HorizontalBicubic(uint8_t *VS_RESTRICT pDst8, const uint8_t *VS_RESTRICT pSrc8,
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) noexcept {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < nHeight; j++) {
        pDst[0] = (pSrc[0] + pSrc[1] + 1) >> 1;
        for (int i = 1; i < nWidth - 3; i++) {
            pDst[i] = std::min(pixelMax, std::max(0,
                (-(pSrc[i - 1] + pSrc[i + 2]) + (pSrc[i] + pSrc[i + 1]) * 9 + 8) >> 4));
        }
        for (intptr_t i = nWidth - 3; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];
        pDst += nPitch;
        pSrc += nPitch;
    }
}


template <typename PixelType>
static void Average2(uint8_t *VS_RESTRICT pDst8, const uint8_t *VS_RESTRICT pSrc18, const uint8_t *VS_RESTRICT pSrc28,
    ptrdiff_t nPitch, int nWidth, int nHeight) noexcept {
    PixelType *pDst = (PixelType *)pDst8;
    const PixelType *pSrc1 = (const PixelType *)pSrc18;
    const PixelType *pSrc2 = (const PixelType *)pSrc28;

    nPitch /= sizeof(PixelType);

    for (int j = 0; j < nHeight; j++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = (pSrc1[i] + pSrc2[i] + 1) >> 1;

        pDst += nPitch;
        pSrc1 += nPitch;
        pSrc2 += nPitch;
    }
}


template<typename PixelType>
void PyramidPlane::GeneratePelPlanes(int pel, SharpParam sharp, VSCore *core, const VSAPI *vsapi) noexcept {
    nPel = pel;

    typedef void (*RefineFunction)(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample);

    RefineFunction refine[3];

    if (sharp == SharpParam::Bilinear) {
        refine[0] = HorizontalBilinear<PixelType>;
        refine[1] = VerticalBilinear<PixelType>;
        refine[2] = DiagonalBilinear<PixelType>;
    } else if (sharp == SharpParam::Bicubic) {
        refine[0] = refine[2] = HorizontalBicubic<PixelType>;
        refine[1] = VerticalBicubic<PixelType>;
    } else { // Wiener
        refine[0] = refine[2] = HorizontalWiener<PixelType>;
        refine[1] = VerticalWiener<PixelType>;
    }

    const uint8_t *src[3] = {};
    uint8_t *dst[3] = {};

    const VSVideoFormat *format = vsapi->getVideoFrameFormat(storage[0]);
    if (nPel == 2) {
        for (int i = 0; i < 3; i++) {
            VSFrame *dstFrame = vsapi->newVideoFrame(format, nPaddedWidth, nPaddedHeight, nullptr, core);
            storage[i + 1] = dstFrame;
            dst[i] = vsapi->getWritePtr(dstFrame, 0);
            pPlane[i + 1] = dst[i];
        }
        src[0] = src[1] = pPlane[0];
        if (sharp == SharpParam::Bilinear)
            src[2] = pPlane[0];
        else
            src[2] = pPlane[2];

        for (int i = 0; i < 3; i++)
            refine[i](dst[i], src[i], nPitch, nPaddedWidth, nPaddedHeight, format->bitsPerSample);
    } else if (nPel == 4) {
        uint8_t *pPlaneW[16] = {};
        for (int i = 0; i < 15; i++) {
            VSFrame *dstFrame = vsapi->newVideoFrame(format, nPaddedWidth, nPaddedHeight, nullptr, core);
            storage[i + 1] = dstFrame;
            pPlaneW[i + 1] = vsapi->getWritePtr(dstFrame, 0);
            pPlane[i + 1] = pPlaneW[i + 1];
        }

        dst[0] = pPlaneW[2];
        dst[1] = pPlaneW[8];
        dst[2] = pPlaneW[10];
        src[0] = src[1] = pPlane[0];
        if (sharp == SharpParam::Bilinear)
            src[2] = pPlane[0];
        else
            src[2] = pPlane[8];

        for (int i = 0; i < 3; i++)
            refine[i](dst[i], src[i], nPitch, nPaddedWidth, nPaddedHeight, format->bitsPerSample);

        // now interpolate intermediate
        Average2<PixelType>(pPlaneW[1], pPlane[0], pPlane[2], nPitch, nPaddedWidth, nPaddedHeight);
        Average2<PixelType>(pPlaneW[9], pPlane[8], pPlane[10], nPitch, nPaddedWidth, nPaddedHeight);
        Average2<PixelType>(pPlaneW[4], pPlane[0], pPlane[8], nPitch, nPaddedWidth, nPaddedHeight);
        Average2<PixelType>(pPlaneW[6], pPlane[2], pPlane[10], nPitch, nPaddedWidth, nPaddedHeight);
        Average2<PixelType>(pPlaneW[5], pPlane[4], pPlane[6], nPitch, nPaddedWidth, nPaddedHeight);

        Average2<PixelType>(pPlaneW[3], pPlane[0] + sizeof(PixelType), pPlane[2], nPitch, nPaddedWidth - 1, nPaddedHeight);
        Average2<PixelType>(pPlaneW[11], pPlane[8] + sizeof(PixelType), pPlane[10], nPitch, nPaddedWidth - 1, nPaddedHeight);
        Average2<PixelType>(pPlaneW[12], pPlane[0] + nPitch, pPlane[8], nPitch, nPaddedWidth, nPaddedHeight - 1);
        Average2<PixelType>(pPlaneW[14], pPlane[2] + nPitch, pPlane[10], nPitch, nPaddedWidth, nPaddedHeight - 1);
        Average2<PixelType>(pPlaneW[13], pPlane[12], pPlane[14], nPitch, nPaddedWidth, nPaddedHeight);
        Average2<PixelType>(pPlaneW[7], pPlane[4] + sizeof(PixelType), pPlane[6], nPitch, nPaddedWidth - 1, nPaddedHeight);
        Average2<PixelType>(pPlaneW[15], pPlane[12] + sizeof(PixelType), pPlane[14], nPitch, nPaddedWidth - 1, nPaddedHeight);
    }

    nVPaddingPel = nVPadding * nPel;
    nHPaddingPel = nHPadding * nPel;
}


template<typename PixelType>
void PyramidPlane::SetExtPel2(const VSFrame *pelFrame, int plane, VSCore *core, const VSAPI *vsapi) {
    const PixelType *pSrc2x = reinterpret_cast<const PixelType *>(vsapi->getReadPtr(pelFrame, plane));
    ptrdiff_t nSrc2xPitch = vsapi->getStride(pelFrame, plane);

    PixelType *pp[4] = {};
    const VSVideoFormat *format = vsapi->getVideoFrameFormat(storage[0]);

    for (int i = 1; i < 4; i++) {
        VSFrame *dstFrame = vsapi->newVideoFrame(format, nPaddedWidth, nPaddedHeight, nullptr, core);
        storage[i] = dstFrame;
        pPlane[i] = vsapi->getWritePtr(dstFrame, 0);
        pp[i] = reinterpret_cast<PixelType *>(vsapi->getWritePtr(dstFrame, 0));
    }
    nSrc2xPitch /= sizeof(PixelType);
    ptrdiff_t nPitchTmp = nPitch / sizeof(PixelType);

    ptrdiff_t offset = nPitchTmp * nVPadding + nHPadding;
    pp[1] += offset;
    pp[2] += offset;
    pp[3] += offset;

    for (int h = 0; h < nRealHeight; h++) {
        for (int w = 0; w < nRealWidth; w++) {
            pp[1][w] = pSrc2x[(w << 1) + 1];
            pp[2][w] = pSrc2x[(w << 1) + nSrc2xPitch];
            pp[3][w] = pSrc2x[(w << 1) + nSrc2xPitch + 1];
        }
        pp[1] += nPitchTmp;
        pp[2] += nPitchTmp;
        pp[3] += nPitchTmp;
        pSrc2x += nSrc2xPitch * 2;
    }

    for (int i = 1; i < 4; i++)
        PadPlaneData<PixelType>(i);
}


template<typename PixelType>
void PyramidPlane::SetExtPel4(const VSFrame *pelFrame, int plane, VSCore *core, const VSAPI *vsapi) {
    const PixelType *pSrc2x = reinterpret_cast<const PixelType *>(vsapi->getReadPtr(pelFrame, plane));
    ptrdiff_t nSrc2xPitch = vsapi->getStride(pelFrame, plane);

    PixelType *pp[16] = {};
    const VSVideoFormat *format = vsapi->getVideoFrameFormat(storage[0]);

    for (int i = 1; i < 16; i++) {
        VSFrame *dstFrame = vsapi->newVideoFrame(format, nPaddedWidth, nPaddedHeight, nullptr, core);
        storage[i] = dstFrame;
        pPlane[i] = vsapi->getWritePtr(dstFrame, 0);
        pp[i] = reinterpret_cast<PixelType *>(vsapi->getWritePtr(dstFrame, 0));
    }

    nSrc2xPitch /= sizeof(PixelType);
    ptrdiff_t nPitchTmp = nPitch / sizeof(PixelType);

    ptrdiff_t offset = nPitchTmp * nVPadding + nHPadding;
    for (int i = 1; i < 16; i++)
        pp[i] += offset;

    for (int h = 0; h < nRealHeight; h++) {
        for (int w = 0; w < nRealWidth; w++) {
            pp[1][w] = pSrc2x[(w << 2) + 1];
            pp[2][w] = pSrc2x[(w << 2) + 2];
            pp[3][w] = pSrc2x[(w << 2) + 3];
            pp[4][w] = pSrc2x[(w << 2) + nSrc2xPitch];
            pp[5][w] = pSrc2x[(w << 2) + nSrc2xPitch + 1];
            pp[6][w] = pSrc2x[(w << 2) + nSrc2xPitch + 2];
            pp[7][w] = pSrc2x[(w << 2) + nSrc2xPitch + 3];
            pp[8][w] = pSrc2x[(w << 2) + nSrc2xPitch * 2];
            pp[9][w] = pSrc2x[(w << 2) + nSrc2xPitch * 2 + 1];
            pp[10][w] = pSrc2x[(w << 2) + nSrc2xPitch * 2 + 2];
            pp[11][w] = pSrc2x[(w << 2) + nSrc2xPitch * 2 + 3];
            pp[12][w] = pSrc2x[(w << 2) + nSrc2xPitch * 3];
            pp[13][w] = pSrc2x[(w << 2) + nSrc2xPitch * 3 + 1];
            pp[14][w] = pSrc2x[(w << 2) + nSrc2xPitch * 3 + 2];
            pp[15][w] = pSrc2x[(w << 2) + nSrc2xPitch * 3 + 3];
        }
        for (int i = 1; i < 16; i++)
            pp[i] += nPitchTmp;
        pSrc2x += nSrc2xPitch * 4;
    }

    for (int i = 1; i < 16; i++)
        PadPlaneData<PixelType>(i);
}


template<typename PixelType>
void PyramidPlane::SetExternalPelPlanes(const VSFrame *pelFrame, int pel, int plane, VSCore *core, const VSAPI *vsapi) {
    nPel = pel;

    if (nPel == 2) {
        SetExtPel2<PixelType>(pelFrame, plane, core, vsapi);
    } else if (nPel == 4) {
        SetExtPel4<PixelType>(pelFrame, plane, core, vsapi);
    }

    nVPaddingPel = nVPadding * nPel;
    nHPaddingPel = nHPadding * nPel;
}

template<typename PixelType>
void PyramidPlane::PadPlaneData(int plane) noexcept {
    PixelType *dstP = (PixelType *)(pPlane[plane]);
    ptrdiff_t nUsedPich = nPitch / sizeof(PixelType);

    // Pad sides by extending the edges
    dstP += nUsedPich * nVPadding;

    for (int h = 0; h < nRealHeight; h++) {
        PixelType padValueLeft = dstP[nHPadding];
        for (int w = 0; w < nHPadding; w++)
            dstP[w] = padValueLeft;
        PixelType padValueRight = dstP[nHPadding + nRealWidth - 1];
        for (int w = nRealWidth + nHPadding; w < nPaddedWidth; w++)
            dstP[w] = padValueRight;
        dstP += nUsedPich;
    }

    // Top and bottom padding by copying the first and last actual image line that's already been extended horizontally
    const PixelType *dstPLastLine = dstP - nUsedPich;
    for (int h = 0; h < nPaddedHeight - nVPadding - nRealHeight; h++) {
        memcpy(dstP, dstPLastLine, nPitch);
        dstP += nUsedPich;
    }

    dstP = (PixelType *)(pPlane[plane]);
    const PixelType *dstPFirstLine = dstP + nUsedPich * nVPadding;
    for (int h = 0; h < nVPadding; h++) {
        memcpy(dstP, dstPFirstLine, nPitch);
        dstP += nUsedPich;
    }
}

void PyramidPlane::FromExternalPlane(const VSFrame *planeFrame, int hPad, int vPad, VSCore *core, const VSAPI *vsapi) noexcept {
    const VSVideoFormat *format = vsapi->getVideoFrameFormat(planeFrame);
    storage[0] = planeFrame;
    pPlane[0] = vsapi->getReadPtr(planeFrame, 0);
    nPitch = vsapi->getStride(planeFrame, 0);
    nHPadding = hPad;
    nVPadding = vPad;
    nHPaddingPel = nHPadding;
    nVPaddingPel = nVPadding;
    nOffsetPadding = nPitch * nVPadding + nHPadding * format->bytesPerSample;

    nPaddedWidth = vsapi->getFrameWidth(planeFrame, 0);
    nPaddedHeight = vsapi->getFrameHeight(planeFrame, 0);

    nWidth = nPaddedWidth - 2 * nHPadding;
    nHeight = nPaddedHeight - 2 * nVPadding;
}

void PyramidPlane::FromExternalPelPlanes(const VSFrame *const *planeFrames, int pel, int hPad, int vPad, VSCore *core, const VSAPI *vsapi) {
    assert(pel == 2 || pel == 4);
    nPel = pel;
    const VSVideoFormat *format = vsapi->getVideoFrameFormat(planeFrames[0]);
    nPitch = vsapi->getStride(planeFrames[0], 0);
    nHPadding = hPad;
    nVPadding = vPad;
    nHPaddingPel = nHPadding * pel;
    nVPaddingPel = nVPadding * pel;

    nOffsetPadding = nPitch * nVPadding + nHPadding * format->bytesPerSample;

    // FIXME, check so everything is the same format and dimensions

    for (int i = 0; i < pel * pel; i++) {
        storage[i] = planeFrames[i];
        pPlane[i] = vsapi->getReadPtr(planeFrames[i], 0);
    }

    nPaddedWidth = vsapi->getFrameWidth(planeFrames[0], 0);
    nPaddedHeight = vsapi->getFrameHeight(planeFrames[0], 0);

    nWidth = nPaddedWidth - 2 * nHPadding;
    nHeight = nPaddedHeight - 2 * nVPadding;
}

int GetPyramidLevelForBlockSize(int blkSizeX, int blkSizeY, int overlapX, int overlapY, int levels) {
    int level = 0;
    while (level < levels - 1) {
        int levelBlkSizeX = (blkSizeX - overlapX) << level;
        int levelBlkSizeY = (blkSizeY - overlapY) << level;
        if (levelBlkSizeX >= 64 || levelBlkSizeY >= 64)
            break;
        level++;
    }
    return level;
}

FramePyramid::FramePyramid(const VSFrame *srcFrame, int levels, int nBlkSizeX, int nBlkSizeY, int nOverlapX, int nOverlapY, int hPad, int vPad, RFilterParam rFilter, VSCore *core, const VSAPI *vsapi)
: core(core), vsapi(vsapi) {
    if (!srcFrame)
        throw SuperPyramidError("Invalid source frame");
    if (levels < 1)
        throw SuperPyramidError("Must have at least one level");
    if (hPad <= 0)
        throw SuperPyramidError("Horizontal padding must be positive");
    if (vPad <= 0)
        throw SuperPyramidError("Vertical padding must be positive");

    pyramidLevels.resize(levels);
    const VSVideoFormat *format = vsapi->getVideoFrameFormat(srcFrame);
    bitsPerSample = format->bitsPerSample;
    bytesPerSample = format->bytesPerSample;
    chroma = (format->colorFamily != cfGray);

    // This works because we only support one level of subsampling
    const VSVideoFormat *srcFormat = vsapi->getVideoFrameFormat(srcFrame);
    assert(srcFormat->subSamplingW <= 1 && srcFormat->subSamplingH <= 1);
    if (chroma) {
        xRatioUV = srcFormat->subSamplingW + 1;
        yRatioUV = srcFormat->subSamplingH + 1;
    }

    nHPad[0] = hPad;
    nHPad[1] = hPad / xRatioUV;
    nHPad[2] = hPad / xRatioUV;

    nVPad[0] = vPad;
    nVPad[1] = vPad / yRatioUV;
    nVPad[2] = vPad / yRatioUV;

    for (int plane = 0; plane < format->numPlanes; plane++) {
        nRealWidth[plane] = vsapi->getFrameWidth(srcFrame, plane);
        nRealHeight[plane] = vsapi->getFrameHeight(srcFrame, plane);

        nWidth[plane] = nRealWidth[plane];
        nHeight[plane] = nRealHeight[plane];
    }



    if (nBlkSizeX > 0 && nOverlapX >= 0) {
        int nBlkX = (nRealWidth[0] - nOverlapX) / (nBlkSizeX - nOverlapX);
        int nWidth_B = (nBlkSizeX - nOverlapX) * nBlkX + nOverlapX;
        if (nWidth_B < nRealWidth[0]) {
            ++nBlkX;
            nWidth[0] = (nBlkSizeX - nOverlapX) * nBlkX + nOverlapX;
            nWidth[1] = nWidth[0] / xRatioUV;
            nWidth[2] = nWidth[0] / xRatioUV;
        }
    }

    if (nBlkSizeY > 0 && nOverlapY >= 0) {
        int nBlkY = (nRealHeight[0] - nOverlapY) / (nBlkSizeY - nOverlapY);
        int nHeight_B = (nBlkSizeY - nOverlapY) * nBlkY + nOverlapY;
        if (nHeight_B < nRealHeight[0]) {
            ++nBlkY;
            nHeight[0] = (nBlkSizeY - nOverlapY) * nBlkY + nOverlapY;
            nHeight[1] = nHeight[0] / yRatioUV;
            nHeight[2] = nHeight[0] / yRatioUV;
        }
    }

    size_t tempBufferSize = (nWidth[0] * format->bytesPerSample * 8); // FIXME, roud up nicer?

    uint8_t *tempBuffer = vsh::vsh_aligned_malloc<uint8_t>(tempBufferSize, 32);

    // FIXME, also limit the number of levels generated based on blksize to save memory
    if (format->bytesPerSample == 1) {
        for (int plane = 0; plane < (chroma ? 3 : 1); plane++) {
            pyramidLevels[0].planes[plane].CopyAndPadPlane<uint8_t>(srcFrame, plane, nHPad[plane], nVPad[plane], nWidth[plane] - nRealWidth[plane], nHeight[plane] - nRealHeight[plane], core, vsapi);
            for (int i = 1; i < levels; i++)
                pyramidLevels[i].planes[plane].ReducePlane<uint8_t>(pyramidLevels[i - 1].planes[plane], xRatioUV, yRatioUV, rFilter, tempBuffer, core, vsapi);
        }
    } else {
        for (int plane = 0; plane < (chroma ? 3 : 1); plane++) {
            pyramidLevels[0].planes[plane].CopyAndPadPlane<uint16_t>(srcFrame, plane, nHPad[plane], nVPad[plane], nWidth[plane] - nRealWidth[plane], nHeight[plane] - nRealHeight[plane], core, vsapi);
            for (int i = 1; i < levels; i++)
                pyramidLevels[i].planes[plane].ReducePlane<uint16_t>(pyramidLevels[i - 1].planes[plane], xRatioUV, yRatioUV, rFilter, tempBuffer, core, vsapi);
        }
    }

    vsh::vsh_aligned_free(tempBuffer);
}


FramePyramid::FramePyramid(const VSFrame *srcFrame, int maxLevel, const std::string &prefix, VSCore *core, const VSAPI *vsapi)
: core(core), vsapi(vsapi) {

    if (!srcFrame)
        throw SuperPyramidError("Invalid source frame");

    serializedData = srcFrame;

    const VSMap *props = vsapi->getFramePropertiesRO(srcFrame);
    int err;
    xRatioUV = vsapi->mapGetIntSaturated(props, (prefix + "SuperXRatioUV").c_str(), 0, &err);
    yRatioUV = vsapi->mapGetIntSaturated(props, (prefix + "SuperYRatioUV").c_str(), 0, &err);
    nWidth[0] = vsapi->mapGetIntSaturated(props, (prefix + "SuperWidth").c_str(), 0, &err);
    nHeight[0] = vsapi->mapGetIntSaturated(props, (prefix + "SuperHeight").c_str(), 0, &err);
    nRealWidth[0] = vsapi->mapGetIntSaturated(props, (prefix + "SuperRealWidth").c_str(), 0, &err);
    nRealHeight[0] = vsapi->mapGetIntSaturated(props, (prefix + "SuperRealHeight").c_str(), 0, &err);
    nHPad[0] = vsapi->mapGetIntSaturated(props, (prefix + "SuperHPad").c_str(), 0, &err);
    nVPad[0] = vsapi->mapGetIntSaturated(props, (prefix + "SuperVPad").c_str(), 0, &err);

    nPel = vsapi->mapGetIntSaturated(props, (prefix + "SuperPel").c_str(), 0, &err);
    int levels = vsapi->mapGetIntSaturated(props, (prefix + "SuperLevels").c_str(), 0, &err);
    chroma = !!vsapi->mapGetInt(props, (prefix + "SuperChroma").c_str(), 0, &err );

    if (xRatioUV < 1 || yRatioUV < 1 || xRatioUV > 2 || yRatioUV > 2 || nRealWidth[0] > nWidth[0] || nRealHeight[0] > nHeight[0] || nVPad[0] < 0 || nHPad[0] < 0 || nRealHeight[0] < 1 || nRealWidth[0] < 1 || levels < 1 || (nPel != 1 && nPel != 2 && nPel != 4))
        throw SuperPyramidError("Invalid super frame metadata");

    if (chroma) {
        nWidth[1] = nWidth[0] / xRatioUV;
        nWidth[2] = nWidth[0] / xRatioUV;
        nHeight[1] = nHeight[0] / yRatioUV;
        nHeight[2] = nHeight[0] / yRatioUV;
        nRealWidth[1] = nRealWidth[0] / xRatioUV;
        nRealWidth[2] = nRealWidth[0] / xRatioUV;
        nRealHeight[1] = nRealHeight[0] / yRatioUV;
        nRealHeight[2] = nRealHeight[0] / yRatioUV;
        nHPad[1] = nHPad[0] / xRatioUV;
        nHPad[2] = nHPad[0] / xRatioUV;
        nVPad[1] = nVPad[0] / yRatioUV;
        nVPad[2] = nVPad[0] / yRatioUV;
    }

    // FIXME, check so all levels match the declared metadata sizes

    int loadLevels = (maxLevel < 0) ? levels : std::min(maxLevel, levels);

    try {

        pyramidLevels.resize(levels);

        if (nPel > 1 && loadLevels > 0) {
            std::string propStr = prefix + "SuperLevel0";
            for (int plane = 0; plane < (chroma ? 3 : 1); plane++) {
                const VSFrame *pelPlanes[16] = {};
                int idxOffset = plane * nPel * nPel;
                for (int i = 0; i < nPel * nPel; i++) {
                    const VSFrame *frame = vsapi->mapGetFrame(props, propStr.c_str(), idxOffset + i, &err);
                    if (!frame)
                        throw SuperPyramidError("Plane data missing in super frame metadata");
                    pelPlanes[i] = frame;
                }
                pyramidLevels[0].planes[plane].FromExternalPelPlanes(pelPlanes, nPel, nHPad[plane], nVPad[plane], core, vsapi);
            }
        }

        for (int level = (nPel > 1) ? 1 : 0; level < loadLevels; level++) {
            std::string propStr = prefix + "SuperLevel" + std::to_string(level);
            for (int plane = 0; plane < (chroma ? 3 : 1); plane++) {
                const VSFrame *frame = vsapi->mapGetFrame(props, propStr.c_str(), plane, &err);
                if (!frame)
                    throw SuperPyramidError("Plane data missing in super frame metadata");
                pyramidLevels[level].planes[plane].FromExternalPlane(frame, nHPad[plane], nVPad[plane], core, vsapi);
            }
        }

    } catch (...) {
        FreeFrames();
        throw;
    }
}

void FramePyramid::FreeFrames() noexcept {
    for (auto &level : pyramidLevels) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 16; j++) {
                vsapi->freeFrame(level.planes[i].storage[j]);
            }
        }
    }

    vsapi->freeFrame(serializedData);
}

// FIXME, this should probably be per level if switched to one frame per level instead of one frame per plane
FramePyramid::~FramePyramid() {
    FreeFrames();
}

void FramePyramid::GeneratePelPlanes(int pel, SharpParam sharp, VSCore *core, const VSAPI *vsapi) {
    if (nPel > 1)
        throw SuperPyramidError("Pel planes have already been generated");
    if (pel != 2 && pel != 4)
        throw SuperPyramidError("Pel value must be 2 or 4");
    if (bytesPerSample == 1) {
        for (int plane = 0; plane < (chroma ? 3 : 1); plane++)
            pyramidLevels[0].planes[plane].GeneratePelPlanes<uint8_t>(pel, sharp, core, vsapi);
    } else {
        for (int plane = 0; plane < (chroma ? 3 : 1); plane++)
            pyramidLevels[0].planes[plane].GeneratePelPlanes<uint16_t>(pel, sharp, core, vsapi);
    }
    nPel = pel;
}

void FramePyramid::SetExternalPelPlanes(const VSFrame *pelFrame, int pel, VSCore *core, const VSAPI *vsapi) {
    if (nPel != 1)
        throw SuperPyramidError("Pel planes already set");
    if (pel != 2 && pel != 4)
        throw SuperPyramidError("Invalid pel value");

    assert(pyramidLevels[0].planes[0].storage[0]);

    const VSFrame *storageFrame = pyramidLevels[0].planes[0].storage[0];

    const VSVideoFormat *pelFormat = vsapi->getVideoFrameFormat(pelFrame);
    const VSVideoFormat *format = vsapi->getVideoFrameFormat(storageFrame);

    if (!vsh::isSameVideoFormat(pelFormat, format))
        throw SuperPyramidError("Pel frame format does not match source frame format");

    if (vsapi->getFrameWidth(pelFrame, 0) != vsapi->getFrameWidth(storageFrame, 0) * pel ||
        vsapi->getFrameHeight(pelFrame, 0) != vsapi->getFrameHeight(storageFrame, 0) * pel)
        throw SuperPyramidError("Pel frame dimensions are not a suitable multiple of the source frame dimensions");

    if (bytesPerSample == 1) {
        for (int plane = 0; plane < (chroma ? 3 : 1); plane++)
            pyramidLevels[0].planes[plane].SetExternalPelPlanes<uint8_t>(pelFrame, pel, plane, core, vsapi);
    } else {
        for (int plane = 0; plane < (chroma ? 3 : 1); plane++)
            pyramidLevels[0].planes[plane].SetExternalPelPlanes<uint16_t>(pelFrame, pel, plane, core, vsapi);
    }
    nPel = pel;
}

void FramePyramid::ExportFrameData(VSFrame *dst, const std::string &prefix) const noexcept {
    VSMap *props = vsapi->getFramePropertiesRW(dst);
    for (int plane = 0; plane < (chroma ? 3 : 1); plane++) {
        assert(pyramidLevels[0].planes[plane].storage[0]);
        for (int i = 0; i < 16; i++) {
            if (pyramidLevels[0].planes[plane].storage[i])
                vsapi->mapSetFrame(props, (prefix + "SuperLevel0").c_str(), pyramidLevels[0].planes[plane].storage[i], maAppend);
        }
    }

    for (int level = 1; level < pyramidLevels.size(); level++) {
        for (int plane = 0; plane < (chroma ? 3 : 1); plane++) {
            assert(pyramidLevels[level].planes[plane].storage[0]);
            vsapi->mapSetFrame(props, (prefix + "SuperLevel" + std::to_string(level)).c_str(), pyramidLevels[level].planes[plane].storage[0], maAppend);
        }
    }

    vsapi->mapSetInt(props, (prefix + "SuperWidth").c_str(), nWidth[0], maReplace);
    vsapi->mapSetInt(props, (prefix + "SuperHeight").c_str(), nHeight[0], maReplace);
    vsapi->mapSetInt(props, (prefix + "SuperRealWidth").c_str(), nRealWidth[0], maReplace);
    vsapi->mapSetInt(props, (prefix + "SuperRealHeight").c_str(), nRealHeight[0], maReplace);
    vsapi->mapSetInt(props, (prefix + "SuperHPad").c_str(), nHPad[0], maReplace);
    vsapi->mapSetInt(props, (prefix + "SuperVPad").c_str(), nVPad[0], maReplace);
    vsapi->mapSetInt(props, (prefix + "SuperPel").c_str(), nPel, maReplace);
    vsapi->mapSetInt(props, (prefix + "SuperLevels").c_str(), pyramidLevels.size(), maReplace);
    vsapi->mapSetInt(props, (prefix + "SuperChroma").c_str(), chroma, maReplace);
    vsapi->mapSetInt(props, (prefix + "SuperXRatioUV").c_str(), xRatioUV, maReplace);
    vsapi->mapSetInt(props, (prefix + "SuperYRatioUV").c_str(), yRatioUV, maReplace);
}

const FramePyramidLevel &FramePyramid::GetLevel(int level) const noexcept {
    assert(level >= 0 && level < static_cast<int>(pyramidLevels.size()));
    return pyramidLevels[level];
}


bool FramePyramid::IsValid() const noexcept {
    return state == State::Valid;
}

bool FramePyramid::IsValidMetadataValid() const noexcept {
    return IsValid() || state == State::ValidMetadataOnly;
}