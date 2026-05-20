#include "SuperPyramid.h"

#include <VSHelper4.h>
#include <cassert>

typedef uint16_t PixelType;
void PyramidPlane::CopyAndPadPlane(const VSFrame *src, int plane, int hPad, int vPad, int nBlkSizePadX, int nBlkSizePadY, VSCore *core, const VSAPI *vsapi) {
    const VSVideoFormat *format = vsapi->getVideoFrameFormat(src);
    VSVideoFormat dstFormat = {};
    vsapi->queryVideoFormat(&dstFormat, cfGray, format->sampleType, format->bitsPerSample, 0, 0, core);
    
    nRealWidth = vsapi->getFrameWidth(src, plane);;
    nRealHeight = vsapi->getFrameHeight(src, plane);;
    nWidth = nRealWidth + nBlkSizePadX;
    nHeight = nRealHeight + nBlkSizePadY;
    nPaddedWidth = nWidth + 2 * hPad;
    nPaddedHeight = nHeight + 2 * vPad;
    nHPadding = hPad;
    nVPadding = vPad;
    nPel = 1;

    VSFrame *dst = vsapi->newVideoFrame(&dstFormat, nPaddedWidth, nPaddedHeight, nullptr, core);
    storage[0] = dst;
    nOffsetPadding = vsapi->getStride(dst, plane) * nVPadding + nHPadding * sizeof(PixelType);

    const PixelType *srcP  = reinterpret_cast<const PixelType *>(vsapi->getReadPtr(src, plane));
    PixelType *dstP = reinterpret_cast<PixelType *>(vsapi->getWritePtr(dst, plane));
    ptrdiff_t srcPitch = vsapi->getStride(src, plane) / sizeof(PixelType);
    ptrdiff_t dstPitch = vsapi->getStride(dst, plane) / sizeof(PixelType);

    // Copy frame data and pad sides by extending the edges
    dstP += dstPitch * vPad;

    for (int h = 0; h < vPad; h++) {
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

    dstP = reinterpret_cast<PixelType *>(vsapi->getWritePtr(dst, plane));
    const PixelType *dstPFirstLine = dstP + dstPitch * vPad;
    for (int h = 0; h < vPad; h++) {
        memcpy(dstP, dstPFirstLine, dstPitch);
        dstP += dstPitch;
    }

    pPlane[0] = vsapi->getReadPtr(storage[0], 0);
}

template <typename PixelType>
static void RB2F_C(uint8_t *VS_RESTRICT pDst8, const uint8_t *VS_RESTRICT pSrc8, ptrdiff_t nDstPitch,
    ptrdiff_t nSrcPitch, int nWidth, int nHeight) {
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

static int PlaneDimensionLuma(int numPixels, int ratioUV, int pad) {
      return (pad >= ratioUV) ? ((numPixels / ratioUV + 1) / 2) * ratioUV : ((numPixels / ratioUV) / 2) * ratioUV;
}


void PyramidPlane::ReducePlane(const PyramidPlane &src, int ratioUV, RFilterParam rFilter, VSCore *core, const VSAPI *vsapi) {
    nVPadding = src.nVPadding;
    nHPadding = src.nHPadding;

    nWidth = PlaneDimensionLuma(src.nWidth, ratioUV, nHPadding);
    nHeight = PlaneDimensionLuma(src.nHeight, ratioUV, nVPadding);

    nPaddedWidth = nWidth + 2 * nHPadding;
    nPaddedHeight = nHeight + 2 * nVPadding;

    VSFrame *dst = vsapi->newVideoFrame(vsapi->getVideoFrameFormat(src.storage[0]), nPaddedWidth, nPaddedHeight, nullptr, core);
    storage[0] = dst;
    uint8_t *dstP = vsapi->getWritePtr(dst, 0);
    pPlane[0] = dstP;
    nOffsetPadding = vsapi->getStride(dst, 0) * nVPadding + nHPadding * sizeof(PixelType);

    if (rFilter == RFilterParam::Simple) {
            RB2F_C<PixelType>(dstP + nOffsetPadding, src.pPlane[0] + src.nOffsetPadding, nPitch, src.nPitch, nWidth, nHeight);
    } else if (rFilter == RFilterParam::Triangle) {
            //RB2Filtered<PixelType>();
    } else if (rFilter == RFilterParam::Bilinear) {
            //RB2BilinearFiltered<PixelType>();
    } else if (rFilter == RFilterParam::Quadratic) {
            //RB2Quadratic<PixelType>();
    } else if (rFilter == RFilterParam::Cubic) {
            //RB2Cubic<PixelType>();
    }
    // FIXME, fix other filters
    RB2F_C<PixelType>(dstP + nOffsetPadding, src.pPlane[0] + src.nOffsetPadding, nPitch, src.nPitch, nWidth, nHeight);

    PadPlaneData(0);
}


template <typename PixelType>
static void VerticalBilinear(uint8_t *VS_RESTRICT pDst8, const uint8_t *VS_RESTRICT pSrc8,
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
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
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
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
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
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
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
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
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
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
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
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
    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
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


// assume all pitches equal
template <typename PixelType>
static void Average2(uint8_t *VS_RESTRICT pDst8, const uint8_t *VS_RESTRICT pSrc18, const uint8_t *VS_RESTRICT pSrc28,
    ptrdiff_t nPitch, int nWidth, int nHeight) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc1 = (PixelType *)pSrc18;
    PixelType *pSrc2 = (PixelType *)pSrc28;

    nPitch /= sizeof(PixelType);

    for (int j = 0; j < nHeight; j++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = (pSrc1[i] + pSrc2[i] + 1) >> 1;

        pDst += nPitch;
        pSrc1 += nPitch;
        pSrc2 += nPitch;
    }
}


void PyramidPlane::GeneratePelPlanes(const VSFrame *pelFrame, int pel, SharpParam sharp, VSCore *core, const VSAPI *vsapi) {
    assert(pel == 2 || pel == 4);
    assert(nPel == 1);
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
}


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
        PadPlaneData(i);
}


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
        PadPlaneData(i);
}


void PyramidPlane::SetExternalPelPlanes(const VSFrame *pelFrame, int pel, int plane, VSCore *core, const VSAPI *vsapi) {
    assert(pel == 2 || pel == 4);
    assert(nPel == 1);
    nPel = pel;

    if (nPel == 2) {
        SetExtPel2(pelFrame, plane, core, vsapi);
    } else if (nPel == 4) {
        SetExtPel4(pelFrame, plane, core, vsapi);
    }
}

void PyramidPlane::PadPlaneData(int plane) {
    PixelType *dstP = (PixelType *)(pPlane[plane]);
    ptrdiff_t nUsedPich = nPitch / sizeof(PixelType);

    // Pad sides by extending the edges
    dstP += nUsedPich * nVPadding;

    for (int h = 0; h < nVPadding; h++) {
        PixelType padValueLeft = dstP[0];
        for (int w = 0; w < nHPadding; w++)
            dstP[w] = padValueLeft;
        PixelType padValueRight = dstP[nRealWidth - 1];
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

FramePyramid::FramePyramid(const VSFrame *srcFrame, int levels, int blkSizeX, int blkSizeY, int overlapX, int overlapY, int hPad, int vPad, RFilterParam rFilter, bool chroma, VSCore *core, const VSAPI *vsapi) {
    assert(levels >= 1);
    pyramidLevels.resize(levels);
    const VSVideoFormat *format = vsapi->getVideoFrameFormat(srcFrame);
    this->chroma = chroma && (format->colorFamily != cfGray);

}