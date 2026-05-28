// Make a motion compensate temporal denoiser
// Copyright(c)2006 A.G.Balakhnin aka Fizick
// See legal notice in Copying.txt for more information

// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA, or visit
// http://www.gnu.org/copyleft/gpl.html .

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <memory>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "MVDegrains.h"
#include "Overlap.h"
#include "CommonMacros.h"
#include "CPU.h"


template<int radius>
struct DegrainData {
    VSNode *node = nullptr;
    VSNode *super = nullptr;
    VSNode *vectors[radius * 2] = {};
    int deltaFrame[radius * 2] = {};

    const VSVideoInfo *vi;

    int64_t thSAD[3];
    int nLimit[3];
    int64_t nSCD1;
    int nSCD2;

    ptrdiff_t dstTempPitch;

    OverlapsFunction OVERS[3];
    DenoiseFunction DEGRAIN[3];
    LimitFunction LimitChanges;
    ToPixelsFunction ToPixels;

    bool process[3];

    int xSubUV;
    int ySubUV;

    int nWidth[3];
    int nHeight[3];
    int nOverlapX[3];
    int nOverlapY[3];
    int nBlkSizeX[3];
    int nBlkSizeY[3];
    int nWidth_B[3];
    int nHeight_B[3];

    OverlapWindows OverWins[3];

    std::string prefix;

    const VSAPI *vsapi;

    DegrainData(const VSAPI *vsapi) : vsapi(vsapi) {}

    ~DegrainData() {
        for (int r = 0; r < radius * 2; r++) {
            if (vectors[r])
                vsapi->freeNode(vectors[r]);
        }
        if (super)
            vsapi->freeNode(super);
        if (node)
            vsapi->freeNode(node);
    }
};

template<int radius, typename PixelType>
static const VSFrame *VS_CC degrainGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) noexcept {
    DegrainData<radius> *d = reinterpret_cast<DegrainData<radius> *>(instanceData);

    if (activationReason == arInitial) {

        for (int r = 0; r < radius * 2; r += 2) {
            //Backward
            vsapi->requestFrameFilter(n, d->vectors[r], frameCtx);
            //Forward
            vsapi->requestFrameFilter(n, d->vectors[r + 1], frameCtx);

            // Backward
            int offB = d->deltaFrame[r];
            if (n + offB < d->vi->numFrames && n + offB >= 0)
                vsapi->requestFrameFilter(n + offB, d->super, frameCtx);

            // Forward
            int offF = d->deltaFrame[r + 1];
            if (n + offF >= 0 && n + offF < d->vi->numFrames)
                vsapi->requestFrameFilter(n + offF, d->super, frameCtx);
        }

        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrame *src = vsapi->getFrameFilter(n, d->node, frameCtx);

        const int pl[] = { 0, 1, 2 };
        const VSFrame *fr[] = {
            d->process[0] ? nullptr : src,
            d->process[1] ? nullptr : src,
            d->process[2] ? nullptr : src
        };

        VSFrame *dst = vsapi->newVideoFrame2(&d->vi->format, d->vi->width, d->vi->height, fr, pl, src, core);

        int bitsPerSample = d->vi->format.bitsPerSample;

        
        constexpr int bytesPerSample = sizeof(PixelType);

        uint8_t *pDst[3] = {};
        uint8_t *pDstCur[3] = {};
        const uint8_t *pSrcCur[3] = {};
        const uint8_t *pSrc[3] = {};
        ptrdiff_t nDstPitches[3] = {};
        ptrdiff_t nSrcPitches[3] = {};
        bool isUsable[radius * 2] = {};

        std::optional<MotionBlockPyramid> fgops[radius * 2];
        
        const VSFrame *refFrames[radius * 2] = {};

        for (int r = 0; r < radius * 2; r++) {
            const VSFrame *frame = vsapi->getFrameFilter(n, d->vectors[r], frameCtx);
            fgops[r].emplace(frame, 0, d->prefix, core, vsapi);
            isUsable[r] = fgops[r]->IsUsable(d->nSCD1, d->nSCD2);
            vsapi->freeFrame(frame);

            if (isUsable[r]) {
                int offset = fgops[r]->nDeltaFrame;
                refFrames[r] = vsapi->getFrameFilter(n + offset, d->super, frameCtx);
            }
        }

        int nLogPel = (fgops[0]->nPel == 4) ? 2 : (fgops[0]->nPel == 2) ? 1 : 0;


        for (int i = 0; i < d->vi->format.numPlanes; i++) {
            pDst[i] = vsapi->getWritePtr(dst, i);
            nDstPitches[i] = vsapi->getStride(dst, i);
            pSrc[i] = vsapi->getReadPtr(src, i);
            nSrcPitches[i] = vsapi->getStride(src, i);

        }

        const int xSubUV = d->xSubUV;
        const int ySubUV = d->ySubUV;
        const int nBlkX = fgops[0]->nBlkX;
        const int nBlkY = fgops[0]->nBlkY;
        const ptrdiff_t dstTempPitch = d->dstTempPitch;
        const int *nWidth = d->nWidth;
        const int *nHeight = d->nHeight;
        const int *nOverlapX = d->nOverlapX;
        const int *nOverlapY = d->nOverlapY;
        const int *nBlkSizeX = d->nBlkSizeX;
        const int *nBlkSizeY = d->nBlkSizeY;
        const int *nWidth_B = d->nWidth_B;
        const int *nHeight_B = d->nHeight_B;
        const int64_t *thSAD = d->thSAD;
        const int *nLimit = d->nLimit;


        std::optional<FramePyramid> pRefGOF[radius * 2];
        for (int r = 0; r < radius * 2; r++)
            if (isUsable[r])
                pRefGOF[r].emplace(refFrames[r], d->prefix, core, vsapi);

        OverlapWindows *OverWins[3] = { &d->OverWins[0], &d->OverWins[1], &d->OverWins[2] };
        uint8_t *DstTemp = nullptr;
        int tmpBlockPitch = nBlkSizeX[0] * bytesPerSample;
        uint8_t *tmpBlock = nullptr;
        if (nOverlapX[0] > 0 || nOverlapY[0] > 0) {
            DstTemp = new uint8_t[dstTempPitch * nBlkSizeY[0]];
            tmpBlock = new uint8_t[tmpBlockPitch * nBlkSizeY[0]];
        }

        const FramePyramidLevel *pPlanes[radius * 2] = {};

        for (int r = 0; r < radius * 2; r++) {
            if (isUsable[r])
                pPlanes[r] = &pRefGOF[r]->GetLevel(0);
        }


        pDstCur[0] = pDst[0];
        pDstCur[1] = pDst[1];
        pDstCur[2] = pDst[2];
        pSrcCur[0] = pSrc[0];
        pSrcCur[1] = pSrc[1];
        pSrcCur[2] = pSrc[2];
        // -----------------------------------------------------------------------------

        for (int plane = 0; plane < d->vi->format.numPlanes; plane++) {
            if (!d->process[plane])
                continue;

            if (nOverlapX[0] == 0 && nOverlapY[0] == 0) {
                for (int by = 0; by < nBlkY; by++) {
                    int xx = 0;
                    for (int bx = 0; bx < nBlkX; bx++) {
                        int i = by * nBlkX + bx;

                        const uint8_t *pointers[radius * 2]; // Moved by the degrain function.
                        ptrdiff_t strides[radius * 2];

                        int WSrc, WRefs[radius * 2];

                        for (int r = 0; r < radius * 2; r++)
                            useBlock<PixelType>(pointers[r], strides[r], WRefs[r], isUsable[r], fgops[r], i, pPlanes[r], pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);

                        normaliseWeights<radius>(WSrc, WRefs);

                        d->DEGRAIN[plane](pDstCur[plane] + xx, nDstPitches[plane], pSrcCur[plane] + xx, nSrcPitches[plane],
                                          pointers, strides,
                                          WSrc, WRefs);

                        xx += nBlkSizeX[plane] * bytesPerSample;

                        if (bx == nBlkX - 1 && nWidth_B[0] < nWidth[0]) // right non-covered region
                            vsh::bitblt(pDstCur[plane] + nWidth_B[plane] * bytesPerSample, nDstPitches[plane],
                                      pSrcCur[plane] + nWidth_B[plane] * bytesPerSample, nSrcPitches[plane],
                                      (nWidth[plane] - nWidth_B[plane]) * bytesPerSample, nBlkSizeY[plane]);
                    }
                    pDstCur[plane] += nBlkSizeY[plane] * (nDstPitches[plane]);
                    pSrcCur[plane] += nBlkSizeY[plane] * (nSrcPitches[plane]);

                    if (by == nBlkY - 1 && nHeight_B[0] < nHeight[0]) // bottom uncovered region
                        vsh::bitblt(pDstCur[plane], nDstPitches[plane],
                                  pSrcCur[plane], nSrcPitches[plane],
                                  nWidth[plane] * bytesPerSample, nHeight[plane] - nHeight_B[plane]);
                }
            } else { // overlap - sliding window
                const int stepY = nBlkSizeY[plane] - nOverlapY[plane];

                // Clear the whole buffer for the first block row
                memset(DstTemp, 0, dstTempPitch * nBlkSizeY[plane]);

                for (int by = 0; by < nBlkY; by++) {
                    int wby = ((by + nBlkY - 3) / (nBlkY - 2)) * 3;
                    int wbx = 0;
                    int xx = 0;

                    // For subsequent rows only clear the new (non-overlap) region;
                    // the top nOverlapY rows were preserved by the memmove below
                    if (by > 0)
                        memset(DstTemp + nOverlapY[plane] * dstTempPitch, 0, dstTempPitch * stepY);

                    for (int bx = 0; bx < nBlkX; bx++) {
                        // select window
                        wbx = bx == nBlkX - 1 ? 2 : wbx;
                        const int16_t *winOver = OverWins[plane]->GetWindow(wby + wbx);

                        int i = by * nBlkX + bx;

                        const uint8_t *pointers[radius * 2]; // Moved by the degrain function.
                        ptrdiff_t strides[radius * 2];

                        int WSrc, WRefs[radius * 2];

                        for (int r = 0; r < radius * 2; r++)
                            useBlock<PixelType>(pointers[r], strides[r], WRefs[r], isUsable[r], fgops[r], i, pPlanes[r], pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);

                        normaliseWeights<radius>(WSrc, WRefs);

                        d->DEGRAIN[plane](tmpBlock, tmpBlockPitch, pSrcCur[plane] + xx, nSrcPitches[plane],
                            pointers, strides,
                            WSrc, WRefs);
                        d->OVERS[plane](DstTemp + xx * 2, dstTempPitch, tmpBlock, tmpBlockPitch, winOver, nBlkSizeX[plane]);

                        xx += (nBlkSizeX[plane] - nOverlapX[plane]) * bytesPerSample;
                        wbx = 1;
                    }

                    // Last block row outputs all nBlkSizeY rows; others output only stepY
                    int rowsToOutput = (by == nBlkY - 1) ? nBlkSizeY[plane] : stepY;
                    int outputHeight = std::min(rowsToOutput, vsapi->getFrameHeight(dst, plane) - by * stepY);
                    int outputWidth = std::min(nWidth_B[plane], vsapi->getFrameWidth(dst, plane));

                    

                    if (outputHeight > 0)
                        d->ToPixels(pDstCur[plane], nDstPitches[plane], DstTemp, dstTempPitch,
                            outputWidth, outputHeight, bitsPerSample);

                    pDstCur[plane] += nDstPitches[plane] * rowsToOutput;
                    pSrcCur[plane] += stepY * nSrcPitches[plane];

                    // Slide: preserve the overlap rows at the top for the next block row
                    if (by < nBlkY - 1)
                        memmove(DstTemp, DstTemp + stepY * dstTempPitch, nOverlapY[plane] * dstTempPitch);
                }

                if (nWidth_B[0] < vsapi->getFrameWidth(dst, plane))
                    vsh::bitblt(pDst[plane] + nWidth_B[plane] * bytesPerSample, nDstPitches[plane],
                        pSrc[plane] + nWidth_B[plane] * bytesPerSample, nSrcPitches[plane],
                        (vsapi->getFrameWidth(dst, plane) - nWidth_B[plane]) * bytesPerSample, nHeight_B[plane]);

                if (nHeight_B[0] < vsapi->getFrameHeight(dst, plane)) // bottom noncovered region
                    vsh::bitblt(pDst[plane] + nDstPitches[plane] * nHeight_B[plane], nDstPitches[plane],
                        pSrc[plane] + nSrcPitches[plane] * nHeight_B[plane], nSrcPitches[plane],
                        nWidth[plane] * bytesPerSample, vsapi->getFrameHeight(dst, plane) - nHeight_B[plane]);
            }

            int pixelMax = (1 << bitsPerSample) - 1;
            if (nLimit[plane] < pixelMax)
                d->LimitChanges(pDst[plane], nDstPitches[plane],
                                pSrc[plane], nSrcPitches[plane],
                                vsapi->getFrameWidth(dst, plane), vsapi->getFrameHeight(dst, plane), nLimit[plane]);
        }


        if (tmpBlock)
            delete[] tmpBlock;

        if (DstTemp)
            delete[] DstTemp;

        vsapi->freeFrame(src);

        return dst;
    }

    return nullptr;
}


template <int radius>
static void VS_CC degrainFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    DegrainData<radius> *d = reinterpret_cast<DegrainData<radius> *>(instanceData);
    delete d;
}


// opt can fit in four bits, if the width and height need more than eight bits each.
#define KEY(width, height, bits, opt) (unsigned)(width) << 24 | (height) << 16 | (bits) << 8 | (opt)

#if defined(MVTOOLS_X86) || defined(MVTOOLS_ARM)
#define DEGRAIN_SSE2(radius, width, height) \
    { KEY(width, height, 8, MVOPT_SSE2), Degrain_sse2<radius, width, height> },

#define DEGRAIN_LEVEL_SSE2(radius)\
    {\
        DEGRAIN_SSE2(radius, 4, 2)\
        DEGRAIN_SSE2(radius, 4, 4)\
        DEGRAIN_SSE2(radius, 4, 8)\
        DEGRAIN_SSE2(radius, 8, 1)\
        DEGRAIN_SSE2(radius, 8, 2)\
        DEGRAIN_SSE2(radius, 8, 4)\
        DEGRAIN_SSE2(radius, 8, 8)\
        DEGRAIN_SSE2(radius, 8, 16)\
        DEGRAIN_SSE2(radius, 16, 1)\
        DEGRAIN_SSE2(radius, 16, 2)\
        DEGRAIN_SSE2(radius, 16, 4)\
        DEGRAIN_SSE2(radius, 16, 8)\
        DEGRAIN_SSE2(radius, 16, 16)\
        DEGRAIN_SSE2(radius, 16, 32)\
        DEGRAIN_SSE2(radius, 32, 8)\
        DEGRAIN_SSE2(radius, 32, 16)\
        DEGRAIN_SSE2(radius, 32, 32)\
        DEGRAIN_SSE2(radius, 32, 64)\
        DEGRAIN_SSE2(radius, 64, 16)\
        DEGRAIN_SSE2(radius, 64, 32)\
        DEGRAIN_SSE2(radius, 64, 64)\
        DEGRAIN_SSE2(radius, 64, 128)\
        DEGRAIN_SSE2(radius, 128, 32)\
        DEGRAIN_SSE2(radius, 128, 64)\
        DEGRAIN_SSE2(radius, 128, 128)\
    }
#else
#define DEGRAIN_SSE2(radius, width, height)
#define DEGRAIN_LEVEL_SSE2(radius)
#endif

#define DEGRAIN(radius, width, height) \
    { KEY(width, height, 8, MVOPT_SCALAR), Degrain_C<radius, width, height, uint8_t> }, \
    { KEY(width, height, 16, MVOPT_SCALAR), Degrain_C<radius, width, height, uint16_t> },

#define DEGRAIN_LEVEL(radius)\
    {\
        DEGRAIN(radius, 2, 2)\
        DEGRAIN(radius, 2, 4)\
        DEGRAIN(radius, 4, 2)\
        DEGRAIN(radius, 4, 4)\
        DEGRAIN(radius, 4, 8)\
        DEGRAIN(radius, 8, 1)\
        DEGRAIN(radius, 8, 2)\
        DEGRAIN(radius, 8, 4)\
        DEGRAIN(radius, 8, 8)\
        DEGRAIN(radius, 8, 16)\
        DEGRAIN(radius, 16, 1)\
        DEGRAIN(radius, 16, 2)\
        DEGRAIN(radius, 16, 4)\
        DEGRAIN(radius, 16, 8)\
        DEGRAIN(radius, 16, 16)\
        DEGRAIN(radius, 16, 32)\
        DEGRAIN(radius, 32, 8)\
        DEGRAIN(radius, 32, 16)\
        DEGRAIN(radius, 32, 32)\
        DEGRAIN(radius, 32, 64)\
        DEGRAIN(radius, 64, 16)\
        DEGRAIN(radius, 64, 32)\
        DEGRAIN(radius, 64, 64)\
        DEGRAIN(radius, 64, 128)\
        DEGRAIN(radius, 128, 32)\
        DEGRAIN(radius, 128, 64)\
        DEGRAIN(radius, 128, 128)\
    }

static const std::unordered_map<uint32_t, DenoiseFunction> degrain_functions[6] = {
    DEGRAIN_LEVEL(1),
    DEGRAIN_LEVEL(2),
    DEGRAIN_LEVEL(3),
    DEGRAIN_LEVEL(4),
    DEGRAIN_LEVEL(5),
    DEGRAIN_LEVEL(6),
};

static const std::unordered_map<uint32_t, DenoiseFunction> degrain_functions_sse2[6] = {
    DEGRAIN_LEVEL_SSE2(1),
    DEGRAIN_LEVEL_SSE2(2),
    DEGRAIN_LEVEL_SSE2(3),
    DEGRAIN_LEVEL_SSE2(4),
    DEGRAIN_LEVEL_SSE2(5),
    DEGRAIN_LEVEL_SSE2(6),
};

static DenoiseFunction selectDegrainFunction(unsigned radius, unsigned width, unsigned height, unsigned bits) {
    DenoiseFunction degrain = degrain_functions[radius - 1].at(KEY(width, height, bits, MVOPT_SCALAR));

    /*
#if defined(MVTOOLS_X86) || defined(MVTOOLS_ARM)
    try {
        degrain = degrain_functions_sse2[radius - 1].at(KEY(width, height, bits, MVOPT_SSE2));
    } catch (std::out_of_range &) { }
#if defined(MVTOOLS_X86)
    if (g_cpuinfo & X264_CPU_AVX2) {
        DenoiseFunction tmp = selectDegrainFunctionAVX2(radius, width, height, bits);
        if (tmp)
            degrain = tmp;
    }
#endif
#endif
*/

    return degrain;
}

#undef DEGRAIN
#undef DEGRAIN_SSE2
#undef DEGRAIN_LEVEL
#undef DEGRAIN_LEVEL_SSE2

#undef KEY


template <int radius>
static void selectFunctions(DegrainData<radius> &d, const MotionBlockPyramid &vectors) {
    const unsigned xRatioUV = vectors.xRatioUV;
    const unsigned yRatioUV = vectors.yRatioUV;
    const unsigned nBlkSizeX = vectors.nBlkSizeX;
    const unsigned nBlkSizeY = vectors.nBlkSizeY;
    const unsigned bits = d.vi->format.bytesPerSample * 8;

    if (d.vi->format.bitsPerSample == 8) {
        d.LimitChanges = LimitChanges_C<uint8_t>;

        d.ToPixels = ToPixels<uint16_t, uint8_t>;

#if defined(MVTOOLS_X86) || defined(MVTOOLS_ARM)
        d.LimitChanges = LimitChanges_sse2;
#endif
    } else {
        d.LimitChanges = LimitChanges_C<uint16_t>;

        d.ToPixels = ToPixels<uint32_t, uint16_t>;
    }

    d.OVERS[0] = selectOverlapsFunction(nBlkSizeX, nBlkSizeY, bits);
    d.DEGRAIN[0] = selectDegrainFunction(radius, nBlkSizeX, nBlkSizeY, bits);

    d.OVERS[1] = d.OVERS[2] = selectOverlapsFunction(nBlkSizeX / xRatioUV, nBlkSizeY / yRatioUV, bits);
    d.DEGRAIN[1] = d.DEGRAIN[2] = selectDegrainFunction(radius, nBlkSizeX / xRatioUV, nBlkSizeY / yRatioUV, bits);
}


template <int radius>
static void VS_CC degrainCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    std::unique_ptr<DegrainData<radius>> d(new DegrainData<radius>(vsapi));

    std::string filter = "Degrain";
    filter.append(std::to_string(radius));

    int err;

    d->thSAD[0] = vsapi->mapGetInt(in, "thsad", 0, &err);
    if (err)
        d->thSAD[0] = 400;

    d->thSAD[1] = d->thSAD[2] = vsapi->mapGetInt(in, "thsadc", 0, &err);
    if (err)
        d->thSAD[1] = d->thSAD[2] = d->thSAD[0];

    // FIXME
    int plane = vsapi->mapGetIntSaturated(in, "plane", 0, &err);
    if (err)
        plane = 4;

    d->nSCD1 = vsapi->mapGetInt(in, "thscd1", 0, &err);
    if (err)
        d->nSCD1 = MV_DEFAULT_SCD1;

    d->nSCD2 = vsapi->mapGetIntSaturated(in, "thscd2", 0, &err);
    if (err)
        d->nSCD2 = MV_DEFAULT_SCD2;

    d->super = vsapi->mapGetNode(in, "super", 0, nullptr);

    const char *prefix = vsapi->mapGetData(in, "prefix", 0, &err);
    if (prefix)
        d->prefix = prefix;
    else
        d->prefix = DEFAULT_MVUTENSILS_PREFIX;

    char errorMsg[ERROR_SIZE + 1] = {};
    const VSFrame *evil = vsapi->getFrame(0, d->super, errorMsg, ERROR_SIZE);
    if (!evil)
        throw std::runtime_error(filter + ": failed to retrieve first frame from super clip. Error message: " + std::string(errorMsg));

        // try FIXME

    FramePyramid evilPyramid(evil, d->prefix, core, vsapi);

    int numVectors = vsapi->mapNumElements(in, "vectors");
    if (numVectors != radius * 2)
        throw std::runtime_error("the number of vector clips must be exactly " + std::to_string(radius * 2));

    std::optional<MotionBlockPyramid> evilVectors[radius * 2];

    for (int r = 0; r < radius * 2; r++) {
        d->vectors[r] = vsapi->mapGetNode(in, "vectors", r, nullptr);
        
        if (vsapi->getNodeType(d->vectors[r]) != mtVideo)
            throw std::runtime_error("invalid vector clip type");

        const VSFrame *vecFrame = vsapi->getFrame(0, d->vectors[r], errorMsg, ERROR_SIZE);
        if (!vecFrame)
            throw std::runtime_error("Failed to retrieve first frame from vector clip " + std::to_string(r) + ": " + std::string(errorMsg));

        // FIXME, free frame if throwing
        evilVectors[r].emplace(vecFrame, 1, d->prefix, core, vsapi);

        vsapi->freeFrame(vecFrame);

        if (r > 0 && !evilVectors[r]->IsCompatible(*evilVectors[r - 1]))
            throw std::runtime_error("Incompatible vector formats");

        d->deltaFrame[r] = evilVectors[r]->nDeltaFrame;
    }

    d->node = vsapi->mapGetNode(in, "clip", 0, nullptr);
    d->vi = vsapi->getVideoInfo(d->node);

    int64_t nSCD1_old = d->nSCD1;
    evilVectors[0]->ScaleThSCD(d->nSCD1, d->nSCD2, d->vi->format.bitsPerSample);


    // bw1, fw1, bw2, fw2, ...

    /*

    for (int r = 0; r < radius * 2; r++)

#define CHECK_VECTORS(rThreshold, backwardN, forwardN, backwardP, forwardP, mvbwN, mvfwN, mvbwP, mvfwP)\
    if (radius > rThreshold) {\
        if (!d->vectors_data[backwardN].isBackward)\
            snprintf(error, ERROR_SIZE, "%s", "mvbw must be generated with isb=True.");\
        if (d->vectors_data[forwardN].isBackward)\
            snprintf(error, ERROR_SIZE, "%s", "mvfw must be generated with isb=False.");\
        if (d->vectors_data[backwardN].nDeltaFrame <= d->vectors_data[backwardP].nDeltaFrame)\
            snprintf(error, ERROR_SIZE, "%s", "mvbwN must have greater delta than mvbwP.");\
        if (d->vectors_data[forwardN].nDeltaFrame <= d->vectors_data[forwardP].nDeltaFrame)\
            snprintf(error, ERROR_SIZE, "%s", "mvfwN must have greater delta than mvfwP.");\
    }

    // Make sure the motion vector clips are correct.
    if (!d->vectors_data[Backward1].isBackward)
        snprintf(error, ERROR_SIZE, "%s", "mvbw must be generated with isb=True.");
    if (d->vectors_data[Forward1].isBackward)
        snprintf(error, ERROR_SIZE, "%s", "mvfw must be generated with isb=False.");

    CHECK_VECTORS(1, Backward2, Forward2, Backward1, Forward1, mvbw2, mvfw2, mvbw, mvfw)
    CHECK_VECTORS(2, Backward3, Forward3, Backward2, Forward2, mvbw3, mvfw3, mvbw2, mvfw2)
    CHECK_VECTORS(3, Backward4, Forward4, Backward3, Forward3, mvbw4, mvfw4, mvbw3, mvfw3)
    CHECK_VECTORS(4, Backward5, Forward5, Backward4, Forward4, mvbw5, mvfw5, mvbw4, mvfw4)
    CHECK_VECTORS(5, Backward6, Forward6, Backward5, Forward5, mvbw6, mvfw6, mvbw5, mvfw5)

#undef CHECK_VECTORS

*/
    d->thSAD[0] = d->thSAD[0] * d->nSCD1 / nSCD1_old;              // normalize to block SAD
    d->thSAD[1] = d->thSAD[2] = d->thSAD[1] * d->nSCD1 / nSCD1_old; // chroma threshold, normalized to block SAD

    if (d->thSAD[0] >= INT_MAX || d->thSAD[1] >= INT_MAX) {
        int64_t maximum = INT_MAX * nSCD1_old / d->nSCD1;

        bool c = d->thSAD[0] < INT_MAX;

        vsapi->mapSetError(out, (filter + ": with this block size and video format, thsad" + (c ? "c" : "") + " must not exceed " + std::to_string(maximum) + " or some calculations would overflow").c_str());

        return;
    }




    const VSVideoInfo *supervi = vsapi->getVideoInfo(d->super);
    int nSuperWidth = supervi->width;

    if (evilVectors[0]->nHeight != evilPyramid.nHeight[0] || evilVectors[0]->nRealHeight != d->vi->height || evilVectors[0]->nWidth != evilPyramid.nWidth[0] || evilVectors[0]->nRealWidth != d->vi->width || evilVectors[0]->nPel != evilPyramid.nPel) {
        vsapi->mapSetError(out, (filter + ": wrong source or super clip frame size").c_str());
        return;
    }

    if (!vsh::isConstantVideoFormat(d->vi) || d->vi->format.bitsPerSample > 16 || d->vi->format.sampleType != stInteger || d->vi->format.subSamplingW > 1 || d->vi->format.subSamplingH > 1 || (d->vi->format.colorFamily != cfYUV && d->vi->format.colorFamily != cfGray)) {
        vsapi->mapSetError(out, (filter + ": input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant dimensions").c_str());
        return;
    }

    int pixelMax = (1 << d->vi->format.bitsPerSample) - 1;

    d->nLimit[0] = vsapi->mapGetIntSaturated(in, "limit", 0, &err);
    if (err)
        d->nLimit[0] = pixelMax;

    d->nLimit[1] = d->nLimit[2] = vsapi->mapGetIntSaturated(in, "limitc", 0, &err);
    if (err)
        d->nLimit[1] = d->nLimit[2] = d->nLimit[0];

    if (d->nLimit[0] < 0 || d->nLimit[0] > pixelMax) {
        vsapi->mapSetError(out, (filter + ": limit must be between 0 and " + std::to_string(pixelMax)).c_str());
        return;
    }

    if (d->nLimit[1] < 0 || d->nLimit[1] > pixelMax) {
        vsapi->mapSetError(out, (filter + ": limitc must be between 0 and " + std::to_string(pixelMax)).c_str());
        return;
    }


    d->dstTempPitch = ((evilVectors[0]->nWidth + 15) / 16) * 16 * d->vi->format.bytesPerSample * 2;


    // FIXME, take planes argument into account
    d->process[0] = true;
    d->process[1] = true;
    d->process[2] = true;

    d->xSubUV = d->vi->format.subSamplingW;
    d->ySubUV = d->vi->format.subSamplingH;

    d->nWidth[0] = evilVectors[0]->nWidth;
    d->nWidth[1] = d->nWidth[2] = d->nWidth[0] >> d->xSubUV;

    d->nHeight[0] = evilVectors[0]->nHeight;
    d->nHeight[1] = d->nHeight[2] = d->nHeight[0] >> d->ySubUV;

    d->nOverlapX[0] = evilVectors[0]->nOverlapX;
    d->nOverlapX[1] = d->nOverlapX[2] = d->nOverlapX[0] >> d->xSubUV;

    d->nOverlapY[0] = evilVectors[0]->nOverlapY;
    d->nOverlapY[1] = d->nOverlapY[2] = d->nOverlapY[0] >> d->ySubUV;

    d->nBlkSizeX[0] = evilVectors[0]->nBlkSizeX;
    d->nBlkSizeX[1] = d->nBlkSizeX[2] = d->nBlkSizeX[0] >> d->xSubUV;

    d->nBlkSizeY[0] = evilVectors[0]->nBlkSizeY;
    d->nBlkSizeY[1] = d->nBlkSizeY[2] = d->nBlkSizeY[0] >> d->ySubUV;

    d->nWidth_B[0] = evilVectors[0]->nBlkX * (d->nBlkSizeX[0] - d->nOverlapX[0]) + d->nOverlapX[0];
    d->nWidth_B[1] = d->nWidth_B[2] = d->nWidth_B[0] >> d->xSubUV;

    d->nHeight_B[0] = evilVectors[0]->nBlkY * (d->nBlkSizeY[0] - d->nOverlapY[0]) + d->nOverlapY[0];
    d->nHeight_B[1] = d->nHeight_B[2] = d->nHeight_B[0] >> d->ySubUV;

    if (d->nOverlapX[0] || d->nOverlapY[0]) {
        d->OverWins[0].Init(d->nBlkSizeX[0], d->nBlkSizeY[0], d->nOverlapX[0], d->nOverlapY[0]);

        if (d->vi->format.colorFamily != cfGray) {
            d->OverWins[1].Init(d->nBlkSizeX[1], d->nBlkSizeY[1], d->nOverlapX[1], d->nOverlapY[1]);
            d->OverWins[2].Init(d->nBlkSizeX[2], d->nBlkSizeY[2], d->nOverlapX[2], d->nOverlapY[2]);
        }
    }

    // FIXME, use reference
    selectFunctions<radius>(*d, *evilVectors[0]);

    // FIXME, what happens when not enough vectors?
    const int numDeps = 2 + radius * 2; // input clip, super, and corresponding backward and forward vectors.
    std::vector<VSFilterDependency> deps;
    deps.reserve(numDeps);
    deps.push_back({ d->node, rpStrictSpatial });
    deps.push_back({ d->super, rpGeneral });
    for (int r = 0; r < radius * 2; r++)
        deps.push_back({ d->vectors[r], rpStrictSpatial });

    assert(numDeps == deps.size());
    
    vsapi->createVideoFilter(out, filter.c_str(), d->vi, (d->vi->format.bytesPerSample == 1) ? degrainGetFrame<radius, uint8_t> : degrainGetFrame<radius, uint16_t>, degrainFree<radius>, fmParallel, deps.data(), numDeps, d.get(), core);
    d.release();
}

constexpr const char *degrain_args =
    "clip:vnode;"
    "super:vnode;"
    "vectors:vnode[];"
    "thsad:int:opt;"
    "thsadc:int:opt;"
    "planes:int[]:opt;"
    "limit:int:opt;"
    "limitc:int:opt;"
    "thscd1:int:opt;"
    "thscd2:int:opt;"
    "prefix:data:opt;";

void degrainsRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) noexcept {
    vspapi->registerFunction("Degrain1",
                 degrain_args,
                 "clip:vnode;",
                 degrainCreate<1>, nullptr, plugin);
    vspapi->registerFunction("Degrain2",
                 degrain_args,
                 "clip:vnode;",
                 degrainCreate<2>, nullptr, plugin);
    vspapi->registerFunction("Degrain3",
                 degrain_args,
                 "clip:vnode;",
                 degrainCreate<3>, nullptr, plugin);
    vspapi->registerFunction("Degrain4",
                 degrain_args,
                "clip:vnode;",
                 degrainCreate<4>, nullptr, plugin);
    vspapi->registerFunction("Degrain5",
                degrain_args,
                "clip:vnode;",
                 degrainCreate<5>, nullptr, plugin);
    vspapi->registerFunction("Degrain6",
                degrain_args,
                "clip:vnode;",
                 degrainCreate<6>, nullptr, plugin);
}
