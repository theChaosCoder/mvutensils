// Create an overlay mask with the motion vectors
// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick - YUY2, occlusion
// See legal notice in Copying.txt for more information
//
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

#include <cmath>
#include <memory>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "SuperPyramid.h"
#include "MotionBlockPyramid.h"
#include "MaskFun.h"
#include "SimpleResize.h"
#include "Common.h"



struct MaskDataExtra {
    // FIXME, why not a const pointer?
    VSVideoInfo vi;

    float ml;
    float fGamma;
    int kind;
    int time256;
    int nSceneChangeValue;
    int64_t thscd1;
    int thscd2;

    float fMaskNormFactor;
    float fMaskNormFactor2;
    float fHalfGamma;

    int nWidthUV;
    int nHeightUV;
    int nWidthB;
    int nHeightB;
    int nWidthBUV;
    int nHeightBUV;

    std::string prefix;

    SimpleResize upsizer;
    SimpleResize upsizerUV;
};

typedef DualNodeData<MaskDataExtra> MaskData;


static uint8_t maskLength(VECTOR v, uint8_t pel, float fMaskNormFactor2, float fHalfGamma) {
    double norme = (double)(v.x * v.x + v.y * v.y) / (pel * pel);

    double l = 255 * pow(norme * fMaskNormFactor2, fHalfGamma);

    return (uint8_t)((l > 255) ? 255 : l);
}

static const VSFrame *VS_CC maskGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MaskData *d =  reinterpret_cast<MaskData *>(instanceData);

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->node1, frameCtx);
        vsapi->requestFrameFilter(n, d->node2, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrame *src = vsapi->getFrameFilter(n, d->node1, frameCtx);
        VSFrame *dst = vsapi->newVideoFrame(&d->vi.format, d->vi.width, d->vi.height, src, core);
        vsapi->freeFrame(src);

        uint8_t *pDst[3];
        ptrdiff_t nDstPitches[3];

        for (int i = 0; i < 3; i++) {
            pDst[i] = vsapi->getWritePtr(dst, i);
            nDstPitches[i] = vsapi->getStride(dst, i);
        }

        const VSFrame *mvn = vsapi->getFrameFilter(n, d->node2, frameCtx);
        MotionBlockPyramid vectors(mvn, 1, d->prefix, core, vsapi);
        vsapi->freeFrame(mvn);

        const int nWidth = vectors.nWidth;
        const int nHeight = vectors.nHeight;
        const int nWidthUV = vectors.nWidthUV;
        const int nHeightUV = vectors.nHeightUV;

        if (vectors.IsUsable(d->thscd1, d->thscd2)) {
            const int nBlkX = vectors.nBlkX;
            const int nBlkY = vectors.nBlkY;
            const int nBlkCount = nBlkX * nBlkY;
            const int nBlkSizeX = vectors.nBlkSizeX;
            const int nBlkSizeY = vectors.nBlkSizeY;
            const int nOverlapX = vectors.nOverlapX;
            const int nOverlapY = vectors.nOverlapY;
            const int nWidthB = vectors.nWidthB;
            const int nHeightB = vectors.nHeightB;
            const int nWidthBUV = vectors.nWidthBUV;
            const int nHeightBUV = vectors.nHeightBUV;
            SimpleResize *upsizer = &d->upsizer;
            SimpleResize *upsizerUV = &d->upsizerUV;
            uint8_t *smallMask = (uint8_t *)malloc(nBlkX * nBlkY);
            uint8_t *smallMaskV = (uint8_t *)malloc(nBlkX * nBlkY);

            // FIXME, only kind 0..2 seem useful and should probably be separated into different filters

            if (d->kind == 0) { // vector length mask
                for (int j = 0; j < nBlkCount; j++)
                    smallMask[j] = maskLength(vectors.GetBlock(j).vector, vectors.nPel, d->fMaskNormFactor2, d->fHalfGamma);
            } else if (d->kind == 1) { // SAD mask
                MakeSADMaskTime(&fgop, nBlkX, nBlkY, 4.0 * d->fMaskNormFactor / (nBlkSizeX * nBlkSizeY), d->fGamma, vectors.nPel, smallMask, nBlkX, d->time256, nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY, d->vi.format.bitsPerSample);
            } else if (d->kind == 2) { // occlusion mask
                MakeVectorOcclusionMaskTime(&fgop, d->vectors_data.isBackward, nBlkX, nBlkY, 1.0 / d->fMaskNormFactor, d->fGamma, vectors.nPel, smallMask, nBlkX, d->time256, nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY);
            }

            upsizer->simpleResize_uint8_t(upsizer, pDst[0], nDstPitches[0], smallMask, nBlkX, 0);
            if (nWidth > nWidthB)
                for (int h = 0; h < nHeight; h++)
                    for (int w = nWidthB; w < nWidth; w++)
                        *(pDst[0] + h * nDstPitches[0] + w) = *(pDst[0] + h * nDstPitches[0] + nWidthB - 1);
            if (nHeight > nHeightB)
                vsh::bitblt(pDst[0] + nHeightB * nDstPitches[0], nDstPitches[0], pDst[0] + (nHeightB - 1) * nDstPitches[0], nDstPitches[0], nWidth, nHeight - nHeightB);

            // chroma
            upsizerUV->simpleResize_uint8_t(upsizerUV, pDst[1], nDstPitches[1], smallMask, nBlkX, 0);

            memcpy(pDst[2], pDst[1], nHeightUV * nDstPitches[1]);

            // EXTEND to cover the unprocessed borders
            if (nWidthUV > nWidthBUV)
                for (int h = 0; h < nHeightUV; h++)
                    for (int w = nWidthBUV; w < nWidthUV; w++) {
                        *(pDst[1] + h * nDstPitches[1] + w) = *(pDst[1] + h * nDstPitches[1] + nWidthBUV - 1);
                        *(pDst[2] + h * nDstPitches[2] + w) = *(pDst[2] + h * nDstPitches[2] + nWidthBUV - 1);
                    }
            if (nHeightUV > nHeightBUV) {
                vsh::bitblt(pDst[1] + nHeightBUV * nDstPitches[1], nDstPitches[1], pDst[1] + (nHeightBUV - 1) * nDstPitches[1], nDstPitches[1], nWidthUV, nHeightUV - nHeightBUV);
                vsh::bitblt(pDst[2] + nHeightBUV * nDstPitches[2], nDstPitches[2], pDst[2] + (nHeightBUV - 1) * nDstPitches[2], nDstPitches[2], nWidthUV, nHeightUV - nHeightBUV);
            }

            free(smallMask);
            free(smallMaskV);
        } else { // not usable
            memset(pDst[0], d->nSceneChangeValue, nHeight * nDstPitches[0]);
            memset(pDst[1], d->nSceneChangeValue, nHeightUV * nDstPitches[1]);
            memset(pDst[2], d->nSceneChangeValue, nHeightUV * nDstPitches[2]);
        }



        return dst;
    }

    return nullptr;
}

static void VS_CC maskCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    std::unique_ptr<MaskData> d(new MaskData(vsapi));

    std::string filterName = "Mask";

    int err;

    d->ml = (float)vsapi->mapGetFloat(in, "ml", 0, &err);
    if (err)
        d->ml = 100.0f;

    d->fGamma = (float)vsapi->mapGetFloat(in, "gamma", 0, &err);
    if (err)
        d->fGamma = 1.0f;

    d->kind = vsapi->mapGetIntSaturated(in, "kind", 0, &err);

    double time = vsapi->mapGetFloat(in, "time", 0, &err);
    if (err)
        time = 100.0;

    d->nSceneChangeValue = vsapi->mapGetIntSaturated(in, "ysc", 0, &err);

    d->thscd1 = vsapi->mapGetInt(in, "thscd1", 0, &err);
    if (err)
        d->thscd1 = MV_DEFAULT_SCD1;

    d->thscd2 = vsapi->mapGetIntSaturated(in, "thscd2", 0, &err);
    if (err)
        d->thscd2 = MV_DEFAULT_SCD2;

    try {

        if (d->fGamma < 0.0f)
            throw std::runtime_error("gamma must not be negative");

        if (d->kind < 0 || d->kind > 5)
            throw std::runtime_error("kind must 0, 1, 2, 3, 4, or 5");

        if (time < 0.0 || time > 100.0)
            throw std::runtime_error("time must be between 0.0 and 100.0");

        if (d->nSceneChangeValue < 0 || d->nSceneChangeValue > 255)
            throw std::runtime_error("ysc must be between 0 and 255");

        const char *prefix = vsapi->mapGetData(in, "prefix", 0, &err);
        if (prefix)
            d->prefix = prefix;
        else
            d->prefix = DEFAULT_MVUTENSILS_PREFIX;

        d->node2 = vsapi->mapGetNode(in, "vectors", 0, nullptr);

        char errorMsg[ERROR_SIZE] = "failed to retrieve first frame from super clip. Error message: ";
        const VSFrame *evil2 = vsapi->getFrame(0, d->node2, errorMsg, ERROR_SIZE);
        if (!evil2)
            throw std::runtime_error(errorMsg);

        MotionBlockPyramid vectors(evil2, 0, d->prefix, core, vsapi);
        vectors.ScaleThSCD(d->thscd1, d->thscd2, d->vi.format.bitsPerSample);

        d->fMaskNormFactor = 1.0f / d->ml;
        d->fMaskNormFactor2 = d->fMaskNormFactor * d->fMaskNormFactor;

        d->fHalfGamma = d->fGamma * 0.5f;

        d->nWidthB = d->vectors_data.nBlkX * (d->vectors_data.nBlkSizeX - d->vectors_data.nOverlapX) + d->vectors_data.nOverlapX;
        d->nHeightB = d->vectors_data.nBlkY * (d->vectors_data.nBlkSizeY - d->vectors_data.nOverlapY) + d->vectors_data.nOverlapY;

        d->nHeightUV = d->vectors_data.nHeight / d->vectors_data.yRatioUV;
        d->nWidthUV = d->vectors_data.nWidth / d->vectors_data.xRatioUV;
        d->nHeightBUV = d->nHeightB / d->vectors_data.yRatioUV;
        d->nWidthBUV = d->nWidthB / d->vectors_data.xRatioUV;


        d->node1 = vsapi->mapGetNode(in, "clip", 0, nullptr);
        d->vi = *vsapi->getVideoInfo(d->node1);

        // FIXME
        if (!vsh::isConstantVideoFormat(&d->vi) || d->vi.format.bitsPerSample > 8 || d->vi.format.subSamplingW > 1 || d->vi.format.subSamplingH > 1 || (d->vi.format.colorFamily != cfYUV && d->vi.format.colorFamily != cfGray))
            throw std::runtime_error("input clip must be GRAY8, YUV420P8, YUV422P8, YUV440P8, or YUV444P8, with constant dimensions.");

        if (d->vi.format.colorFamily == cfGray)
            vsapi->getVideoFormatByID(&d->vi.format, pfYUV444P8, core);

        simpleInit(&d->upsizer, d->nWidthB, d->nHeightB, d->vectors_data.nBlkX, d->vectors_data.nBlkY, d->vectors_data.nWidth, d->vectors_data.nHeight, d->vectors_data.nPel);
        simpleInit(&d->upsizerUV, d->nWidthBUV, d->nHeightBUV, d->vectors_data.nBlkX, d->vectors_data.nBlkY, d->nWidthUV, d->nHeightUV, d->vectors_data.nPel);

        d->time256 = (int)(time * 256 / 100);

    } catch (std::runtime_error &e) {
        vsapi->mapSetError(out, (filterName + ": " + e.what()).c_str());
        return;
    }

    VSFilterDependency deps[2] = { 
        {d->node1, rpStrictSpatial}, 
        {d->node2, rpStrictSpatial},
    };

    vsapi->createVideoFilter(out, "Mask", &d->vi, maskGetFrame, filterFree<MaskData>, fmParallel, deps, ARRAY_SIZE(deps), d.get(), core);
    d.release();
}


void maskRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("Mask",
                 "clip:vnode;"
                 "vectors:vnode;"
                 "ml:float:opt;"
                 "gamma:float:opt;"
                 "kind:int:opt;"
                 "time:float:opt;"
                 "ysc:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "prefix:data:opt;",
                 "clip:vnode;",
                 maskCreate, nullptr, plugin);
}
