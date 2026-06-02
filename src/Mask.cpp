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
#include "Common.h"

struct MaskDataExtra {
    VSVideoInfo vi;

    float ml;
    float fGamma;
    int kind;
    int time256;
    int nSceneChangeValue;
    int64_t thscd1;
    int thscd2;

    float fMaskNormFactor;

    int nWidthUV;
    int nHeightUV;
    int nWidthB;
    int nHeightB;
    int nWidthBUV;
    int nHeightBUV;

    std::string prefix;
    std::string filterName;
};

typedef DualNodeData<MaskDataExtra> MaskData;


static const VSFrame *VS_CC maskGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MaskData *d =  reinterpret_cast<MaskData *>(instanceData);

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->node1, frameCtx);
        vsapi->requestFrameFilter(n, d->node2, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrame *src = vsapi->getFrameFilter(n, d->node1, frameCtx);
        VSFrame *dst = vsapi->newVideoFrame(&d->vi.format, d->vi.width, d->vi.height, src, core);
        vsapi->freeFrame(src);

        try {
            const VSFrame *mvn = vsapi->getFrameFilter(n, d->node2, frameCtx);
            MotionBlockPyramid vectors(mvn, 1, d->prefix, core, vsapi);
            vsapi->freeFrame(mvn);

            if (vectors.IsUsable(d->thscd1, d->thscd2)) {
                std::unique_ptr<uint8_t[]> smallMask(new uint8_t[vectors.nBlkX * vectors.nBlkY]);

                if (d->kind == 0) {
                    vectors.MakeVectorLengthMask(d->fMaskNormFactor, d->fGamma, smallMask.get(), vectors.nBlkX, d->time256);
                } else if (d->kind == 1) {
                    vectors.MakeSADMask(d->fMaskNormFactor, d->fGamma, smallMask.get(), vectors.nBlkX, d->time256);
                } else if (d->kind == 2) {
                    vectors.MakeVectorOcclusionMask(d->fMaskNormFactor, d->fGamma, smallMask.get(), vectors.nBlkX, d->time256);
                }

                upsizer->simpleResize_uint8_t(upsizer, pDst[0], nDstPitches[0], smallMask, nBlkX, 0);
                upsizerUV->simpleResize_uint8_t(upsizerUV, pDst[1], nDstPitches[1], smallMask, nBlkX, 0);

                memcpy(pDst[2], pDst[1], nHeightUV * nDstPitches[1]);

            } else {
                for (int plane = 0; plane < d->vi.format.numPlanes; plane++)
                    memset(vsapi->getWritePtr(dst, plane), d->nSceneChangeValue, vsapi->getStride(dst, plane) * vsapi->getFrameHeight(dst, plane));
            }

            return dst;

        } catch (std::runtime_error &e) {
            vsapi->freeFrame(dst);
            vsapi->setFilterError((d->filterName + ": " + e.what()).c_str(), frameCtx);
        }
    }

    return nullptr;
}

static void VS_CC maskCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    std::unique_ptr<MaskData> d(new MaskData(vsapi));

    d->filterName = "Mask";

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

        char errorMsg[ERROR_SIZE] = {};
        const VSFrame *evil2 = vsapi->getFrame(0, d->node2, errorMsg, ERROR_SIZE);
        if (!evil2)
            throw std::runtime_error("failed to retrieve first frame from vectors clip. Error message: " + std::string(errorMsg));

        MotionBlockPyramid vectors(evil2, 0, d->prefix, core, vsapi);
        vectors.ScaleThSCD(d->thscd1, d->thscd2, d->vi.format.bitsPerSample);

        d->fMaskNormFactor = 1.0f / d->ml;

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
        vsapi->mapSetError(out, (d->filterName + ": " + e.what()).c_str());
        return;
    }

    VSFilterDependency deps[2] = { 
        {d->node1, rpStrictSpatial}, 
        {d->node2, rpStrictSpatial},
    };

    vsapi->createVideoFilter(out, d->filterName.c_str(), &d->vi, maskGetFrame, filterFree<MaskData>, fmParallel, deps, ARRAY_SIZE(deps), d.get(), core);
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
