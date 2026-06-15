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
#include <VSConstants4.h>

#include "SuperPyramid.h"
#include "MotionBlockPyramid.h"
#include "MaskResize.h"
#include "Common.h"

struct MaskData {
    VSNode *node = nullptr;
    VSVideoInfo vi;

    float fGamma;
    intptr_t kind;
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

    const VSAPI *vsapi;

    MaskData(const VSAPI *vsapi) : vsapi(vsapi) {};

    ~MaskData() {
        vsapi->freeNode(node);
    }
};


template<typename PixelType>
static const VSFrame *VS_CC maskGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MaskData *d =  reinterpret_cast<MaskData *>(instanceData);

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        VSFrame *dst = vsapi->newVideoFrame(&d->vi.format, d->vi.width, d->vi.height, nullptr, core);

        try {
            MotionBlockPyramid vectors(vsapi->getFrameFilter(n, d->node, frameCtx), 1, d->prefix, core, vsapi);

            if (vectors.IsUsable(d->thscd1, d->thscd2)) {
                std::unique_ptr<BlockMask<PixelType>> Mask;
                if (d->kind == 0) {
                    Mask = vectors.MakeVectorLengthMask<PixelType>(d->fMaskNormFactor, d->fGamma);
                } else if (d->kind == 1) {
                    Mask = vectors.MakeSADMask<PixelType>(d->fMaskNormFactor, d->fGamma, d->time256);
                } else if (d->kind == 2) {
                    Mask = vectors.MakeVectorOcclusionMask<PixelType>(d->fMaskNormFactor, d->fGamma, d->time256);
                }

                BilinearUpsizeBlockMask(vsapi->getWritePtr(dst, 0), vsapi->getStride(dst, 0), vsapi->getFrameWidth(dst, 0), vsapi->getFrameHeight(dst, 0),
                    Mask->mask, Mask->stride, vectors.nBlkX, vectors.nBlkY, vectors.nBlkSizeX, vectors.nBlkSizeY, vectors.nOverlapX, vectors.nOverlapY, d->vi.format.bitsPerSample);
            } else {
                PixelType *dstPtr = reinterpret_cast<PixelType *>(vsapi->getWritePtr(dst, 0));
                std::fill(dstPtr, dstPtr + (vsapi->getStride(dst, 0) / sizeof(PixelType)) * vsapi->getFrameHeight(dst, 0), d->nSceneChangeValue);
            }

            vsapi->mapSetInt(vsapi->getFramePropertiesRW(dst), "_Range", VSC_RANGE_FULL, maAppend);

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

    int err;

    d->kind = (intptr_t)userData;
    
    if (d->kind == 0)
        d->filterName = "VectorLengthMask";
    else if (d->kind == 1)
        d->filterName = "SADMask";
    else if (d->kind == 2)
        d->filterName = "OcclusionMask";

    float ml = (float)vsapi->mapGetFloat(in, "ml", 0, &err);
    if (err)
        ml = 100.0f;

    d->fGamma = (float)vsapi->mapGetFloat(in, "gamma", 0, &err);
    if (err)
        d->fGamma = 1.0f;

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

        if (time < 0.0 || time > 100.0)
            throw std::runtime_error("time must be between 0.0 and 100.0");

        const char *prefix = vsapi->mapGetData(in, "prefix", 0, &err);
        if (prefix)
            d->prefix = prefix;
        else
            d->prefix = DEFAULT_MVUTENSILS_PREFIX;

        d->node = vsapi->mapGetNode(in, "vectors", 0, nullptr);
        MotionBlockPyramid vectors(d->node, d->prefix, core, vsapi);

        d->vi = *vsapi->getVideoInfo(d->node);
        d->vi.width = vectors.nRealWidth;
        d->vi.height = vectors.nRealHeight;
        vsapi->queryVideoFormat(&d->vi.format, cfGray, stInteger, vectors.bitsPerSample, 0, 0, core);

        int maxVal = (1 << vectors.bitsPerSample) - 1;
        if (d->nSceneChangeValue < 0 || d->nSceneChangeValue > maxVal)
            throw std::runtime_error("ysc must be between 0 and " + std::to_string(maxVal));

        vectors.ScaleThSCD(d->thscd1, d->thscd2, d->vi.format.bitsPerSample);

        d->fMaskNormFactor = 1.0f / ml;

        d->time256 = (int)(time * 256 / 100);

    } catch (std::runtime_error &e) {
        vsapi->mapSetError(out, (d->filterName + ": " + e.what()).c_str());
        return;
    }

    VSFilterDependency deps[1] = { 
        {d->node, rpStrictSpatial}, 
    };

    vsapi->createVideoFilter(out, d->filterName.c_str(), &d->vi, d->vi.format.bitsPerSample == 8 ? maskGetFrame<uint8_t> : maskGetFrame<uint16_t>, filterFree<MaskData>, fmParallel, deps, ARRAY_SIZE(deps), d.get(), core);
    d.release();
}

static constexpr char filterArgs[] =
    "vectors:vnode;"
    "ml:float:opt;"
    "gamma:float:opt;"
    "time:float:opt;"
    "ysc:int:opt;"
    "thscd1:int:opt;"
    "thscd2:int:opt;"
    "prefix:data:opt;";

void maskRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("VectorLengthMask",
        filterArgs,
        "clip:vnode;",
        maskCreate, nullptr, plugin);
    vspapi->registerFunction("SADMask",
        filterArgs,
        "clip:vnode;",
        maskCreate, (void *)1, plugin);
    vspapi->registerFunction("OcclusionMask",
        filterArgs,
        "clip:vnode;",
        maskCreate, (void *)2, plugin);
}

