// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick - YUY2
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

#include <VapourSynth4.h>
#include <VSHelper4.h>
#include <memory>
#include <stdexcept>

#include "SuperPyramid.h"
#include "MotionBlockPyramid.h"
#include "Common.h"


struct SCDetectionDataExtra {
    const VSVideoInfo *vi;

    int64_t thscd1;
    int thscd2;

    std::string prefix;
};

typedef DualNodeData<SCDetectionDataExtra> SCDetectionData;


static const VSFrame *VS_CC scdetectionGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) noexcept {
    SCDetectionData *d = reinterpret_cast<SCDetectionData *>(instanceData);

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->node1, frameCtx);
        vsapi->requestFrameFilter(n, d->node2, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrame *src = vsapi->getFrameFilter(n, d->node1, frameCtx);
        VSFrame *dst = vsapi->copyFrame(src, core);
        vsapi->freeFrame(src);

        try {
            const VSFrame *mvn = vsapi->getFrameFilter(n, d->node2, frameCtx);
            MotionBlockPyramid vectors(mvn, 1, d->prefix, core, vsapi);
            vsapi->freeFrame(mvn);

            constexpr const char *propNames[2] = { "_SceneChangePrev", "_SceneChangeNext" };
            VSMap *props = vsapi->getFramePropertiesRW(dst);
            vsapi->mapSetInt(props, propNames[vectors.nDeltaFrame > 0], !vectors.IsUsable(d->thscd1, d->thscd2), maReplace);
        } catch (std::runtime_error &e) {
            vsapi->setFilterError(("SCDetection: " + std::string(e.what())).c_str(), frameCtx);
            vsapi->freeFrame(dst);
            return nullptr;
        }

        return dst;
    }

    return nullptr;
}


static void VS_CC scdetectionFree(void *instanceData, VSCore *core, const VSAPI *vsapi) noexcept {
    SCDetectionData *d = reinterpret_cast<SCDetectionData *>(instanceData);
    delete d;
}


static void VS_CC scdetectionCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) noexcept {
    std::unique_ptr<SCDetectionData> d(new SCDetectionData(vsapi));

    int err;

    d->thscd1 = vsapi->mapGetInt(in, "thscd1", 0, &err);
    if (err)
        d->thscd1 = MV_DEFAULT_SCD1;

    d->thscd2 = vsapi->mapGetIntSaturated(in, "thscd2", 0, &err);
    if (err)
        d->thscd2 = MV_DEFAULT_SCD2;

    const char *prefix = vsapi->mapGetData(in, "prefix", 0, &err);
    if (prefix)
        d->prefix = prefix;
    else
        d->prefix = DEFAULT_MVUTENSILS_PREFIX;


    d->node1 = vsapi->mapGetNode(in, "clip", 0, nullptr);
    d->node2 = vsapi->mapGetNode(in, "vectors", 0, nullptr);

    d->vi = vsapi->getVideoInfo(d->node1);

    try {
        MotionBlockPyramid vectors(d->node2, d->prefix, core, vsapi);

        vectors.ScaleThSCD(d->thscd1, d->thscd2, d->vi->format.bitsPerSample);
    } catch (std::runtime_error &e) {
        vsapi->mapSetError(out, ("SCDetection: " + std::string(e.what())).c_str());
        return;
    }


    VSFilterDependency deps[2] = { 
        {d->node1, rpStrictSpatial},
        {d->node2, rpStrictSpatial}
    };

    vsapi->createVideoFilter(out, "SCDetection", d->vi, scdetectionGetFrame, scdetectionFree, fmParallel, deps, ARRAY_SIZE(deps), d.get(), core);
    d.release();
}

void scdetectionRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) noexcept {
    vspapi->registerFunction("SCDetection",
                 "clip:vnode;"
                 "vectors:vnode;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "prefix:data:opt;",
                 "clip:vnode;",
                 scdetectionCreate, nullptr, plugin);
}
