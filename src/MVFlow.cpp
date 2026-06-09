// Pixels flow motion function
// Copyright(c)2005 A.G.Balakhnin aka Fizick

// See legal notice in Copying.txt for more information

// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; version 2 of the License.
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

#include <cstdint>
#include <cstring>
#include <memory>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "Common.h"
#include "SuperPyramid.h"
#include "MotionBlockPyramid.h"



struct FlowData {
    VSNode *clip;
    VSNode *super;
    VSNode *vectors;
    const VSVideoInfo *vi;
    
    int deltaFrame;
    int time256;
    int fields;
    int64_t thscd1;
    int thscd2;
    bool tff;
    bool tff_exists;

    int pixel_max;

    std::string prefix;

    const VSAPI *vsapi;

    FlowData(const VSAPI *vsapi) : vsapi(vsapi) {};

    ~FlowData() {
        vsapi->freeNode(clip);
        vsapi->freeNode(super);
        vsapi->freeNode(vectors);
    }
};


template <typename PixelType>
static void flowFetch(uint8_t *pdst8, ptrdiff_t dst_pitch, const PyramidPlane &pref, int16_t *VXFull, ptrdiff_t VXPitch, int16_t *VYFull, ptrdiff_t VYPitch, int width, int height, int time256) {
    PixelType *pdst = (PixelType *)pdst8;

    dst_pitch /= sizeof(PixelType);
    int nPelLog = ilog2(pref.nPel);

    // fetch mode
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            // use interpolated image
            int vx = (VXFull[w] * time256 + 128) >> 8;
            int vy = (VYFull[w] * time256 + 128) >> 8;
            // FIXME, maybe template this on npel as well for speed?
            // FIXME, should shift w and h by ilog2(npel) to have 
            pdst[w] = *reinterpret_cast<const PixelType *>(pref.GetPointer((w << nPelLog) + vx, (h << nPelLog) + vy));
        }
        pdst += dst_pitch;
        VXFull += VXPitch;
        VYFull += VYPitch;
    }
}


typedef uint8_t PixelType;

static const VSFrame *VS_CC flowGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    FlowData *d =  reinterpret_cast<FlowData *>(instanceData);

    int nref = n + d->deltaFrame;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->clip, frameCtx);
        vsapi->requestFrameFilter(n, d->vectors, frameCtx);

        if (nref >= 0 && nref < d->vi->numFrames) {
            if (n < nref) {
                vsapi->requestFrameFilter(n, d->super, frameCtx);
                vsapi->requestFrameFilter(nref, d->super, frameCtx);
            } else {
                vsapi->requestFrameFilter(nref, d->super, frameCtx);
                vsapi->requestFrameFilter(n, d->super, frameCtx);
            }
        }
    } else if (activationReason == arAllFramesReady) {
        const VSFrame *mvn = vsapi->getFrameFilter(n, d->vectors, frameCtx);
        MotionBlockPyramid vectors(mvn, 1, d->prefix, core, vsapi);
        vsapi->freeFrame(mvn);

        if (vectors.IsUsable(d->thscd1, d->thscd2)) {
            const VSFrame *ref = vsapi->getFrameFilter(nref, d->super, frameCtx);
            FramePyramid refGOF(ref, 1, d->prefix, core, vsapi);

            const VSFrame *propSrc = vsapi->getFrameFilter(n, d->clip, frameCtx);
            VSFrame *dst = vsapi->newVideoFrame(&d->vi->format, d->vi->width, d->vi->height, propSrc, core);
            vsapi->freeFrame(propSrc);

            size_t full_size = d->vi * VPitchY * sizeof(int16_t);
            ptrdiff_t small_size_stride = roundUpTo64(vectors.nBlkX * sizeof(int16_t));
            size_t small_size = vectors.nBlkY * small_size_stride;

            int16_t *VXFullY = (int16_t *)malloc(full_size);
            int16_t *VYFullY = (int16_t *)malloc(full_size);
            int16_t *VXSmallY = (int16_t *)malloc(small_size);
            int16_t *VYSmallY = (int16_t *)malloc(small_size);


            // The output from this is always used upsized and possinly adjusted for UV as well
            MakeVectorSmallMasks(&fgop, nBlkX, nBlkY, VXSmallY, nBlkXP, VYSmallY, nBlkXP);


            int fieldShift = 0;
            if (d->fields && vectors.nPel > 1 && ((nref - n) % 2 != 0)) {

                const VSFrame *src = vsapi->getFrameFilter(n, d->super, frameCtx);
                FramePyramid srcGOF(src, 1, d->prefix, core, vsapi);

                int err;
                const VSMap *props = vsapi->getFramePropertiesRO(src);
                int src_top_field = !!vsapi->mapGetInt(props, "_Field", 0, &err);
                if (err && !d->tff_exists)
                    throw std::runtime_error("_Field property not found in super frame. Therefore, you must pass tff argument");

                if (d->tff_exists)
                    src_top_field = d->tff ^ (n % 2);

                props = vsapi->getFramePropertiesRO(ref);
                int ref_top_field = !!vsapi->mapGetInt(props, "_Field", 0, &err);
                if (err && !d->tff_exists) 
                    throw std::runtime_error("_Field property not found in super frame. Therefore, you must pass tff argument");

                if (d->tff_exists)
                    ref_top_field = d->tff ^ (nref % 2);

                fieldShift = (src_top_field && !ref_top_field) ? vectors.nPel / 2 : ((ref_top_field && !src_top_field) ? -(vectors.nPel / 2) : 0);
                // vertical shift of fields for fieldbased video at finest level pel2
            }

            for (int j = 0; j < nBlkYP; j++) {
                for (int i = 0; i < nBlkXP; i++) {
                    VYSmallY[j * nBlkXP + i] += fieldShift;
                }
            }

            d->upsizer.simpleResize_int16_t(&d->upsizer, VXFullY, VPitchY, VXSmallY, nBlkXP, 1);
            d->upsizer.simpleResize_int16_t(&d->upsizer, VYFullY, VPitchY, VYSmallY, nBlkXP, 0);


            flowFetch<PixelType>(vsapi->getWritePtr(dst, 0), vsapi->getStride(dst, 0), refGOF.GetLevel(0).planes[0],
                    VXFullY, VPitchY, VYFullY, VPitchY,
                    d->vi->width, d->vi->height, d->time256);

            if (d->vi->format.colorFamily != cfGray) {
                size_t full_size_uv = nHeightPUV * VPitchUV * sizeof(int16_t);
                size_t small_size_uv = nBlkYP * nBlkXP * sizeof(int16_t);

                int16_t *VXFullUV = (int16_t *)malloc(full_size_uv);
                int16_t *VYFullUV = (int16_t *)malloc(full_size_uv);

                int16_t *VXSmallUV = (int16_t *)malloc(small_size_uv);
                int16_t *VYSmallUV = (int16_t *)malloc(small_size_uv);

                // This divides the vectors by the subsampling in both directions, memcpy if no subsampling
                VectorSmallMaskYToHalfUV(VXSmallY, nBlkXP, nBlkYP, VXSmallUV, xRatioUV);
                VectorSmallMaskYToHalfUV(VYSmallY, nBlkXP, nBlkYP, VYSmallUV, yRatioUV);

                d->upsizerUV.simpleResize_int16_t(&d->upsizerUV, VXFullUV, VPitchUV, VXSmallUV, nBlkXP, 1);
                d->upsizerUV.simpleResize_int16_t(&d->upsizerUV, VYFullUV, VPitchUV, VYSmallUV, nBlkXP, 0);


                flowFetch<PixelType>(vsapi->getWritePtr(dst, 1), vsapi->getStride(dst, 1), refGOF.GetLevel(0).planes[1],
                        VXFullUV, VPitchUV, VYFullUV, VPitchUV,
                        nWidthUV, nHeightUV, time256);
                flowFetch<PixelType>(vsapi->getWritePtr(dst, 2), vsapi->getStride(dst, 2), refGOF.GetLevel(0).planes[2],
                        VXFullUV, VPitchUV, VYFullUV, VPitchUV,
                        nWidthUV, nHeightUV, time256);

                free(VXFullUV);
                free(VYFullUV);
                free(VXSmallUV);
                free(VYSmallUV);
            }

            free(VXFullY);
            free(VYFullY);
            free(VXSmallY);
            free(VYSmallY);

            return dst;
        } else { // not usable
            return vsapi->getFrameFilter(n, d->clip, frameCtx);
        }
    }

    return nullptr;
}


static void VS_CC flowCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    std::unique_ptr<FlowData> d(new FlowData(vsapi));

    int err;

    double time = vsapi->mapGetFloat(in, "time", 0, &err);
    if (err)
        time = 100.0;

    d->fields = !!vsapi->mapGetInt(in, "fields", 0, &err);

    d->thscd1 = vsapi->mapGetInt(in, "thscd1", 0, &err);
    if (err)
        d->thscd1 = MV_DEFAULT_SCD1;

    d->thscd2 = vsapi->mapGetIntSaturated(in, "thscd2", 0, &err);
    if (err)
        d->thscd2 = MV_DEFAULT_SCD2;

    d->tff = !!vsapi->mapGetInt(in, "tff", 0, &err);
    d->tff_exists = !err;

    try {

        if (time < 0.0 || time > 100.0) {
            vsapi->mapSetError(out, "Flow: time must be between 0 and 100 % (inclusive).");
            return;
        }

        d->time256 = (int)(time * 256.0 / 100.0);

        const char *prefix = vsapi->mapGetData(in, "prefix", 0, &err);
        if (prefix)
            d->prefix = prefix;
        else
            d->prefix = DEFAULT_MVUTENSILS_PREFIX;

        d->super = vsapi->mapGetNode(in, "super", 0, nullptr);

        char errorMsg[ERROR_SIZE] = {};
        const VSFrame *evil = vsapi->getFrame(0, d->super, errorMsg, ERROR_SIZE);
        if (!evil)
            throw std::runtime_error("failed to retrieve first frame from super clip. Error message: " + std::string(errorMsg));

        FramePyramid super(evil, 0, d->prefix, core, vsapi);

        d->vectors = vsapi->mapGetNode(in, "vectors", 0, nullptr);

        d->clip = vsapi->mapGetNode(in, "clip", 0, nullptr);
        d->vi = vsapi->getVideoInfo(d->clip);

        if (!super.IsCompatibleWithSource(d->vi))
            throw std::runtime_error("source clip isn't compatible with super clip");

        const VSFrame *evil2 = vsapi->getFrame(0, d->vectors, errorMsg, ERROR_SIZE);
        if (!evil2)
            throw std::runtime_error("failed to retrieve first frame from vectors clip. Error message: " + std::string(errorMsg));

        MotionBlockPyramid vectors(evil2, 0, d->prefix, core, vsapi);
        vsapi->freeFrame(evil2);

        vectors.ScaleThSCD(d->thscd1, d->thscd2, d->vi->format.bitsPerSample);

        d->deltaFrame = vectors.nDeltaFrame;

        if (!vectors.IsCompatibleForAnalysis(super))
            throw std::runtime_error("wrong source or super clip frame size");

        // FIXME
        //simpleInit(&d.upsizer, d.nWidthP, d.nHeightP, d.nBlkXP, d.nBlkYP, d.vectors_data.nWidth, d.vectors_data.nHeight, d.vectors_data.nPel, d.opt);
        //if (d.vi->format.colorFamily != cfGray)
        //    simpleInit(&d.upsizerUV, d.nWidthPUV, d.nHeightPUV, d.nBlkXP, d.nBlkYP, d.nWidthUV, d.nHeightUV, d.vectors_data.nPel, d.opt);


    } catch (std::runtime_error &e) {
        vsapi->mapSetError(out, ("Flow: " + std::string(e.what())).c_str());
        return;
    }

    VSFilterDependency deps[3] = {
        {d->clip, rpStrictSpatial},
        {d->super, rpStrictSpatial},
        {d->vectors, rpStrictSpatial},
    };

    vsapi->createVideoFilter(out, "Flow", d->vi, flowGetFrame, filterFree<FlowData>, fmParallel, deps, ARRAY_SIZE(deps), d.get(), core);
    d.release();
}

void flowRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("Flow",
                 "clip:vnode;"
                 "super:vnode;"
                 "vectors:vnode;"
                 "time:float:opt;"
                 "mode:int:opt;"
                 "fields:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "tff:int:opt;"
                 "prefix:data:opt;",
                 "clip:vnode;",
                 flowCreate, nullptr, plugin);
}
