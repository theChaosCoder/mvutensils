#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "SuperPyramid.h"
#include "CommonMacros.h"



typedef struct MVSuperData {
    VSNode *node;
    VSVideoInfo vi;

    VSNode *pelclip; // upsized source clip with doubled frame width and heigth (used for pel=2)

    int nHPad;
    int nVPad;
    int nPel;
    int nLevels;
    SharpParam sharp;
    RFilterParam rfilter; // frame reduce filter mode

    int nWidth;
    int nHeight;

    int yRatioUV;
    int xRatioUV;

    int nSuperWidth;
    int nSuperHeight;

    int nBlkSizeX;
    int nBlkSizeY;
    int nOverlapX;
    int nOverlapY;

    bool usePelClip;
} MVSuperData;


static const VSFrame *VS_CC mvsuperGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    MVSuperData *d = (MVSuperData *)instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->node, frameCtx);
        if (d->usePelClip)
            vsapi->requestFrameFilter(n, d->pelclip, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrame *src = vsapi->getFrameFilter(n, d->node, frameCtx);

        FramePyramid pyramid(src, d->nLevels, d->nBlkSizeX, d->nBlkSizeY, d->nOverlapX, d->nOverlapY, d->nHPad, d->nVPad, d->rfilter, core, vsapi);

        if (d->usePelClip) {
            const VSFrame *srcPel = vsapi->getFrameFilter(n, d->pelclip, frameCtx);
            pyramid.SetExternalPelPlanes(srcPel, d->nPel, 0, core, vsapi);
            vsapi->freeFrame(srcPel);
        } else if (d->nPel > 1) {
            pyramid.GeneratePelPlanes(d->nPel, d->sharp, core, vsapi);
        }

        VSFrame *dst = vsapi->copyFrame(src, core);
        pyramid.ExportFrameData(dst, "MVUtensils");
        vsapi->freeFrame(src);

        return dst;
    }

    return nullptr;
}


static void VS_CC mvsuperFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVSuperData *d = (MVSuperData *)instanceData;

    vsapi->freeNode(d->node);
    vsapi->freeNode(d->pelclip);
    free(d);
}


static void VS_CC mvsuperCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    MVSuperData d;
    MVSuperData *data;

    int err;

    d.nHPad = vsapi->mapGetIntSaturated(in, "hpad", 0, &err);
    if (err)
        d.nHPad = 16;

    d.nVPad = vsapi->mapGetIntSaturated(in, "vpad", 0, &err);
    if (err)
        d.nVPad = 16;

    d.nPel = vsapi->mapGetIntSaturated(in, "pel", 0, &err);
    if (err)
        d.nPel = 2;

    d.nLevels = vsapi->mapGetIntSaturated(in, "levels", 0, &err);

    d.sharp = static_cast<SharpParam>(vsapi->mapGetIntSaturated(in, "sharp", 0, &err)); // pel2 interpolation type
    if (err)
        d.sharp = SharpParam::Wiener;

    d.rfilter = static_cast<RFilterParam>(vsapi->mapGetIntSaturated(in, "rfilter", 0, &err));
    if (err)
        d.rfilter = RFilterParam::Bilinear;

    if ((d.nPel != 1) && (d.nPel != 2) && (d.nPel != 4)) {
        vsapi->mapSetError(out, "Super: pel must be 1, 2, or 4.");
        return;
    }

    if (d.sharp < SharpParam::Bilinear || d.sharp > SharpParam::Wiener) {
        vsapi->mapSetError(out, "Super: sharp must be between 0 and 2 (inclusive).");
        return;
    }

    if (d.rfilter < RFilterParam::Simple || d.rfilter > RFilterParam::Cubic) {
        vsapi->mapSetError(out, "Super: rfilter must be between 0 and 4 (inclusive).");
        return;
    }


    d.node = vsapi->mapGetNode(in, "clip", 0, 0);

    // Make a copy of the video info, so we can reference
    // it and modify it below.
    d.vi = *vsapi->getVideoInfo(d.node);

    d.nWidth = d.vi.width;
    d.nHeight = d.vi.height;

    if (!vsh::isConstantVideoFormat(&d.vi) || d.vi.format.bitsPerSample > 16 || d.vi.format.sampleType != stInteger ||
            d.vi.format.subSamplingW > 1 || d.vi.format.subSamplingH > 1 || (d.vi.format.colorFamily != cfYUV && d.vi.format.colorFamily != cfGray)) {
        vsapi->mapSetError(out, "Super: input clip must be GRAY, YUV420, YUV422, YUV440, or YUV444, up to 16 bits, with constant dimensions.");
        vsapi->freeNode(d.node);
        return;
    }

    d.xRatioUV = 1 << d.vi.format.subSamplingW;
    d.yRatioUV = 1 << d.vi.format.subSamplingH;

    // at last two pixels width and height of chroma
    int nLevelsMax = 0;
    int nLevelWidth = d.vi.width;
    int nLevelHeight = d.vi.height;
    while (true) {
        nLevelHeight = PlaneDimensionLuma(nLevelHeight, d.yRatioUV, d.nVPad);
        nLevelWidth = PlaneDimensionLuma(nLevelWidth, d.xRatioUV, d.nHPad);
        if (nLevelHeight < d.yRatioUV * 2 || nLevelWidth < d.xRatioUV * 2)
            break;
        nLevelsMax++;
    }
    if (d.nLevels <= 0 || d.nLevels > nLevelsMax)
        d.nLevels = nLevelsMax;

    d.pelclip = vsapi->mapGetNode(in, "pelclip", 0, &err);
    const VSVideoInfo *pelvi = d.pelclip ? vsapi->getVideoInfo(d.pelclip) : NULL;

    if (d.pelclip && (!vsh::isConstantVideoFormat(pelvi) || !vsh::isSameVideoFormat(&pelvi->format, &d.vi.format))) {
        vsapi->mapSetError(out, "Super: pelclip must have the same format as the input clip, and it must have constant dimensions.");
        vsapi->freeNode(d.node);
        vsapi->freeNode(d.pelclip);
        return;
    }

    d.usePelClip = false;
    if (d.pelclip && (d.nPel >= 2)) {
        if ((pelvi->width == d.vi.width * d.nPel) &&
            (pelvi->height == d.vi.height * d.nPel)) {
            d.usePelClip = true;
        } else {
            vsapi->mapSetError(out, "Super: pelclip's dimensions must be a multiple of the input clip's dimensions.");
            vsapi->freeNode(d.pelclip);
            vsapi->freeNode(d.node);
            return;
        }
    }

    d.nBlkSizeX = vsapi->mapGetIntSaturated(in, "blksize", 0, &err);
    d.nBlkSizeY = vsapi->mapGetIntSaturated(in, "blksizev", 0, &err);
    if (err)
        d.nBlkSizeY = d.nBlkSizeX;

    d.nOverlapX = vsapi->mapGetIntSaturated(in, "overlap", 0, &err);
    d.nOverlapY = vsapi->mapGetIntSaturated(in, "overlapv", 0, &err);
    if (err)
        d.nOverlapY = d.nOverlapX;

    data = (MVSuperData *)malloc(sizeof(d));
    *data = d;

    VSFilterDependency deps[1] = { 
        {data->node, rpStrictSpatial}
    };

    vsapi->createVideoFilter(out, "Super", &data->vi, mvsuperGetFrame, mvsuperFree, fmParallel, deps, ARRAY_SIZE(deps), data, core);
}


void mvsuperRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("Super", 
                 "clip:vnode;"
                 "hpad:int:opt;"
                 "vpad:int:opt;"
                 "blksize:int:opt;"
                 "blksizev:int:opt;"
                 "overlap:int:opt;"
                 "overlapv:int:opt;"
                 "pel:int:opt;"
                 "levels:int:opt;"
                 "sharp:int:opt;"
                 "rfilter:int:opt;"
                 "pelclip:vnode:opt;",
                 "clip:vnode;",
                 mvsuperCreate, 0, plugin);
}
