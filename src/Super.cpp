#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "SuperPyramid.h"
#include "CommonMacros.h"


struct SuperDataExtra {
    VSVideoInfo vi;

    int nHPad;
    int nVPad;
    int nPel;
    int nLevels;
    SharpParam sharp;
    RFilterParam rfilter;

    int nBlkSizeX;
    int nBlkSizeY;
    int nOverlapX;
    int nOverlapY;

    bool usePelClip;

    std::string prefix;
};

typedef DualNodeData<SuperDataExtra> SuperData;


static const VSFrame *VS_CC superGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) noexcept {
    SuperData *d = reinterpret_cast<SuperData *>(instanceData);

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->node1, frameCtx);
        if (d->usePelClip)
            vsapi->requestFrameFilter(n, d->node2, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrame *src = vsapi->getFrameFilter(n, d->node1, frameCtx);

        FramePyramid pyramid(src, d->nLevels, d->nBlkSizeX, d->nBlkSizeY, d->nOverlapX, d->nOverlapY, d->nHPad, d->nVPad, d->rfilter, core, vsapi);

        if (d->usePelClip) {
            const VSFrame *srcPel = vsapi->getFrameFilter(n, d->node2, frameCtx);
            pyramid.SetExternalPelPlanes(srcPel, d->nPel, core, vsapi);
            vsapi->freeFrame(srcPel);
        } else if (d->nPel > 1) {
            pyramid.GeneratePelPlanes(d->nPel, d->sharp, core, vsapi);
        }

        VSFrame *dst = vsapi->copyFrame(src, core);
        vsapi->freeFrame(src);

        pyramid.ExportFrameData(dst, d->prefix);

        return dst;
    }

    return nullptr;
}

static void VS_CC superCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) noexcept {
    std::unique_ptr<SuperData> d(new SuperData(vsapi));

    int err;

    d->nHPad = vsapi->mapGetIntSaturated(in, "hpad", 0, &err);
    if (err)
        d->nHPad = 16;

    d->nVPad = vsapi->mapGetIntSaturated(in, "vpad", 0, &err);
    if (err)
        d->nVPad = 16;

    d->nPel = vsapi->mapGetIntSaturated(in, "pel", 0, &err);
    if (err)
        d->nPel = 2;

    d->nLevels = vsapi->mapGetIntSaturated(in, "levels", 0, &err);

    d->sharp = static_cast<SharpParam>(vsapi->mapGetIntSaturated(in, "sharp", 0, &err));
    if (err)
        d->sharp = SharpParam::Wiener;

    d->rfilter = static_cast<RFilterParam>(vsapi->mapGetIntSaturated(in, "rfilter", 0, &err));
    if (err)
        d->rfilter = RFilterParam::Bilinear;

    const char *prefix = vsapi->mapGetData(in, "prefix", 0, &err);
    if (prefix)
        d->prefix = prefix;
    else
        d->prefix = DEFAULT_MVUTENSILS_PREFIX;

    try {

        if ((d->nPel != 1) && (d->nPel != 2) && (d->nPel != 4))
            throw std::runtime_error("pel must be 1, 2, or 4");

        if (d->sharp != SharpParam::Bilinear && d->sharp != SharpParam::Bicubic && d->sharp != SharpParam::Wiener)
            throw std::runtime_error("sharp must be between 0 and 2");

        if (d->rfilter != RFilterParam::Simple && d->rfilter != RFilterParam::Bilinear && d->rfilter != RFilterParam::Cubic)
            throw std::runtime_error("rfilter must be between 0 and 2");

        d->node1 = vsapi->mapGetNode(in, "clip", 0, 0);
        d->vi = *vsapi->getVideoInfo(d->node1);

        if (!vsh::isConstantVideoFormat(&d->vi) || d->vi.format.bitsPerSample > 16 || d->vi.format.sampleType != stInteger ||
            d->vi.format.subSamplingW > 1 || d->vi.format.subSamplingH > 1 || (d->vi.format.colorFamily != cfYUV && d->vi.format.colorFamily != cfGray))
            throw std::runtime_error("input clip must be GRAY, YUV420, YUV422, YUV440, or YUV444, up to 16 bits, with constant dimensions");

        int xRatioUV = 1 << d->vi.format.subSamplingW;
        int yRatioUV = 1 << d->vi.format.subSamplingH;

        // FIXME, maybe have a static helper function for this calculation since it's needed in multiple places, and it should be per plane or level
        // at least two pixels width and height of chroma
        int nLevelsMax = 0;
        int nLevelWidth = d->vi.width;
        int nLevelHeight = d->vi.height;
        while (true) {
            nLevelHeight = PlaneDimensionLuma(nLevelHeight, yRatioUV, d->nVPad);
            nLevelWidth = PlaneDimensionLuma(nLevelWidth, xRatioUV, d->nHPad);
            if (nLevelHeight < yRatioUV * 2 || nLevelWidth < xRatioUV * 2)
                break;
            nLevelsMax++;
        }
        if (d->nLevels <= 0 || d->nLevels > nLevelsMax)
            d->nLevels = nLevelsMax;

        d->node2 = vsapi->mapGetNode(in, "pelclip", 0, &err);
        const VSVideoInfo *pelvi = d->node2 ? vsapi->getVideoInfo(d->node2) : nullptr;

        if (pelvi && (!vsh::isConstantVideoFormat(pelvi) || !vsh::isSameVideoFormat(&pelvi->format, &d->vi.format)))
            throw std::runtime_error("pelclip must have the same format as the input clip, and it must have constant dimensions");

        d->usePelClip = false;
        if (pelvi && (d->nPel >= 2)) {
            if ((pelvi->width == d->vi.width * d->nPel) &&
                (pelvi->height == d->vi.height * d->nPel)) {
                d->usePelClip = true;
            } else {
                throw std::runtime_error("pelclip's dimensions must be a multiple of the input clip's dimensions");
            }
        }

        d->nBlkSizeX = vsapi->mapGetIntSaturated(in, "blksizeh", 0, &err);
        d->nBlkSizeY = vsapi->mapGetIntSaturated(in, "blksizev", 0, &err);
        if (err)
            d->nBlkSizeY = d->nBlkSizeX;

        if (d->nBlkSizeX < 0 || d->nBlkSizeY < 0)
            throw std::runtime_error("block size must be non-negative");

        d->nOverlapX = vsapi->mapGetIntSaturated(in, "overlaph", 0, &err);
        d->nOverlapY = vsapi->mapGetIntSaturated(in, "overlapv", 0, &err);
        if (err)
            d->nOverlapY = d->nOverlapX;

        if (d->nOverlapY < 0 || d->nOverlapX < 0)
            throw std::runtime_error("overlap must be non-negative");

    } catch (std::runtime_error &e) {
        vsapi->mapSetError(out, ("Super: " + std::string(e.what())).c_str());
        return;
    }

    VSFilterDependency deps[2] = { 
        { d->node1, rpStrictSpatial },
        { d->node2, rpStrictSpatial }
    };

    vsapi->createVideoFilter(out, "Super", &d->vi, superGetFrame, filterFree<SuperData>, fmParallel, deps, d->usePelClip ? 2 : 1, d.get(), core);
    d.release();
}


void superRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) noexcept {
    vspapi->registerFunction("Super", 
                 "clip:vnode;"
                 "hpad:int:opt;"
                 "vpad:int:opt;"
                 "blksizeh:int:opt;"
                 "blksizev:int:opt;"
                 "overlaph:int:opt;"
                 "overlapv:int:opt;"
                 "levels:int:opt;"
                 "sharp:int:opt;"
                 "rfilter:int:opt;"
                 "pel:int:opt;"
                 "pelclip:vnode:opt;"
                 "prefix:data:opt;",
                 "clip:vnode;",
                 superCreate, nullptr, plugin);
}
