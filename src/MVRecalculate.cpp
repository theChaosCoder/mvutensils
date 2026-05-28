#include <memory>
#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "SuperPyramid.h"
#include "MotionBlockPyramid.h"
#include "CommonMacros.h"

struct RecalculateDataExtra {
    const VSVideoInfo *vi;

    int deltaFrame;

    int nLambda;

    int nBlkSizeX;
    int nBlkSizeY;
    int nOverlapX;
    int nOverlapY;

    SearchType searchType;

    int nPel;
    int pnew;  
    int divideExtra; 
    int meander;

    bool useSatd;

    int searchparam;
    bool chroma;
    bool truemotion;
    bool smooth;
    int64_t thSAD;

    bool fields;
    bool tff;
    bool tff_exists;

    std::string prefix;
};

typedef DualNodeData<RecalculateDataExtra> RecalculateData;

static const VSFrame *VS_CC recalculateGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) noexcept {
    RecalculateData *d = reinterpret_cast<RecalculateData *>(instanceData);

    int nref = n + d->deltaFrame;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->node2, frameCtx);

        if (nref >= 0 && nref < d->vi->numFrames) {
            if (n < nref) {
                vsapi->requestFrameFilter(n, d->node1, frameCtx);
                vsapi->requestFrameFilter(nref, d->node1, frameCtx);
            } else {
                vsapi->requestFrameFilter(nref, d->node1, frameCtx);
                vsapi->requestFrameFilter(n, d->node1, frameCtx);
            }
        } else { // too close to beginning/end of clip
            vsapi->requestFrameFilter(n, d->node1, frameCtx);
        }
    } else if (activationReason == arAllFramesReady) {
        // FIXME, how is this set when recalculating?
        gopInit(&vectorFields, d->analysisData.nBlkSizeX, d->analysisData.nBlkSizeY, d->analysisData.nLvCount, d->analysisData.nPel, d->analysisData.nMotionFlags, d->analysisData.nOverlapX, d->analysisData.nOverlapY, d->analysisData.nBlkX, d->analysisData.nBlkY, d->analysisData.xRatioUV, d->analysisData.yRatioUV, d->divideExtra, d->vi->format.bitsPerSample);

        const VSFrame *src = vsapi->getFrameFilter(n, d->node1, frameCtx);
        FramePyramid pSrcGOF(src, 1, d->prefix, core, vsapi);
            

        const VSMap *srcprops = vsapi->getFramePropertiesRO(src);
        int err;

        int src_top_field = !!vsapi->mapGetInt(srcprops, "_Field", 0, &err);
        if (err && d->fields && !d->tff_exists) {
            vsapi->setFilterError("Recalculate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
            return NULL;
        }

        // if tff was passed, it overrides _Field.
        if (d->tff_exists)
            src_top_field = d->tff ^ (n % 2);

        const VSFrame *mvn = vsapi->getFrameFilter(n, d->node2, frameCtx);
        MotionBlockPyramid fgop(mvn, 1, d->prefix, core, vsapi);
        vsapi->freeFrame(mvn);

        if (fgop.HasMotionVectors() && nref >= 0 && nref < d->vi->numFrames) {
            const VSFrame *ref = vsapi->getFrameFilter(nref, d->node1, frameCtx);
            FramePyramid pRefGOF(ref, 1, d->prefix, core, vsapi);
            const VSMap *refprops = vsapi->getFramePropertiesRO(ref);

            int ref_top_field = !!vsapi->mapGetInt(refprops, "_Field", 0, &err);
            if (err && d->fields && !d->tff_exists) {
                vsapi->setFilterError("Recalculate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                return nullptr;
            }

            // if tff was passed, it overrides _Field.
            if (d->tff_exists)
                ref_top_field = d->tff ^ (nref % 2);

            int fieldShift = 0;
            if (d->fields && d->analysisData.nPel > 1 && (d->analysisData.nDeltaFrame % 2)) {
                fieldShift = (src_top_field && !ref_top_field) ? d->analysisData.nPel / 2 : ((ref_top_field && !src_top_field) ? -(d->analysisData.nPel / 2) : 0);
                // vertical shift of fields for fieldbased video at finest level pel2
            }

            fgop.RecalculateMVs(pSrcGOF, pRefGOF, d->searchType, d->searchparam, d->nLambda, d->pnew, fieldShift, d->thSAD, d->useSatd, d->smooth, d->meander);

            fgop.DivideVectorsExtra(d->divideExtra);

        }

        VSFrame *dst = vsapi->copyFrame(src, core);
        fgop.ExportFrameData(dst, true, d->prefix, core, vsapi);

        return dst;
    }

    return nullptr;
}


static void VS_CC recalculateFree(void *instanceData, VSCore *core, const VSAPI *vsapi) noexcept {
    RecalculateData *d = reinterpret_cast<RecalculateData *>(instanceData);
    delete d;
}


static void VS_CC recalculateCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) noexcept {
    std::unique_ptr<RecalculateData> d(new RecalculateData(vsapi));

    int err;

    d->thSAD = vsapi->mapGetInt(in, "thsad", 0, &err);
    if (err)
        d->thSAD = 200;

    d->smooth = !!vsapi->mapGetIntSaturated(in, "smooth", 0, &err);
    if (err)
        d->smooth = true;

    d->nBlkSizeX = vsapi->mapGetIntSaturated(in, "blksizeh", 0, &err);
    if (err)
        d->nBlkSizeX = 8;

    d->nBlkSizeY = vsapi->mapGetIntSaturated(in, "blksizev", 0, &err);
    if (err)
        d->nBlkSizeY = d->nBlkSizeX;

    d->searchType = static_cast<SearchType>(vsapi->mapGetIntSaturated(in, "search", 0, &err));
    if (err)
        d->searchType = SearchType::Hex2;

    d->searchparam = std::max(vsapi->mapGetIntSaturated(in, "searchparam", 0, &err), 1);
    if (err)
        d->searchparam = 2;

    d->chroma = !!vsapi->mapGetInt(in, "chroma", 0, &err);
    if (err)
        d->chroma = 1;

    d->truemotion = !!vsapi->mapGetInt(in, "truemotion", 0, &err);
    if (err)
        d->truemotion = 1;

    d->nLambda = vsapi->mapGetIntSaturated(in, "lambda", 0, &err);
    if (err)
        d->nLambda = d->truemotion ? (1000 * d->nBlkSizeX * d->nBlkSizeY / 64) : 0;

    d->pnew = vsapi->mapGetIntSaturated(in, "pnew", 0, &err);
    if (err)
        d->pnew = d->truemotion ? 50 : 0; // relative to 256

    d->nOverlapX = vsapi->mapGetIntSaturated(in, "overlaph", 0, &err);

    d->nOverlapY = vsapi->mapGetIntSaturated(in, "overlapv", 0, &err);
    if (err)
        d->nOverlapY = d->nOverlapX;

    d->useSatd = !!vsapi->mapGetInt(in, "satd", 0, &err);

    d->divideExtra = vsapi->mapGetIntSaturated(in, "divide", 0, &err);

    d->meander = !!vsapi->mapGetInt(in, "meander", 0, &err);
    if (err)
        d->meander = 1;

    d->fields = !!vsapi->mapGetInt(in, "fields", 0, &err);

    d->tff = !!vsapi->mapGetInt(in, "tff", 0, &err);
    d->tff_exists = !err;

    const char *prefix = vsapi->mapGetData(in, "prefix", 0, &err);
    if (prefix)
        d->prefix = prefix;
    else
        d->prefix = DEFAULT_MVUTENSILS_PREFIX;

    if (d->searchType != SearchType::Logarithmic && d->searchType != SearchType::Exhaustive && d->searchType != SearchType::Hex2 && d->searchType != SearchType::UnevenMultiHexagon && d->searchType != SearchType::Horizontal && d->searchType != SearchType::Vertical)
        throw std::runtime_error("Analyse: search must be between 0 and 5");

    if (d->useSatd && d->nBlkSizeX == 16 && d->nBlkSizeY == 2) {
        vsapi->mapSetError(out, "satd cannot work with 16x2 blocks.");
        return;
    }

    if (d->divideExtra < 0 || d->divideExtra > 2) {
        vsapi->mapSetError(out, "divide must be between 0 and 2 (inclusive).");
        return;
    }


    if ((d->nBlkSizeX != 4 || d->nBlkSizeY != 4) &&
        (d->nBlkSizeX != 8 || d->nBlkSizeY != 4) &&
        (d->nBlkSizeX != 8 || d->nBlkSizeY != 8) &&
        (d->nBlkSizeX != 16 || d->nBlkSizeY != 2) &&
        (d->nBlkSizeX != 16 || d->nBlkSizeY != 8) &&
        (d->nBlkSizeX != 16 || d->nBlkSizeY != 16) &&
        (d->nBlkSizeX != 32 || d->nBlkSizeY != 16) &&
        (d->nBlkSizeX != 32 || d->nBlkSizeY != 32) &&
        (d->nBlkSizeX != 64 || d->nBlkSizeY != 32) &&
        (d->nBlkSizeX != 64 || d->nBlkSizeY != 64) &&
        (d->nBlkSizeX != 128 || d->nBlkSizeY != 64) &&
        (d->nBlkSizeX != 128 || d->nBlkSizeY != 128)) {

        vsapi->mapSetError(out, "Recalculate: the block size must be 4x4, 8x4, 8x8, 16x2, 16x8, 16x16, 32x16, 32x32, 64x32, 64x64, 128x64, or 128x128.");
        return;
    }


    if (d->pnew < 0 || d->pnew > 256) {
        vsapi->mapSetError(out, "Recalculate: pnew must be between 0 and 256 (inclusive).");
        return;
    }


    if (d->nOverlapX < 0 || d->nOverlapX > d->nBlkSizeX / 2 ||
        d->nOverlapY < 0 || d->nOverlapY > d->nBlkSizeY / 2) {
        vsapi->mapSetError(out, "Recalculate: overlap must be at most half of blksize, overlapv must be at most half of blksizev, and they both need to be at least 0.");
        return;
    }

    if (d->divideExtra && (d->nBlkSizeX < 8 || d->nBlkSizeY < 8)) {
        vsapi->mapSetError(out, "Recalculate: blksize and blksizev must be at least 8 when divide=True.");
        return;
    }

    d->node1 = vsapi->mapGetNode(in, "super", 0, 0);
    d->vi = vsapi->getVideoInfo(d->node1);

    if (d->nOverlapX % (1 << d->vi->format.subSamplingW) ||
        d->nOverlapY % (1 << d->vi->format.subSamplingH)) {
        vsapi->mapSetError(out, "The requested overlap is incompatible with the super clip's subsampling.");
        return;
    }

    if (d->divideExtra && (d->nOverlapX % (2 << d->vi->format.subSamplingW) ||
                          d->nOverlapY % (2 << d->vi->format.subSamplingH))) { // subsampling times 2
        vsapi->mapSetError(out, "overlaph and overlapv must be multiples of 2 or 4 when divide=True, depending on the super clip's subsampling.");
        return;
    }

    char errorMsg[ERROR_SIZE] = "Recalculate: failed to retrieve first frame from super clip. Error message: ";
    size_t errorLen = strlen(errorMsg);
    const VSFrame *evil = vsapi->getFrame(0, d->node1, errorMsg + errorLen, ERROR_SIZE - (int)errorLen);

    if (!evil) {
        vsapi->mapSetError(out, errorMsg);
        return;
    }
    const VSMap *props = vsapi->getFramePropertiesRO(evil);
    int evil_err[6];
    int nHeight = vsapi->mapGetIntSaturated(props, "Super_height", 0, &evil_err[0]);
    d->nSuperHPad = vsapi->mapGetIntSaturated(props, "Super_hpad", 0, &evil_err[1]);
    d->nSuperVPad = vsapi->mapGetIntSaturated(props, "Super_vpad", 0, &evil_err[2]);
    d->nSuperPel = vsapi->mapGetIntSaturated(props, "Super_pel", 0, &evil_err[3]);
    d->nSuperModeYUV = vsapi->mapGetIntSaturated(props, "Super_modeyuv", 0, &evil_err[4]);
    d->nSuperLevels = vsapi->mapGetIntSaturated(props, "Super_levels", 0, &evil_err[5]);
    vsapi->freeFrame(evil);

    for (int i = 0; i < 6; i++)
        if (evil_err[i]) {
            vsapi->mapSetError(out, "required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            return;
        }

    d->node2 = vsapi->mapGetNode(in, "vectors", 0, NULL);

    char error[ERROR_SIZE + 1] = {};
    const char *filter_name = "Recalculate";

    adataFromVectorClip(&d.vectors_data, d.vectors, filter_name, "vectors", vsapi, error, ERROR_SIZE);


    d.analysisData.yRatioUV = d.vectors_data.yRatioUV;
    d.analysisData.xRatioUV = d.vectors_data.xRatioUV;

    d.analysisData.nWidth = d.vectors_data.nWidth;
    d.analysisData.nHeight = d.vectors_data.nHeight;

    d.analysisData.nDeltaFrame = d.vectors_data.nDeltaFrame;
    d.analysisData.isBackward = d.vectors_data.isBackward;


    d.analysisData.bitsPerSample = d.vi->format.bitsPerSample;

    int pixelMax = (1 << d.vi->format.bitsPerSample) - 1;
    d->thSAD = (int64_t)((double)d->thSAD * pixelMax / 255.0 + 0.5);
    d->nLambda = (int)((double)d->nLambda * pixelMax / 255.0 + 0.5);

    // normalize threshold to block size
    int referenceBlockSize = 8 * 8;
    d->thSAD = d->thSAD * (d->analysisData.nBlkSizeX * d->analysisData.nBlkSizeY) / referenceBlockSize;
    if (d->chroma)
        d->thSAD += d->thSAD / (d->analysisData.xRatioUV * d->analysisData.yRatioUV) * 2;

    d->nPel = d->nSuperPel; //x

    int nSuperWidth = d->vi->width;
    if (nHeight != d->analysisData.nHeight || nSuperWidth - 2 * d->nSuperHPad != d->analysisData.nWidth) {
        vsapi->mapSetError(out, "Recalculate: wrong frame size.");
        return;
    }

    d.analysisData.nHPadding = d.nSuperHPad; //v2.0    //x
    d.analysisData.nVPadding = d.nSuperVPad;


    int nBlkX = (d->analysisData.nWidth - d->analysisData.nOverlapX) / (d->analysisData.nBlkSizeX - d->analysisData.nOverlapX); //x
    int nBlkY = (d->analysisData.nHeight - d->analysisData.nOverlapY) / (d->analysisData.nBlkSizeY - d->analysisData.nOverlapY);

    d->analysisData.nBlkX = nBlkX;
    d->analysisData.nBlkY = nBlkY;

    d->analysisData.nLvCount = 1;


    if (d->divideExtra) { //v1.8.1
        memcpy(&d->analysisDataDivided, &d->analysisData, sizeof(d->analysisData));
        d->analysisDataDivided.nBlkX = d->analysisData.nBlkX * 2;
        d->analysisDataDivided.nBlkY = d->analysisData.nBlkY * 2;
        d->analysisDataDivided.nBlkSizeX = d->analysisData.nBlkSizeX / 2;
        d->analysisDataDivided.nBlkSizeY = d->analysisData.nBlkSizeY / 2;
        d->analysisDataDivided.nOverlapX = d->analysisData.nOverlapX / 2;
        d->analysisDataDivided.nOverlapY = d->analysisData.nOverlapY / 2;
        d->analysisDataDivided.nLvCount = d->analysisData.nLvCount + 1;
    }

    VSFilterDependency deps[2] = { 
        {d->node1, rpGeneral},
        {d->node2, rpStrictSpatial},
    };

    vsapi->createVideoFilter(out, "Recalculate", d->vi, recalculateGetFrame, recalculateFree, fmParallel, deps, ARRAY_SIZE(deps), d.get(), core);
    d.release();
}


void recalculateRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) noexcept {
    vspapi->registerFunction("Recalculate",
                 "super:vnode;"
                 "vectors:vnode;"
                 "thsad:int:opt;"
                 "smooth:int:opt;"
                 "blksizeh:int:opt;"
                 "blksizev:int:opt;"
                 "search:int:opt;"
                 "searchparam:int:opt;"
                 "lambda:int:opt;"
                 "chroma:int:opt;"
                 "truemotion:int:opt;"
                 "pnew:int:opt;"
                 "overlaph:int:opt;"
                 "overlapv:int:opt;"
                 "divide:int:opt;"
                 "meander:int:opt;"
                 "fields:int:opt;"
                 "tff:int:opt;"
                 "satd:int:opt;"
                 "prefix:data:opt;"
                 "clip:vnode;",
                 recalculateCreate, nullptr, plugin);
}
