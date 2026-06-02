#include <climits>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "Common.h"
#include "SuperPyramid.h"
#include "MotionBlockPyramid.h"


struct AnalyseDataExtra {
    const VSVideoInfo *vi;
    const VSVideoInfo *supervi;

    int nBlkSizeX;
    int nBlkSizeY;
    int nOverlapX;
    int nOverlapY;

    int deltaFrame;

    int nLambda;

    SearchType searchType;
    SearchType searchTypeCoarse;

    int nSearchParam; // usually search radius
    int nPelSearch; // search radius at finest level

    int lsad;        // SAD limit for lambda using - added by Fizick
    int pnew;        // penalty to cost for new canditate - added by Fizick
    int plevel;      // penalty factors (lambda, plen) level scaling - added by Fizick
    bool global;     // use global motion predictor
    int pglobal;     // penalty factor for global motion predictor
    int pzero;       // penalty factor for zero vector
    int64_t badSAD;  //  SAD threshold to make more wide search for bad vectors
    int badrange;    // range (radius) of wide search
    bool meander;    //meander (alternate) scan blocks (even row left to right, odd row right to left
    bool tryMany;    // try refine around many predictors
    bool useSatd;

    int levels;
    int searchparam;
    bool chroma;
    bool truemotion;

    bool fields;
    bool tff;
    bool tff_exists;

    MotionBlockPyramid::DivideExtra divideExtra;

    std::string prefix;
};

typedef SingleNodeData<AnalyseDataExtra> AnalyseData;

static const VSFrame *VS_CC analyseGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) noexcept {
    AnalyseData *d = reinterpret_cast<AnalyseData *>(instanceData);

    int nref = n + d->deltaFrame;

    if (activationReason == arInitial) {
        if (nref >= 0 && nref < d->vi->numFrames) {
            if (n < nref) {
                vsapi->requestFrameFilter(n, d->node, frameCtx);
                vsapi->requestFrameFilter(nref, d->node, frameCtx);
            } else {
                vsapi->requestFrameFilter(nref, d->node, frameCtx);
                vsapi->requestFrameFilter(n, d->node, frameCtx);
            }
        } else { // too close to beginning/end of clip
            vsapi->requestFrameFilter(n, d->node, frameCtx);
        }
    } else if (activationReason == arAllFramesReady) {
        try {
            const VSFrame *src = vsapi->getFrameFilter(n, d->node, frameCtx);
            FramePyramid srcFramePyramid(src, -1, d->prefix, core, vsapi);

            const VSMap *srcProps = vsapi->getFramePropertiesRO(src);
            int err;

            bool src_top_field = !!vsapi->mapGetInt(srcProps, "_Field", 0, &err);
            if (err && d->fields && !d->tff_exists) {
                vsapi->setFilterError("Analyse: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                return nullptr;
            }

            // if tff was passed, it overrides _Field.
            if (d->tff_exists)
                src_top_field = d->tff ^ (n % 2);

            MotionBlockPyramid vectorFields(srcFramePyramid, d->nBlkSizeX, d->nBlkSizeY, d->nOverlapX, d->nOverlapY, d->levels, d->chroma, d->deltaFrame);

            if (nref >= 0 && nref < d->vi->numFrames) {
                const VSFrame *ref = vsapi->getFrameFilter(nref, d->node, frameCtx);
                FramePyramid refFramePyramid(ref, -1, d->prefix, core, vsapi);

                const VSMap *refProps = vsapi->getFramePropertiesRO(ref);

                bool ref_top_field = !!vsapi->mapGetInt(refProps, "_Field", 0, &err);
                if (err && d->fields && !d->tff_exists) {
                    vsapi->setFilterError("Analyse: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                    return nullptr;
                }

                // if tff was passed, it overrides _Field.
                if (d->tff_exists)
                    ref_top_field = d->tff ^ (nref % 2);

                int fieldShift = 0;
                if (d->fields && srcFramePyramid.nPel > 1 && (d->deltaFrame % 2)) {
                    fieldShift = (src_top_field && !ref_top_field) ? srcFramePyramid.nPel / 2 : ((ref_top_field && !src_top_field) ? -(srcFramePyramid.nPel / 2) : 0);
                    // vertical shift of fields for fieldbased video at finest level pel2
                }

                vectorFields.SearchMVs(srcFramePyramid, refFramePyramid, d->searchType, d->nSearchParam, d->nPelSearch, d->nLambda, d->lsad, d->pnew, d->plevel, d->global, fieldShift, d->useSatd, d->pzero, d->pglobal, d->badSAD, d->badrange, d->meander, d->tryMany, d->searchTypeCoarse, d->chroma);

                vectorFields.DivideVectorsExtra(d->divideExtra);
            }

            VSFrame *dst = vsapi->copyFrame(src, core);
            vectorFields.ExportFrameData(dst, true, d->prefix, core, vsapi);

            return dst;

        } catch (std::runtime_error &e) {
            // Note that exceptions can only happen before SearchMVs MMX code
            vsapi->setFilterError(("Analyse: " + std::string(e.what())).c_str(), frameCtx);
            return nullptr;
        }
    }

    return nullptr;
}


static void VS_CC analyseCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) noexcept {
    std::unique_ptr<AnalyseData> d(new AnalyseData(vsapi));

    int err;

    d->nBlkSizeX = vsapi->mapGetIntSaturated(in, "blksizeh", 0, &err);
    if (err)
        d->nBlkSizeX = 8;

    d->nBlkSizeY = vsapi->mapGetIntSaturated(in, "blksizev", 0, &err);
    if (err)
        d->nBlkSizeY = d->nBlkSizeX;

    d->levels = vsapi->mapGetIntSaturated(in, "levels", 0, &err);

    d->searchType = static_cast<SearchType>(vsapi->mapGetIntSaturated(in, "search", 0, &err));
    if (err)
        d->searchType = SearchType::Hex2;

    d->searchTypeCoarse = static_cast<SearchType>(vsapi->mapGetIntSaturated(in, "search_coarse", 0, &err));
    if (err)
        d->searchTypeCoarse = SearchType::Exhaustive;

    d->searchparam = vsapi->mapGetIntSaturated(in, "searchparam", 0, &err);
    if (err)
        d->searchparam = 2;

    d->nPelSearch = vsapi->mapGetIntSaturated(in, "pelsearch", 0, &err);

    d->chroma = !!vsapi->mapGetInt(in, "chroma", 0, &err);
    if (err)
        d->chroma = true;

    d->deltaFrame = vsapi->mapGetIntSaturated(in, "delta", 0, &err);
    if (err)
        d->deltaFrame = 1;

    d->truemotion = !!vsapi->mapGetInt(in, "truemotion", 0, &err);
    if (err)
        d->truemotion = true;

    d->nLambda = vsapi->mapGetIntSaturated(in, "lambda", 0, &err);
    if (err)
        d->nLambda = d->truemotion ? (1000 * d->nBlkSizeX * d->nBlkSizeY / 64) : 0;

    d->lsad = vsapi->mapGetIntSaturated(in, "lsad", 0, &err);
    if (err)
        d->lsad = d->truemotion ? 1200 : 400;

    d->plevel = vsapi->mapGetIntSaturated(in, "plevel", 0, &err);
    if (err)
        d->plevel = d->truemotion ? 1 : 0;

    d->global = !!vsapi->mapGetInt(in, "global", 0, &err);
    if (err)
        d->global = d->truemotion;

    d->pnew = vsapi->mapGetIntSaturated(in, "pnew", 0, &err);
    if (err)
        d->pnew = d->truemotion ? 50 : 0; // relative to 256

    d->pzero = vsapi->mapGetIntSaturated(in, "pzero", 0, &err);
    if (err)
        d->pzero = d->pnew;

    d->pglobal = vsapi->mapGetIntSaturated(in, "pglobal", 0, &err);

    d->nOverlapX = vsapi->mapGetIntSaturated(in, "overlaph", 0, &err);

    d->nOverlapY = vsapi->mapGetIntSaturated(in, "overlapv", 0, &err);
    if (err)
        d->nOverlapY = d->nOverlapX;

    d->useSatd = !!vsapi->mapGetInt(in, "satd", 0, &err);

    d->divideExtra = static_cast<MotionBlockPyramid::DivideExtra>(vsapi->mapGetIntSaturated(in, "divide", 0, &err));

    d->badSAD = vsapi->mapGetIntSaturated(in, "badsad", 0, &err);
    if (err)
        d->badSAD = 10000;

    d->badrange = vsapi->mapGetIntSaturated(in, "badrange", 0, &err);
    if (err)
        d->badrange = 24;

    d->meander = !!vsapi->mapGetInt(in, "meander", 0, &err);
    if (err)
        d->meander = true;

    d->tryMany = !!vsapi->mapGetInt(in, "trymany", 0, &err);

    d->fields = !!vsapi->mapGetInt(in, "fields", 0, &err);

    d->tff = !!vsapi->mapGetInt(in, "tff", 0, &err);
    d->tff_exists = !err;

    const char *prefix = vsapi->mapGetData(in, "prefix", 0, &err);
    if (prefix)
        d->prefix = prefix;
    else
        d->prefix = DEFAULT_MVUTENSILS_PREFIX;

    try {

        if (d->searchType != SearchType::Logarithmic && d->searchType != SearchType::Exhaustive && d->searchType != SearchType::Hex2 && d->searchType != SearchType::UnevenMultiHexagon && d->searchType != SearchType::Horizontal && d->searchType != SearchType::Vertical)
            throw std::runtime_error("search must be between 0 and 5");

        if (d->searchTypeCoarse != SearchType::Logarithmic && d->searchTypeCoarse != SearchType::Exhaustive && d->searchTypeCoarse != SearchType::Hex2 && d->searchTypeCoarse != SearchType::UnevenMultiHexagon && d->searchTypeCoarse != SearchType::Horizontal && d->searchTypeCoarse != SearchType::Vertical)
            throw std::runtime_error("search_coarse must be between 0 and 5");

        if (d->useSatd && d->nBlkSizeX == 16 && d->nBlkSizeY == 2)
            throw std::runtime_error("satd cannot work with 16x2 blocks");

        if (d->divideExtra != MotionBlockPyramid::DivideExtra::No && d->divideExtra != MotionBlockPyramid::DivideExtra::Point && d->divideExtra != MotionBlockPyramid::DivideExtra::Median)
            throw std::runtime_error("divide must be between 0 and 2");


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
            (d->nBlkSizeX != 128 || d->nBlkSizeY != 128))
            throw std::runtime_error("the block size must be 4x4, 8x4, 8x8, 16x2, 16x8, 16x16, 32x16, 32x32, 64x32, 64x64, 128x64 or 128x128");


        if (d->plevel < 0 || d->plevel > 2)
            throw std::runtime_error("plevel must be between 0 and 2");

        if (d->pnew < 0 || d->pnew > 256)
            throw std::runtime_error("pnew must be between 0 and 256");

        if (d->pzero < 0 || d->pzero > 256)
            throw std::runtime_error("pzero must be between 0 and 256");

        if (d->pglobal < 0 || d->pglobal > 256)
            throw std::runtime_error("pglobal must be between 0 and 256");

        if (d->nOverlapX < 0 || d->nOverlapX > d->nBlkSizeX / 2 ||
            d->nOverlapY < 0 || d->nOverlapY > d->nBlkSizeY / 2)
            throw std::runtime_error("overlaph must be at most half of blksizeh, overlapv must be at most half of blksizev, and they both need to be at least 0");

        if (d->divideExtra != MotionBlockPyramid::DivideExtra::No && (d->nBlkSizeX < 8 || d->nBlkSizeY < 8))
            throw std::runtime_error("blksize and blksizev must be at least 8 when divide>0");


        d->nSearchParam = std::max(d->searchparam, 1);


        d->node = vsapi->mapGetNode(in, "super", 0, 0);
        d->supervi = vsapi->getVideoInfo(d->node);
        d->vi = d->supervi;

        if (!vsh::isConstantVideoFormat(d->vi) || d->vi->format.bitsPerSample > 16 || d->vi->format.sampleType != stInteger || d->vi->format.subSamplingW > 1 || d->vi->format.subSamplingH > 1 || (d->vi->format.colorFamily != cfYUV && d->vi->format.colorFamily != cfGray))
            throw std::runtime_error("Input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant format and dimensions");

        if (d->vi->format.colorFamily == cfGray)
            d->chroma = false;

        int pixelMax = (1 << d->vi->format.bitsPerSample) - 1;
        d->lsad = (int)((double)d->lsad * pixelMax / 255.0 + 0.5);
        d->badSAD = (int)((double)d->badSAD * pixelMax / 255.0 + 0.5);
        d->nLambda = (int)((double)d->nLambda * pixelMax / 255.0 + 0.5);

        d->lsad = (int64_t)d->lsad * (d->nBlkSizeX * d->nBlkSizeY) / 64;
        d->badSAD = d->badSAD * (d->nBlkSizeX * d->nBlkSizeY) / 64;

        if (d->nOverlapX % (1 << d->vi->format.subSamplingW) ||
            d->nOverlapY % (1 << d->vi->format.subSamplingH))
            throw std::runtime_error("The requested overlap is incompatible with the super clip's subsampling");
       

        if ((d->divideExtra != MotionBlockPyramid::DivideExtra::No) && (d->nOverlapX % (2 << d->vi->format.subSamplingW) ||
                              d->nOverlapY % (2 << d->vi->format.subSamplingH)))
            throw std::runtime_error("overlaph and overlapv must be multiples of 2 or 4 when divide > 0, depending on the super clip's subsampling");

        if (d->deltaFrame == 0)
            throw std::runtime_error("delta can't be 0");

        char errorMsg[ERROR_SIZE] = {};
        const VSFrame *evil = vsapi->getFrame(0, d->node, errorMsg, ERROR_SIZE);
        if (!evil)
            throw std::runtime_error("failed to retrieve first frame from super clip. Error message: " + std::string(errorMsg));

        FramePyramid super(evil, -1, d->prefix, core, vsapi);

        if (d->nPelSearch <= 0)
            d->nPelSearch = super.nPel;

        MotionBlockPyramid DryRun(super, d->nBlkSizeX, d->nBlkSizeY, d->nOverlapX, d->nOverlapY, d->levels, d->chroma, d->deltaFrame);

    } catch (std::runtime_error &e) {
        vsapi->mapSetError(out, ("Analyse: " + std::string(e.what())).c_str());
        return;
    }

    VSFilterDependency deps[1] = { 
        {d->node, rpGeneral}
    };

    vsapi->createVideoFilter(out, "Analyse", d->vi, analyseGetFrame, filterFree<AnalyseData>, fmParallel, deps, ARRAY_SIZE(deps), d.get(), core);
    d.release();
}


void analyseRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) noexcept {
    vspapi->registerFunction("Analyse",
                 "super:vnode;"
                 "blksizeh:int:opt;"
                 "blksizev:int:opt;"
                 "levels:int:opt;"
                 "search:int:opt;"
                 "searchparam:int:opt;"
                 "pelsearch:int:opt;"
                 "lambda:int:opt;"
                 "chroma:int:opt;"
                 "delta:int:opt;"
                 "truemotion:int:opt;"
                 "lsad:int:opt;"
                 "plevel:int:opt;"
                 "global:int:opt;"
                 "pnew:int:opt;"
                 "pzero:int:opt;"
                 "pglobal:int:opt;"
                 "overlaph:int:opt;"
                 "overlapv:int:opt;"
                 "divide:int:opt;"
                 "badsad:int:opt;"
                 "badrange:int:opt;"
                 "meander:int:opt;"
                 "trymany:int:opt;"
                 "fields:int:opt;"
                 "tff:int:opt;"
                 "search_coarse:int:opt;"
                 "satd:int:opt;"
                 "prefix:data:opt;",
                 "clip:vnode;",
                 analyseCreate, nullptr, plugin);
}
