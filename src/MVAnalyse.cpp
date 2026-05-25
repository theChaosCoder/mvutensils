#include <climits>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "CPU.h"
#include "CommonMacros.h"
#include "SuperPyramid.h"
#include "MotionBlockPyramid.h"

template<typename T>
struct SingleNodeData : public T {
private:
    const VSAPI *vsapi;
public:
    VSNode *node = nullptr;

    explicit SingleNodeData(const VSAPI *vsapi) noexcept : T(), vsapi(vsapi) {
    }

    ~SingleNodeData() {
        vsapi->freeNode(node);
    }
};


struct MVAnalyseDataExtra {
    VSNode *node;
    const VSVideoInfo *vi;
    const VSVideoInfo *supervi;

    int nBlkSizeX;
    int nBlkSizeY;
    int nOverlapX;
    int nOverlapY;

    int deltaFrame;

    /*! \brief motion vecteur cost factor */
    int nLambda;

    /*! \brief search type chosen for refinement in the EPZ */
    SearchType searchType;

    SearchType searchTypeCoarse;

    /*! \brief additionnal parameter for this search */
    int nSearchParam; // usually search radius

    int nPelSearch; // search radius at finest level

    int lsad;        // SAD limit for lambda using - added by Fizick
    int pnew;        // penalty to cost for new canditate - added by Fizick
    int plevel;      // penalty factors (lambda, plen) level scaling - added by Fizick
    bool global;     // use global motion predictor
    int pglobal;     // penalty factor for global motion predictor
    int pzero;       // penalty factor for zero vector
    int divideExtra; // divide blocks on sublocks with median motion
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
    int tff;
    bool tff_exists;
};

typedef SingleNodeData<MVAnalyseDataExtra> MVAnalyseData;

static const VSFrame *VS_CC mvanalyseGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVAnalyseData *d = (MVAnalyseData *)instanceData;
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
        const VSFrame *src = vsapi->getFrameFilter(n, d->node, frameCtx);
        const VSMap *srcprops = vsapi->getFramePropertiesRO(src);
        int err;

        int src_top_field = !!vsapi->mapGetInt(srcprops, "_Field", 0, &err);
        if (err && d->fields && !d->tff_exists) {
            vsapi->setFilterError("Analyse: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
            vsapi->freeFrame(src);
            return NULL;
        }

        // if tff was passed, it overrides _Field.
        if (d->tff_exists)
            src_top_field = d->tff ^ (n % 2);

        FramePyramid pSrcGOF(src, "MVUtensils", core, vsapi);
        MotionBlockPyramid vectorFields(pSrcGOF, d->nBlkSizeX, d->nBlkSizeY, d->nOverlapX, d->nOverlapY, d->levels, d->chroma, d->deltaFrame, d->supervi->format.bitsPerSample);

        if (nref >= 0 && nref < d->vi->numFrames) {
            const VSFrame *ref = vsapi->getFrameFilter(nref, d->node, frameCtx);
            const VSMap *refprops = vsapi->getFramePropertiesRO(ref);

            int ref_top_field = !!vsapi->mapGetInt(refprops, "_Field", 0, &err);
            if (err && d->fields && !d->tff_exists) {
                vsapi->setFilterError("Analyse: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                vsapi->freeFrame(src);
                vsapi->freeFrame(ref);
                return NULL;
            }

            // if tff was passed, it overrides _Field.
            if (d->tff_exists)
                ref_top_field = d->tff ^ (nref % 2);

            FramePyramid pRefGOF(ref, "MVUtensils", core, vsapi);

            int fieldShift = 0;
            if (d->fields && pSrcGOF.nPel > 1 && (d->deltaFrame % 2)) {
                fieldShift = (src_top_field && !ref_top_field) ? pSrcGOF.nPel / 2 : ((ref_top_field && !src_top_field) ? -(pSrcGOF.nPel / 2) : 0);
                // vertical shift of fields for fieldbased video at finest level pel2
            }

            // FIXME, chroma has a different meaning here? this one controls if chroma is used for ME
            vectorFields.SearchMVs(pSrcGOF, pRefGOF, d->searchType, d->nSearchParam, d->nPelSearch, d->nLambda, d->lsad, d->pnew, d->plevel, d->global, fieldShift, d->useSatd, d->pzero, d->pglobal, d->badSAD, d->badrange, d->meander, d->tryMany, d->searchTypeCoarse, d->chroma);

            vsapi->freeFrame(ref);

            if (d->divideExtra) {
                // make extra level with divided sublocks with median (not estimated) motion
                vectorFields.DivideVectorsExtra(d->divideExtra);
            }

#if defined(MVTOOLS_X86)
            // FIXME: Get rid of all mmx shit.
            mvtools_cpu_emms();
#endif
        }

        VSFrame *dst = vsapi->copyFrame(src, core);
        vectorFields.ExportFrameData(dst, "MVUtensils", core, vsapi);
        vsapi->freeFrame(src);

        return dst;
    }

    return nullptr;
}


static void VS_CC mvanalyseFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVAnalyseData *d = (MVAnalyseData *)instanceData;
    delete d;
}


static void VS_CC mvanalyseCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    std::unique_ptr<MVAnalyseData> d(new MVAnalyseData(vsapi));
    int err;

    d->nBlkSizeX = vsapi->mapGetIntSaturated(in, "blksize", 0, &err);
    if (err)
        d->nBlkSizeX = 8;

    d->nBlkSizeY = vsapi->mapGetIntSaturated(in, "blksizev", 0, &err);
    if (err)
        d->nBlkSizeY = d->nBlkSizeX;

    d->levels = vsapi->mapGetIntSaturated(in, "levels", 0, &err);

    d->searchType = (SearchType)(vsapi->mapGetIntSaturated(in, "search", 0, &err));
    if (err)
        d->searchType = SearchType::Hex2;

    d->searchTypeCoarse = (SearchType)(vsapi->mapGetIntSaturated(in, "search_coarse", 0, &err));
    if (err)
        d->searchTypeCoarse = SearchType::Exhaustive;

    d->searchparam = vsapi->mapGetIntSaturated(in, "searchparam", 0, &err);
    if (err)
        d->searchparam = 2;

    d->nPelSearch = vsapi->mapGetIntSaturated(in, "pelsearch", 0, &err);

    d->chroma = !!vsapi->mapGetInt(in, "chroma", 0, &err);
    if (err)
        d->chroma = true;

    // FIXME, check so deltaframe isn't 0 in a good way
    d->deltaFrame = vsapi->mapGetIntSaturated(in, "delta", 0, &err);
    if (err || !d->deltaFrame)
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

    d->nOverlapX = vsapi->mapGetIntSaturated(in, "overlap", 0, &err);

    d->nOverlapY = vsapi->mapGetIntSaturated(in, "overlapv", 0, &err);
    if (err)
        d->nOverlapY = d->nOverlapX;

    d->useSatd = !!vsapi->mapGetInt(in, "satd", 0, &err);

    d->divideExtra = vsapi->mapGetIntSaturated(in, "divide", 0, &err);

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

    // FIXME, restore check
    /*
    if (d->searchType < 0 || d->searchType > 7) {
        vsapi->mapSetError(out, "Analyse: search must be between 0 and 7 (inclusive).");
        return;
    }

    if (d->searchTypeCoarse < 0 || d->searchTypeCoarse > 7) {
        vsapi->mapSetError(out, "Analyse: search_coarse must be between 0 and 7 (inclusive).");
        return;
    }
    */

    if (d->useSatd && d->nBlkSizeX == 16 && d->nBlkSizeY == 2) {
        vsapi->mapSetError(out, "Analyse: satd cannot work with 16x2 blocks.");
        return;
    }

    if (d->divideExtra < 0 || d->divideExtra > 2) {
        vsapi->mapSetError(out, "Analyse: divide must be between 0 and 2 (inclusive).");
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

        vsapi->mapSetError(out, "Analyse: the block size must be 4x4, 8x4, 8x8, 16x2, 16x8, 16x16, 32x16, 32x32, 64x32, 64x64, 128x64, or 128x128.");
        return;
    }


    if (d->plevel < 0 || d->plevel > 2) {
        vsapi->mapSetError(out, "Analyse: plevel must be between 0 and 2 (inclusive).");
        return;
    }


    if (d->pnew < 0 || d->pnew > 256) {
        vsapi->mapSetError(out, "Analyse: pnew must be between 0 and 256 (inclusive).");
        return;
    }


    if (d->pzero < 0 || d->pzero > 256) {
        vsapi->mapSetError(out, "Analyse: pzero must be between 0 and 256 (inclusive).");
        return;
    }


    if (d->pglobal < 0 || d->pglobal > 256) {
        vsapi->mapSetError(out, "Analyse: pglobal must be between 0 and 256 (inclusive).");
        return;
    }


    if (d->nOverlapX < 0 || d->nOverlapX > d->nBlkSizeX / 2 ||
        d->nOverlapY < 0 || d->nOverlapY > d->nBlkSizeY / 2) {
        vsapi->mapSetError(out, "Analyse: overlap must be at most half of blksize, overlapv must be at most half of blksizev, and they both need to be at least 0.");
        return;
    }

    if (d->divideExtra && (d->nBlkSizeX < 8 || d->nBlkSizeY < 8)) {
        vsapi->mapSetError(out, "Analyse: blksize and blksizev must be at least 8 when divide=True.");
        return;
    }


    d->nSearchParam = std::max(d->searchparam, 1);


    d->node = vsapi->mapGetNode(in, "super", 0, 0);
    d->supervi = vsapi->getVideoInfo(d->node);
    d->vi = d->supervi;

    if (!vsh::isConstantVideoFormat(d->vi) || d->vi->format.bitsPerSample > 16 || d->vi->format.sampleType != stInteger || d->vi->format.subSamplingW > 1 || d->vi->format.subSamplingH > 1 || (d->vi->format.colorFamily != cfYUV && d->vi->format.colorFamily != cfGray)) {
        vsapi->mapSetError(out, "Analyse: Input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant format and dimensions.");
        return;
    }

    if (d->vi->format.colorFamily == cfGray)
        d->chroma = false;

    int pixelMax = (1 << d->vi->format.bitsPerSample) - 1;
    d->lsad = (int)((double)d->lsad * pixelMax / 255.0 + 0.5);
    d->badSAD = (int)((double)d->badSAD * pixelMax / 255.0 + 0.5);
    d->nLambda = (int)((double)d->nLambda * pixelMax / 255.0 + 0.5);

    d->lsad = (int64_t)d->lsad * (d->nBlkSizeX * d->nBlkSizeY) / 64;
    d->badSAD = d->badSAD * (d->nBlkSizeX * d->nBlkSizeY) / 64;

    if (d->nOverlapX % (1 << d->vi->format.subSamplingW) ||
        d->nOverlapY % (1 << d->vi->format.subSamplingH)) {
        vsapi->mapSetError(out, "Analyse: The requested overlap is incompatible with the super clip's subsampling.");
        return;
    }

    if (d->divideExtra && (d->nOverlapX % (2 << d->vi->format.subSamplingW) ||
                          d->nOverlapY % (2 << d->vi->format.subSamplingH))) { // subsampling times 2
        vsapi->mapSetError(out, "Analyse: overlap and overlapv must be multiples of 2 or 4 when divide=True, depending on the super clip's subsampling.");
        return;
    }

#define ERROR_SIZE 1024
    char errorMsg[ERROR_SIZE] = "Analyse: failed to retrieve first frame from super clip. Error message: ";
    size_t errorLen = strlen(errorMsg);
    const VSFrame *evil = vsapi->getFrame(0, d->node, errorMsg + errorLen, ERROR_SIZE - (int)errorLen);
#undef ERROR_SIZE
    if (!evil) {
        vsapi->mapSetError(out, errorMsg);
        return;
    }

    FramePyramid super(evil, "MVUtensils", core, vsapi);

    // Note that this invalidates all data pointers in super
    vsapi->freeFrame(evil);

    // fill in missing fields
    if (d->nPelSearch <= 0)
        d->nPelSearch = super.nPel; // not below value of 0 at finest level //x

    MotionBlockPyramid DryRun(super, d->nBlkSizeX, d->nBlkSizeY, d->nOverlapX, d->nOverlapY, d->levels, d->chroma, d->deltaFrame, d->supervi->format.bitsPerSample);

    VSFilterDependency deps[1] = { 
        {d->node, rpGeneral} //super
    };

    vsapi->createVideoFilter(out, "Analyse", d->vi, mvanalyseGetFrame, mvanalyseFree, fmParallel, deps, ARRAY_SIZE(deps), d.get(), core);
    d.release();
}


void mvanalyseRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("Analyse",
                 "super:vnode;"
                 "blksize:int:opt;"
                 "blksizev:int:opt;"
                 "levels:int:opt;"
                 "search:int:opt;"
                 "searchparam:int:opt;"
                 "pelsearch:int:opt;"
                 "isb:int:opt;"
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
                 "overlap:int:opt;"
                 "overlapv:int:opt;"
                 "divide:int:opt;"
                 "badsad:int:opt;"
                 "badrange:int:opt;"
                 "meander:int:opt;"
                 "trymany:int:opt;"
                 "fields:int:opt;"
                 "tff:int:opt;"
                 "search_coarse:int:opt;"
                 "satd:int:opt;",
                 "clip:vnode;",
                 mvanalyseCreate, 0, plugin);
}
