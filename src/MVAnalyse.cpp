#include <climits>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "CPU.h"
#include "GroupOfPlanes.h"
#include "MVAnalysisData.h"
#include "CommonMacros.h"
#include "SuperPyramid.h"


typedef struct MVAnalyseData {
    VSNode *node;
    const VSVideoInfo *vi;
    const VSVideoInfo *supervi;

    MVAnalysisData analysisData = {};
    MVAnalysisData analysisDataDivided = {};

    /*! \brief optimisations enabled */
    int opt;

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
    int plen;        // penalty factor (similar to lambda) for vector length - added by Fizick
    int plevel;      // penalty factors (lambda, plen) level scaling - added by Fizick
    int global;     // use global motion predictor
    int pglobal;     // penalty factor for global motion predictor
    int pzero;       // penalty factor for zero vector
    int divideExtra; // divide blocks on sublocks with median motion
    int64_t badSAD;  //  SAD threshold to make more wide search for bad vectors
    int badrange;    // range (radius) of wide search
    int meander;    //meander (alternate) scan blocks (even row left to right, odd row right to left
    int tryMany;    // try refine around many predictors

    bool useSatd;

    int nSuperHPad;
    int nSuperVPad;
    int nSuperPel;
    int nSuperModeYUV;

    int levels;
    int searchparam;
    bool chroma;
    int truemotion;

    int fields;
    int tff;
    int tff_exists;
} MVAnalyseData;



static const VSFrame *VS_CC mvanalyseGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    MVAnalyseData *d = (MVAnalyseData *)instanceData;

    if (activationReason == arInitial) {
        int nref;

        if (d->analysisData.nDeltaFrame > 0) {
            int offset = (d->analysisData.isBackward) ? d->analysisData.nDeltaFrame : -d->analysisData.nDeltaFrame;
            nref = n + offset;

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
        } else {                                 // special static mode
            nref = -d->analysisData.nDeltaFrame; // positive fixed frame number

            if (n < nref) {
                vsapi->requestFrameFilter(n, d->node, frameCtx);
                vsapi->requestFrameFilter(nref, d->node, frameCtx);
            } else {
                vsapi->requestFrameFilter(nref, d->node, frameCtx);
                vsapi->requestFrameFilter(n, d->node, frameCtx);
            }
        }
    } else if (activationReason == arAllFramesReady) {

        GroupOfPlanes vectorFields;

        gopInit(&vectorFields, d->analysisData.nBlkSizeX, d->analysisData.nBlkSizeY, d->analysisData.nLvCount, d->analysisData.nPel, d->analysisData.nMotionFlags, d->analysisData.nCPUFlags, d->analysisData.nOverlapX, d->analysisData.nOverlapY, d->analysisData.nBlkX, d->analysisData.nBlkY, d->analysisData.xRatioUV, d->analysisData.yRatioUV, d->divideExtra, d->supervi->format.bitsPerSample);


        int nref;

        if (d->analysisData.nDeltaFrame > 0) {
            int offset = (d->analysisData.isBackward) ? d->analysisData.nDeltaFrame : -d->analysisData.nDeltaFrame;
            nref = n + offset;
        } else {                                 // special static mode
            nref = -d->analysisData.nDeltaFrame; // positive fixed frame number
        }

        const VSFrame *src = vsapi->getFrameFilter(n, d->node, frameCtx);
        const VSMap *srcprops = vsapi->getFramePropertiesRO(src);
        int err;

        int src_top_field = !!vsapi->mapGetInt(srcprops, "_Field", 0, &err);
        if (err && d->fields && !d->tff_exists) {
            vsapi->setFilterError("Analyse: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
            gopDeinit(&vectorFields);
            vsapi->freeFrame(src);
            return NULL;
        }

        // if tff was passed, it overrides _Field.
        if (d->tff_exists)
            src_top_field = d->tff ^ (n % 2);


        MVArraySizeType vectors_size = gopGetArraySize(&vectorFields);
        uint8_t *vectors = (uint8_t *)malloc(vectors_size);


        if (nref >= 0 && nref < d->vi->numFrames) {
            const VSFrame *ref = vsapi->getFrameFilter(nref, d->node, frameCtx);
            const VSMap *refprops = vsapi->getFramePropertiesRO(ref);

            int ref_top_field = !!vsapi->mapGetInt(refprops, "_Field", 0, &err);
            if (err && d->fields && !d->tff_exists) {
                vsapi->setFilterError("Analyse: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                gopDeinit(&vectorFields);
                vsapi->freeFrame(src);
                vsapi->freeFrame(ref);
                free(vectors);
                return NULL;
            }

            // if tff was passed, it overrides _Field.
            if (d->tff_exists)
                ref_top_field = d->tff ^ (nref % 2);

            int fieldShift = 0;
            if (d->fields && d->analysisData.nPel > 1 && (d->analysisData.nDeltaFrame % 2)) {
                fieldShift = (src_top_field && !ref_top_field) ? d->analysisData.nPel / 2 : ((ref_top_field && !src_top_field) ? -(d->analysisData.nPel / 2) : 0);
                // vertical shift of fields for fieldbased video at finest level pel2
            }


            FramePyramid pSrcGOF(src, "MVUtensils", core, vsapi);
            FramePyramid pRefGOF(ref, "MVUtensils", core, vsapi);

            // FIXME, chroma has a different meaning here? this one controls if chroma is used for ME
            gopSearchMVs(&vectorFields, &pSrcGOF, &pRefGOF, d->searchType, d->nSearchParam, d->nPelSearch, d->nLambda, d->lsad, d->pnew, d->plevel, d->global, vectors, fieldShift, d->useSatd, d->pzero, d->pglobal, d->badSAD, d->badrange, d->meander, d->tryMany, d->searchTypeCoarse, d->chroma);

            if (d->divideExtra) {
                // make extra level with divided sublocks with median (not estimated) motion
                gopExtraDivide(&vectorFields, vectors);
            }

            gopDeinit(&vectorFields);
            vsapi->freeFrame(ref);
        } else { // too close to the beginning or end to do anything
            gopWriteDefaultToArray(&vectorFields, vectors);
            gopDeinit(&vectorFields);
        }

        VSFrame *dst = vsapi->copyFrame(src, core);
        VSMap *dstprops = vsapi->getFramePropertiesRW(dst);

        vsapi->mapSetData(dstprops,
                           prop_MVTools_MVAnalysisData,
                           (const char *)(d->divideExtra ? &d->analysisDataDivided : &d->analysisData),
                           sizeof(MVAnalysisData),
                           dtBinary,
                           maReplace);

        vsapi->mapSetData(dstprops,
                           prop_MVTools_vectors,
                           (const char *)vectors,
                           vectors_size,
                           dtBinary,
                           maReplace);

        free(vectors);

#if defined(MVTOOLS_X86)
        // FIXME: Get rid of all mmx shit.
        mvtools_cpu_emms();
#endif

        vsapi->freeFrame(src);

        return dst;
    }

    return 0;
}


static void VS_CC mvanalyseFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVAnalyseData *d = (MVAnalyseData *)instanceData;

    vsapi->freeNode(d->node);
    free(d);
}


static void VS_CC mvanalyseCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    MVAnalyseData d = {};
    MVAnalyseData *data;

    int err;

    d.analysisData.nBlkSizeX = vsapi->mapGetIntSaturated(in, "blksize", 0, &err);
    if (err)
        d.analysisData.nBlkSizeX = 8;

    d.analysisData.nBlkSizeY = vsapi->mapGetIntSaturated(in, "blksizev", 0, &err);
    if (err)
        d.analysisData.nBlkSizeY = d.analysisData.nBlkSizeX;

    d.levels = vsapi->mapGetIntSaturated(in, "levels", 0, &err);

    d.searchType = (SearchType)(vsapi->mapGetIntSaturated(in, "search", 0, &err));
    if (err)
        d.searchType = SearchHex2;

    d.searchTypeCoarse = (SearchType)(vsapi->mapGetIntSaturated(in, "search_coarse", 0, &err));
    if (err)
        d.searchTypeCoarse = SearchExhaustive;

    d.searchparam = vsapi->mapGetIntSaturated(in, "searchparam", 0, &err);
    if (err)
        d.searchparam = 2;

    d.nPelSearch = vsapi->mapGetIntSaturated(in, "pelsearch", 0, &err);

    d.analysisData.isBackward = !!vsapi->mapGetInt(in, "isb", 0, &err);

    d.chroma = !!vsapi->mapGetInt(in, "chroma", 0, &err);
    if (err)
        d.chroma = true;

    d.analysisData.nDeltaFrame = vsapi->mapGetIntSaturated(in, "delta", 0, &err);
    if (err)
        d.analysisData.nDeltaFrame = 1;

    d.truemotion = !!vsapi->mapGetInt(in, "truemotion", 0, &err);
    if (err)
        d.truemotion = 1;

    d.nLambda = vsapi->mapGetIntSaturated(in, "lambda", 0, &err);
    if (err)
        d.nLambda = d.truemotion ? (1000 * d.analysisData.nBlkSizeX * d.analysisData.nBlkSizeY / 64) : 0;

    d.lsad = vsapi->mapGetIntSaturated(in, "lsad", 0, &err);
    if (err)
        d.lsad = d.truemotion ? 1200 : 400;

    d.plevel = vsapi->mapGetIntSaturated(in, "plevel", 0, &err);
    if (err)
        d.plevel = d.truemotion ? 1 : 0;

    d.global = !!vsapi->mapGetInt(in, "global", 0, &err);
    if (err)
        d.global = d.truemotion ? 1 : 0;

    d.pnew = vsapi->mapGetIntSaturated(in, "pnew", 0, &err);
    if (err)
        d.pnew = d.truemotion ? 50 : 0; // relative to 256

    d.pzero = vsapi->mapGetIntSaturated(in, "pzero", 0, &err);
    if (err)
        d.pzero = d.pnew;

    d.pglobal = vsapi->mapGetIntSaturated(in, "pglobal", 0, &err);

    d.analysisData.nOverlapX = vsapi->mapGetIntSaturated(in, "overlap", 0, &err);

    d.analysisData.nOverlapY = vsapi->mapGetIntSaturated(in, "overlapv", 0, &err);
    if (err)
        d.analysisData.nOverlapY = d.analysisData.nOverlapX;

    d.useSatd = !!vsapi->mapGetInt(in, "satd", 0, &err);

    d.divideExtra = vsapi->mapGetIntSaturated(in, "divide", 0, &err);

    d.badSAD = vsapi->mapGetIntSaturated(in, "badsad", 0, &err);
    if (err)
        d.badSAD = 10000;

    d.badrange = vsapi->mapGetIntSaturated(in, "badrange", 0, &err);
    if (err)
        d.badrange = 24;

    d.opt = !!vsapi->mapGetInt(in, "opt", 0, &err);
    if (err)
        d.opt = 1;

    d.meander = !!vsapi->mapGetInt(in, "meander", 0, &err);
    if (err)
        d.meander = 1;

    d.tryMany = !!vsapi->mapGetInt(in, "trymany", 0, &err);

    d.fields = !!vsapi->mapGetInt(in, "fields", 0, &err);

    d.tff = !!vsapi->mapGetInt(in, "tff", 0, &err);
    d.tff_exists = !err;


    if (d.searchType < 0 || d.searchType > 7) {
        vsapi->mapSetError(out, "Analyse: search must be between 0 and 7 (inclusive).");
        return;
    }

    if (d.searchTypeCoarse < 0 || d.searchTypeCoarse > 7) {
        vsapi->mapSetError(out, "Analyse: search_coarse must be between 0 and 7 (inclusive).");
        return;
    }

    if (d.useSatd && d.analysisData.nBlkSizeX == 16 && d.analysisData.nBlkSizeY == 2) {
        vsapi->mapSetError(out, "Analyse: satd cannot work with 16x2 blocks.");
        return;
    }

    if (d.divideExtra < 0 || d.divideExtra > 2) {
        vsapi->mapSetError(out, "Analyse: divide must be between 0 and 2 (inclusive).");
        return;
    }


    if ((d.analysisData.nBlkSizeX != 4 || d.analysisData.nBlkSizeY != 4) &&
        (d.analysisData.nBlkSizeX != 8 || d.analysisData.nBlkSizeY != 4) &&
        (d.analysisData.nBlkSizeX != 8 || d.analysisData.nBlkSizeY != 8) &&
        (d.analysisData.nBlkSizeX != 16 || d.analysisData.nBlkSizeY != 2) &&
        (d.analysisData.nBlkSizeX != 16 || d.analysisData.nBlkSizeY != 8) &&
        (d.analysisData.nBlkSizeX != 16 || d.analysisData.nBlkSizeY != 16) &&
        (d.analysisData.nBlkSizeX != 32 || d.analysisData.nBlkSizeY != 16) &&
        (d.analysisData.nBlkSizeX != 32 || d.analysisData.nBlkSizeY != 32) &&
        (d.analysisData.nBlkSizeX != 64 || d.analysisData.nBlkSizeY != 32) &&
        (d.analysisData.nBlkSizeX != 64 || d.analysisData.nBlkSizeY != 64) &&
        (d.analysisData.nBlkSizeX != 128 || d.analysisData.nBlkSizeY != 64) &&
        (d.analysisData.nBlkSizeX != 128 || d.analysisData.nBlkSizeY != 128)) {

        vsapi->mapSetError(out, "Analyse: the block size must be 4x4, 8x4, 8x8, 16x2, 16x8, 16x16, 32x16, 32x32, 64x32, 64x64, 128x64, or 128x128.");
        return;
    }


    if (d.plevel < 0 || d.plevel > 2) {
        vsapi->mapSetError(out, "Analyse: plevel must be between 0 and 2 (inclusive).");
        return;
    }


    if (d.pnew < 0 || d.pnew > 256) {
        vsapi->mapSetError(out, "Analyse: pnew must be between 0 and 256 (inclusive).");
        return;
    }


    if (d.pzero < 0 || d.pzero > 256) {
        vsapi->mapSetError(out, "Analyse: pzero must be between 0 and 256 (inclusive).");
        return;
    }


    if (d.pglobal < 0 || d.pglobal > 256) {
        vsapi->mapSetError(out, "Analyse: pglobal must be between 0 and 256 (inclusive).");
        return;
    }


    if (d.analysisData.nOverlapX < 0 || d.analysisData.nOverlapX > d.analysisData.nBlkSizeX / 2 ||
        d.analysisData.nOverlapY < 0 || d.analysisData.nOverlapY > d.analysisData.nBlkSizeY / 2) {
        vsapi->mapSetError(out, "Analyse: overlap must be at most half of blksize, overlapv must be at most half of blksizev, and they both need to be at least 0.");
        return;
    }

    if (d.divideExtra && (d.analysisData.nBlkSizeX < 8 || d.analysisData.nBlkSizeY < 8)) {
        vsapi->mapSetError(out, "Analyse: blksize and blksizev must be at least 8 when divide=True.");
        return;
    }


    if (d.searchType == SearchNstep)
        d.nSearchParam = (d.searchparam < 0) ? 0 : d.searchparam;
    else
        d.nSearchParam = (d.searchparam < 1) ? 1 : d.searchparam;


    d.node = vsapi->mapGetNode(in, "super", 0, 0);
    d.supervi = vsapi->getVideoInfo(d.node);
    d.vi = d.supervi;

    if (!vsh::isConstantVideoFormat(d.vi) || d.vi->format.bitsPerSample > 16 || d.vi->format.sampleType != stInteger || d.vi->format.subSamplingW > 1 || d.vi->format.subSamplingH > 1 || (d.vi->format.colorFamily != cfYUV && d.vi->format.colorFamily != cfGray)) {
        vsapi->mapSetError(out, "Analyse: Input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant format and dimensions.");
        vsapi->freeNode(d.node);
        return;
    }

    if (d.vi->format.colorFamily == cfGray)
        d.chroma = false;

    d.analysisData.bitsPerSample = d.vi->format.bitsPerSample;

    int pixelMax = (1 << d.vi->format.bitsPerSample) - 1;
    d.lsad = (int)((double)d.lsad * pixelMax / 255.0 + 0.5);
    d.badSAD = (int)((double)d.badSAD * pixelMax / 255.0 + 0.5);
    d.nLambda = (int)((double)d.nLambda * pixelMax / 255.0 + 0.5);

    d.lsad = (int64_t)d.lsad * (d.analysisData.nBlkSizeX * d.analysisData.nBlkSizeY) / 64;
    d.badSAD = d.badSAD * (d.analysisData.nBlkSizeX * d.analysisData.nBlkSizeY) / 64;


    d.analysisData.nMotionFlags = 0;
    d.analysisData.nMotionFlags |= d.opt ? MOTION_USE_SIMD : 0;
    d.analysisData.nMotionFlags |= d.analysisData.isBackward ? MOTION_IS_BACKWARD : 0;
    d.analysisData.nMotionFlags |= d.chroma ? MOTION_USE_CHROMA_MOTION : 0;


    if (d.opt) {
        d.analysisData.nCPUFlags = g_cpuinfo;
    }

    if (d.analysisData.nOverlapX % (1 << d.vi->format.subSamplingW) ||
        d.analysisData.nOverlapY % (1 << d.vi->format.subSamplingH)) {
        vsapi->mapSetError(out, "Analyse: The requested overlap is incompatible with the super clip's subsampling.");
        vsapi->freeNode(d.node);
        return;
    }

    if (d.divideExtra && (d.analysisData.nOverlapX % (2 << d.vi->format.subSamplingW) ||
                          d.analysisData.nOverlapY % (2 << d.vi->format.subSamplingH))) { // subsampling times 2
        vsapi->mapSetError(out, "Analyse: overlap and overlapv must be multiples of 2 or 4 when divide=True, depending on the super clip's subsampling.");
        vsapi->freeNode(d.node);
        return;
    }

    if (d.analysisData.nDeltaFrame <= 0 && (-d.analysisData.nDeltaFrame) >= d.vi->numFrames) {
        vsapi->mapSetError(out, "Analyse: delta points to frame past the input clip's end.");
        vsapi->freeNode(d.node);
        return;
    }

    d.analysisData.yRatioUV = 1 << d.vi->format.subSamplingH;
    d.analysisData.xRatioUV = 1 << d.vi->format.subSamplingW;


#define ERROR_SIZE 1024
    char errorMsg[ERROR_SIZE] = "Analyse: failed to retrieve first frame from super clip. Error message: ";
    size_t errorLen = strlen(errorMsg);
    const VSFrame *evil = vsapi->getFrame(0, d.node, errorMsg + errorLen, ERROR_SIZE - (int)errorLen);
#undef ERROR_SIZE
    if (!evil) {
        vsapi->mapSetError(out, errorMsg);
        vsapi->freeNode(d.node);
        return;
    }

    FramePyramid FPEvil(evil, "MVUtensils", core, vsapi);



    // fill in missing fields
    d.analysisData.nWidth = FPEvil.nWidth[0];

    d.analysisData.nHeight = FPEvil.nHeight[0];

    d.analysisData.nPel = FPEvil.nPel;

    d.analysisData.nHPadding = FPEvil.nHPad[0];
    d.analysisData.nVPadding = FPEvil.nVPad[0];


    int nBlkX = (d.analysisData.nWidth - d.analysisData.nOverlapX) / (d.analysisData.nBlkSizeX - d.analysisData.nOverlapX); //x
    int nBlkY = (d.analysisData.nHeight - d.analysisData.nOverlapY) / (d.analysisData.nBlkSizeY - d.analysisData.nOverlapY);

    d.analysisData.nBlkX = nBlkX;
    d.analysisData.nBlkY = nBlkY;

    int nWidth_B = (d.analysisData.nBlkSizeX - d.analysisData.nOverlapX) * nBlkX + d.analysisData.nOverlapX; // covered by blocks
    int nHeight_B = (d.analysisData.nBlkSizeY - d.analysisData.nOverlapY) * nBlkY + d.analysisData.nOverlapY;

    // calculate valid levels
    int nLevelsMax = 0;
    while (((nWidth_B >> nLevelsMax) - d.analysisData.nOverlapX) / (d.analysisData.nBlkSizeX - d.analysisData.nOverlapX) > 0 &&
           ((nHeight_B >> nLevelsMax) - d.analysisData.nOverlapY) / (d.analysisData.nBlkSizeY - d.analysisData.nOverlapY) > 0) // at last one block
    {
        nLevelsMax++;
    }

    d.analysisData.nLvCount = d.levels > 0 ? d.levels : nLevelsMax + d.levels;

    if (d.analysisData.nLvCount < 1 || d.analysisData.nLvCount > nLevelsMax) {
        vsapi->mapSetError(out, "Analyse: invalid number of levels.");
        vsapi->freeNode(d.node);
        return;
    }

    if (d.analysisData.nLvCount > FPEvil.pyramidLevels.size()) { //x
#define ERROR_SIZE 512
        char error_msg[ERROR_SIZE + 1] = { 0 };
        snprintf(error_msg, ERROR_SIZE, "Analyse: super clip has %d levels. Analyse needs %d levels.", (int)FPEvil.pyramidLevels.size(), d.analysisData.nLvCount);
#undef ERROR_SIZE
        vsapi->mapSetError(out, error_msg);
        vsapi->freeNode(d.node);
        return;
    }


    if (d.nPelSearch <= 0)
        d.nPelSearch = d.analysisData.nPel; // not below value of 0 at finest level //x


    if (d.divideExtra) { //v1.8.1
        memcpy(&d.analysisDataDivided, &d.analysisData, sizeof(d.analysisData));
        d.analysisDataDivided.nBlkX = d.analysisData.nBlkX * 2;
        d.analysisDataDivided.nBlkY = d.analysisData.nBlkY * 2;
        d.analysisDataDivided.nBlkSizeX = d.analysisData.nBlkSizeX / 2;
        d.analysisDataDivided.nBlkSizeY = d.analysisData.nBlkSizeY / 2;
        d.analysisDataDivided.nOverlapX = d.analysisData.nOverlapX / 2;
        d.analysisDataDivided.nOverlapY = d.analysisData.nOverlapY / 2;
        d.analysisDataDivided.nLvCount = d.analysisData.nLvCount + 1;
    }


    data = (MVAnalyseData *)malloc(sizeof(d));
    *data = d;

    VSFilterDependency deps[1] = { 
        {data->node, rpGeneral} //super
    };

    vsapi->createVideoFilter(out, "Analyse", data->vi, mvanalyseGetFrame, mvanalyseFree, fmParallel, deps, ARRAY_SIZE(deps), data, core);
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
                 "opt:int:opt;"
                 "meander:int:opt;"
                 "trymany:int:opt;"
                 "fields:int:opt;"
                 "tff:int:opt;"
                 "search_coarse:int:opt;"
                 "satd:int:opt;",
                 "clip:vnode;",
                 mvanalyseCreate, 0, plugin);
}
