#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>
#include <string>
#include <cassert>
#include <VapourSynth4.h>
#include "MVAnalysisData.h"
#include "SuperPyramid.h"
#include "CopyCode.h"
#include "SADFunctions.h"

// A motion block level is the motion estimation context euivalent
// to a level of the super pyramid, it contains all the data and parameters needed to perform motion estimation on this level

struct MotionBlockLevel {
private:
    static constexpr size_t MAX_PREDICTOR = 5;
public:
    int nBlkX;        /* width in number of blocks */
    int nBlkY;        /* height in number of blocks */
    int nBlkSizeX;    /* size of a block */
    int nBlkSizeY;    /* size of a block */
    int nBlkCount;    /* number of blocks in the plane */
    int nPel;         /* pel refinement accuracy */
    int nLogPel;      /* logarithm of the pel refinement accuracy */
    int nScale;       /* scaling factor of the plane */
    int nLogScale;    /* logarithm of the scaling factor */
    int nOverlapX; // overlap size
    int nOverlapY; // overlap size
    int xRatioUV;
    int yRatioUV;
    int nLogxRatioUV; // log of xRatioUV (0 for 1 and 1 for 2)
    int nLogyRatioUV; // log of yRatioUV (0 for 1 and 1 for 2)
    int bytesPerSample;

    SADFunction SAD;   /* function which computes the sad */
    COPYFunction BLITLUMA;
    COPYFunction BLITCHROMA;
    SADFunction SADCHROMA;
    SADFunction SATD; /* SATD function, (similar to SAD), used as replacement to dct */

    VECTOR *vectors; /* motion vectors of the blocks */
    /* before the search, contains the hierachal predictor */
    /* after the search, contains the best motion vector */

    bool smallestPlane; /* say whether vectors can used predictors from a smaller plane */
    bool chroma;        /* do we do chroma me */

    /* working fields */

    const FramePyramidLevel *pSrcFrame;
    const FramePyramidLevel *pRefFrame;

    ptrdiff_t nSrcPitch[3];
    const uint8_t *pSrc[3]; // the alignment of this array is important for speed for some reason (cacheline?)
    ptrdiff_t nRefPitch[3];

    VECTOR bestMV;    /* best vector found so far during the search */
    int64_t nMinCost; /* minimum cost ( sad + mv cost ) found so far */
    VECTOR predictor; /* best predictor for the current vector */

    VECTOR predictors[MAX_PREDICTOR]; /* set of predictors for the current block */

    int nDxMin; /* minimum x coordinate for the vector */
    int nDyMin; /* minimum y coordinate for the vector */
    int nDxMax; /* maximum x corrdinate for the vector */
    int nDyMax; /* maximum y coordinate for the vector */

    int x[3];       /* absolute x coordinate of the origin of the block in the reference frame */
    int y[3];       /* absolute y coordinate of the origin of the block in the reference frame */
    int blkx;       /* x coordinate in blocks */
    int blky;       /* y coordinate in blocks */
    int blkIdx;     /* index of the block */
    int blkScanDir; // direction of scan (1 is left to rught, -1 is right to left)

    /* search parameters */

    SearchType searchType; /* search type used */
    int nSearchParam;      /* additionnal parameter for this search */
    int64_t nLambda;       /* vector cost factor */
    int64_t LSAD;          // SAD limit for lambda using - Fizick
    int penaltyNew;        // cost penalty factor for new candidates
    int penaltyZero;       // cost penalty factor for zero vector
    int pglobal;           // cost penalty factor for global predictor
    //   int nLambdaLen; //  penalty factor (lambda) for vector length
    int64_t badSAD;   // SAD threshold for more wide search
    int badrange;     // wide search radius
    int badcount;     // number of bad blocks refined
    bool tryMany;     // try refine around many predictors

    VECTOR globalMVPredictor;  // predictor of global motion vector
    VECTOR zeroMVfieldShifted; // zero motion vector for fieldbased video at finest level pel2

    bool useSatd;
    int *freqArray; // temporary array for global motion estimaton
    int freqSize;   // size of freqArray
    int64_t verybigSAD;

    int nSrcPitch_temp[3];
    uint8_t *pSrc_temp[3]; //for easy WRITE access to temp block

    std::vector<VECTOR> out;
};

class MotionBlockPyramid {
public:
    int nBlkSizeX;
    int nBlkSizeY;
    int nLevelCount;
    int nOverlapX;
    int nOverlapY;
    int xRatioUV;
    int yRatioUV;
    int divideExtra;
    std::vector<MotionBlockLevel> pyramidLevels;
public: //? ?????

    // construct from 
    MotionBlockPyramid(const FramePyramid &src, const FramePyramid &ref);


};