// See legal notice in Copying.txt for more information

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

#ifndef PLANEOFBLOCKS_H
#define PLANEOFBLOCKS_H

#include <cstdlib>

#include "Fakery.h"
#include "CopyCode.h"
#include "SADFunctions.h"
#include "CommonFunctions.h"
#include "SuperPyramid.h"


#define MAX_PREDICTOR 5 // right now 5 should be enough (TSchniede)


typedef struct PlaneOfBlocks {

    /* fields set at initialization */

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
    // the frame to compare to, may be possible to pass prefframe instead of whole pob to some functions
    const FramePyramidLevel *pRefFrame;
    ptrdiff_t nRefPitch[3];

    VECTOR bestMV;    /* best vector found so far during the search */
    int64_t nMinCost; /* minimum cost ( sad + mv cost ) found so far */
    VECTOR predictor; /* best predictor for the current vector */

    VECTOR predictors[MAX_PREDICTOR]; /* set of predictors for the current block */

    // Maximum vector bounds possible, used to reject vectors, recalculated each block
    int nDxMin; /* minimum x coordinate for the vector */
    int nDyMin; /* minimum y coordinate for the vector */
    int nDxMax; /* maximum x corrdinate for the vector */
    int nDyMax; /* maximum y coordinate for the vector */

    // loop variables used deep in motion estimation
    int x[3];       /* absolute x coordinate of the origin of the block in the reference frame */
    int y[3];       /* absolute y coordinate of the origin of the block in the reference frame */

    /* search parameters */

    // static parameters during the whole search process
    SearchType searchType; /* search type used */
    int nSearchParam;      /* additionnal parameter for this search */
    int64_t LSAD;          // SAD limit for lambda using - Fizick
    int penaltyNew;        // cost penalty factor for new candidates
    int penaltyZero;       // cost penalty factor for zero vector
    int pglobal;           // cost penalty factor for global predictor
    int64_t badSAD;   // SAD threshold for more wide search
    int badrange;     // wide search radius
    int64_t verybigSAD;

    // passed deep into motion estimation code, hard to detangle
    int64_t nLambda;       /* vector cost factor */

    // unknown
    VECTOR globalMVPredictor;  // predictor of global motion vector
    VECTOR zeroMVfieldShifted; // zero motion vector for fieldbased video at finest level pel2


    // only used as temporary space in pobEstimateGlobalMVDoubled, should be a temp space pointer provided by the pyramid object instead
    std::vector<int> freqArray;

    // Set for each block
    int nSrcPitch_temp[3];
    uint8_t *pSrc_temp[3]; //for easy WRITE access to temp block
} PlaneOfBlocks;


void pobInit(PlaneOfBlocks *pob, int _nBlkX, int _nBlkY, int _nBlkSizeX, int _nBlkSizeY, int _nPel, int _nLevel, bool smallestPlane, bool chroma, int _nOverlapX, int _nOverlapY, int _xRatioUV, int _yRatioUV, int bitsPerSample);

void pobDeinit(PlaneOfBlocks *pob);

void pobEstimateGlobalMVDoubled(PlaneOfBlocks *pob, VECTOR *globalMVec);

MVArraySizeType pobGetArraySize(const PlaneOfBlocks *pob, int divideMode);

void pobInterpolatePrediction(PlaneOfBlocks *pob, const PlaneOfBlocks *pob2);

void pobRecalculateMVs(PlaneOfBlocks *pob, const FakeGroupOfPlanes *fgop, const FramePyramidLevel *pSrcFrame, const FramePyramidLevel *pRefFrame, SearchType st, int stp, int lambda, int pnew, uint8_t *out, int fieldShift, int64_t thSAD, bool useSatd, int smooth, bool meander);

void pobSearchMVs(PlaneOfBlocks *pob, const FramePyramidLevel *pSrcFrame, const FramePyramidLevel *pRefFrame, SearchType st, int stp, int lambda, int lsad, int pnew, int plevel, uint8_t *out, VECTOR *globalMVec, int fieldShift, bool useSatd, int pzero, int pglobal, int64_t badSAD, int badrange, bool meander, bool tryMany, bool chroma);

MVArraySizeType pobWriteDefaultToArray(const PlaneOfBlocks *pob, uint8_t *array, int divideMode);


#endif
