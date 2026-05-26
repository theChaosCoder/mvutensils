#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>
#include <string>
#include <cassert>
#include <VapourSynth4.h>
#include "SuperPyramid.h"
#include "CopyCode.h"
#include "SADFunctions.h"

struct VECTOR {
    int x;
    int y;
    int64_t sad;
};

struct BlockData {
    int x;
    int y;
    VECTOR vector;
};

enum class SearchType {
    Logarithmic = 2,
    Exhaustive = 3,
    Hex2 = 4,
    UnevenMultiHexagon = 5,
    Horizontal = 6,
    Vertical = 7
};


static constexpr const int MV_DEFAULT_SCD1 = 400;
static constexpr const int MV_DEFAULT_SCD2 = 130;

static constexpr const VECTOR zeroMV = { 0, 0, -1 };


// A motion block level is the motion estimation context euivalent
// to a level of the super pyramid, it contains all the data and parameters needed to perform motion estimation on this level

class MotionBlockLevel {
    friend class MotionBlockPyramid;
private:
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

    // static parameters during the whole search process
    SearchType searchType; /* search type used */
    int nSearchParam;      /* additionnal parameter for this search */
    int64_t LSAD;          // SAD limit for lambda using - Fizick
    int penaltyNew;        // cost penalty factor for new candidates
    int penaltyZero;       // cost penalty factor for zero vector
    int pglobal;           // cost penalty factor for global predictor
    int64_t verybigSAD;

    SADFunction SAD = nullptr;   /* function which computes the sad */
    SADFunction SADCHROMA = nullptr;
    COPYFunction BLITLUMA = nullptr;
    COPYFunction BLITCHROMA = nullptr;

    std::vector<VECTOR> vectors; /* motion vectors of the blocks */
    /* before the search, contains the hierachal predictor */
    /* after the search, contains the best motion vector */

    bool smallestPlane; /* say whether vectors can used predictors from a smaller plane */
    bool chroma;        /* do we do chroma me */

    /* working fields */
    // the frame to compare to, may be possible to pass prefframe instead of whole pob to some functions
    // constant the whole time
    const FramePyramidLevel *pRefFrame = nullptr;
    ptrdiff_t nRefPitch[3] = {};

    // These are all used deep in motion estimation
    VECTOR bestMV;    /* best vector found so far during the search */
    int64_t nMinCost; /* minimum cost ( sad + mv cost ) found so far */
    VECTOR predictor; /* best predictor for the current vector */

    // Maximum vector bounds possible
    int nDxMin; /* minimum x coordinate for the vector */
    int nDyMin; /* minimum y coordinate for the vector */
    int nDxMax; /* maximum x corrdinate for the vector */
    int nDyMax; /* maximum y coordinate for the vector */
    int x[3];       /* absolute x coordinate of the origin of the block in the reference frame */
    int y[3];       /* absolute y coordinate of the origin of the block in the reference frame */
    int64_t nLambda;       /* vector cost factor */

    // unknown
    VECTOR globalMVPredictor;  // predictor of global motion vector

    // always constant? constant for the whole frame?
    VECTOR zeroMVfieldShifted; // zero motion vector for fieldbased video at finest level pel2

    // Set for each block
    int nSrcPitch_temp[3];
    uint8_t *pSrc_temp[3] = {}; //for easy WRITE access to temp block
private:
    template <int nLogPel, typename PixelType>
    const uint8_t *GetRefBlock(int nVx, int nVy) const;

    template <int nLogPel, typename PixelType>
    const uint8_t *GetRefBlockU(int nVx, int nVy) const;

    template <int nLogPel, typename PixelType>
    const uint8_t *GetRefBlockV(int nVx, int nVy) const;

    template <int nLogPel, int flags, typename PixelType>
    void CheckMV_Template(int vx, int vy, int *dir, int val);

    template <int nLogPel, typename PixelType>
    void CheckMV0(int vx, int vy);

    template <int nLogPel, typename PixelType>
    void CheckMV(int vx, int vy);

    template <int nLogPel, typename PixelType>
    void CheckMV2(int vx, int vy, int *dir, int val);

    template <int nLogPel, typename PixelType>
    void CheckMVdir(int vx, int vy, int *dir, int val);

    template <int nLogPel, typename PixelType>
    void DiamondSearch(int length);

    template <int nLogPel, typename PixelType>
    void ExpandingSearch(int r, int s, int mvx, int mvy);

    template <int nLogPel, typename PixelType>
    void Hex2Search(int i_me_range);

    template <int nLogPel, typename PixelType>
    void CrossSearch(int start, int x_max, int y_max, int mvx, int mvy);

    template <int nLogPel, typename PixelType>
    void UMHSearch(int i_me_range, int omx, int omy);

    template <int nLogPel, typename PixelType>
    void Refine();

    template <int nLogPel, typename PixelType>
    void PseudoEPZSearch(int blkIdx, int blkx, int blky, int blkScanDir, int64_t badSAD, int badrange, bool tryMany, int &badcount);

    template <int nLogPel, typename PixelType>
    void doRecalculateMVs(const FramePyramidLevel &pSrcFrame, const FramePyramidLevel &pRefFrame,
        SearchType st, int stp, int lambda, int pnew,
        int fieldShift, int64_t thSAD, int smooth, bool meander);

    template <int nLogPel, typename PixelType>
    void DoSearchMVs(const FramePyramidLevel &pSrcFrame, const FramePyramidLevel &pRefFrame,
        SearchType st, int stp, int lambda, int lsad, int pnew,
        int plevel, VECTOR *globalMVec, int fieldShift,
        int pzero, int pglobal, int64_t badSAD, int badrange, bool meander, bool tryMany, bool chroma);

    bool IsVectorOK(int vx, int vy) const;
    int MotionDistorsion(int vx, int vy) const;
    VECTOR ClipMV(VECTOR v) const;
    void FetchPredictors(int blkidx, int blkx, int blky, int blkScanDir, VECTOR predictors[5]);
    void InitMotionEstimationFields(bool useSatd, bool chroma);
public:
    void SearchMVs(const FramePyramidLevel &pSrcFrame, const FramePyramidLevel &pRefFrame,
        SearchType st, int stp, int lambda, int lsad, int pnew,
        int plevel, VECTOR *globalMVec, int fieldShift, bool useSatd,
        int pzero, int pglobal, int64_t badSAD, int badrange, bool meander, bool tryMany, bool chroma);

    void RecalculateMVs(const FramePyramidLevel &pSrcFrame, const FramePyramidLevel &pRefFrame,
        SearchType st, int stp, int lambda, int pnew,
        int fieldShift, int64_t thSAD, bool useSatd, int smooth, bool meander);

    void EstimateGlobalMVDoubled(VECTOR &globalMVec);
    void InterpolatePredictorsFromParent(const MotionBlockLevel &parentLevel);

    bool IsSceneChange(int64_t nTh1, int nTh2) const;

    void Initialize(int _nBlkX, int _nBlkY, int _nBlkSizeX, int _nBlkSizeY, int _nPel, int _nLevel, bool smallestPlane, bool chroma, int _nOverlapX, int _nOverlapY, int _xRatioUV, int _yRatioUV, int bitsPerSample);

    MotionBlockLevel() = default;
    ~MotionBlockLevel();
};

class MotionBlockPyramid {
public:
    bool valid = false;

    // exported
    int nBlkSizeX;
    int nBlkSizeY;
    int nLevelCount;
    int nOverlapX;
    int nOverlapY;
    int xRatioUV;
    int yRatioUV;
    int nPel;
    int nDeltaFrame;
    bool chroma;
    int nRealWidth;
    int nRealHeight;
    int nWidth;
    int nHeight;
    int nBlkX;
    int nBlkY;
    int nHPadding;
    int nVPadding;
    
private:
    // not exported

    int divideExtra = 0;

    std::vector<VECTOR> dividedVectors;

    std::vector<MotionBlockLevel> pyramidLevels;
public:
    void DivideVectorsExtra(int divideExtra);
    // construct from 
    MotionBlockPyramid(const FramePyramid &src, int nBlkSizeX, int nBlkSizeY, int nOverlapX, int nOverlapY, int nLevels, bool chroma, int nDeltaFrame, int bitsPerSample);
    MotionBlockPyramid(const VSFrame *src, int maxLevel, const std::string &prefix, VSCore *core, const VSAPI *vsapi); // de-serialization from a frame, can choose to omit some levels by setting maxLevel, if -1 all levels are loaded, only need to pass 0 for recalculate and using motion vectors
    void ExportFrameData(VSFrame *dst, const std::string &prefix, VSCore *core, const VSAPI *vsapi); // serialization to a frame

    void SearchMVs(const FramePyramid &pSrcGOF, const FramePyramid &pRefGOF,
        SearchType searchType, int nSearchParam, int nPelSearch, int nLambda,
        int lsad, int pnew, int plevel, bool global, int fieldShift, bool useSatd,
        int pzero, int pglobal, int64_t badSAD, int badrange, int meander, int tryMany,
        SearchType coarseSearchType, bool chroma);

    void RecalculateMVs(const FramePyramid &pSrcGOF, const FramePyramid &pRefGOF,
        SearchType searchType, int nSearchParam, int nLambda, int pnew,
        int fieldShift, int64_t thSAD, bool useSatd, int smooth, int meander);

    bool IsUsable(int64_t thscd1, int thscd2) const;
    BlockData GetBlock(int nBlk) const;
    void ScaleThSCD(int64_t &thscd1, int &thscd2, int bitsPerSample) const;
};

