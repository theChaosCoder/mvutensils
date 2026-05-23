#ifndef MVTOOLS_FAKERY_H
#define MVTOOLS_FAKERY_H


#include "MVAnalysisData.h"


typedef struct FakeBlockData {
    int x;
    int y;
    VECTOR vector;
} FakeBlockData;


typedef struct FakePlaneOfBlocks {
    int nBlkX;
    int nBlkY;
    int nBlkSizeX;
    int nBlkSizeY;
    int nBlkCount;
    int nPel;
    int nOverlapX;
    int nOverlapY;

    FakeBlockData *blocks;
} FakePlaneOfBlocks;


typedef struct FakeGroupOfPlanes {
    int nLvCount;
    int validity;

    FakePlaneOfBlocks **planes;
} FakeGroupOfPlanes;


// FakePlaneOfBlocks

void fpobInit(FakePlaneOfBlocks *fpob, int sizeX, int sizeY, int pel, int nOverlapX, int nOverlapY, int nBlkX, int nBlkY);

void fpobDeinit(FakePlaneOfBlocks *fpob);

// FakeGroupOfPlanes

void fgopInit(FakeGroupOfPlanes *fgop, const MVAnalysisData *ad);

void fgopDeinit(FakeGroupOfPlanes *fgop);

void fgopUpdate(FakeGroupOfPlanes *fgop, const uint8_t *array);

int fgopIsValid(const FakeGroupOfPlanes *fgop);

const FakePlaneOfBlocks *fgopGetPlane0(const FakeGroupOfPlanes *fgop);

const FakeBlockData *fgopGetBlock(const FakeGroupOfPlanes *fgop, int nLevel, int nBlk);

int fgopIsUsable(const FakeGroupOfPlanes *fgop, int64_t thscd1, int thscd2);


#endif // MVTOOLS_FAKERY_H
