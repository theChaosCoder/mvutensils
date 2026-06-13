#pragma once

#include <cstdint>
#include <vector>
#define ZIMGXX_NAMESPACE mvuzimgxx
#include <zimg++.hpp>

void BilinearUpsizeBlockMask(uint8_t *dst, ptrdiff_t dststride, int dstwidth, int dstheight, const void *src, ptrdiff_t srcstride, int nBlkX, int nBlkY, int nBlkSizeX, int nBlkSizeY, int nOverlapX, int nOverlapY, int bitsPerSample);

class MaskTile {
public:
    int dstX;
    int dstY;
    int dstWidth;
    int dstHeight;
    mvuzimgxx::FilterGraph graph;
};

class MaskResizer {
public:
    class MaskTile {
    public:
        int dstX;
        int dstY;
        int dstWidth;
        int dstHeight;
        mvuzimgxx::FilterGraph graph;
    };

    constexpr static int TileSize = 64;
    std::vector<MaskTile> tiles;
    size_t tmpSize = 0;

    void Init(int nBlkX, int nBlkY, int nBlkSizeX, int nBlkSizeY, int nOverlapX, int nOverlapY, int dstWidth, int dstHeight);
};