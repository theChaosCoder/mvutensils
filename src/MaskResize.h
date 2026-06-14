#pragma once

#include <cstdint>
#include <vector>
#include <memory>
#include <VSHelper4.h>
#include "Common.h"

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

    static const zimg_image_buffer_const MakeSrcBuffer(const void *data, ptrdiff_t stride = 0) {
        zimg_image_buffer_const buf;
        buf.plane[0].data = const_cast<void *>(data);
        buf.plane[0].stride = stride ? stride : GetTileBufferStride();
        buf.plane[0].mask = ZIMG_BUFFER_MAX;
        return buf;
    }

    static const zimg_image_buffer MakeDstBuffer(void *data, ptrdiff_t stride = 0) {
        zimg_image_buffer buf;
        buf.plane[0].data = data;
        buf.plane[0].stride = stride ? stride : GetTileBufferStride();
        buf.plane[0].mask = ZIMG_BUFFER_MAX;
        return buf;
    }

    static constexpr ptrdiff_t GetTileBufferStride() {
        return roundUpTo64(TileSize * sizeof(uint16_t));
    }

    static std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> GetTileBuffer() {
        return std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)>{ vsh::vsh_aligned_malloc<uint16_t>(GetTileBufferStride() * TileSize, 64), vsh::vsh_aligned_free };
    }

    static std::unique_ptr<void, decltype(&vsh::vsh_aligned_free)> GetTmpBuffer(size_t size) {
        return std::unique_ptr<void, decltype(&vsh::vsh_aligned_free)>{ vsh::vsh_aligned_malloc<void>(size, 64), vsh::vsh_aligned_free };
    }
};