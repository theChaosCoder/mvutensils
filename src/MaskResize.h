#pragma once

#include <cstdint>
#include <vector>
#include <memory>
#include <tuple>
#include <utility>
#include "Common.h"

#define ZIMGXX_NAMESPACE mvuzimgxx
#include <zimg++.hpp>

void BilinearUpsizeBlockMask(uint8_t *dst, ptrdiff_t dststride, int dstwidth, int dstheight, const void *src, ptrdiff_t srcstride, int nBlkX, int nBlkY, int nBlkSizeX, int nBlkSizeY, int nOverlapX, int nOverlapY, int bitsPerSample);

class MaskResizer {
public:
    struct BufferPair {
        mvuzimgxx::zimage_buffer_const src;
        mvuzimgxx::zimage_buffer dst;
    };

    class MaskTile {
    public:
        int dstX;
        int dstY;
        int dstWidth;
        int dstHeight;
        mvuzimgxx::FilterGraph graph;

        template <typename... Pairs>
        void Process(void *tmp, const Pairs&... pairs) const {
            (graph.process(pairs.src, pairs.dst, tmp), ...);
        }
    };

    constexpr static int TileSize = 64;
    std::vector<MaskTile> tiles;
    size_t tmpSize = 0;

    void Init(int nBlkX, int nBlkY, int nBlkSizeX, int nBlkSizeY, int nOverlapX, int nOverlapY, int dstWidth, int dstHeight);


    static BufferPair MakeBufferPair(const void *src, ptrdiff_t srcStride, void *dst, ptrdiff_t dstStride = 0) {
        return { MakeSrcBuffer(src, srcStride), MakeDstBuffer(dst, dstStride) };
    }

    static constexpr ptrdiff_t GetTileBufferStride() {
        return roundUpTo64(TileSize * sizeof(uint16_t));
    }

    template <size_t N>
    static auto GetTileBuffers() {
        return GetTileBuffersImpl(std::make_index_sequence<N>{});
    }

    static std::unique_ptr<void, decltype(&mvu_aligned_free)> GetTmpBuffer(size_t size) {
        return std::unique_ptr<void, decltype(&mvu_aligned_free)>{ mvu_aligned_malloc<void>(size, 64), mvu_aligned_free };
    }

private:
    static std::unique_ptr<uint16_t, decltype(&mvu_aligned_free)> GetTileBuffer() {
        return std::unique_ptr<uint16_t, decltype(&mvu_aligned_free)>{ mvu_aligned_malloc<uint16_t>(GetTileBufferStride() *TileSize, 64), mvu_aligned_free };
    }

    template <size_t... Is>
    static auto GetTileBuffersImpl(std::index_sequence<Is...>) {
        return std::make_tuple(((void)Is, GetTileBuffer())...);
    }

    static const mvuzimgxx::zimage_buffer_const MakeSrcBuffer(const void *data, ptrdiff_t stride = 0) {
        mvuzimgxx::zimage_buffer_const buf;
        buf.plane[0].data = const_cast<void *>(data);
        buf.plane[0].stride = stride ? stride : GetTileBufferStride();
        buf.plane[0].mask = ZIMG_BUFFER_MAX;
        return buf;
    }

    static const mvuzimgxx::zimage_buffer MakeDstBuffer(void *data, ptrdiff_t stride = 0) {
        mvuzimgxx::zimage_buffer buf;
        buf.plane[0].data = data;
        buf.plane[0].stride = stride ? stride : GetTileBufferStride();
        buf.plane[0].mask = ZIMG_BUFFER_MAX;
        return buf;
    }
};