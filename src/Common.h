#pragma once

#include <vector>
#include <stdexcept>
#ifdef _WIN32
#include <malloc.h>
#else 
#include <cstdlib>
#endif
#include <VapourSynth4.h>

#define MVU_RESTRICT __restrict

class MVUtensilsError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

template<typename T, size_t U>
constexpr int ARRAY_SIZE(const T (&arr)[U]) {
    return static_cast<int>(U);
}

template<typename T>
static void VS_CC filterFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    delete reinterpret_cast<T *>(instanceData);
}

constexpr int roundUpTo64(int value) {
    return ((value + 63) / 64) * 64;
}

constexpr int ERROR_SIZE = 1024;

static constexpr const int MV_DEFAULT_SCD1 = 400;
static constexpr const int MV_DEFAULT_SCD2 = 130;

constexpr char DEFAULT_MVUTENSILS_PREFIX[] = "MVUtensils";

static inline void mvu_bitblt(void *dstp, ptrdiff_t dst_stride, const void *srcp, ptrdiff_t src_stride, size_t row_size, size_t height) {
    if (height) {
        if (src_stride == dst_stride && src_stride == (ptrdiff_t)row_size) {
            memcpy(dstp, srcp, row_size * height);
        } else {
            const uint8_t *srcp8 = (const uint8_t *)srcp;
            uint8_t *dstp8 = (uint8_t *)dstp;
            size_t i;
            for (i = 0; i < height; i++) {
                memcpy(dstp8, srcp8, row_size);
                srcp8 += src_stride;
                dstp8 += dst_stride;
            }
        }
    }
}

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


// FIXME, drop this for naming clarity?
template<typename T>
struct DualNodeData : public T {
private:
    const VSAPI *vsapi;
public:
    VSNode *node1 = nullptr;
    VSNode *node2 = nullptr;

    explicit DualNodeData(const VSAPI *vsapi) noexcept : T(), vsapi(vsapi) {
    }

    ~DualNodeData() {
        vsapi->freeNode(node1);
        vsapi->freeNode(node2);
    }
};

template<typename T>
struct VariableNodeData : public T {
private:
    const VSAPI *vsapi;
public:
    std::vector<VSNode *> nodes;

    explicit VariableNodeData(const VSAPI *vsapi) noexcept : T(), vsapi(vsapi) {
    }

    ~VariableNodeData() {
        for (auto iter : nodes)
            vsapi->freeNode(iter);
    }
};

/* returns the biggest integer x such as 2^x <= i */
inline static int ilog2(int i) {
    int result = 0;
    while (i > 1) {
        i /= 2;
        result++;
    }
    return result;
}


template<typename T>
static inline T *mvu_aligned_malloc(size_t size, size_t alignment) {
#ifdef _WIN32
    return (T *)_aligned_malloc(size, alignment);
#else
    void *tmp = NULL;
    if (posix_memalign(&tmp, alignment, size))
        tmp = 0;
    return (T *)tmp;
#endif
}

static inline void mvu_aligned_free(void *ptr) {
#ifdef _WIN32
    _aligned_free(ptr);
#else 
    free(ptr);
#endif
}