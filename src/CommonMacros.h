#pragma once

#include <vector>
#include <stdexcept>


class MVUtensilsError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

template<typename T, size_t U>
constexpr int ARRAY_SIZE(const T (&arr)[U]) {
    return static_cast<int>(U);
}

/*
* FIXME, maybe introduce soemthing like this
template<typename T>
static void VS_CC filterFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    delete reinterpret_cast<T *>(instanceData);
}
*/

// FIXME, probably should be called something else

#define RETERROR(x) do { vsapi->mapSetError(out, (x)); return; } while (0)

constexpr int ERROR_SIZE = 1024;

static constexpr const int MV_DEFAULT_SCD1 = 400;
static constexpr const int MV_DEFAULT_SCD2 = 130;

constexpr char DEFAULT_MVUTENSILS_PREFIX[] = "MVUtensils";

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
