// CPU feature detection. Only AVX2 and the AVX-512 feature levels are reported (see CPU.h).
// Uses the CPUID / XGETBV compiler intrinsics -- no external assembly.

#include <cstdint>

#include "CPU.h"


#if defined(MVTOOLS_X86)

#if defined(_MSC_VER)
  #include <intrin.h>
#elif defined(__GNUC__)
  #include <cpuid.h>
#endif

namespace {

// do_cpuid / do_xgetbv borrowed from zimg (src/zimg/common/x86/cpuinfo_x86.cpp).

// Execute CPUID with the given leaf (eax) and subleaf (ecx); regs receives eax,ebx,ecx,edx.
void do_cpuid(int regs[4], int eax, int ecx) {
#if defined(_MSC_VER)
    __cpuidex(regs, eax, ecx);
#elif defined(__GNUC__)
    __cpuid_count(eax, ecx, regs[0], regs[1], regs[2], regs[3]);
#else
    regs[0] = regs[1] = regs[2] = regs[3] = 0;
#endif
}

// Execute XGETBV; returns (edx << 32) | eax.
unsigned long long do_xgetbv(unsigned ecx) {
#if defined(_MSC_VER)
    return _xgetbv(ecx);
#elif defined(__GNUC__)
    unsigned eax, edx;
    __asm__("xgetbv" : "=a"(eax), "=d"(edx) : "c"(ecx));
    return (static_cast<unsigned long long>(edx) << 32) | eax;
#else
    return 0;
#endif
}

} // namespace


uint32_t cpu_detect() {
    uint32_t cpu = 0;
    int regs[4] = { 0 };

    do_cpuid(regs, 0, 0);
    if (regs[0] < 7) // leaf 7 carries the AVX2 / AVX-512 feature bits
        return 0;

    // OS support for the vector state must be checked via XGETBV before trusting the feature bits.
    do_cpuid(regs, 1, 0);
    bool osxsave = (regs[2] & (1u << 27)) != 0;
    unsigned long long xcr0 = osxsave ? do_xgetbv(0) : 0;
    bool ymm = (xcr0 & 0x06) == 0x06;          // XMM + YMM state
    bool zmm = (xcr0 & 0xE0) == 0xE0;          // opmask + ZMM_Hi256 + Hi16_ZMM state

    do_cpuid(regs, 7, 0);
    unsigned ebx = (unsigned)regs[1], ecx = (unsigned)regs[2], edx = (unsigned)regs[3];

    if (ymm && (ebx & (1u << 5)))
        cpu |= MVU_CPU_AVX2;

    if (zmm) {
        // x86-64-v4 baseline = AVX-512 F + DQ + CD + BW + VL -> one flag.
        const unsigned avx512_base = (1u << 16) | (1u << 17) | (1u << 28) | (1u << 30) | (1u << 31);
        if ((ebx & avx512_base) == avx512_base) cpu |= MVU_CPU_AVX512_BASE;

        if (ebx & (1u << 21)) cpu |= MVU_CPU_AVX512IFMA;
        if (ecx & (1u << 1))  cpu |= MVU_CPU_AVX512VBMI;
        if (ecx & (1u << 6))  cpu |= MVU_CPU_AVX512VBMI2;
        if (ecx & (1u << 11)) cpu |= MVU_CPU_AVX512VNNI;
        if (ecx & (1u << 12)) cpu |= MVU_CPU_AVX512BITALG;
        if (ecx & (1u << 14)) cpu |= MVU_CPU_AVX512VPOPCNTDQ;
        if (edx & (1u << 8))  cpu |= MVU_CPU_AVX512VP2INTERSECT;
        if (edx & (1u << 23)) cpu |= MVU_CPU_AVX512FP16;

        do_cpuid(regs, 7, 1);
        if ((unsigned)regs[0] & (1u << 5)) cpu |= MVU_CPU_AVX512BF16;
    }

    return cpu;
}

#else // not MVTOOLS_X86

uint32_t cpu_detect() {
    return 0;
}

#endif
