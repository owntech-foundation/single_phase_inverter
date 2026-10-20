/* Host shim for the CMSIS-DSP symbols used by control_library */
#pragma once
#include <math.h>
#include <stdint.h>
typedef float float32_t;
typedef double float64_t;
typedef int32_t q31_t;
#ifndef PI
#define PI 3.14159265358979f
#endif
static inline float32_t arm_sin_f32(float32_t x) { return sinf(x); }
static inline float32_t arm_cos_f32(float32_t x) { return cosf(x); }
/* Only the declaration of the FIR instance is needed (fir.h) */
typedef struct { uint16_t numTaps; float32_t *pState; const float32_t *pCoeffs; } arm_fir_instance_f32;
static inline void arm_fir_init_f32(arm_fir_instance_f32 *S, uint16_t numTaps,
    const float32_t *pCoeffs, float32_t *pState, uint32_t blockSize)
{ S->numTaps = numTaps; S->pCoeffs = pCoeffs; S->pState = pState; (void)blockSize; }
static inline void arm_fir_f32(const arm_fir_instance_f32 *S, const float32_t *pSrc,
    float32_t *pDst, uint32_t blockSize) { (void)S; (void)pSrc; (void)pDst; (void)blockSize; }
