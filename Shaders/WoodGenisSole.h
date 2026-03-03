#ifndef WOOD_GENIS_SOLE_H
#define WOOD_GENIS_SOLE_H

#include "WoodCommon.h"

static uint gsHash(uint2 x) {
    x = uint2(0x3504f333u, 0xf1bbcdcbu) * x;
    return (x.x ^ x.y) * 741103597u;
}

static float gsFloat01(uint x) {
    return as_type<float>((x >> 9u) | 0x3f800000u) - 1.0;
}

static float gsFloat11(uint x) {
    return as_type<float>((x >> 9u) | 0x40000000u) - 3.0;
}

static float2 gsHash22(float2 p) {
    uint2 x = as_type<uint2>(p);
    return float2(gsFloat11(gsHash(x)),
                  gsFloat11(gsHash(x + 868867u)));
}

static float gsHash21(float2 p) {
    uint2 x = as_type<uint2>(p);
    return gsFloat01(gsHash(x));
}

static float gsHash11(float p) {
    uint x = as_type<uint>(p);
    return gsFloat01(gsHash(uint2(x, 324034u)));
}

static float gsVnoise2(float2 p) {
    float2 i = floor(p);
    float2 f = fract(p);
    float a = gsHash21(i);
    float b = gsHash21(i + float2(1.0, 0.0));
    float c = gsHash21(i + float2(0.0, 1.0));
    float d = gsHash21(i + float2(1.0, 1.0));
    float2 u = f * f * (3.0 - 2.0 * f);
    return a + u.x * (b - a) + u.y * (c - a) + u.x * u.y * (d - c - b + a);
}

static float gsVnoise1(float p) {
    float i = floor(p);
    float f = fract(p);
    float a = gsHash11(i);
    float b = gsHash11(i + 1.0);
    float u = f * f * (3.0 - 2.0 * f);
    return mix(a, b, u);
}

static float gsFbm2(float2 p) {
    return gsVnoise2(p) * 0.5 + gsVnoise2(p * 2.0) * 0.25 + gsVnoise2(p * 4.0) * 0.125;
}

static float gsFbm1(float p) {
    return gsVnoise1(p) * 0.5 + gsVnoise1(p * 2.0) * 0.25 + gsVnoise1(p * 4.0) * 0.125;
}

static float gsGnoise2(float2 p) {
    float2 i = floor(p);
    float2 f = fract(p);
    float a = dot(gsHash22(i), f);
    float b = dot(gsHash22(i + float2(1.0, 0.0)), f - float2(1.0, 0.0));
    float c = dot(gsHash22(i + float2(0.0, 1.0)), f - float2(0.0, 1.0));
    float d = dot(gsHash22(i + float2(1.0)), f - float2(1.0));
    float2 q = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    return a + q.x * (b - a) + q.y * (c - a) + q.x * q.y * (d - c - b + a);
}

static float3 gsWood3D(float3 p) {
    float sn = sin(0.25);
    float cn = cos(0.25);
    float3 axis = normalize(float3(1.0, 1.0, 1.0));
    p += 2.0 * cross(axis * sn, cross(axis * sn, p) + cn * p);

    float2 U = 40.0 * p.xz + 15.0;

    float w = length(U);
    w += gsFbm1(w * 0.5);
    w *= log(max(0.001f, w * 0.2)) * 0.1;
    w += gsFbm2(U * 0.05 + p.y * 0.5 + 500.0);
    float ringFloor = floor(w);
    float ringFrac = fract(w);
    float rings = ringFrac * smoothstep(0.0, 1.0, 1.0 - ringFrac) * 2.7;

    float grain = 0.5 * (1.0 + gsGnoise2(U * 8.0 + 100.0 + p.y * 5.0)) * 0.5 +
                  gsGnoise2((ringFrac + ringFloor + 100.0 + gsFbm2(U + 200.0) * 0.06) * 20.0) * 0.5;
    float combined = mix(rings, grain, 0.5);

    return clamp((combined * 1.5 + 0.3) * float3(0.7, 0.4, 0.2), 0.0, 1.0);
}

static float3 gsBoardWoodTexture(float2 worldXY, float elevation, float seed) {
    float seedAngle = seed * 1.93;
    float sa = sin(seedAngle), ca = cos(seedAngle);
    float2 rotated = float2(worldXY.x * ca - worldXY.y * sa,
                            worldXY.x * sa + worldXY.y * ca);
    float2 offset = float2(wc_hash21(float2(seed * 3.17, 1.0)) * 20.0 - 10.0,
                           wc_hash21(float2(1.0, seed * 4.31)) * 20.0 - 10.0);
    float3 p = float3((rotated + offset) * 0.5, elevation * 2.0 + seed * 7.0);
    float3 raw = gsWood3D(p);

    float3 colDark, colMid, colLight;
    wc_woodPalette(seed, colDark, colMid, colLight);

    float lum = dot(raw, float3(0.299, 0.587, 0.114));
    float3 col = mix(colDark, mix(colMid, colLight, saturate(lum * 2.0 - 1.0)), saturate(lum * 2.0));
    return saturate(col);
}

#endif
