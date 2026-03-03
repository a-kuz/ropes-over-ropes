#ifndef WOOD_COMMON_H
#define WOOD_COMMON_H

#include <metal_stdlib>
using namespace metal;

static float wc_hash21(float2 p) {
    float n = sin(dot(p, float2(127.1, 311.7)));
    return fract(n * 43758.5453123);
}

static float wc_noise3d(float3 p) {
    const float3 s = float3(7.0, 157.0, 113.0);
    float3 ip = floor(p);
    float3 fp = fract(p);
    fp = fp * fp * (3.0 - 2.0 * fp);
    float4 h = float4(0.0, s.yz, s.y + s.z) + dot(ip, s);
    h = mix(fract(sin(h) * 43758.545), fract(sin(h + s.x) * 43758.545), fp.x);
    h.xy = mix(h.xz, h.yw, fp.y);
    return mix(h.x, h.y, fp.z);
}

static float wc_fbm3d(float3 p, int octaves, float roughness) {
    float sum = 0.0, amp = 1.0, tot = 0.0;
    roughness = saturate(roughness);
    for (int i = 0; i < octaves; i++) {
        sum += amp * wc_noise3d(p);
        tot += amp;
        amp *= roughness;
        p *= 2.0;
    }
    return sum / tot;
}

static float3 wc_randomPos3(float seed) {
    float4 s = float4(seed, 0.0, 1.0, 2.0);
    return float3(wc_hash21(s.xy), wc_hash21(s.xz), wc_hash21(s.xw)) * 100.0 + 100.0;
}

static float wc_fbmDistorted(float3 p) {
    p += (float3(wc_noise3d(p + wc_randomPos3(0.0)),
                 wc_noise3d(p + wc_randomPos3(1.0)),
                 wc_noise3d(p + wc_randomPos3(2.0))) * 2.0 - 1.0) * 1.12;
    return wc_fbm3d(p, 8, 0.5);
}

static float wc_musgraveFbm(float3 p, float octaves, float dimension, float lacunarity) {
    float sum = 0.0, amp = 1.0;
    float m = pow(lacunarity, -dimension);
    for (float i = 0.0; i < octaves; i += 1.0) {
        float n = wc_noise3d(p) * 2.0 - 1.0;
        sum += n * amp;
        amp *= m;
        p *= lacunarity;
    }
    return sum;
}

static float3 wc_waveFbmX(float3 p) {
    float n = p.x * 20.0;
    n += 0.4 * wc_fbm3d(p * 3.0, 3, 3.0);
    return float3(sin(n) * 0.5 + 0.5, p.yz);
}

static float wc_remap01(float f, float in1, float in2) {
    return saturate((f - in1) / (in2 - in1));
}

static void wc_woodPalette(float seed, thread float3& colDark, thread float3& colMid, thread float3& colLight) {
    float hue = fract(seed * 0.618033988);
    if (hue < 0.07) {
        colDark  = float3(0.04, 0.04, 0.05);
        colMid   = float3(0.12, 0.12, 0.13);
        colLight = float3(0.24, 0.24, 0.26);
    } else if (hue < 0.14) {
        colDark  = float3(0.08, 0.09, 0.10);
        colMid   = float3(0.20, 0.21, 0.23);
        colLight = float3(0.36, 0.37, 0.40);
    } else if (hue < 0.21) {
        colDark  = float3(0.10, 0.09, 0.08);
        colMid   = float3(0.24, 0.22, 0.20);
        colLight = float3(0.42, 0.38, 0.35);
    } else if (hue < 0.28) {
        colDark  = float3(0.06, 0.07, 0.09);
        colMid   = float3(0.15, 0.16, 0.20);
        colLight = float3(0.28, 0.30, 0.35);
    } else if (hue < 0.35) {
        colDark  = float3(0.08, 0.07, 0.06);
        colMid   = float3(0.20, 0.17, 0.14);
        colLight = float3(0.36, 0.30, 0.26);
    } else if (hue < 0.42) {
        colDark  = float3(0.06, 0.04, 0.03);
        colMid   = float3(0.18, 0.12, 0.08);
        colLight = float3(0.32, 0.22, 0.16);
    } else if (hue < 0.49) {
        colDark  = float3(0.03, 0.03, 0.03);
        colMid   = float3(0.10, 0.09, 0.08);
        colLight = float3(0.20, 0.18, 0.16);
    } else if (hue < 0.56) {
        colDark  = float3(0.14, 0.10, 0.06);
        colMid   = float3(0.30, 0.22, 0.14);
        colLight = float3(0.50, 0.38, 0.26);
    } else if (hue < 0.63) {
        colDark  = float3(0.18, 0.17, 0.16);
        colMid   = float3(0.34, 0.32, 0.30);
        colLight = float3(0.52, 0.50, 0.47);
    } else if (hue < 0.70) {
        colDark  = float3(0.06, 0.08, 0.10);
        colMid   = float3(0.16, 0.19, 0.24);
        colLight = float3(0.30, 0.34, 0.40);
    } else if (hue < 0.77) {
        colDark  = float3(0.18, 0.08, 0.06);
        colMid   = float3(0.34, 0.18, 0.12);
        colLight = float3(0.52, 0.30, 0.22);
    } else if (hue < 0.84) {
        colDark  = float3(0.14, 0.14, 0.12);
        colMid   = float3(0.30, 0.28, 0.26);
        colLight = float3(0.48, 0.46, 0.42);
    } else if (hue < 0.92) {
        colDark  = float3(0.05, 0.05, 0.06);
        colMid   = float3(0.14, 0.14, 0.16);
        colLight = float3(0.26, 0.26, 0.30);
    } else {
        colDark  = float3(0.12, 0.10, 0.09);
        colMid   = float3(0.26, 0.22, 0.20);
        colLight = float3(0.44, 0.38, 0.34);
    }
}

#endif
