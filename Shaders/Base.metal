#include <metal_stdlib>
#include "WoodCommon.h"
#include "WoodOtavio.h"
#include "WoodGenisSole.h"
using namespace metal;

struct VSOut {
    float4 position [[position]];
    float2 uv;
};

struct FrameUniforms {
    float4x4 viewProj;
    float4x4 invViewProj;
    float4x4 lightViewProj;
    float4 lightDir_intensity;
    float4 ambientColor;
    float4 cameraPos;
    float4 orthoHalfSize_shadowBias;
    float4 shadowInvSize_unused;
    float4 timeDrag;
    float4 woodBoundsMin;
    float4 woodBoundsMax;
    float4 holeTint;
    float4 visualParams;
    float4 lightingParams;
    float4 tableParams;
    float4 tableParams2;
    float4 ropeMatParams;
    float4 ropeMatParams2;
    float4 ropeMatParams3;
    float4 cartoonParams;
    float4 wormParams1;
    float4 wormParams2;
    float4 wormParams3;
    float4 wormParams4;
};

static float shadowVisibility(float3 worldPos, float3 worldN, constant FrameUniforms& frame, depth2d<float> shadowMap);

static float3 woodSolidTexture(float2 worldXY, float seed) {
    float seedAngle = seed * 2.399;
    float sa = sin(seedAngle), ca = cos(seedAngle);
    float2 rotated = float2(worldXY.x * ca - worldXY.y * sa,
                            worldXY.x * sa + worldXY.y * ca);
    float2 offset = float2(wc_hash21(float2(seed, seed * 7.13)) * 40.0 - 20.0,
                           wc_hash21(float2(seed * 3.71, seed)) * 40.0 - 20.0);
    float3 p = float3((rotated + offset) * 1.1, floor(fract(seed * 0.37) * 8.0));

    float scaleVar = mix(0.8, 1.2, wc_hash21(float2(seed * 1.23, 0.0)));
    p.xy *= scaleVar;

    float n1 = wc_fbmDistorted(p * float3(7.8, 1.17, 1.17));
    n1 = mix(n1, 1.0, 0.2);
    float n2 = mix(wc_musgraveFbm(float3(n1 * 4.6), 8.0, 0.0, 2.5), n1, 0.85);
    float dirt = 1.0 - wc_musgraveFbm(wc_waveFbmX(p * float3(0.01, 0.15, 0.15)), 15.0, 0.26, 2.4) * 0.4;
    float grain = 1.0 - smoothstep(0.2, 1.0, wc_musgraveFbm(p * float3(500.0, 6.0, 1.0), 2.0, 2.0, 2.5)) * 0.2;
    n2 *= dirt * grain;

    float3 colDark, colMid, colLight;
    wc_woodPalette(seed, colDark, colMid, colLight);

    float3 col = mix(mix(colDark, colMid, wc_remap01(n2, 0.19, 0.56)), colLight, wc_remap01(n2, 0.56, 1.0));
    return saturate(pow(col, float3(0.88)));
}

static float3 plankWoodTexture(float2 worldXY, float seed) {
    float seedAngle = seed * 1.73;
    float sa = sin(seedAngle), ca = cos(seedAngle);
    float2 rotated = float2(worldXY.x * ca - worldXY.y * sa,
                            worldXY.x * sa + worldXY.y * ca);
    float2 globalOffset = float2(wc_hash21(float2(seed, seed * 5.17)) * 30.0 - 15.0,
                                 wc_hash21(float2(seed * 2.91, seed)) * 30.0 - 15.0);
    float2 wp = rotated + globalOffset;

    float plankWidth = mix(0.28, 0.45, wc_hash21(float2(seed * 3.14, 7.0)));
    float gapWidth = 0.006;

    float plankY = wp.y / plankWidth;
    float plankRow = floor(plankY);
    float plankFrac = fract(plankY);

    float rowOffset = wc_hash21(float2(plankRow, seed * 11.3)) * 2.0;
    float plankLen = mix(0.8, 1.6, wc_hash21(float2(plankRow, seed * 4.7)));
    float plankX = (wp.x + rowOffset) / plankLen;
    float plankCol = floor(plankX);
    float plankFracX = fract(plankX);

    float gapY = smoothstep(0.0, gapWidth / plankWidth, plankFrac) *
                 smoothstep(0.0, gapWidth / plankWidth, 1.0 - plankFrac);
    float gapX = smoothstep(0.0, gapWidth / plankLen, plankFracX) *
                 smoothstep(0.0, gapWidth / plankLen, 1.0 - plankFracX);
    float gapMask = gapX * gapY;

    float plankId = wc_hash21(float2(plankRow * 17.3 + plankCol * 7.1, seed));
    float plankSeed = seed + plankId * 100.0;

    float bendK = (wc_hash21(float2(plankSeed, 3.3)) - 0.5) * 0.08;
    float localX = plankFracX * plankLen;
    float localY = plankFrac * plankWidth;
    float bentY = localY + sin(localX * 3.14159 / plankLen) * bendK;

    float3 p = float3(localX * 0.016, bentY * 0.016, plankId * 7.0);

    float n1 = wc_fbmDistorted(p * float3(1.0, 0.15, 0.15) * 8.0);
    n1 = mix(n1, 1.0, 0.2);
    float n2 = wc_musgraveFbm(float3(n1 * 4.6), 8.0, 0.0, 2.5);
    float n3 = mix(n2, n1, 0.85);
    float3 q = wc_waveFbmX(p * float3(0.01, 0.15, 0.15));
    float dirt = 1.0 - wc_musgraveFbm(q, 15.0, 0.26, 2.4) * 0.4;
    float grain = 1.0 - smoothstep(0.2, 1.0, wc_musgraveFbm(p * float3(500.0, 6.0, 1.0), 2.0, 2.0, 2.5)) * 0.2;
    n3 *= dirt * grain;

    float3 colDark, colMid, colLight;
    wc_woodPalette(plankSeed, colDark, colMid, colLight);

    float plankBrightness = mix(0.85, 1.15, wc_hash21(float2(plankSeed * 1.7, 2.0)));
    float3 col = mix(mix(colDark, colMid, wc_remap01(n3, 0.185, 0.565)), colLight, wc_remap01(n3, 0.565, 1.0));
    col *= plankBrightness;

    float edgeDarken = smoothstep(0.0, 0.04, plankFrac) * smoothstep(0.0, 0.04, 1.0 - plankFrac) *
                       smoothstep(0.0, 0.03, plankFracX) * smoothstep(0.0, 0.03, 1.0 - plankFracX);
    col *= mix(0.7, 1.0, edgeDarken);

    float3 gapColor = colDark * 0.15;
    col = mix(gapColor, col, gapMask);

    return saturate(pow(col, float3(0.88)));
}

static float3 gradientTexture(float2 worldXY, float seed) {
    float seedAngle = seed * 1.618;
    float sa = sin(seedAngle), ca = cos(seedAngle);
    float2 rp = float2(worldXY.x * ca - worldXY.y * sa,
                        worldXY.x * sa + worldXY.y * ca);
    float2 off = float2(wc_hash21(float2(seed * 2.3, 1.0)) * 20.0 - 10.0,
                        wc_hash21(float2(1.0, seed * 3.7)) * 20.0 - 10.0);
    rp += off;

    float gradType = fract(seed * 0.7236);

    float3 c1, c2, c3;
    float palIdx = fract(seed * 0.4618);
    if (palIdx < 0.06) {
        c1 = float3(0.82, 0.82, 0.84); c2 = float3(0.62, 0.63, 0.66); c3 = float3(0.44, 0.45, 0.48);
    } else if (palIdx < 0.12) {
        c1 = float3(0.90, 0.88, 0.85); c2 = float3(0.70, 0.68, 0.64); c3 = float3(0.50, 0.48, 0.44);
    } else if (palIdx < 0.18) {
        c1 = float3(0.86, 0.86, 0.88); c2 = float3(0.56, 0.57, 0.62); c3 = float3(0.30, 0.32, 0.38);
    } else if (palIdx < 0.24) {
        c1 = float3(0.78, 0.76, 0.72); c2 = float3(0.52, 0.50, 0.46); c3 = float3(0.30, 0.28, 0.26);
    } else if (palIdx < 0.30) {
        c1 = float3(0.88, 0.86, 0.82); c2 = float3(0.58, 0.54, 0.48); c3 = float3(0.34, 0.30, 0.26);
    } else if (palIdx < 0.36) {
        c1 = float3(0.06, 0.06, 0.08); c2 = float3(0.18, 0.18, 0.22); c3 = float3(0.34, 0.34, 0.40);
    } else if (palIdx < 0.42) {
        c1 = float3(0.04, 0.05, 0.07); c2 = float3(0.12, 0.14, 0.20); c3 = float3(0.24, 0.28, 0.38);
    } else if (palIdx < 0.48) {
        c1 = float3(0.08, 0.07, 0.06); c2 = float3(0.22, 0.20, 0.18); c3 = float3(0.40, 0.36, 0.32);
    } else if (palIdx < 0.54) {
        c1 = float3(0.02, 0.04, 0.06); c2 = float3(0.08, 0.16, 0.24); c3 = float3(0.18, 0.30, 0.42);
    } else if (palIdx < 0.60) {
        c1 = float3(0.84, 0.84, 0.82); c2 = float3(0.48, 0.48, 0.46); c3 = float3(0.20, 0.20, 0.20);
    } else if (palIdx < 0.66) {
        c1 = float3(0.04, 0.06, 0.04); c2 = float3(0.12, 0.20, 0.14); c3 = float3(0.24, 0.38, 0.28);
    } else if (palIdx < 0.72) {
        c1 = float3(0.05, 0.03, 0.06); c2 = float3(0.16, 0.10, 0.22); c3 = float3(0.30, 0.20, 0.40);
    } else if (palIdx < 0.78) {
        c1 = float3(0.92, 0.90, 0.86); c2 = float3(0.74, 0.72, 0.68); c3 = float3(0.56, 0.54, 0.50);
    } else if (palIdx < 0.84) {
        c1 = float3(0.10, 0.10, 0.12); c2 = float3(0.26, 0.26, 0.30); c3 = float3(0.46, 0.46, 0.52);
    } else if (palIdx < 0.92) {
        c1 = float3(0.80, 0.82, 0.86); c2 = float3(0.54, 0.56, 0.62); c3 = float3(0.32, 0.34, 0.40);
    } else {
        c1 = float3(0.06, 0.05, 0.04); c2 = float3(0.18, 0.16, 0.14); c3 = float3(0.36, 0.32, 0.28);
    }

    float t;
    if (gradType < 0.25) {
        float warp = wc_noise3d(float3(rp * 0.4, seed)) * 0.15;
        t = saturate(rp.y * 0.25 + 0.5 + warp);
    } else if (gradType < 0.45) {
        float dist = length(rp) * 0.6;
        float warp = wc_noise3d(float3(rp * 1.5, seed)) * 0.2;
        t = saturate((dist + warp) * 0.4);
    } else if (gradType < 0.60) {
        float diag = (rp.x + rp.y) * 0.3;
        float warp = wc_noise3d(float3(rp * 0.6, seed)) * 0.2;
        t = saturate(diag + warp + 0.5);
    } else if (gradType < 0.75) {
        float angle = atan2(rp.y, rp.x) / 6.28318 + 0.5;
        float warp = wc_noise3d(float3(rp * 2.0, seed * 1.3)) * 0.08;
        t = fract(angle + warp);
    } else if (gradType < 0.88) {
        float wave = sin(rp.x * 2.0 + wc_noise3d(float3(rp * 0.8, seed)) * 1.5) * 0.5 + 0.5;
        float wave2 = sin(rp.y * 1.8 + wc_noise3d(float3(rp * 0.6, seed + 5.0)) * 1.2) * 0.5 + 0.5;
        t = saturate(wave * 0.55 + wave2 * 0.45);
    } else {
        float n = wc_fbm3d(float3(rp * 0.4, seed * 0.7), 5, 0.5);
        t = saturate(n * 0.8 + 0.3);
    }

    float3 col;
    if (t < 0.5) {
        col = mix(c1, c2, t * 2.0);
    } else {
        col = mix(c2, c3, (t - 0.5) * 2.0);
    }

    float micro = wc_noise3d(float3(rp * 8.0, seed * 2.0)) * 0.03 - 0.015;
    col += micro;

    return saturate(col);
}

static float3 woodTexture(float2 uv, float2 worldXY, float seed) {
    float style = fract(seed * 0.3819);
    if (style < 0.20) {
        return woodSolidTexture(worldXY, seed);
    } else if (style < 0.40) {
        return plankWoodTexture(worldXY, seed);
    } else if (style < 0.60) {
        return gradientTexture(worldXY, seed);
    } else {
        return otavioWoodTexture(worldXY, seed);
    }
}

static float3 boardWoodTexture(float2 worldXY, float elevation, float seed) {
    float style = fract(seed * 0.5173);
    if (style < 0.5) {
        return otavioBoardWoodTexture(worldXY, elevation, seed);
    } else {
        return gsBoardWoodTexture(worldXY, elevation, seed);
    }
}

static float toonStep(float nl, int levels) {
    if (levels <= 1) return 1.0;
    float n = float(levels - 1);
    return floor(nl * n + 0.5) / n;
}

static float celStep(float ndl, int levels, float shadowBright) {
    if (levels <= 1) return ndl > 0.0 ? 1.0 : shadowBright;
    float n = float(levels);
    float bucket = floor(ndl * n);
    bucket = clamp(bucket, 0.0, n - 1.0);
    float t = bucket / (n - 1.0);
    return mix(shadowBright, 1.0, t);
}

static float3 celRopeShading(float3 baseColor, float3 n, float3 l, float3 v, int levels, float4 cp) {
    float shadowBright = cp.x;
    float wrap = cp.y;
    float ndl = dot(n, l);
    float lit = celStep(saturate((ndl + wrap) / (1.0 + wrap)), levels, shadowBright);
    return baseColor * lit;
}

static float3 celTableShading(float3 baseColor, float3 n, float3 l, int levels, float4 cp) {
    float shadowBright = cp.x;
    float wrap = cp.y;
    float ndl = dot(n, l);
    float lit = celStep(saturate((ndl + wrap * 2.0) / (1.0 + wrap * 2.0)), levels, shadowBright);
    return baseColor * lit;
}

static float3 celHoleShading(float3 baseColor, float3 n, float3 l, int levels, float4 cp) {
    float shadowBright = cp.x;
    float wrap = cp.y;
    float ndl = dot(n, l);
    float lit = celStep(saturate((ndl + wrap) / (1.0 + wrap)), levels, shadowBright);
    return baseColor * lit;
}

static float3 rubberPBR(float3 baseColor, float3 n, float3 l, float3 v,
                        float roughness, float taut, float vCoord, float cartoonMode, int cartoonLevels,
                        float4 matP, float4 matP2, float stretchGloss, float stretchSpec) {
    float matteAmount = matP.x;
    float glossAmount = matP.y;
    float diffuseWrap = matP.z;
    float subsurface = matP.w;
    float edgeLight = matP2.x;
    float saturation = matP2.y;

    float nl = saturate(dot(n, l));
    float nv = saturate(dot(n, v));
    float3 h = normalize(l + v);
    float nh = saturate(dot(n, h));
    float vh = saturate(dot(v, h));

    float radial = abs(vCoord - 0.5) * 2.0;
    float coreDarken = 1.0 - (1.0 - radial) * (1.0 - radial) * mix(0.15, 0.45, matteAmount);
    float edgeBrighten = pow(radial, 2.5) * mix(0.15, 0.02, matteAmount);
    float3 albedo = baseColor * coreDarken + float3(edgeBrighten);

    float grey = dot(albedo, float3(0.299, 0.587, 0.114));
    albedo = mix(float3(grey), albedo, saturation);

    float wrapDiff = saturate((nl + diffuseWrap) / (1.0 + diffuseWrap));
    if (cartoonMode > 0.5) wrapDiff = toonStep(wrapDiff, cartoonLevels);

    float sssNL = saturate(dot(-n, l));
    float sssWrap = saturate((sssNL + 0.3) / 1.3);
    float sssContrib = sssWrap * subsurface * 0.3;

    float ambientBase = mix(0.20, 0.45, matteAmount);
    float3 diff = albedo * (ambientBase + (1.0 - ambientBase) * wrapDiff + sssContrib);

    float rough = mix(0.18, 0.92, matteAmount) + roughness * 0.1;
    float taut2 = taut * taut;
    float roughFloor = mix(0.85, 0.25, taut) * stretchGloss + 0.85 * (1.0 - stretchGloss);
    rough = mix(rough, rough * roughFloor, taut);
    rough = clamp(rough, 0.05, 0.99);
    float alpha = rough * rough;
    float alpha2 = alpha * alpha;
    float denom = nh * nh * (alpha2 - 1.0) + 1.0;
    float D = alpha2 / (3.14159265 * denom * denom + 1e-5);
    float k = (rough + 1.0) * (rough + 1.0) / 8.0;
    float G1l = nl / (nl * (1.0 - k) + k);
    float G1v = nv / (nv * (1.0 - k) + k);
    float G = G1l * G1v;
    float F0base = mix(0.08, 0.01, matteAmount);
    float F0 = F0base + taut2 * 0.12 * stretchSpec;
    float F = F0 + (1.0 - F0) * pow(1.0 - vh, 5.0);
    float spec = D * G * F / max(4.0 * nl * nv, 0.001);
    float specBoost = 1.0 + (taut * 1.5 + taut2 * 3.0) * stretchSpec;
    float specIntensity = glossAmount * mix(1.2, 0.08, matteAmount);
    float3 specColor = float3(1.0) * spec * specIntensity * specBoost;
    if (cartoonMode > 0.5) specColor *= 0.3;

    float rimPow = mix(3.0, 6.0, matteAmount) - taut * 1.5 * stretchGloss;
    rimPow = max(1.5, rimPow);
    float rim = pow(1.0 - nv, rimPow) * edgeLight * (1.0 + taut2 * 2.0 * stretchSpec);
    diff += albedo * rim;

    return diff + specColor;
}

struct HoleInstance {
    float4 position_radius;
};

struct SceneVSOut {
    float4 position [[position]];
    float2 local;
    float2 worldXY;
};

vertex VSOut fullscreenVertex(uint vid [[vertex_id]]) {
    float2 p;
    if (vid == 0) p = float2(-1.0, -1.0);
    else if (vid == 1) p = float2(3.0, -1.0);
    else p = float2(-1.0, 3.0);

    VSOut out;
    out.position = float4(p, 0.0, 1.0);
    out.uv = p * 0.5 + 0.5;
    return out;
}

fragment float4 solidColorFragment(VSOut in [[stage_in]]) {
    float2 uv = in.uv;
    float3 c = mix(float3(0.08, 0.09, 0.13), float3(0.12, 0.13, 0.20), uv.y);
    return float4(c, 1.0);
}

struct TableOut {
    float4 color [[color(0)]];
    float depth [[depth(any)]];
};

struct HoleCountBuf {
    uint count;
};

struct BakeWoodParams {
    float2 worldMin;
    float2 worldMax;
    float seed;
    float brightness;
    float patternScale;
    float2 _pad;
};

kernel void bakeWoodKernel(texture2d<float, access::write> dst [[texture(0)]],
                           constant BakeWoodParams& params [[buffer(0)]],
                           uint2 gid [[thread_position_in_grid]]) {
    uint w = dst.get_width();
    uint h = dst.get_height();
    if (gid.x >= w || gid.y >= h) return;

    float2 uv = (float2(gid) + 0.5) / float2(w, h);
    float2 worldXY = mix(params.worldMin, params.worldMax, uv);
    worldXY *= params.patternScale;

    float3 col = woodTexture(uv, worldXY, params.seed);
    col *= params.brightness;
    dst.write(float4(col, 1.0), gid);
}

struct BakeBoardWoodVolumeParams {
    float4 worldMin;
    float4 worldMax;
    float seed;
    float brightness;
    float2 _pad;
};

kernel void bakeBoardWoodVolumeKernel(texture3d<float, access::write> dst [[texture(0)]],
                                      constant BakeBoardWoodVolumeParams& params [[buffer(0)]],
                                      uint3 gid [[thread_position_in_grid]]) {
    uint w = dst.get_width();
    uint h = dst.get_height();
    uint d = dst.get_depth();
    if (gid.x >= w || gid.y >= h || gid.z >= d) return;

    float3 uvw = (float3(gid) + 0.5) / float3(w, h, d);
    float3 worldPos = mix(params.worldMin.xyz, params.worldMax.xyz, uvw);

    float3 col = boardWoodTexture(worldPos.xy, worldPos.z, params.seed);
    col *= params.brightness;
    dst.write(float4(col, 1.0), gid);
}

fragment TableOut tableFragment(VSOut in [[stage_in]],
                              constant FrameUniforms& frame [[buffer(1)]],
                              constant HoleCountBuf& holeCounts [[buffer(3)]],
                              const device HoleInstance* holes [[buffer(4)]],
                              depth2d<float> shadowMap [[texture(2)]],
                              texture2d<float> woodTex [[texture(3)]]) {
    constexpr sampler woodSampler(address::clamp_to_edge, filter::linear);

    float2 uv = in.uv;
    float2 ndc = uv * 2.0 - 1.0;

    float4 nearW = frame.invViewProj * float4(ndc.x, ndc.y, 0.0, 1.0);
    float4 farW  = frame.invViewProj * float4(ndc.x, ndc.y, 1.0, 1.0);
    float3 nearP = nearW.xyz / nearW.w;
    float3 farP  = farW.xyz / farW.w;
    float3 dir = farP - nearP;
    float t = -nearP.z / dir.z;
    float2 worldXY = nearP.xy + dir.xy * t;

    float3 worldPos = float3(worldXY, 0.0);

    float4 clipPos = frame.viewProj * float4(worldPos, 1.0);
    float tableDepth = clipPos.z / clipPos.w + 0.0004;

    float squareHoles = frame.tableParams2.w;
    for (uint i = 0; i < holeCounts.count; i++) {
        float holeElev = holes[i].position_radius.z;
        if (holeElev > 0.01) continue;
        float2 holeCenter = holes[i].position_radius.xy;
        float holeR = holes[i].position_radius.w * 0.76;
        float2 d = abs(worldXY - holeCenter);
        float dist = squareHoles > 0.5 ? max(d.x, d.y) : length(worldXY - holeCenter);
        if (dist < holeR) {
            tableDepth = 1.0;
            break;
        }
    }

    float3 worldN = float3(0.0, 0.0, 1.0);

    float tableStyle = frame.tableParams.x;
    float3 baseColor;
    if (tableStyle < 0.5) {
        if (woodTex.get_width() > 1) {
            float2 woodUV = (worldXY - frame.woodBoundsMin.xy) / (frame.woodBoundsMax.xy - frame.woodBoundsMin.xy);
            baseColor = woodTex.sample(woodSampler, woodUV).rgb;
        } else {
            float levelSeed = frame.timeDrag.z;
            baseColor = woodTexture(uv, worldXY, levelSeed);
        }

        baseColor = saturate(baseColor);
    } else if (tableStyle < 1.5) {
        float3 c1 = frame.tableParams.yzw;
        float3 c2 = frame.tableParams2.xyz;
        baseColor = mix(c1, c2, uv.y);
    } else {
        baseColor = frame.tableParams.yzw;
    }

    float3 l = normalize(frame.lightDir_intensity.xyz);
    float lightI = frame.lightDir_intensity.w;
    float3 v = normalize(frame.cameraPos.xyz - worldPos);
    float nl = saturate(dot(worldN, l));
    float nv = saturate(dot(worldN, v));
    float3 h = normalize(l + v);
    float nh = saturate(dot(worldN, h));

    float cartoonMode = frame.visualParams.z;
    int cartoonLevels = int(frame.visualParams.w);

    float3 c;
    if (cartoonMode > 0.5) {
        float3 flatColor = float3(0.88, 0.88, 0.88);
        c = celTableShading(flatColor, worldN, l, cartoonLevels, frame.cartoonParams);
    } else {
        float wrap = 0.4;
        float wrapTerm = saturate((nl + wrap) / (1.0 + wrap));
        float3 diff = baseColor * mix(0.25, 0.95, wrapTerm) * lightI;

        float fresnel = pow(1.0 - nv, 3.0);
        float roughness = 0.75;
        float alpha = roughness * roughness;
        float alpha2 = alpha * alpha;
        float denom = nh * nh * (alpha2 - 1.0) + 1.0;
        float d = alpha2 / (3.14159265 * denom * denom + 1e-5);
        float k = alpha * 0.5 + 1e-4;
        float gl = nl / (nl * (1.0 - k) + k);
        float gv = nv / (nv * (1.0 - k) + k);
        float spec = d * gl * gv;
        float3 specColor = float3(0.9, 0.85, 0.75) * spec * 0.12 * (0.2 + 0.8 * fresnel) * lightI;

        c = diff + specColor;

        float shadow = 1.0;
        if (shadowMap.get_width() > 0) {
            shadow = shadowVisibility(worldPos, worldN, frame, shadowMap);
        }
        shadow = pow(shadow, 2.2);
        float shadowDark = frame.lightingParams.y;
        c *= mix(shadowDark, 1.0, shadow);

        float ambient = frame.lightingParams.x;
        c += baseColor * ambient;
    }

    TableOut out;
    out.color = float4(c, 1.0);
    out.depth = tableDepth;
    return out;
}

struct HoleIn {
    float3 position [[attribute(0)]];
    float3 normal [[attribute(1)]];
};

struct HoleOut {
    float4 position [[position]];
    float3 normal;
    float3 worldPos;
    float highlight;
    float holeId;
};

vertex HoleOut holeVertex(const device HoleIn* vertices [[buffer(0)]],
                          uint vid [[vertex_id]],
                          uint iid [[instance_id]],
                          constant FrameUniforms& frame [[buffer(1)]],
                          const device HoleInstance* holes [[buffer(2)]]) {
    HoleInstance inst = holes[iid];
    float radius = inst.position_radius.w;
    float elevation = inst.position_radius.z;
    float3 lp = vertices[vid].position * radius;
    float3 wp = float3(inst.position_radius.x + lp.x, inst.position_radius.y + lp.y, elevation + lp.z);

    float hlIdx = frame.ambientColor.w;
    float isHighlight = (hlIdx >= 0.0 && abs(float(iid) - hlIdx) < 0.5) ? 1.0 : 0.0;

    HoleOut o;
    o.worldPos = wp;
    o.position = frame.viewProj * float4(wp, 1.0);
    o.normal = normalize(vertices[vid].normal);
    o.highlight = isHighlight;
    o.holeId = float(iid);
    return o;
}

fragment float4 holeFragment(HoleOut in [[stage_in]],
                             constant FrameUniforms& frame [[buffer(1)]],
                             depth2d<float> shadowMap [[texture(2)]]) {
    float3 n = normalize(in.normal);
    float3 l = normalize(frame.lightDir_intensity.xyz);
    float3 v = normalize(frame.cameraPos.xyz - in.worldPos);

    float3 baseCol = float3(0.12, 0.13, 0.15);
    float3 topCol = float3(0.18, 0.19, 0.22);
    float specPower = 120.0;
    float specStrength = 0.55;

    float3 col = mix(baseCol, topCol, smoothstep(0.15, 0.75, n.z));
    float3 tint = frame.holeTint.xyz;
    float tintAmt = frame.holeTint.w;
    col = mix(col, col * tint, tintAmt);

    if (in.highlight > 0.5) {
        float3 hlCol = float3(0.55, 0.85, 1.0);
        col = mix(col, hlCol, 0.6);
    }

    float lightI = frame.lightDir_intensity.w;
    float cartoonMode = frame.visualParams.z;
    int cartoonLevels = int(frame.visualParams.w);

    float3 lit;
    if (cartoonMode > 0.5) {
        lit = celHoleShading(col, n, l, cartoonLevels, frame.cartoonParams);
    } else {
        float ndl = saturate(dot(n, l));
        float3 h = normalize(l + v);
        float ndh = saturate(dot(n, h));
        float nv = saturate(dot(n, v));
        float fresnel = pow(1.0 - nv, 4.0);
        float spec = pow(ndh, specPower) * specStrength * (0.3 + 0.7 * fresnel);
        float3 specCol = mix(col, float3(1.0), 0.5) * spec * lightI;
        lit = col * (0.22 + 0.78 * ndl) * lightI + specCol;

        float shadow = 1.0;
        if (shadowMap.get_width() > 0) {
            shadow = shadowVisibility(in.worldPos, n, frame, shadowMap);
        }
        shadow = pow(shadow, 2.0);
        float shadowDark = frame.lightingParams.y;
        lit *= mix(shadowDark, 1.0, shadow);
    }

    if (in.highlight > 0.5) {
        lit += float3(0.08, 0.15, 0.22);
    }

    return float4(lit, 1.0);
}

struct RopeIn {
    float3 position [[attribute(0)]];
    float3 normal [[attribute(1)]];
    float3 color [[attribute(2)]];
    float2 uv [[attribute(3)]];
    float4 params [[attribute(4)]];
};

struct RopeOut {
    float4 position [[position]];
    float3 normal;
    float3 color;
    float3 worldPos;
    float2 uv;
    float4 params;
};

vertex RopeOut ropeVertex(RopeIn in [[stage_in]],
                          constant FrameUniforms& frame [[buffer(1)]]) {
    RopeOut o;
    float time = frame.timeDrag.x;
    float energy = frame.timeDrag.y;
    float dragActive = frame.timeDrag.w;
    float u = in.uv.x;
    float pinch = in.params.y;
    float isWorm = in.params.w;

    float w = sin(u * 3.14159265);
    w = w * w;

    float3 displaced;
    if (isWorm > 0.5) {
        float crawlSpeed = frame.wormParams4.x;
        float crawlAmp = frame.wormParams4.y;
        float sideAmpP = frame.wormParams4.z;
        float crawlWave = sin(u * 12.0 - time * crawlSpeed) * crawlAmp * w;
        float sideWave = sin(u * 8.0 - time * crawlSpeed * 0.57) * sideAmpP * w;
        float3 nDir = normalize(in.normal);
        float3 sideDir = cross(nDir, float3(0, 0, 1));
        if (length(sideDir) < 1e-3) sideDir = cross(nDir, float3(0, 1, 0));
        sideDir = normalize(sideDir);
        displaced = in.position + nDir * crawlWave + sideDir * sideWave;
    } else {
        float amp = (0.002 + 0.010 * pinch) * energy * (0.25 + 0.75 * dragActive) * w;
        float wave = sin(u * 24.0 + time * 16.0) * 0.65 + sin(u * 11.0 - time * 9.0) * 0.35;
        displaced = in.position + normalize(in.normal) * (wave * amp);
    }

    o.worldPos = displaced;
    o.position = frame.viewProj * float4(displaced, 1.0);
    o.normal = in.normal;
    o.color = in.color;
    o.uv = in.uv;
    o.params = in.params;
    return o;
}

static float3 wormShading(float3 baseColor, float3 n, float3 l, float3 v, float3 worldPos,
                          float2 uv, float time, constant FrameUniforms& frame) {
    float u = uv.x;
    float vCoord = uv.y;

    float grooveDepth = frame.wormParams1.x;
    float bellyBright = frame.wormParams1.y;
    float backDark = frame.wormParams1.z;
    float skinNoiseAmt = frame.wormParams1.w;
    float sssStr = frame.wormParams2.x;
    float rough = frame.wormParams2.y;
    float specStr = frame.wormParams2.z;
    float rimStr = frame.wormParams2.w;
    float eyeSize = frame.wormParams3.x;
    float pulseSpeed = frame.wormParams3.y;
    float pulseAmp = frame.wormParams3.z;
    float segFreq = frame.wormParams3.w;

    float segPhase = u * segFreq * 3.14159265 * 2.0;
    float segGroove = smoothstep(0.85, 1.0, abs(sin(segPhase)));
    float grooveDarken = 1.0 - segGroove * grooveDepth;

    float bodyGrad = sin(u * 3.14159265);
    float3 bellyColor = baseColor * float3(bellyBright, bellyBright * 0.957, bellyBright * 0.87);
    float3 backColor = baseColor * float3(backDark, backDark * 1.07, backDark);
    float bellySide = smoothstep(0.3, 0.7, vCoord);
    float3 skinColor = mix(bellyColor, backColor, bellySide);

    float skinNoise = wc_hash21(worldPos.xy * 80.0 + u * 5.0) * skinNoiseAmt - skinNoiseAmt * 0.5;
    skinColor += skinNoise;
    skinColor *= grooveDarken;

    float ndl = saturate(dot(n, l));
    float wrap = 0.45;
    float wrapDiff = saturate((ndl + wrap) / (1.0 + wrap));

    float sssNL = saturate(dot(-n, l));
    float sss = sssNL * sssStr * bodyGrad;
    float3 sssColor = baseColor * float3(1.3, 0.5, 0.3);

    float3 diff = skinColor * (0.25 + 0.75 * wrapDiff) + sssColor * sss;

    float nv = saturate(dot(n, v));
    float3 h = normalize(l + v);
    float nh = saturate(dot(n, h));

    float alpha = rough * rough;
    float alpha2 = alpha * alpha;
    float denom = nh * nh * (alpha2 - 1.0) + 1.0;
    float D = alpha2 / (3.14159265 * denom * denom + 1e-5);
    float k = (rough + 1.0) * (rough + 1.0) / 8.0;
    float G1l = ndl / (ndl * (1.0 - k) + k + 1e-4);
    float G1v = nv / (nv * (1.0 - k) + k + 1e-4);
    float G = G1l * G1v;
    float F0 = 0.06;
    float F = F0 + (1.0 - F0) * pow(1.0 - saturate(dot(v, h)), 5.0);
    float spec = D * G * F / max(4.0 * ndl * nv, 0.001);
    float3 specColor = float3(0.95, 0.97, 1.0) * spec * specStr;

    float fresnel = pow(1.0 - nv, 3.5);
    float3 rim = float3(rimStr, rimStr * 1.5, rimStr * 1.25) * fresnel;

    float headDist = min(u, 1.0 - u);
    float eyeZone = smoothstep(0.06, 0.02, headDist);
    float eyeAngle = vCoord * 3.14159265 * 2.0;
    float eyeLeft = smoothstep(eyeSize, eyeSize * 0.33, abs(eyeAngle - 1.2));
    float eyeRight = smoothstep(eyeSize, eyeSize * 0.33, abs(eyeAngle - 5.08));
    float eyeDot = (eyeLeft + eyeRight) * eyeZone;
    float3 eyeColor = float3(0.02, 0.02, 0.02);

    float3 c = diff + specColor + rim;
    c = mix(c, eyeColor, eyeDot * 0.9);

    float pulse = sin(time * pulseSpeed + u * 8.0) * pulseAmp + 1.0;
    c *= pulse;

    return c;
}

fragment float4 ropeFragment(RopeOut in [[stage_in]],
                             constant FrameUniforms& frame [[buffer(1)]],
                             depth2d<float> shadowMap [[texture(2)]],
                             texture2d<float> noiseTex [[texture(3)]]) {
    float3 l = normalize(frame.lightDir_intensity.xyz);
    float lightI = frame.lightDir_intensity.w;
    float3 v = normalize(frame.cameraPos.xyz - in.worldPos);
    float3 n = normalize(in.normal);
    float taut = saturate(in.params.x);
    float pinch = saturate(in.params.y);
    float repel = saturate(in.params.z);
    float isWorm = in.params.w;

    float microBump = frame.ropeMatParams2.z;
    float contactAOStr = frame.ropeMatParams2.w;
    float liftGlowStr = frame.ropeMatParams3.x;
    float bumpScale = max(frame.ropeMatParams3.y, 0.5);

    float stretchDamp = 1.0 / (1.0 + taut * 2.5);

    constexpr sampler noiseSampler(filter::linear, address::repeat);
    float2 baseUV = in.uv * float2(bumpScale, bumpScale * 0.27);
    float2 ns1 = noiseTex.sample(noiseSampler, baseUV).rg;
    float2 ns2 = noiseTex.sample(noiseSampler, baseUV * 2.7 + float2(0.31, 0.73)).rg;
    float2 ns3 = noiseTex.sample(noiseSampler, baseUV * 5.3 + float2(0.67, 0.19)).rg;
    float n0 = ns1.r * 0.5 + ns2.r * 0.3 + ns3.r * 0.2;
    float n1 = ns1.g * 0.5 + ns2.g * 0.3 + ns3.g * 0.2;

    float3 tVec = cross(n, float3(0.0, 0.0, 1.0));
    if (length(tVec) < 1e-3) tVec = cross(n, float3(0.0, 1.0, 0.0));
    tVec = normalize(tVec);
    float3 bVec = normalize(cross(n, tVec));

    if (isWorm > 0.5) {
        float wormBumpAmp = 0.06;
        float wSegFreq = frame.wormParams3.w;
        float segBump = sin(in.uv.x * wSegFreq * 3.14159265 * 2.0);
        n = normalize(n + tVec * segBump * wormBumpAmp + (tVec * (n0 - 0.5) + bVec * (n1 - 0.5)) * 0.03);
    } else {
        float microAmp = (microBump + pinch * 0.12) * stretchDamp;
        n = normalize(n + (tVec * (n0 - 0.5) + bVec * (n1 - 0.5)) * microAmp);
    }

    float3 base = in.color;
    float microAO = (n0 + n1 - 1.0) * microBump * stretchDamp * 3.0;
    base *= 1.0 + microAO;
    base = mix(base, base * 1.3, pinch * 0.15);

    float roughNoise = noiseTex.sample(noiseSampler, baseUV * 1.7 + 0.37).r;
    float rough = roughNoise + pinch * 0.06 + repel * 0.04;

    float repelAO = smoothstep(0.0, 0.5, repel) * contactAOStr;
    float pinchAO = smoothstep(0.0, 0.3, pinch) * contactAOStr * 0.6;
    float contactAO = 1.0 - saturate(repelAO + pinchAO);

    float cartoonMode = frame.visualParams.z;
    int cartoonLevels = int(frame.visualParams.w);

    float3 c;
    if (isWorm > 0.5) {
        float time = frame.timeDrag.x;
        c = wormShading(base, n, l, v, in.worldPos, in.uv, time, frame) * lightI;
        c *= contactAO;

        float shadow = 1.0;
        if (shadowMap.get_width() > 0) {
            shadow = shadowVisibility(in.worldPos, n, frame, shadowMap);
        }
        shadow = pow(shadow, 2.0);
        float ambient = frame.lightingParams.x + 0.06;
        c *= mix(ambient, 1.0, shadow);
    } else if (cartoonMode > 0.5) {
        c = celRopeShading(in.color, normalize(in.normal), l, v, cartoonLevels, frame.cartoonParams);
    } else {
        c = rubberPBR(base, n, l, v, rough, taut, in.uv.y, cartoonMode, cartoonLevels,
                      frame.ropeMatParams, frame.ropeMatParams2,
                      frame.ropeMatParams3.z, frame.ropeMatParams3.w) * lightI;
        c *= contactAO;

        float liftGlow = saturate(in.worldPos.z / 0.35);
        c += float3(0.03, 0.04, 0.06) * liftGlow * liftGlowStr;

        float shadow = 1.0;
        if (shadowMap.get_width() > 0) {
            shadow = shadowVisibility(in.worldPos, n, frame, shadowMap);
        }
        shadow = pow(shadow, 2.0);
        float ambient = frame.lightingParams.x + 0.04 * taut;
        c *= mix(ambient, 1.0, shadow);
    }

    return float4(c, 1.0);
}

struct ShadowOut {
    float4 position [[position]];
};

vertex ShadowOut ropeShadowVertex(RopeIn in [[stage_in]],
                                  constant FrameUniforms& frame [[buffer(1)]]) {
    ShadowOut o;
    float time = frame.timeDrag.x;
    float energy = frame.timeDrag.y;
    float dragActive = frame.timeDrag.w;
    float u = in.uv.x;
    float pinch = in.params.y;
    float isWorm = in.params.w;
    float w = sin(u * 3.14159265);
    w = w * w;

    float3 displaced;
    if (isWorm > 0.5) {
        float crawlSpeed = frame.wormParams4.x;
        float crawlAmp = frame.wormParams4.y;
        float sideAmpP = frame.wormParams4.z;
        float crawlWave = sin(u * 12.0 - time * crawlSpeed) * crawlAmp * w;
        float sideWave = sin(u * 8.0 - time * crawlSpeed * 0.57) * sideAmpP * w;
        float3 nDir = normalize(in.normal);
        float3 sideDir = cross(nDir, float3(0, 0, 1));
        if (length(sideDir) < 1e-3) sideDir = cross(nDir, float3(0, 1, 0));
        sideDir = normalize(sideDir);
        displaced = in.position + nDir * crawlWave + sideDir * sideWave;
    } else {
        float amp = (0.002 + 0.010 * pinch) * energy * (0.25 + 0.75 * dragActive) * w;
        float wave = sin(u * 24.0 + time * 16.0) * 0.65 + sin(u * 11.0 - time * 9.0) * 0.35;
        displaced = in.position + normalize(in.normal) * (wave * amp);
    }

    o.position = frame.lightViewProj * float4(displaced, 1.0);
    return o;
}

vertex ShadowOut holeShadowVertex(const device HoleIn* vertices [[buffer(0)]],
                                  uint vid [[vertex_id]],
                                  uint iid [[instance_id]],
                                  constant FrameUniforms& frame [[buffer(1)]],
                                  const device HoleInstance* holes [[buffer(2)]]) {
    HoleInstance inst = holes[iid];
    float radius = inst.position_radius.w;
    float elevation = inst.position_radius.z;
    float3 lp = vertices[vid].position * radius;
    float3 wp = float3(inst.position_radius.x + lp.x, inst.position_radius.y + lp.y, elevation + lp.z);
    ShadowOut o;
    o.position = frame.lightViewProj * float4(wp, 1.0);
    return o;
}

// MARK: - Board (elevated surface) shaders

struct BoardIn {
    float3 position [[attribute(0)]];
    float3 normal   [[attribute(1)]];
    float2 worldXY  [[attribute(2)]];
};

struct BoardOut {
    float4 position [[position]];
    float3 worldPos;
    float3 normal;
    float2 worldXY;
    float elevation;
};

struct BoardFragOut {
    float4 color [[color(0)]];
    float depth [[depth(any)]];
};

vertex BoardOut boardVertex(const device BoardIn* vertices [[buffer(0)]],
                            uint vid [[vertex_id]],
                            constant FrameUniforms& frame [[buffer(1)]]) {
    BoardIn v = vertices[vid];
    BoardOut o;
    o.worldPos = v.position;
    o.position = frame.viewProj * float4(v.position, 1.0);
    o.normal = v.normal;
    o.worldXY = v.worldXY;
    o.elevation = v.position.z;
    return o;
}

fragment BoardFragOut boardFragment(BoardOut in [[stage_in]],
                              constant FrameUniforms& frame [[buffer(1)]],
                              constant HoleCountBuf& holeCounts [[buffer(3)]],
                              const device HoleInstance* holes [[buffer(4)]],
                              depth2d<float> shadowMap [[texture(2)]],
                              texture3d<float> boardWoodVolumeTex [[texture(3)]]) {
    constexpr sampler woodVolumeSampler(address::clamp_to_edge, filter::linear);

    float3 n = normalize(in.normal);
    bool isTopFace = n.z > 0.5;

    BoardFragOut out;
    float4 clipPos = frame.viewProj * float4(in.worldPos, 1.0);
    out.depth = clipPos.z / clipPos.w;

    if (isTopFace) {
        float sqHoles = frame.tableParams2.w;
        for (uint i = 0; i < holeCounts.count; i++) {
            float holeElev = holes[i].position_radius.z;
            if (abs(holeElev - in.elevation) > 0.05) continue;
            float2 holeCenter = holes[i].position_radius.xy;
            float holeR = holes[i].position_radius.w;
            float2 dd = abs(in.worldXY - holeCenter);
            float dist = sqHoles > 0.5 ? max(dd.x, dd.y) : length(in.worldXY - holeCenter);
            if (dist < holeR) {
                out.depth = 1.0;
                out.color = float4(0, 0, 0, 0);
                return out;
            }
        }
    }

    float3 l = normalize(frame.lightDir_intensity.xyz);
    float lightI = frame.lightDir_intensity.w;

    float levelSeed = frame.timeDrag.z;
    float3 baseColor;
    if (boardWoodVolumeTex.get_width() > 1) {
        float3 woodUVW = (in.worldPos - frame.woodBoundsMin.xyz) / (frame.woodBoundsMax.xyz - frame.woodBoundsMin.xyz);
        baseColor = boardWoodVolumeTex.sample(woodVolumeSampler, woodUVW).rgb;
    } else {
        baseColor = boardWoodTexture(in.worldXY, in.worldPos.z, levelSeed);
    }

    float ndl = saturate(dot(n, l));
    float ambient = frame.lightingParams.x;
    float shadow = shadowVisibility(in.worldPos, n, frame, shadowMap);
    float3 lit = baseColor * (ambient + ndl * lightI * shadow);

    float3 v = normalize(frame.cameraPos.xyz - in.worldPos);
    float3 h = normalize(l + v);
    float spec = pow(saturate(dot(n, h)), 60.0) * 0.2 * shadow;
    lit += spec;

    float edgeDarken = 1.0;
    if (!isTopFace) {
        edgeDarken = 0.65 + 0.35 * saturate(in.worldPos.z / max(in.elevation, 0.01));
    }
    lit *= edgeDarken;

    out.color = float4(lit, 1.0);
    return out;
}

vertex ShadowOut boardShadowVertex(const device BoardIn* vertices [[buffer(0)]],
                                   uint vid [[vertex_id]],
                                   constant FrameUniforms& frame [[buffer(1)]]) {
    ShadowOut o;
    o.position = frame.lightViewProj * float4(vertices[vid].position, 1.0);
    return o;
}

constant float2 poissonDisk[32] = {
    float2(-0.613392, 0.617481),
    float2(0.170019, -0.040254),
    float2(-0.299417, 0.791925),
    float2(0.645680, 0.493210),
    float2(-0.651784, 0.717887),
    float2(0.421003, 0.027070),
    float2(-0.817194, -0.271096),
    float2(-0.705374, -0.668203),
    float2(0.977050, -0.108615),
    float2(0.063326, 0.142369),
    float2(0.203528, 0.214331),
    float2(-0.667531, 0.326090),
    float2(-0.098422, -0.295755),
    float2(-0.885922, 0.215369),
    float2(0.566637, 0.605213),
    float2(0.039766, -0.396100),
    float2(0.308439, -0.723416),
    float2(-0.345912, -0.938257),
    float2(0.854412, 0.263352),
    float2(-0.367833, 0.440661),
    float2(0.234208, 0.887153),
    float2(-0.951050, -0.240556),
    float2(0.587940, -0.598885),
    float2(-0.102601, 0.515472),
    float2(0.798181, -0.179661),
    float2(-0.435220, -0.589435),
    float2(0.142256, -0.897236),
    float2(0.468750, 0.750000),
    float2(-0.750000, 0.468750),
    float2(0.750000, -0.468750),
    float2(-0.468750, -0.750000),
    float2(0.250000, 0.866025)
};

static float findBlocker(depth2d<float> shadowMap, float2 uv, float depthRef, float searchRadius) {
    constexpr sampler depthSampler(coord::normalized, address::clamp_to_edge, filter::linear);
    
    float blockerSum = 0.0;
    float blockerCount = 0.0;
    
    int sampleCount = 24;
    for (int i = 0; i < sampleCount; i++) {
        float2 offset = poissonDisk[i] * searchRadius;
        float2 suv = uv + offset;
        float sampleDepth = shadowMap.sample(depthSampler, suv);
        
        if (sampleDepth < depthRef) {
            blockerSum += sampleDepth;
            blockerCount += 1.0;
        }
    }
    
    if (blockerCount < 1.0) return -1.0;
    return blockerSum / blockerCount;
}

static float pcssFilter(depth2d<float> shadowMap, float2 uv, float depthRef, float filterRadius) {
    constexpr sampler shadowSampler(coord::normalized, address::clamp_to_edge, filter::linear, compare_func::less_equal);
    
    float sum = 0.0;
    
    int sampleCount = 32;
    for (int i = 0; i < sampleCount; i++) {
        float2 offset = poissonDisk[i] * filterRadius;
        float2 suv = uv + offset;
        sum += shadowMap.sample_compare(shadowSampler, suv, depthRef);
    }
    
    return sum / float(sampleCount);
}

static float shadowMapSample(depth2d<float> shadowMap, float2 uv, float depthRef) {
    constexpr sampler shadowSampler(coord::normalized, address::clamp_to_edge, filter::linear, compare_func::less_equal);
    return shadowMap.sample_compare(shadowSampler, uv, depthRef);
}

static float shadowVisibility(float3 worldPos, float3 worldN, constant FrameUniforms& frame, depth2d<float> shadowMap) {
    if (frame.lightingParams.w < 0.5) return 1.0;

    float4 lp = frame.lightViewProj * float4(worldPos, 1.0);
    float3 ndc = lp.xyz / max(1e-6, lp.w);
    float2 uv = float2(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) return 1.0;

    float biasBase = frame.orthoHalfSize_shadowBias.z;
    float ndl = saturate(dot(normalize(worldN), normalize(frame.lightDir_intensity.xyz)));
    float bias = biasBase + (1.0 - ndl) * biasBase * 2.2;

    float2 invSize = frame.shadowInvSize_unused.xy;

    float depthRef = ndc.z - bias;
    float shadowType = frame.orthoHalfSize_shadowBias.w;

    if (shadowType < 0.5) {
        float shadow = shadowMapSample(shadowMap, uv, depthRef);
        return shadow;
    }

    if (shadowType < 1.5) {
        float filterRadius = invSize.x * 4.0;
        float shadow = pcssFilter(shadowMap, uv, depthRef, filterRadius);
        shadow = smoothstep(0.0, 1.0, shadow);
        return shadow;
    }

    float lightSize = max(0.001, frame.lightingParams.z);
    float nearPlane = 0.01;
    float blockerSearchRadius = lightSize * (depthRef - nearPlane) / depthRef;
    blockerSearchRadius *= 0.65;
    blockerSearchRadius = clamp(blockerSearchRadius, invSize.x * 1.5, invSize.x * 10.0);

    float avgBlockerDepth = findBlocker(shadowMap, uv, depthRef, blockerSearchRadius);

    if (avgBlockerDepth < 0.0) {
        return 1.0;
    }

    float penumbraRadius = lightSize * (depthRef - avgBlockerDepth) / avgBlockerDepth;
    penumbraRadius = max(0.0001, penumbraRadius);

    float filterRadius = penumbraRadius * 1.4;
    filterRadius = clamp(filterRadius, invSize.x * 3.0, invSize.x * 20.0);

    float shadow = pcssFilter(shadowMap, uv, depthRef, filterRadius);

    shadow = smoothstep(0.0, 1.0, shadow);
    shadow = pow(shadow, 1.2);

    return shadow;
}

kernel void bloomThreshold(texture2d<float, access::read> src [[texture(0)]],
                           texture2d<float, access::write> dst [[texture(1)]],
                           uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= dst.get_width() || gid.y >= dst.get_height()) return;
    float2 uv = (float2(gid) + 0.5) / float2(dst.get_width(), dst.get_height());
    float2 suv = uv * float2(src.get_width(), src.get_height());
    uint2 sid = uint2(suv);
    sid.x = min(sid.x, src.get_width() - 1);
    sid.y = min(sid.y, src.get_height() - 1);
    float3 c = src.read(sid).xyz;
    float lum = dot(c, float3(0.2126, 0.7152, 0.0722));
    float t = smoothstep(0.35, 0.75, lum);
    float3 bright = c * t;
    bright += max(c - 0.8, float3(0.0));
    dst.write(float4(bright, 1.0), gid);
}

kernel void bloomBlurH(texture2d<float, access::read> src [[texture(0)]],
                       texture2d<float, access::write> dst [[texture(1)]],
                       uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= dst.get_width() || gid.y >= dst.get_height()) return;
    int2 p = int2(gid);
    int2 mx = int2(dst.get_width()-1, dst.get_height()-1);
    float4 acc = float4(0.0);
    float weights[] = { 0.1574, 0.1476, 0.1216, 0.0879, 0.0559, 0.0311, 0.0152, 0.0065, 0.0024 };
    acc += src.read(uint2(p)) * weights[0];
    for (int i = 1; i <= 8; i++) {
        acc += src.read(uint2(clamp(p + int2(i, 0), int2(0), mx))) * weights[i];
        acc += src.read(uint2(clamp(p - int2(i, 0), int2(0), mx))) * weights[i];
    }
    dst.write(acc, gid);
}

kernel void bloomBlurV(texture2d<float, access::read> src [[texture(0)]],
                       texture2d<float, access::write> dst [[texture(1)]],
                       uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= dst.get_width() || gid.y >= dst.get_height()) return;
    int2 p = int2(gid);
    int2 mx = int2(dst.get_width()-1, dst.get_height()-1);
    float4 acc = float4(0.0);
    float weights[] = { 0.1574, 0.1476, 0.1216, 0.0879, 0.0559, 0.0311, 0.0152, 0.0065, 0.0024 };
    acc += src.read(uint2(p)) * weights[0];
    for (int i = 1; i <= 8; i++) {
        acc += src.read(uint2(clamp(p + int2(0, i), int2(0), mx))) * weights[i];
        acc += src.read(uint2(clamp(p - int2(0, i), int2(0), mx))) * weights[i];
    }
    dst.write(acc, gid);
}

struct PostParams {
    float exposure;
    float bloomStrength;
    float cartoonEdgeStrength;
    float cartoonMode;
    float cartoonEdgeSmooth;
    float _pad0;
    float _pad1;
    float _pad2;
};

static float edgeDetect(float2 uv, depth2d<float> depthTex, float edgeSmooth) {
    constexpr sampler depthSmp(coord::normalized, address::clamp_to_edge, filter::linear);
    uint w = depthTex.get_width();
    uint h = depthTex.get_height();
    float spread = mix(1.0, 3.0, edgeSmooth);
    float2 dims = float2(w, h);
    float2 st = spread / dims;
    float2 c = uv;
    float d00 = depthTex.sample(depthSmp, c + float2(-1, -1) * st);
    float d10 = depthTex.sample(depthSmp, c + float2(0, -1) * st);
    float d20 = depthTex.sample(depthSmp, c + float2(1, -1) * st);
    float d01 = depthTex.sample(depthSmp, c + float2(-1, 0) * st);
    float d21 = depthTex.sample(depthSmp, c + float2(1, 0) * st);
    float d02 = depthTex.sample(depthSmp, c + float2(-1, 1) * st);
    float d12 = depthTex.sample(depthSmp, c + float2(0, 1) * st);
    float d22 = depthTex.sample(depthSmp, c + float2(1, 1) * st);
    float gx = -d00 - 2.0 * d01 - d02 + d20 + 2.0 * d21 + d22;
    float gy = -d00 - 2.0 * d10 - d20 + d02 + 2.0 * d12 + d22;
    float grad = sqrt(gx * gx + gy * gy);
    float threshold = mix(0.002, 0.0005, edgeSmooth);
    return smoothstep(threshold * 0.5, threshold * 1.5, grad);
}

fragment float4 postFragment(VSOut in [[stage_in]],
                             texture2d<float> hdr [[texture(0)]],
                             texture2d<float> bloom [[texture(1)]],
                             depth2d<float> depthTex [[texture(2)]],
                             constant PostParams& post [[buffer(0)]]) {
    constexpr sampler s(address::clamp_to_edge, filter::linear);
    float2 uv = in.uv;
    float3 c = hdr.sample(s, uv).xyz;
    float3 b = bloom.sample(s, uv).xyz;
    c += b * post.bloomStrength;
    float3 mapped = 1.0 - exp(-c * post.exposure);
    mapped = pow(saturate(mapped), float3(1.0 / 2.2));
    if (post.cartoonMode > 0.5) {
        mapped = floor(mapped * 32.0 + 0.5) / 32.0;

        if (depthTex.get_width() > 0 && post.cartoonEdgeStrength > 0.01) {
            float edge = edgeDetect(uv, depthTex, post.cartoonEdgeSmooth);
            float darken = 1.0 - post.cartoonEdgeStrength;
            mapped = mix(mapped, mapped * darken, edge);
        }
    }
    return float4(mapped, 1.0);
}

