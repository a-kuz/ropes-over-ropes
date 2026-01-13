#include <metal_stdlib>
using namespace metal;

struct VSOut {
    float4 position [[position]];
    float2 uv;
};

struct FrameUniforms {
    float4x4 viewProj;
    float4x4 lightViewProj;
    float4 lightDir_intensity;
    float4 ambientColor;
    float4 cameraPos;
    float4 orthoHalfSize_shadowBias;
    float4 shadowInvSize_unused;
    float4 timeDrag;
    float4 levelSeed;
};

static float shadowVisibility(float3 worldPos, float3 worldN, constant FrameUniforms& frame, depth2d<float> shadowMap);

static float hash21(float2 p) {
    float n = sin(dot(p, float2(127.1, 311.7)));
    return fract(n * 43758.5453123);
}

static float noise2d(float2 p) {
    float2 i = floor(p);
    float2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    
    float a = hash21(i);
    float b = hash21(i + float2(1.0, 0.0));
    float c = hash21(i + float2(0.0, 1.0));
    float d = hash21(i + float2(1.0, 1.0));
    
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

static float fbm2d(float2 p, int octaves) {
    float value = 0.0;
    float amplitude = 0.5;
    float frequency = 1.0;
    for (int i = 0; i < octaves; i++) {
        value += amplitude * noise2d(p * frequency);
        amplitude *= 0.5;
        frequency *= 2.0;
    }
    return value;
}

static float3 woodTexture(float2 uv, float2 worldXY, float seed) {
    float2 p = worldXY * 0.28;
    float seedOffset = seed * 137.5;
    
    float angle = atan2(p.y + seedOffset * 0.01, p.x + seedOffset * 0.007);
    float ringDist = length(p);
    
    float ringNoise = fbm2d(p * 3.2 + float2(angle * 0.5 + seedOffset, seedOffset * 0.3), 4);
    float ringWarp = ringNoise * 0.15;
    float warpedDist = ringDist + ringWarp;
    
    float ringPhase = warpedDist * 10.5 + fbm2d(p * 1.8 + seedOffset, 3) * 1.4;
    float rings = sin(ringPhase) * 0.5 + 0.5;
    rings = pow(rings, 2.2);
    
    float ringVariation = fbm2d(p * 4.5 + seedOffset * 0.5, 3);
    rings = mix(rings, rings * 0.7, ringVariation * 0.4);
    
    float densityNoise = fbm2d(p * 6.0 + seedOffset * 0.2, 4);
    float density = mix(0.85, 1.15, densityNoise);
    rings *= density;
    
    float grainDir = angle + ringDist * 0.3;
    float grainScale = 52.0;
    float grainNoise = fbm2d(float2(p.y * grainScale + seedOffset, p.x * 0.25 + grainDir * 2.0), 5);
    float grainPattern = sin(p.y * grainScale + grainNoise * 4.5) * 0.5 + 0.5;
    grainPattern = pow(grainPattern, 5.5);
    
    float grainIntensity = mix(0.65, 0.95, fbm2d(p * 8.0 + seedOffset, 3));
    grainPattern = grainPattern * grainIntensity;
    
    float medullaryRays = sin(angle * 8.0 + ringDist * 3.0 + seedOffset * 0.1) * 0.5 + 0.5;
    medullaryRays = pow(medullaryRays, 12.0) * 0.3;
    medullaryRays *= smoothstep(0.2, 1.5, ringDist);
    
    float hueShift = fract(seed * 0.618);
    float3 heartwoodDark = float3(0.38 + hueShift * 0.08, 0.24 - hueShift * 0.04, 0.16 + hueShift * 0.02);
    float3 heartwoodMid = float3(0.52 + hueShift * 0.06, 0.36 - hueShift * 0.03, 0.24 + hueShift * 0.03);
    float3 heartwoodLight = float3(0.68 + hueShift * 0.04, 0.52 - hueShift * 0.02, 0.38 + hueShift * 0.02);
    float3 sapwoodLight = float3(0.82 + hueShift * 0.02, 0.70 - hueShift * 0.01, 0.56 + hueShift * 0.01);
    
    float ringMask = rings;
    float3 heartwoodColor = mix(heartwoodDark, heartwoodMid, ringMask);
    heartwoodColor = mix(heartwoodColor, heartwoodLight, smoothstep(0.3, 0.7, rings));
    
    float sapwoodMask = smoothstep(0.0, 0.35, ringDist) * smoothstep(1.4, 0.95, ringDist);
    float3 woodColor = mix(heartwoodColor, sapwoodLight, sapwoodMask * 0.65);
    
    float grainEffect = grainPattern * 0.45;
    woodColor = mix(woodColor, heartwoodLight * 1.15, grainEffect);
    
    woodColor = mix(woodColor, woodColor * 1.12, medullaryRays);
    
    float colorVariation = fbm2d(p * 0.95 + seedOffset * 0.1, 4);
    woodColor *= mix(0.88, 1.12, colorVariation);
    
    float textureVariation = fbm2d(p * 2.3 + seedOffset * 0.15, 5);
    woodColor *= mix(0.92, 1.08, textureVariation);
    
    return saturate(woodColor);
}

static float3 marbleTexture(float2 uv, float2 worldXY, float seed) {
    float2 p = worldXY * 0.35 + seed * 17.3;
    
    float3 baseWhite = float3(0.94, 0.93, 0.91);
    float3 veinDark = float3(0.25, 0.28, 0.32);
    float3 veinMid = float3(0.55, 0.58, 0.62);
    
    float veinHue = fract(seed * 0.314);
    if (veinHue > 0.5) {
        veinDark = float3(0.35, 0.25, 0.22);
        veinMid = float3(0.65, 0.55, 0.50);
        baseWhite = float3(0.95, 0.92, 0.88);
    }
    
    float largeVein = fbm2d(p * 1.2 + float2(seed * 0.8, seed * 0.5), 4);
    float mediumVein = fbm2d(p * 3.5 + float2(seed * 1.2, seed * 0.7), 5);
    float fineVein = fbm2d(p * 8.0 + float2(seed * 0.4, seed * 0.9), 4);
    
    float veinPattern = largeVein * 0.6 + mediumVein * 0.3 + fineVein * 0.1;
    veinPattern = pow(abs(sin(veinPattern * 6.0 + p.x * 2.0 + p.y * 1.5)), 3.0);
    
    float crystalNoise = fbm2d(p * 25.0 + seed, 3);
    float sparkle = pow(crystalNoise, 8.0) * 0.15;
    
    float3 color = mix(baseWhite, veinMid, veinPattern * 0.6);
    color = mix(color, veinDark, pow(veinPattern, 2.0) * 0.4);
    color += float3(sparkle);
    
    float variation = fbm2d(p * 0.8 + seed * 2.1, 3);
    color *= mix(0.95, 1.05, variation);
    
    return saturate(color);
}

static float3 leatherTexture(float2 uv, float2 worldXY, float seed) {
    float2 p = worldXY * 0.6 + seed * 23.7;
    
    float hue = fract(seed * 0.437);
    float3 baseColor;
    if (hue < 0.25) {
        baseColor = float3(0.22, 0.14, 0.10);
    } else if (hue < 0.5) {
        baseColor = float3(0.12, 0.08, 0.06);
    } else if (hue < 0.75) {
        baseColor = float3(0.28, 0.22, 0.18);
    } else {
        baseColor = float3(0.18, 0.10, 0.08);
    }
    
    float grain = fbm2d(p * 18.0 + seed * 3.1, 5);
    float pores = fbm2d(p * 45.0 + seed * 7.2, 4);
    pores = pow(pores, 3.0);
    
    float creases = fbm2d(p * 2.5 + seed * 1.5, 4);
    creases = pow(abs(sin(creases * 8.0)), 4.0);
    
    float stitchPattern = 0.0;
    float stitchFreq = 12.0;
    float stitchY = fract(p.y * stitchFreq);
    float stitchX = fract(p.x * stitchFreq + floor(p.y * stitchFreq) * 0.5);
    if (abs(stitchY - 0.5) < 0.05 && abs(stitchX - 0.5) < 0.15) {
        stitchPattern = 0.08;
    }
    
    float3 color = baseColor;
    color *= mix(0.85, 1.15, grain);
    color -= float3(pores * 0.08);
    color *= mix(0.92, 1.0, 1.0 - creases * 0.3);
    color += float3(stitchPattern);
    
    return saturate(color);
}

static float3 brushedMetalTexture(float2 uv, float2 worldXY, float seed) {
    float2 p = worldXY * 0.4 + seed * 31.2;
    
    float metalType = fract(seed * 0.529);
    float3 baseColor;
    float3 highlightColor;
    if (metalType < 0.33) {
        baseColor = float3(0.72, 0.74, 0.78);
        highlightColor = float3(0.88, 0.90, 0.94);
    } else if (metalType < 0.66) {
        baseColor = float3(0.82, 0.68, 0.52);
        highlightColor = float3(0.95, 0.85, 0.72);
    } else {
        baseColor = float3(0.38, 0.42, 0.45);
        highlightColor = float3(0.55, 0.60, 0.65);
    }
    
    float brushAngle = fract(seed * 0.777) * 3.14159;
    float2 brushDir = float2(cos(brushAngle), sin(brushAngle));
    float brushCoord = dot(p, brushDir);
    
    float brushStrokes = 0.0;
    brushStrokes += sin(brushCoord * 120.0 + fbm2d(p * 2.0 + seed, 3) * 8.0) * 0.3;
    brushStrokes += sin(brushCoord * 280.0 + fbm2d(p * 5.0 + seed * 2.0, 2) * 4.0) * 0.2;
    brushStrokes += fbm2d(float2(brushCoord * 400.0, p.y * 2.0) + seed, 3) * 0.15;
    brushStrokes = brushStrokes * 0.5 + 0.5;
    
    float scratches = fbm2d(p * 35.0 + seed * 5.3, 4);
    scratches = pow(scratches, 4.0) * 0.12;
    
    float3 color = mix(baseColor, highlightColor, brushStrokes * 0.4);
    color += float3(scratches);
    
    float reflection = fbm2d(p * 0.5 + seed * 0.8, 3);
    color *= mix(0.92, 1.08, reflection);
    
    return saturate(color);
}

static float3 graniteTexture(float2 uv, float2 worldXY, float seed) {
    float2 p = worldXY * 0.45 + seed * 19.8;
    
    float grainType = fract(seed * 0.381);
    float3 baseGray, speckDark, speckLight, crystalColor;
    if (grainType < 0.33) {
        baseGray = float3(0.45, 0.45, 0.48);
        speckDark = float3(0.15, 0.15, 0.18);
        speckLight = float3(0.75, 0.75, 0.72);
        crystalColor = float3(0.65, 0.60, 0.55);
    } else if (grainType < 0.66) {
        baseGray = float3(0.55, 0.50, 0.48);
        speckDark = float3(0.25, 0.20, 0.18);
        speckLight = float3(0.85, 0.80, 0.75);
        crystalColor = float3(0.75, 0.65, 0.60);
    } else {
        baseGray = float3(0.38, 0.40, 0.42);
        speckDark = float3(0.12, 0.14, 0.16);
        speckLight = float3(0.68, 0.70, 0.72);
        crystalColor = float3(0.55, 0.58, 0.62);
    }
    
    float largeGrain = fbm2d(p * 3.5 + seed * 2.1, 4);
    float mediumGrain = fbm2d(p * 12.0 + seed * 4.3, 4);
    float fineGrain = fbm2d(p * 35.0 + seed * 8.7, 3);
    
    float3 color = baseGray;
    
    float darkSpeckMask = step(0.65, fineGrain) * step(mediumGrain, 0.45);
    color = mix(color, speckDark, darkSpeckMask * 0.8);
    
    float lightSpeckMask = step(0.7, mediumGrain) * step(0.55, fineGrain);
    color = mix(color, speckLight, lightSpeckMask * 0.6);
    
    float crystalMask = step(0.75, largeGrain) * step(0.6, mediumGrain);
    color = mix(color, crystalColor, crystalMask * 0.5);
    
    float sparkle = pow(fbm2d(p * 80.0 + seed * 12.0, 2), 10.0) * 0.2;
    color += float3(sparkle);
    
    float variation = fbm2d(p * 1.2 + seed * 0.9, 3);
    color *= mix(0.9, 1.1, variation);
    
    return saturate(color);
}

static float3 fabricTexture(float2 uv, float2 worldXY, float seed) {
    float2 p = worldXY * 0.5 + seed * 41.3;
    
    float colorChoice = fract(seed * 0.617);
    float3 warpColor, weftColor;
    if (colorChoice < 0.2) {
        warpColor = float3(0.15, 0.32, 0.22);
        weftColor = float3(0.12, 0.28, 0.18);
    } else if (colorChoice < 0.4) {
        warpColor = float3(0.35, 0.18, 0.15);
        weftColor = float3(0.30, 0.14, 0.12);
    } else if (colorChoice < 0.6) {
        warpColor = float3(0.18, 0.22, 0.38);
        weftColor = float3(0.14, 0.18, 0.32);
    } else if (colorChoice < 0.8) {
        warpColor = float3(0.28, 0.26, 0.22);
        weftColor = float3(0.24, 0.22, 0.18);
    } else {
        warpColor = float3(0.42, 0.38, 0.32);
        weftColor = float3(0.38, 0.34, 0.28);
    }
    
    float threadFreq = 60.0;
    float warp = sin(p.x * threadFreq + fbm2d(p * 5.0 + seed, 2) * 0.8) * 0.5 + 0.5;
    float weft = sin(p.y * threadFreq + fbm2d(p * 5.0 + seed * 2.0, 2) * 0.8) * 0.5 + 0.5;
    
    float weavePattern = warp * (1.0 - weft * 0.3) + weft * 0.3;
    
    float3 color = mix(warpColor, weftColor, weavePattern);
    
    float fuzz = fbm2d(p * 150.0 + seed * 15.0, 3) * 0.08;
    color += float3(fuzz);
    
    float lint = pow(fbm2d(p * 80.0 + seed * 20.0, 2), 6.0) * 0.1;
    color += float3(lint);
    
    float threadVar = fbm2d(p * 25.0 + seed * 3.0, 3);
    color *= mix(0.88, 1.12, threadVar);
    
    return saturate(color);
}

static float3 concreteTexture(float2 uv, float2 worldXY, float seed) {
    float2 p = worldXY * 0.38 + seed * 27.4;
    
    float toneChoice = fract(seed * 0.743);
    float3 baseColor;
    if (toneChoice < 0.33) {
        baseColor = float3(0.62, 0.60, 0.58);
    } else if (toneChoice < 0.66) {
        baseColor = float3(0.55, 0.54, 0.52);
    } else {
        baseColor = float3(0.68, 0.65, 0.62);
    }
    
    float surface = fbm2d(p * 8.0 + seed * 3.2, 5);
    float pebbles = fbm2d(p * 25.0 + seed * 7.1, 4);
    float sandGrain = fbm2d(p * 60.0 + seed * 11.3, 3);
    
    float cracks = fbm2d(p * 2.0 + seed * 1.5, 4);
    cracks = pow(abs(sin(cracks * 4.0 + p.x * 0.5)), 8.0) * 0.15;
    
    float3 color = baseColor;
    color *= mix(0.85, 1.15, surface);
    
    float pebbleMask = step(0.65, pebbles);
    float3 pebbleColor = baseColor * mix(0.7, 1.3, fbm2d(p * 40.0 + seed * 5.0, 2));
    color = mix(color, pebbleColor, pebbleMask * 0.4);
    
    color += float3(sandGrain * 0.06 - 0.03);
    color -= float3(cracks);
    
    return saturate(color);
}

static float3 getTableTexture(float2 uv, float2 worldXY, float seed) {
    int textureType = int(fract(seed * 0.618 + 0.1) * 7.0);
    
    if (textureType == 0) {
        return woodTexture(uv, worldXY, seed);
    } else if (textureType == 1) {
        return marbleTexture(uv, worldXY, seed);
    } else if (textureType == 2) {
        return leatherTexture(uv, worldXY, seed);
    } else if (textureType == 3) {
        return brushedMetalTexture(uv, worldXY, seed);
    } else if (textureType == 4) {
        return graniteTexture(uv, worldXY, seed);
    } else if (textureType == 5) {
        return fabricTexture(uv, worldXY, seed);
    } else {
        return concreteTexture(uv, worldXY, seed);
    }
}

struct LightParams {
    float3 direction;
    float intensity;
    float3 warmTint;
};

static LightParams getLightParams(float seed) {
    LightParams params;
    
    float angleVariation = fract(seed * 0.823) * 0.6 - 0.3;
    float heightVariation = fract(seed * 0.567) * 0.3 + 0.2;
    
    float baseAngle = -0.92 + angleVariation;
    float baseHeight = 0.35 + heightVariation;
    params.direction = normalize(float3(baseAngle, -0.18 + fract(seed * 0.432) * 0.2 - 0.1, baseHeight));
    
    params.intensity = 4.5 + fract(seed * 0.291) * 2.0;
    
    float warmth = fract(seed * 0.719);
    if (warmth < 0.33) {
        params.warmTint = float3(1.0, 0.95, 0.88);
    } else if (warmth < 0.66) {
        params.warmTint = float3(0.95, 0.98, 1.0);
    } else {
        params.warmTint = float3(1.0, 0.98, 0.92);
    }
    
    return params;
}

static float3 matteRubber(float3 baseColor, float3 n, float3 l, float3 v, float rough, float fiber) {
    float nl = saturate(dot(n, l));
    float nv = saturate(dot(n, v));
    float3 h = normalize(l + v);
    float nh = saturate(dot(n, h));

    float wrap = mix(0.32, 0.68, rough);
    float horizon = pow(1.0 - nv, 4.0);
    float fd90 = 0.4 + 2.4 * nh * nh * rough;
    float lightScatter = mix(1.0, fd90, pow(1.0 - nl, 5.0));
    float viewScatter = mix(1.0, fd90, pow(1.0 - nv, 5.0));
    float wrapTerm = saturate((nl + wrap) / (1.0 + wrap));
    float microShadow = saturate(nl * nv * 4.0);
    float coreDark = mix(0.18, 0.10, rough);
    float edgeLift = mix(0.70, 0.88, rough);
    float lobe = saturate((nl * 0.7 + nv * 0.3));
    float3 diff = baseColor * mix(coreDark, edgeLift, wrapTerm) * lightScatter * viewScatter;
    diff *= mix(0.82, 1.02, microShadow);
    diff *= mix(1.0, 0.84, horizon);
    float sheenMix = mix(0.25, 0.55, fiber);
    diff = mix(diff, diff * float3(1.05, 1.02, 0.98), sheenMix * pow(1.0 - nv, 2.5));

    float sheen = pow(1.0 - nh, 4.2) * (0.06 + 0.24 * fiber);
    float3 sheenCol = mix(baseColor, float3(1.0), 0.26) * sheen;

    float alpha = max(0.08, rough * rough);
    float alpha2 = alpha * alpha;
    float denom = nh * nh * (alpha2 - 1.0) + 1.0;
    float d = alpha2 / (3.14159265 * denom * denom + 1e-5);
    float k = alpha * 0.5 + 1e-4;
    float gl = nl / (nl * (1.0 - k) + k);
    float gv = nv / (nv * (1.0 - k) + k);
    float spec = d * gl * gv;
    float fres = pow(1.0 - nv, 5.0);
    float3 spTint = mix(float3(0.05, 0.05, 0.04), baseColor, 0.35);
    float grazeFres = pow(1.0 - nv, 2.5);
    float3 sp = spTint * spec * (0.06 + 0.36 * fiber) * (0.08 + 0.90 * fres);
    float forward = pow(saturate(dot(h, v)), 8.0) * (0.08 + 0.12 * fiber);
    sp += spTint * forward;

    float subsurface = (1.0 - nl) * (0.14 + 0.12 * (1.0 - rough));
    float rim = pow(1.0 - nv, 3.2) * (0.08 + 0.16 * fiber);
    float graze = pow(1.0 - nv, 3.6) * (0.06 + 0.12 * (1.0 - rough));
    diff *= 1.0 + subsurface;
    diff += baseColor * rim;
    diff += baseColor * graze;
    return diff + sheenCol + sp;
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

fragment float4 tableFragment(VSOut in [[stage_in]],
                              constant FrameUniforms& frame [[buffer(1)]],
                              depth2d<float> shadowMap [[texture(2)]]) {
    float2 uv = in.uv;
    
    float halfW = frame.orthoHalfSize_shadowBias.x;
    float halfH = frame.orthoHalfSize_shadowBias.y;
    float2 worldXY = float2((uv.x * 2.0 - 1.0) * halfW, (uv.y * 2.0 - 1.0) * halfH);
    float3 worldPos = float3(worldXY.x, worldXY.y, 0.0);
    float3 worldN = float3(0.0, 0.0, 1.0);
    
    float seed = frame.levelSeed.x;
    float3 baseColor = getTableTexture(uv, worldXY, seed);
    LightParams light = getLightParams(seed);
    
    float3 l = normalize(frame.lightDir_intensity.xyz);
    float3 v = normalize(frame.cameraPos.xyz - worldPos);
    float nl = saturate(dot(worldN, l));
    float nv = saturate(dot(worldN, v));
    float3 h = normalize(l + v);
    float nh = saturate(dot(worldN, h));
    
    float wrap = 0.4;
    float wrapTerm = saturate((nl + wrap) / (1.0 + wrap));
    float3 diff = baseColor * mix(0.25, 0.95, wrapTerm) * light.warmTint;
    
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
    float3 specColor = light.warmTint * spec * 0.12 * (0.2 + 0.8 * fresnel);
    
    float3 c = diff + specColor;
    
    float shadow = 1.0;
    if (shadowMap.get_width() > 0) {
        shadow = shadowVisibility(worldPos, worldN, frame, shadowMap);
    }
    shadow = pow(shadow, 2.0);
    c *= mix(0.22, 1.0, shadow);
    
    float ambient = 0.15;
    c += baseColor * ambient * light.warmTint;
    
    return float4(c, 1.0);
}

struct HoleIn {
    float3 position [[attribute(0)]];
    float3 normal [[attribute(1)]];
};

struct HoleOut {
    float4 position [[position]];
    float3 normal;
    float3 worldPos;
};

vertex HoleOut holeVertex(const device HoleIn* vertices [[buffer(0)]],
                          uint vid [[vertex_id]],
                          uint iid [[instance_id]],
                          constant FrameUniforms& frame [[buffer(1)]],
                          const device HoleInstance* holes [[buffer(2)]]) {
    HoleInstance inst = holes[iid];
    float radius = inst.position_radius.w;
    float3 lp = vertices[vid].position * radius;
    float3 wp = float3(inst.position_radius.x + lp.x, inst.position_radius.y + lp.y, lp.z);

    HoleOut o;
    o.worldPos = wp;
    o.position = frame.viewProj * float4(wp, 1.0);
    o.normal = normalize(vertices[vid].normal);
    return o;
}

fragment float4 holeFragment(HoleOut in [[stage_in]],
                             constant FrameUniforms& frame [[buffer(1)]],
                             depth2d<float> shadowMap [[texture(2)]]) {
    float3 n = normalize(in.normal);
    float3 l = normalize(frame.lightDir_intensity.xyz);
    float3 v = normalize(frame.cameraPos.xyz - in.worldPos);

    float3 topCol = float3(0.90, 0.92, 0.97);
    float3 wallCol = float3(0.70, 0.74, 0.82);
    float3 col = mix(wallCol, topCol, smoothstep(0.15, 0.65, n.z));

    float ndl = saturate(dot(n, l));
    float3 h = normalize(l + v);
    float ndh = saturate(dot(n, h));
    float spec = pow(ndh, 32.0) * 0.18;
    float3 lit = col * (0.25 + 0.75 * ndl) + float3(spec);

    float shadow = 1.0;
    if (shadowMap.get_width() > 0) {
        shadow = shadowVisibility(in.worldPos, n, frame, shadowMap);
    }
    shadow = pow(shadow, 1.8);
    lit *= mix(0.25, 1.0, shadow);
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

    float w = sin(u * 3.14159265);
    w = w * w;
    float amp = (0.002 + 0.010 * pinch) * energy * (0.25 + 0.75 * dragActive) * w;
    float wave = sin(u * 24.0 + time * 16.0) * 0.65 + sin(u * 11.0 - time * 9.0) * 0.35;
    float3 displaced = in.position + normalize(in.normal) * (wave * amp);

    o.worldPos = displaced;
    o.position = frame.viewProj * float4(displaced, 1.0);
    o.normal = in.normal;
    o.color = in.color;
    o.uv = in.uv;
    o.params = in.params;
    return o;
}

static float3 rubberShading(float3 baseColor, float3 n, float3 l, float3 v) {
    float ndl = saturate(dot(n, l));
    float3 h = normalize(l + v);
    float ndh = saturate(dot(n, h));

    float wrap = saturate((ndl + 0.48) / 1.48);
    float3 diff = baseColor * (0.18 + 0.82 * wrap);

    float nv = saturate(dot(n, v));
    float fres = pow(1.0 - nv, 6.0);

    float specPow = 2.2;
    float spec = pow(ndh, specPow) * 0.018;
    float3 sp = float3(spec) * (0.15 + 0.85 * fres);

    float subsurface = (1.0 - ndl) * 0.06;
    return diff * (1.0 + subsurface) + sp;
}

fragment float4 ropeFragment(RopeOut in [[stage_in]],
                             constant FrameUniforms& frame [[buffer(1)]],
                             depth2d<float> shadowMap [[texture(2)]]) {
    float3 l = normalize(frame.lightDir_intensity.xyz);
    float3 v = normalize(frame.cameraPos.xyz - in.worldPos);
    float3 n = normalize(in.normal);
    float taut = saturate(in.params.x);
    float pinch = saturate(in.params.y);
    float repel = saturate(in.params.z);

    float2 p = in.worldPos.xy * 42.0 + in.uv.x * 13.0;
    float n0 = hash21(p);
    float n1 = hash21(p.yx + 17.3);
    float roughNoise = hash21(p * 1.7 + in.uv.xy * 5.3);
    float rough = mix(0.28, 0.64, roughNoise);
    rough = saturate(rough - taut * 0.04 + pinch * 0.06 + repel * 0.03);
    float fiber = saturate(abs(n0 - n1) * 2.2 + pinch * 0.25);
    float3 t = cross(n, float3(0.0, 0.0, 1.0));
    if (length(t) < 1e-3) {
        t = cross(n, float3(0.0, 1.0, 0.0));
    }
    t = normalize(t);
    float3 b = normalize(cross(n, t));
    float microAmp = mix(0.05, 0.12, rough);
    float3 micro = (t * (n0 - 0.5) + b * (n1 - 0.5)) * microAmp;
    n = normalize(n + micro);

    float3 base = in.color;
    base = mix(base, float3(1.0), pinch * 0.22 + repel * 0.08);
    base *= 1.0 + pinch * 0.10;

    float ao = mix(0.78, 1.0, saturate(taut * 0.5 + (1.0 - pinch) * 0.3));
    float3 c = matteRubber(base, n, l, v, rough, fiber);

    float h = saturate(in.worldPos.z / 0.35);
    c += float3(0.08, 0.10, 0.16) * h * 0.6;
    c *= ao;

    float shadow = 1.0;
    if (shadowMap.get_width() > 0) {
        shadow = shadowVisibility(in.worldPos, n, frame, shadowMap);
    }
    shadow = pow(shadow, 1.85);
    float lift = 0.22 + 0.06 * taut;
    c *= mix(lift, 1.0, shadow);
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
    float w = sin(u * 3.14159265);
    w = w * w;
    float amp = (0.002 + 0.010 * pinch) * energy * (0.25 + 0.75 * dragActive) * w;
    float wave = sin(u * 24.0 + time * 16.0) * 0.65 + sin(u * 11.0 - time * 9.0) * 0.35;
    float3 displaced = in.position + normalize(in.normal) * (wave * amp);
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
    float3 lp = vertices[vid].position * radius;
    float3 wp = float3(inst.position_radius.x + lp.x, inst.position_radius.y + lp.y, lp.z);
    ShadowOut o;
    o.position = frame.lightViewProj * float4(wp, 1.0);
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

static float shadowVisibility(float3 worldPos, float3 worldN, constant FrameUniforms& frame, depth2d<float> shadowMap) {
    float4 lp = frame.lightViewProj * float4(worldPos, 1.0);
    float3 ndc = lp.xyz / max(1e-6, lp.w);
    float2 uv = float2(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) return 1.0;

    float biasBase = frame.orthoHalfSize_shadowBias.z;
    float ndl = saturate(dot(normalize(worldN), normalize(frame.lightDir_intensity.xyz)));
    float bias = biasBase + (1.0 - ndl) * biasBase * 2.2;

    float2 invSize = frame.shadowInvSize_unused.xy;

    float depthRef = ndc.z - bias;
    
    float lightSize = 0.045;
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
    shadow = pow(shadow, 0.75);
    
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
    float t = smoothstep(0.92, 1.25, lum);
    dst.write(float4(c * t, 1.0), gid);
}

kernel void bloomBlurH(texture2d<float, access::read> src [[texture(0)]],
                       texture2d<float, access::write> dst [[texture(1)]],
                       uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= dst.get_width() || gid.y >= dst.get_height()) return;
    int2 p = int2(gid);
    float4 acc = float4(0.0);
    float w0 = 0.227027;
    float w1 = 0.1945946;
    float w2 = 0.1216216;
    float w3 = 0.054054;
    float w4 = 0.016216;
    acc += src.read(uint2(p)) * w0;
    acc += src.read(uint2(clamp(p + int2(1,0), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w1;
    acc += src.read(uint2(clamp(p - int2(1,0), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w1;
    acc += src.read(uint2(clamp(p + int2(2,0), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w2;
    acc += src.read(uint2(clamp(p - int2(2,0), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w2;
    acc += src.read(uint2(clamp(p + int2(3,0), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w3;
    acc += src.read(uint2(clamp(p - int2(3,0), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w3;
    acc += src.read(uint2(clamp(p + int2(4,0), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w4;
    acc += src.read(uint2(clamp(p - int2(4,0), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w4;
    dst.write(acc, gid);
}

kernel void bloomBlurV(texture2d<float, access::read> src [[texture(0)]],
                       texture2d<float, access::write> dst [[texture(1)]],
                       uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= dst.get_width() || gid.y >= dst.get_height()) return;
    int2 p = int2(gid);
    float4 acc = float4(0.0);
    float w0 = 0.227027;
    float w1 = 0.1945946;
    float w2 = 0.1216216;
    float w3 = 0.054054;
    float w4 = 0.016216;
    acc += src.read(uint2(p)) * w0;
    acc += src.read(uint2(clamp(p + int2(0,1), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w1;
    acc += src.read(uint2(clamp(p - int2(0,1), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w1;
    acc += src.read(uint2(clamp(p + int2(0,2), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w2;
    acc += src.read(uint2(clamp(p - int2(0,2), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w2;
    acc += src.read(uint2(clamp(p + int2(0,3), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w3;
    acc += src.read(uint2(clamp(p - int2(0,3), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w3;
    acc += src.read(uint2(clamp(p + int2(0,4), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w4;
    acc += src.read(uint2(clamp(p - int2(0,4), int2(0), int2(dst.get_width()-1, dst.get_height()-1)))) * w4;
    dst.write(acc, gid);
}

fragment float4 postFragment(VSOut in [[stage_in]],
                             texture2d<float> hdr [[texture(0)]],
                             texture2d<float> bloom [[texture(1)]]) {
    constexpr sampler s(address::clamp_to_edge, filter::linear);
    float2 uv = in.uv;
    float3 c = hdr.sample(s, uv).xyz;
    float3 b = bloom.sample(s, uv).xyz;
    c += b * 0.18;
    float exposure = 1.35;
    float3 mapped = 1.0 - exp(-c * exposure);
    mapped = pow(saturate(mapped), float3(1.0 / 2.2));
    return float4(mapped, 1.0);
}

