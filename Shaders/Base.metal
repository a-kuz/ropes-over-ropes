#include <metal_stdlib>
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
    float seedAngle = seed * 2.399;
    float sa = sin(seedAngle);
    float ca = cos(seedAngle);
    float2 rotated = float2(worldXY.x * ca - worldXY.y * sa,
                            worldXY.x * sa + worldXY.y * ca);
    float2 offset = float2(hash21(float2(seed, seed * 7.13)) * 40.0 - 20.0,
                           hash21(float2(seed * 3.71, seed)) * 40.0 - 20.0);
    float2 p = (rotated + offset) * 1.1;

    float scaleVar = mix(0.8, 1.2, hash21(float2(seed * 1.23, 0.0)));
    p *= scaleVar;

    float angle = atan2(p.y, p.x);
    float ringDist = length(p);

    float ringNoise = fbm2d(p * 3.2 + float2(angle * 0.5, 0.0), 5);
    float ringWarp = ringNoise * 0.12;
    float warpedDist = ringDist + ringWarp;

    float ringDensity = mix(10.0, 18.0, hash21(float2(seed * 2.17, 1.0)));
    float ringPhase = warpedDist * ringDensity + fbm2d(p * 2.2, 4) * 1.2;
    float rings = sin(ringPhase) * 0.5 + 0.5;
    float ringSharp = mix(2.5, 4.0, hash21(float2(seed * 5.77, 3.0)));
    rings = pow(rings, ringSharp);

    float ringVariation = fbm2d(p * 5.5, 4);
    rings = mix(rings, rings * 0.65, ringVariation * 0.45);

    float densityNoise = fbm2d(p * 8.0, 5);
    float density = mix(0.88, 1.12, densityNoise);
    rings *= density;

    float grainDir = angle + ringDist * 0.3;
    float grainScale = mix(45.0, 80.0, hash21(float2(seed * 4.31, 2.0)));
    float grainNoise = fbm2d(float2(p.y * grainScale, p.x * 0.25 + grainDir * 2.0), 6);
    float grainPattern = sin(p.y * grainScale + grainNoise * 5.0) * 0.5 + 0.5;
    grainPattern = pow(grainPattern, 6.0);

    float grainIntensity = mix(0.6, 0.95, fbm2d(p * 10.0, 4));
    grainPattern *= grainIntensity;

    float fineGrain = fbm2d(p * 28.0, 4) * 0.08;

    float medullaryRays = sin(angle * 10.0 + ringDist * 4.0) * 0.5 + 0.5;
    medullaryRays = pow(medullaryRays, 14.0) * 0.25;
    medullaryRays *= smoothstep(0.15, 1.2, ringDist);

    float pores = hash21(floor(p * 120.0));
    pores = smoothstep(0.92, 0.98, pores) * 0.06;

    float hue = fract(seed * 0.618033988);
    float3 heartwoodDark, heartwoodMid, heartwoodLight, sapwoodLight;
    if (hue < 0.125) {
        // oak — classic warm
        heartwoodDark  = float3(0.38, 0.24, 0.16);
        heartwoodMid   = float3(0.52, 0.36, 0.24);
        heartwoodLight = float3(0.68, 0.52, 0.38);
        sapwoodLight   = float3(0.82, 0.70, 0.56);
    } else if (hue < 0.25) {
        // walnut — dark brown
        heartwoodDark  = float3(0.18, 0.12, 0.08);
        heartwoodMid   = float3(0.30, 0.20, 0.14);
        heartwoodLight = float3(0.42, 0.30, 0.22);
        sapwoodLight   = float3(0.55, 0.42, 0.32);
    } else if (hue < 0.375) {
        // maple — light blonde
        heartwoodDark  = float3(0.62, 0.52, 0.38);
        heartwoodMid   = float3(0.74, 0.64, 0.50);
        heartwoodLight = float3(0.84, 0.76, 0.62);
        sapwoodLight   = float3(0.92, 0.86, 0.74);
    } else if (hue < 0.5) {
        // wenge — very dark
        heartwoodDark  = float3(0.12, 0.08, 0.06);
        heartwoodMid   = float3(0.22, 0.15, 0.10);
        heartwoodLight = float3(0.32, 0.22, 0.16);
        sapwoodLight   = float3(0.44, 0.32, 0.24);
    } else if (hue < 0.625) {
        // cherry — reddish
        heartwoodDark  = float3(0.42, 0.20, 0.14);
        heartwoodMid   = float3(0.58, 0.32, 0.22);
        heartwoodLight = float3(0.72, 0.46, 0.32);
        sapwoodLight   = float3(0.85, 0.62, 0.48);
    } else if (hue < 0.75) {
        // ebony — near black
        heartwoodDark  = float3(0.06, 0.05, 0.04);
        heartwoodMid   = float3(0.14, 0.11, 0.08);
        heartwoodLight = float3(0.22, 0.17, 0.13);
        sapwoodLight   = float3(0.34, 0.26, 0.20);
    } else if (hue < 0.875) {
        // mahogany — deep red-brown
        heartwoodDark  = float3(0.26, 0.12, 0.08);
        heartwoodMid   = float3(0.40, 0.22, 0.14);
        heartwoodLight = float3(0.54, 0.32, 0.22);
        sapwoodLight   = float3(0.68, 0.48, 0.34);
    } else {
        // ash — pale grey-brown
        heartwoodDark  = float3(0.52, 0.46, 0.38);
        heartwoodMid   = float3(0.64, 0.58, 0.50);
        heartwoodLight = float3(0.76, 0.70, 0.62);
        sapwoodLight   = float3(0.88, 0.84, 0.76);
    }

    float ringMask = rings;
    float3 heartwoodColor = mix(heartwoodDark, heartwoodMid, ringMask);
    heartwoodColor = mix(heartwoodColor, heartwoodLight, smoothstep(0.25, 0.65, rings));

    float sapwoodMask = smoothstep(0.0, 0.3, ringDist) * smoothstep(1.4, 0.9, ringDist);
    float3 woodColor = mix(heartwoodColor, sapwoodLight, sapwoodMask * 0.6);

    float grainEffect = grainPattern * 0.4;
    woodColor = mix(woodColor, heartwoodLight * 1.1, grainEffect);
    woodColor += fineGrain;

    woodColor = mix(woodColor, woodColor * 1.1, medullaryRays);
    woodColor -= pores;

    float colorVariation = fbm2d(p * 1.2, 5);
    woodColor *= mix(0.9, 1.1, colorVariation);

    float textureVariation = fbm2d(p * 3.0, 6);
    woodColor *= mix(0.93, 1.07, textureVariation);

    return saturate(woodColor);
}

static float3 rubberPBR(float3 baseColor, float3 n, float3 l, float3 v,
                        float roughness, float taut, float vCoord) {
    float nl = saturate(dot(n, l));
    float nv = saturate(dot(n, v));
    float3 h = normalize(l + v);
    float nh = saturate(dot(n, h));
    float vh = saturate(dot(v, h));

    float radial = abs(vCoord - 0.5) * 2.0;
    float coreDarken = 1.0 - (1.0 - radial) * (1.0 - radial) * 0.25;
    float edgeBrighten = pow(radial, 2.5) * 0.12;
    float3 albedo = baseColor * coreDarken + float3(edgeBrighten);

    float wrap = 0.25;
    float wrapDiff = saturate((nl + wrap) / (1.0 + wrap));
    float3 diff = albedo * (0.28 + 0.72 * wrapDiff);

    float rough = mix(0.22, 0.42, roughness);
    rough = mix(rough, rough * 0.75, taut);
    float alpha = rough * rough;
    float alpha2 = alpha * alpha;
    float denom = nh * nh * (alpha2 - 1.0) + 1.0;
    float D = alpha2 / (3.14159265 * denom * denom + 1e-5);
    float k = (rough + 1.0) * (rough + 1.0) / 8.0;
    float G1l = nl / (nl * (1.0 - k) + k);
    float G1v = nv / (nv * (1.0 - k) + k);
    float G = G1l * G1v;
    float F0 = 0.04;
    float F = F0 + (1.0 - F0) * pow(1.0 - vh, 5.0);
    float spec = D * G * F / max(4.0 * nl * nv, 0.001);
    float specBoost = 1.0 + taut * 0.6;
    float3 specColor = float3(1.0) * spec * 0.65 * specBoost;

    float rim = pow(1.0 - nv, 4.0) * 0.08;
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
    float _pad;
};

kernel void bakeWoodKernel(texture2d<float, access::write> dst [[texture(0)]],
                           constant BakeWoodParams& params [[buffer(0)]],
                           uint2 gid [[thread_position_in_grid]]) {
    uint w = dst.get_width();
    uint h = dst.get_height();
    if (gid.x >= w || gid.y >= h) return;

    float2 uv = (float2(gid) + 0.5) / float2(w, h);
    float2 worldXY = mix(params.worldMin, params.worldMax, uv);

    float3 col = woodTexture(uv, worldXY, params.seed);
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

    for (uint i = 0; i < holeCounts.count; i++) {
        float2 holeCenter = holes[i].position_radius.xy;
        float holeR = holes[i].position_radius.w * 0.76;
        float dist = length(worldXY - holeCenter);
        if (dist < holeR) {
            tableDepth = 1.0;
            break;
        }
    }

    float3 worldN = float3(0.0, 0.0, 1.0);

    float3 baseColor;
    if (woodTex.get_width() > 1) {
        float2 woodUV = (worldXY - frame.woodBoundsMin.xy) / (frame.woodBoundsMax.xy - frame.woodBoundsMin.xy);
        baseColor = woodTex.sample(woodSampler, woodUV).rgb;
    } else {
        float levelSeed = frame.timeDrag.z;
        baseColor = woodTexture(uv, worldXY, levelSeed);
    }

    float3 l = normalize(frame.lightDir_intensity.xyz);
    float3 v = normalize(frame.cameraPos.xyz - worldPos);
    float nl = saturate(dot(worldN, l));
    float nv = saturate(dot(worldN, v));
    float3 h = normalize(l + v);
    float nh = saturate(dot(worldN, h));

    float wrap = 0.4;
    float wrapTerm = saturate((nl + wrap) / (1.0 + wrap));
    float3 diff = baseColor * mix(0.25, 0.95, wrapTerm);

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
    float3 specColor = float3(0.9, 0.85, 0.75) * spec * 0.12 * (0.2 + 0.8 * fresnel);

    float3 c = diff + specColor;

    float shadow = 1.0;
    if (shadowMap.get_width() > 0) {
        shadow = shadowVisibility(worldPos, worldN, frame, shadowMap);
    }
    shadow = pow(shadow, 2.0);
    c *= mix(0.22, 1.0, shadow);

    float ambient = 0.15;
    c += baseColor * ambient;

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
    float3 lp = vertices[vid].position * radius;
    float3 wp = float3(inst.position_radius.x + lp.x, inst.position_radius.y + lp.y, lp.z);

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

    float levelSeed = frame.timeDrag.z;
    float matSeed = hash21(float2(levelSeed * 1.37, 0.0));

    float3 topCol, wallCol;
    float specPower, specStrength;

    if (matSeed < 0.2) {
        topCol  = float3(0.90, 0.92, 0.97);
        wallCol = float3(0.70, 0.74, 0.82);
        specPower = 48.0; specStrength = 0.25;
    } else if (matSeed < 0.4) {
        topCol  = float3(0.78, 0.68, 0.48);
        wallCol = float3(0.55, 0.45, 0.30);
        specPower = 64.0; specStrength = 0.35;
    } else if (matSeed < 0.6) {
        topCol  = float3(0.35, 0.35, 0.38);
        wallCol = float3(0.20, 0.20, 0.22);
        specPower = 80.0; specStrength = 0.40;
    } else if (matSeed < 0.8) {
        topCol  = float3(0.92, 0.82, 0.55);
        wallCol = float3(0.70, 0.58, 0.32);
        specPower = 96.0; specStrength = 0.45;
    } else {
        topCol  = float3(0.72, 0.50, 0.42);
        wallCol = float3(0.48, 0.30, 0.24);
        specPower = 56.0; specStrength = 0.30;
    }

    float perHole = hash21(float2(in.holeId * 3.17, levelSeed * 2.31));
    topCol  *= mix(0.92, 1.08, perHole);
    wallCol *= mix(0.92, 1.08, perHole);

    float3 col = mix(wallCol, topCol, smoothstep(0.15, 0.65, n.z));

    if (in.highlight > 0.5) {
        float3 hlCol = float3(0.55, 0.85, 1.0);
        col = mix(col, hlCol, 0.6);
    }

    float ndl = saturate(dot(n, l));
    float3 h = normalize(l + v);
    float ndh = saturate(dot(n, h));
    float nv = saturate(dot(n, v));
    float fresnel = pow(1.0 - nv, 4.0);
    float spec = pow(ndh, specPower) * specStrength * (0.3 + 0.7 * fresnel);
    float3 specCol = mix(col, float3(1.0), 0.5) * spec;
    float3 lit = col * (0.22 + 0.78 * ndl) + specCol;

    if (in.highlight > 0.5) {
        lit += float3(0.08, 0.15, 0.22);
    }

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

fragment float4 ropeFragment(RopeOut in [[stage_in]],
                             constant FrameUniforms& frame [[buffer(1)]],
                             depth2d<float> shadowMap [[texture(2)]]) {
    float3 l = normalize(frame.lightDir_intensity.xyz);
    float3 v = normalize(frame.cameraPos.xyz - in.worldPos);
    float3 n = normalize(in.normal);
    float taut = saturate(in.params.x);
    float pinch = saturate(in.params.y);
    float repel = saturate(in.params.z);

    float2 noiseP = in.worldPos.xy * 48.0 + in.uv.x * 11.0;
    float n0 = hash21(noiseP);
    float n1 = hash21(noiseP.yx + 17.3);
    float3 tVec = cross(n, float3(0.0, 0.0, 1.0));
    if (length(tVec) < 1e-3) tVec = cross(n, float3(0.0, 1.0, 0.0));
    tVec = normalize(tVec);
    float3 bVec = normalize(cross(n, tVec));
    float microAmp = 0.03 + pinch * 0.04;
    n = normalize(n + (tVec * (n0 - 0.5) + bVec * (n1 - 0.5)) * microAmp);

    float3 base = in.color;
    base = mix(base, base * 1.3, pinch * 0.15);

    float roughNoise = hash21(noiseP * 1.7 + in.uv.xy * 5.3);
    float rough = mix(0.08, 0.22, roughNoise);
    rough += pinch * 0.08 + repel * 0.05;

    float contactAO = 1.0 - repel * 0.35 - pinch * 0.15;

    float3 c = rubberPBR(base, n, l, v, rough, taut, in.uv.y);
    c *= contactAO;

    float liftGlow = saturate(in.worldPos.z / 0.35);
    c += float3(0.04, 0.05, 0.08) * liftGlow * 0.4;

    float shadow = 1.0;
    if (shadowMap.get_width() > 0) {
        shadow = shadowVisibility(in.worldPos, n, frame, shadowMap);
    }
    shadow = pow(shadow, 1.6);
    float ambient = 0.18 + 0.05 * taut;
    c *= mix(ambient, 1.0, shadow);

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
    float exposure = 1.05;
    float3 mapped = 1.0 - exp(-c * exposure);
    mapped = pow(saturate(mapped), float3(1.0 / 2.2));
    return float4(mapped, 1.0);
}

