#version 300 es
precision highp float;
precision highp sampler2DShadow;

layout(std140) uniform FrameBlock {
    mat4 uViewProj;
    mat4 uInvViewProj;
    mat4 uLightViewProj;
    vec4 uLightDirIntensity;
    vec4 uAmbientColor;
    vec4 uCameraPos;
    vec4 uOrthoHalfSizeShadowBias;
    vec4 uShadowInvSize;
    vec4 uTimeDrag;
    vec4 uWoodBoundsMin;
    vec4 uWoodBoundsMax;
    vec4 uHoleTint;
    vec4 uVisualParams;
    vec4 uLightingParams;
    vec4 uTableParams;
    vec4 uTableParams2;
    vec4 uRopeMatParams;
    vec4 uRopeMatParams2;
    vec4 uRopeMatParams3;
    vec4 uCartoonParams;
    vec4 uWormParams1;
    vec4 uWormParams2;
    vec4 uWormParams3;
    vec4 uWormParams4;
    vec4 uRopeMatParams4;
};

uniform sampler2DShadow uShadowMap;

in vec3 vNormal;
in vec3 vWorldPos;
in float vHighlight;
in float vHoleId;

out vec4 fragColor;

// ---------- Poisson Disk (32 samples) ----------
const vec2 poissonDisk[32] = vec2[32](
    vec2(-0.613392, 0.617481),
    vec2( 0.170019,-0.040254),
    vec2(-0.299417, 0.791925),
    vec2( 0.645680, 0.493210),
    vec2(-0.651784, 0.717887),
    vec2( 0.421003, 0.027070),
    vec2(-0.817194,-0.271096),
    vec2(-0.705374,-0.668203),
    vec2( 0.977050,-0.108615),
    vec2( 0.063326, 0.142369),
    vec2( 0.203528, 0.214331),
    vec2(-0.667531, 0.326090),
    vec2(-0.098422,-0.295755),
    vec2(-0.885922, 0.215369),
    vec2( 0.566637, 0.605213),
    vec2( 0.039766,-0.396100),
    vec2( 0.308439,-0.723416),
    vec2(-0.345912,-0.938257),
    vec2( 0.854412, 0.263352),
    vec2(-0.367833, 0.440661),
    vec2( 0.234208, 0.887153),
    vec2(-0.951050,-0.240556),
    vec2( 0.587940,-0.598885),
    vec2(-0.102601, 0.515472),
    vec2( 0.798181,-0.179661),
    vec2(-0.435220,-0.589435),
    vec2( 0.142256,-0.897236),
    vec2( 0.468750, 0.750000),
    vec2(-0.750000, 0.468750),
    vec2( 0.750000,-0.468750),
    vec2(-0.468750,-0.750000),
    vec2( 0.250000, 0.866025)
);

float shadowMapSample(vec2 uv, float depthRef) {
    return texture(uShadowMap, vec3(uv, depthRef));
}

float pcssFilter(vec2 uv, float depthRef, float filterRadius) {
    float sum = 0.0;
    for (int i = 0; i < 32; i++) {
        vec2 offset = poissonDisk[i] * filterRadius;
        sum += texture(uShadowMap, vec3(uv + offset, depthRef));
    }
    return sum / 32.0;
}

float findBlocker(vec2 uv, float depthRef, float searchRadius) {
    float blockerSum = 0.0;
    float blockerCount = 0.0;
    for (int i = 0; i < 24; i++) {
        vec2 offset = poissonDisk[i] * searchRadius;
        float cmp = texture(uShadowMap, vec3(uv + offset, depthRef));
        if (cmp < 0.5) {
            blockerSum += depthRef - 0.005;
            blockerCount += 1.0;
        }
    }
    if (blockerCount < 1.0) return -1.0;
    return blockerSum / blockerCount;
}

float shadowVisibility(vec3 worldPos, vec3 worldN) {
    if (uLightingParams.w < 0.5) return 1.0;

    vec4 lp = uLightViewProj * vec4(worldPos, 1.0);
    vec3 ndc = lp.xyz / max(1e-6, lp.w);
    vec2 uv = vec2(ndc.x * 0.5 + 0.5, ndc.y * 0.5 + 0.5);
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) return 1.0;

    float biasBase = uOrthoHalfSizeShadowBias.z;
    float ndl = clamp(dot(normalize(worldN), normalize(uLightDirIntensity.xyz)), 0.0, 1.0);
    float bias = biasBase + (1.0 - ndl) * biasBase * 2.2;
    vec2 invSize = uShadowInvSize.xy;
    float depthRef = ndc.z * 0.5 + 0.5 - bias;
    float shadowType = uOrthoHalfSizeShadowBias.w;

    if (shadowType < 0.5) {
        return shadowMapSample(uv, depthRef);
    }
    if (shadowType < 1.5) {
        float filterRadius = invSize.x * 4.0;
        float shadow = pcssFilter(uv, depthRef, filterRadius);
        return smoothstep(0.0, 1.0, shadow);
    }

    float lightSize = max(0.001, uLightingParams.z);
    float nearPlane = 0.01;
    float blockerSearchRadius = lightSize * (depthRef - nearPlane) / depthRef;
    blockerSearchRadius *= 0.65;
    blockerSearchRadius = clamp(blockerSearchRadius, invSize.x * 1.5, invSize.x * 10.0);
    float avgBlockerDepth = findBlocker(uv, depthRef, blockerSearchRadius);
    if (avgBlockerDepth < 0.0) return 1.0;
    float penumbraRadius = lightSize * (depthRef - avgBlockerDepth) / avgBlockerDepth;
    penumbraRadius = max(0.0001, penumbraRadius);
    float filterRadius = penumbraRadius * 1.4;
    filterRadius = clamp(filterRadius, invSize.x * 3.0, invSize.x * 20.0);
    float shadow = pcssFilter(uv, depthRef, filterRadius);
    shadow = smoothstep(0.0, 1.0, shadow);
    shadow = pow(shadow, 1.2);
    return shadow;
}

// ---------- Cel shading ----------
float celStep(float ndl, int levels, float shadowBright) {
    if (levels <= 1) return ndl > 0.0 ? 1.0 : shadowBright;
    float n = float(levels);
    float bucket = floor(ndl * n);
    bucket = clamp(bucket, 0.0, n - 1.0);
    float t = bucket / (n - 1.0);
    return mix(shadowBright, 1.0, t);
}

vec3 celHoleShading(vec3 baseColor, vec3 n, vec3 l, int levels, vec4 cp) {
    float shadowBright = cp.x;
    float wrap = cp.y;
    float ndl = dot(n, l);
    float lit = celStep(clamp((ndl + wrap) / (1.0 + wrap), 0.0, 1.0), levels, shadowBright);
    return baseColor * lit;
}

void main() {
    vec3 n = normalize(vNormal);
    vec3 l = normalize(uLightDirIntensity.xyz);
    vec3 v = normalize(uCameraPos.xyz - vWorldPos);

    vec3 baseCol = vec3(0.12, 0.13, 0.15);
    vec3 topCol  = vec3(0.18, 0.19, 0.22);
    float specPower    = 120.0;
    float specStrength = 0.55;

    vec3 col = mix(baseCol, topCol, smoothstep(0.15, 0.75, n.z));
    vec3 tint    = uHoleTint.xyz;
    float tintAmt = uHoleTint.w;
    col = mix(col, col * tint, tintAmt);

    if (vHighlight > 0.5) {
        vec3 hlCol = vec3(0.55, 0.85, 1.0);
        col = mix(col, hlCol, 0.6);
    }

    float lightI = uLightDirIntensity.w;
    float cartoonMode   = uVisualParams.z;
    int   cartoonLevels = int(uVisualParams.w);

    vec3 lit;
    if (cartoonMode > 0.5) {
        lit = celHoleShading(col, n, l, cartoonLevels, uCartoonParams);
    } else {
        float ndl = clamp(dot(n, l), 0.0, 1.0);
        vec3 h  = normalize(l + v);
        float ndh = clamp(dot(n, h), 0.0, 1.0);
        float nv  = clamp(dot(n, v), 0.0, 1.0);
        float fresnel = pow(1.0 - nv, 4.0);
        float spec = pow(ndh, specPower) * specStrength * (0.3 + 0.7 * fresnel);
        vec3 specCol = mix(col, vec3(1.0), 0.5) * spec * lightI;
        lit = col * (0.22 + 0.78 * ndl) * lightI + specCol;

        float shadow = shadowVisibility(vWorldPos, n);
        shadow = pow(shadow, 2.0);
        float shadowDark = uLightingParams.y;
        lit *= mix(shadowDark, 1.0, shadow);
    }

    if (vHighlight > 0.5) {
        lit += vec3(0.08, 0.15, 0.22);
    }

    fragColor = vec4(lit, 1.0);
}
