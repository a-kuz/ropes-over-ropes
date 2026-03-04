#version 300 es
precision highp float;
precision highp sampler2D;
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
};

uniform sampler2DShadow uShadowMap;  // binding 2
uniform sampler2D uNoiseTex;         // binding 3

in vec3 vNormal;
in vec3 vColor;
in vec3 vWorldPos;
in vec2 vUV;
in vec4 vParams;

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

// ---------- Shadow helpers ----------
float shadowMapSample(vec2 uv, float depthRef) {
    return texture(uShadowMap, vec3(uv, depthRef));
}

float findBlocker(vec2 uv, float depthRef, float searchRadius) {
    float blockerSum = 0.0;
    float blockerCount = 0.0;
    // Use the shadow map as a regular texture for depth reads
    // In GLES 3.0 we can read via textureProj or texelFetch; for simplicity
    // we sample with compare and infer blocker from comparison results.
    // A more precise approach reads from a separate depth texture.
    // Here we approximate: sample with slight offsets and collect depth.
    // NOTE: For a true 1:1 port we need a non-comparison sampler for blocker search.
    // We'll use the comparison sampler and estimate blocker depth from depthRef.
    int sampleCount = 24;
    for (int i = 0; i < sampleCount; i++) {
        vec2 offset = poissonDisk[i] * searchRadius;
        vec2 suv = uv + offset;
        float cmp = texture(uShadowMap, vec3(suv, depthRef));
        if (cmp < 0.5) { // sample is in shadow => blocker exists
            blockerSum += depthRef - 0.005; // approximate blocker depth
            blockerCount += 1.0;
        }
    }
    if (blockerCount < 1.0) return -1.0;
    return blockerSum / blockerCount;
}

float pcssFilter(vec2 uv, float depthRef, float filterRadius) {
    float sum = 0.0;
    int sampleCount = 32;
    for (int i = 0; i < sampleCount; i++) {
        vec2 offset = poissonDisk[i] * filterRadius;
        vec2 suv = uv + offset;
        sum += texture(uShadowMap, vec3(suv, depthRef));
    }
    return sum / float(sampleCount);
}

float shadowVisibility(vec3 worldPos, vec3 worldN) {
    if (uLightingParams.w < 0.5) return 1.0;

    vec4 lp = uLightViewProj * vec4(worldPos, 1.0);
    vec3 ndc = lp.xyz / max(1e-6, lp.w);
    // OpenGL: no Y-flip needed (Metal flips Y, OpenGL doesn't)
    vec2 uv = vec2(ndc.x * 0.5 + 0.5, ndc.y * 0.5 + 0.5);
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) return 1.0;

    float biasBase = uOrthoHalfSizeShadowBias.z;
    float ndl = clamp(dot(normalize(worldN), normalize(uLightDirIntensity.xyz)), 0.0, 1.0);
    float bias = biasBase + (1.0 - ndl) * biasBase * 2.2;

    vec2 invSize = uShadowInvSize.xy;

    // OpenGL NDC Z is [-1,1], depth buffer stores [0,1]. Convert:
    float depthRef = ndc.z * 0.5 + 0.5 - bias;
    float shadowType = uOrthoHalfSizeShadowBias.w;

    if (shadowType < 0.5) {
        return shadowMapSample(uv, depthRef);
    }

    if (shadowType < 1.5) {
        float filterRadius = invSize.x * 4.0;
        float shadow = pcssFilter(uv, depthRef, filterRadius);
        shadow = smoothstep(0.0, 1.0, shadow);
        return shadow;
    }

    // PCSS
    float lightSize = max(0.001, uLightingParams.z);
    float nearPlane = 0.01;
    float blockerSearchRadius = lightSize * (depthRef - nearPlane) / depthRef;
    blockerSearchRadius *= 0.65;
    blockerSearchRadius = clamp(blockerSearchRadius, invSize.x * 1.5, invSize.x * 10.0);

    float avgBlockerDepth = findBlocker(uv, depthRef, blockerSearchRadius);

    if (avgBlockerDepth < 0.0) {
        return 1.0;
    }

    float penumbraRadius = lightSize * (depthRef - avgBlockerDepth) / avgBlockerDepth;
    penumbraRadius = max(0.0001, penumbraRadius);

    float filterRadius = penumbraRadius * 1.4;
    filterRadius = clamp(filterRadius, invSize.x * 3.0, invSize.x * 20.0);

    float shadow = pcssFilter(uv, depthRef, filterRadius);

    shadow = smoothstep(0.0, 1.0, shadow);
    shadow = pow(shadow, 1.2);

    return shadow;
}

// ---------- Toon / Cel helpers ----------
float toonStep(float nl, int levels) {
    if (levels <= 1) return 1.0;
    float n = float(levels - 1);
    return floor(nl * n + 0.5) / n;
}

float celStep(float ndl, int levels, float shadowBright) {
    if (levels <= 1) return ndl > 0.0 ? 1.0 : shadowBright;
    float n = float(levels);
    float bucket = floor(ndl * n);
    bucket = clamp(bucket, 0.0, n - 1.0);
    float t = bucket / (n - 1.0);
    return mix(shadowBright, 1.0, t);
}

vec3 celRopeShading(vec3 baseColor, vec3 n, vec3 l, vec3 v, int levels, vec4 cp) {
    float shadowBright = cp.x;
    float wrap = cp.y;
    float ndl = dot(n, l);
    float lit = celStep(clamp((ndl + wrap) / (1.0 + wrap), 0.0, 1.0), levels, shadowBright);
    return baseColor * lit;
}

// ---------- Rubber PBR ----------
vec3 rubberPBR(vec3 baseColor, vec3 n, vec3 l, vec3 v,
               float roughness, float taut, float vCoord, float cartoonMode, int cartoonLevels,
               vec4 matP, vec4 matP2, float stretchGloss, float stretchSpec) {
    float matteAmount = matP.x;
    float glossAmount = matP.y;
    float diffuseWrap = matP.z;
    float subsurface  = matP.w;
    float edgeLight   = matP2.x;
    float saturation  = matP2.y;

    float nl = clamp(dot(n, l), 0.0, 1.0);
    float nv = clamp(dot(n, v), 0.0, 1.0);
    vec3 h  = normalize(l + v);
    float nh = clamp(dot(n, h), 0.0, 1.0);
    float vh = clamp(dot(v, h), 0.0, 1.0);

    float radial = abs(vCoord - 0.5) * 2.0;
    float coreDarken   = 1.0 - (1.0 - radial) * (1.0 - radial) * mix(0.15, 0.45, matteAmount);
    float edgeBrighten = pow(radial, 2.5) * mix(0.15, 0.02, matteAmount);
    vec3 albedo = baseColor * coreDarken + vec3(edgeBrighten);

    float grey = dot(albedo, vec3(0.299, 0.587, 0.114));
    albedo = mix(vec3(grey), albedo, saturation);

    float wrapDiff = clamp((nl + diffuseWrap) / (1.0 + diffuseWrap), 0.0, 1.0);
    if (cartoonMode > 0.5) wrapDiff = toonStep(wrapDiff, cartoonLevels);

    float sssNL     = clamp(dot(-n, l), 0.0, 1.0);
    float sssWrap   = clamp((sssNL + 0.3) / 1.3, 0.0, 1.0);
    float sssContrib = sssWrap * subsurface * 0.3;

    float ambientBase = mix(0.20, 0.45, matteAmount);
    vec3 diff = albedo * (ambientBase + (1.0 - ambientBase) * wrapDiff + sssContrib);

    float rough = mix(0.18, 0.92, matteAmount) + roughness * 0.1;
    float taut2 = taut * taut;
    float roughFloor = mix(0.85, 0.25, taut) * stretchGloss + 0.85 * (1.0 - stretchGloss);
    rough = mix(rough, rough * roughFloor, taut);
    rough = clamp(rough, 0.05, 0.99);
    float alpha  = rough * rough;
    float alpha2 = alpha * alpha;
    float denom  = nh * nh * (alpha2 - 1.0) + 1.0;
    float D      = alpha2 / (3.14159265 * denom * denom + 1e-5);
    float k      = (rough + 1.0) * (rough + 1.0) / 8.0;
    float G1l    = nl / (nl * (1.0 - k) + k);
    float G1v    = nv / (nv * (1.0 - k) + k);
    float G      = G1l * G1v;
    float F0base = mix(0.08, 0.01, matteAmount);
    float F0     = F0base + taut2 * 0.12 * stretchSpec;
    float F      = F0 + (1.0 - F0) * pow(1.0 - vh, 5.0);
    float spec   = D * G * F / max(4.0 * nl * nv, 0.001);
    float specBoost = 1.0 + (taut * 1.5 + taut2 * 3.0) * stretchSpec;
    float specIntensity = glossAmount * mix(1.2, 0.08, matteAmount);
    vec3 specColor = vec3(1.0) * spec * specIntensity * specBoost;
    if (cartoonMode > 0.5) specColor *= 0.3;

    float rimPow = mix(3.0, 6.0, matteAmount) - taut * 1.5 * stretchGloss;
    rimPow = max(1.5, rimPow);
    float rim = pow(1.0 - nv, rimPow) * edgeLight * (1.0 + taut2 * 2.0 * stretchSpec);
    diff += albedo * rim;

    return diff + specColor;
}

// ---------- Worm shading ----------
float wc_hash21(vec2 p) {
    float n = sin(dot(p, vec2(127.1, 311.7)));
    return fract(n * 43758.5453123);
}

vec3 wormShading(vec3 baseColor, vec3 n, vec3 l, vec3 v, vec3 worldPos,
                 vec2 uv, float time) {
    float u = uv.x;
    float vCoord = uv.y;

    float grooveDepth  = uWormParams1.x;
    float bellyBright  = uWormParams1.y;
    float backDark     = uWormParams1.z;
    float skinNoiseAmt = uWormParams1.w;
    float sssStr       = uWormParams2.x;
    float rough        = uWormParams2.y;
    float specStr      = uWormParams2.z;
    float rimStr       = uWormParams2.w;
    float eyeSize      = uWormParams3.x;
    float pulseSpeed   = uWormParams3.y;
    float pulseAmp     = uWormParams3.z;
    float segFreq      = uWormParams3.w;

    float segPhase   = u * segFreq * 3.14159265 * 2.0;
    float segGroove  = smoothstep(0.85, 1.0, abs(sin(segPhase)));
    float grooveDarken = 1.0 - segGroove * grooveDepth;

    float bodyGrad = sin(u * 3.14159265);
    vec3 bellyColor = baseColor * vec3(bellyBright, bellyBright * 0.957, bellyBright * 0.87);
    vec3 backColor  = baseColor * vec3(backDark, backDark * 1.07, backDark);
    float bellySide = smoothstep(0.3, 0.7, vCoord);
    vec3 skinColor  = mix(bellyColor, backColor, bellySide);

    float skinNoise = wc_hash21(worldPos.xy * 80.0 + u * 5.0) * skinNoiseAmt - skinNoiseAmt * 0.5;
    skinColor += skinNoise;
    skinColor *= grooveDarken;

    float ndl  = clamp(dot(n, l), 0.0, 1.0);
    float wrap = 0.45;
    float wrapDiff = clamp((ndl + wrap) / (1.0 + wrap), 0.0, 1.0);

    float sssNL   = clamp(dot(-n, l), 0.0, 1.0);
    float sss     = sssNL * sssStr * bodyGrad;
    vec3 sssColor = baseColor * vec3(1.3, 0.5, 0.3);

    vec3 diff = skinColor * (0.25 + 0.75 * wrapDiff) + sssColor * sss;

    float nv = clamp(dot(n, v), 0.0, 1.0);
    vec3 h   = normalize(l + v);
    float nh = clamp(dot(n, h), 0.0, 1.0);

    float alpha  = rough * rough;
    float alpha2 = alpha * alpha;
    float denom  = nh * nh * (alpha2 - 1.0) + 1.0;
    float D      = alpha2 / (3.14159265 * denom * denom + 1e-5);
    float k      = (rough + 1.0) * (rough + 1.0) / 8.0;
    float G1l    = ndl / (ndl * (1.0 - k) + k + 1e-4);
    float G1v    = nv  / (nv  * (1.0 - k) + k + 1e-4);
    float G      = G1l * G1v;
    float F0     = 0.06;
    float F      = F0 + (1.0 - F0) * pow(1.0 - clamp(dot(v, h), 0.0, 1.0), 5.0);
    float spec   = D * G * F / max(4.0 * ndl * nv, 0.001);
    vec3 specColor = vec3(0.95, 0.97, 1.0) * spec * specStr;

    float fresnel = pow(1.0 - nv, 3.5);
    vec3 rim = vec3(rimStr, rimStr * 1.5, rimStr * 1.25) * fresnel;

    float headDist = min(u, 1.0 - u);
    float eyeZone  = smoothstep(0.06, 0.02, headDist);
    float eyeAngle = vCoord * 3.14159265 * 2.0;
    float eyeLeft  = smoothstep(eyeSize, eyeSize * 0.33, abs(eyeAngle - 1.2));
    float eyeRight = smoothstep(eyeSize, eyeSize * 0.33, abs(eyeAngle - 5.08));
    float eyeDot   = (eyeLeft + eyeRight) * eyeZone;
    vec3 eyeColor  = vec3(0.02, 0.02, 0.02);

    vec3 c = diff + specColor + rim;
    c = mix(c, eyeColor, eyeDot * 0.9);

    float pulse = sin(time * pulseSpeed + u * 8.0) * pulseAmp + 1.0;
    c *= pulse;

    return c;
}

// ---------- Main ----------
void main() {
    vec3 l = normalize(uLightDirIntensity.xyz);
    float lightI = uLightDirIntensity.w;
    vec3 v = normalize(uCameraPos.xyz - vWorldPos);
    vec3 n = normalize(vNormal);
    float taut  = clamp(vParams.x, 0.0, 1.0);
    float pinch = clamp(vParams.y, 0.0, 1.0);
    float repel = clamp(vParams.z, 0.0, 1.0);
    float isWorm = vParams.w;

    float microBump    = uRopeMatParams2.z;
    float contactAOStr = uRopeMatParams2.w;
    float liftGlowStr  = uRopeMatParams3.x;
    float bumpScale    = max(uRopeMatParams3.y, 0.5);

    float stretchDamp = 1.0 / (1.0 + taut * 2.5);

    vec2 baseUV = vUV * vec2(bumpScale, bumpScale * 0.27);
    // Sample noise texture; if not bound, texture() returns (0,0,0,1).
    // Detect missing texture by checking if all three samples sum to ~0 and use procedural fallback.
    vec2 ns1 = texture(uNoiseTex, baseUV).rg;
    vec2 ns2 = texture(uNoiseTex, baseUV * 2.7 + vec2(0.31, 0.73)).rg;
    vec2 ns3 = texture(uNoiseTex, baseUV * 5.3 + vec2(0.67, 0.19)).rg;
    float noiseSum = ns1.r + ns1.g + ns2.r + ns2.g + ns3.r + ns3.g;
    if (noiseSum < 0.001) {
        // Procedural noise fallback (wc_hash21-based)
        ns1 = vec2(wc_hash21(baseUV * 73.17), wc_hash21(baseUV * 91.31 + vec2(1.37, 2.79)));
        ns2 = vec2(wc_hash21(baseUV * 197.3 + vec2(0.31, 0.73)), wc_hash21(baseUV * 211.7 + vec2(1.68, 3.52)));
        ns3 = vec2(wc_hash21(baseUV * 371.9 + vec2(0.67, 0.19)), wc_hash21(baseUV * 401.3 + vec2(2.34, 0.86)));
    }
    float n0 = ns1.r * 0.5 + ns2.r * 0.3 + ns3.r * 0.2;
    float n1 = ns1.g * 0.5 + ns2.g * 0.3 + ns3.g * 0.2;

    vec3 tVec = cross(n, vec3(0.0, 0.0, 1.0));
    if (length(tVec) < 1e-3) tVec = cross(n, vec3(0.0, 1.0, 0.0));
    tVec = normalize(tVec);
    vec3 bVec = normalize(cross(n, tVec));

    if (isWorm > 0.5) {
        float wormBumpAmp = 0.06;
        float wSegFreq = uWormParams3.w;
        float segBump = sin(vUV.x * wSegFreq * 3.14159265 * 2.0);
        n = normalize(n + tVec * segBump * wormBumpAmp + (tVec * (n0 - 0.5) + bVec * (n1 - 0.5)) * 0.03);
    } else {
        float microAmp = (microBump + pinch * 0.12) * stretchDamp;
        n = normalize(n + (tVec * (n0 - 0.5) + bVec * (n1 - 0.5)) * microAmp);
    }

    vec3 base = vColor;
    float microAO = (n0 + n1 - 1.0) * microBump * stretchDamp * 3.0;
    base *= 1.0 + microAO;
    base = mix(base, base * 1.3, pinch * 0.15);

    float roughNoise = texture(uNoiseTex, baseUV * 1.7 + 0.37).r;
    if (roughNoise < 0.001 && noiseSum < 0.001) {
        roughNoise = wc_hash21(baseUV * 137.9 + vec2(0.37));
    }
    float rough = roughNoise + pinch * 0.06 + repel * 0.04;

    float repelAO  = smoothstep(0.0, 0.5, repel) * contactAOStr;
    float pinchAO  = smoothstep(0.0, 0.3, pinch) * contactAOStr * 0.6;
    float contactAO = 1.0 - clamp(repelAO + pinchAO, 0.0, 1.0);

    float cartoonMode   = uVisualParams.z;
    int   cartoonLevels = int(uVisualParams.w);

    vec3 c;
    if (isWorm > 0.5) {
        float time = uTimeDrag.x;
        c = wormShading(base, n, l, v, vWorldPos, vUV, time) * lightI;
        c *= contactAO;

        float shadow = shadowVisibility(vWorldPos, n);
        shadow = pow(shadow, 2.0);
        float ambient = uLightingParams.x + 0.06;
        c *= mix(ambient, 1.0, shadow);
    } else if (cartoonMode > 0.5) {
        c = celRopeShading(vColor, normalize(vNormal), l, v, cartoonLevels, uCartoonParams);
    } else {
        c = rubberPBR(base, n, l, v, rough, taut, vUV.y, cartoonMode, cartoonLevels,
                      uRopeMatParams, uRopeMatParams2,
                      uRopeMatParams3.z, uRopeMatParams3.w) * lightI;
        c *= contactAO;

        float liftGlow = clamp(vWorldPos.z / 0.35, 0.0, 1.0);
        c += vec3(0.03, 0.04, 0.06) * liftGlow * liftGlowStr;

        float shadow = shadowVisibility(vWorldPos, n);
        shadow = pow(shadow, 2.0);
        float ambient = uLightingParams.x + 0.04 * taut;
        c *= mix(ambient, 1.0, shadow);
    }

    fragColor = vec4(c, 1.0);
}
