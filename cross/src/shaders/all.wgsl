// ── Uniforms & types ──

struct FrameUniforms {
    viewProj: mat4x4<f32>,
    invViewProj: mat4x4<f32>,
    lightViewProj: mat4x4<f32>,
    lightDir_intensity: vec4<f32>,
    ambientColor: vec4<f32>,
    cameraPos: vec4<f32>,
    orthoHalfSize_shadowBias: vec4<f32>,
    shadowInvSize_unused: vec4<f32>,
    timeDrag: vec4<f32>,
};

struct HoleInstance {
    position_radius: vec4<f32>,
};

struct HoleInstanceArray {
    data: array<HoleInstance>,
};

struct ShadowSegment {
    ax: f32, ay: f32, az: f32, radius: f32,
    bx: f32, by: f32, bz: f32, rope_id: f32,
};

struct ShadowSegmentArray {
    count: u32,
    _p0: u32, _p1: u32, _p2: u32,
    data: array<ShadowSegment>,
};

// ── Bindings ──

@group(0) @binding(0) var<uniform> frame: FrameUniforms;
@group(0) @binding(1) var shadow_map: texture_depth_2d;
@group(0) @binding(2) var shadow_sampler: sampler_comparison;
@group(0) @binding(3) var depth_sampler: sampler;

@group(1) @binding(0) var<storage, read> hole_instances: HoleInstanceArray;
@group(1) @binding(1) var<storage, read> shadow_segments: ShadowSegmentArray;

@group(2) @binding(0) var hdr_texture: texture_2d<f32>;
@group(2) @binding(1) var bloom_texture: texture_2d<f32>;
@group(2) @binding(2) var linear_sampler: sampler;

@group(3) @binding(0) var bloom_src: texture_2d<f32>;
@group(3) @binding(1) var bloom_dst: texture_storage_2d<rgba16float, write>;

// ── Poisson disk ──

const POISSON_DISK: array<vec2<f32>, 32> = array<vec2<f32>, 32>(
    vec2<f32>(-0.613392, 0.617481),
    vec2<f32>(0.170019, -0.040254),
    vec2<f32>(-0.299417, 0.791925),
    vec2<f32>(0.645680, 0.493210),
    vec2<f32>(-0.651784, 0.717887),
    vec2<f32>(0.421003, 0.027070),
    vec2<f32>(-0.817194, -0.271096),
    vec2<f32>(-0.705374, -0.668203),
    vec2<f32>(0.977050, -0.108615),
    vec2<f32>(0.063326, 0.142369),
    vec2<f32>(0.203528, 0.214331),
    vec2<f32>(-0.667531, 0.326090),
    vec2<f32>(-0.098422, -0.295755),
    vec2<f32>(-0.885922, 0.215369),
    vec2<f32>(0.566637, 0.605213),
    vec2<f32>(0.039766, -0.396100),
    vec2<f32>(0.308439, -0.723416),
    vec2<f32>(-0.345912, -0.938257),
    vec2<f32>(0.854412, 0.263352),
    vec2<f32>(-0.367833, 0.440661),
    vec2<f32>(0.234208, 0.887153),
    vec2<f32>(-0.951050, -0.240556),
    vec2<f32>(0.587940, -0.598885),
    vec2<f32>(-0.102601, 0.515472),
    vec2<f32>(0.798181, -0.179661),
    vec2<f32>(-0.435220, -0.589435),
    vec2<f32>(0.142256, -0.897236),
    vec2<f32>(0.468750, 0.750000),
    vec2<f32>(-0.750000, 0.468750),
    vec2<f32>(0.750000, -0.468750),
    vec2<f32>(-0.468750, -0.750000),
    vec2<f32>(0.250000, 0.866025),
);

// ── Noise ──

fn hash21(p: vec2<f32>) -> f32 {
    let n = sin(dot(p, vec2<f32>(127.1, 311.7)));
    return fract(n * 43758.5453123);
}

fn noise2d(p: vec2<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let ff = f * f * (3.0 - 2.0 * f);
    let a = hash21(i);
    let b = hash21(i + vec2<f32>(1.0, 0.0));
    let c = hash21(i + vec2<f32>(0.0, 1.0));
    let d = hash21(i + vec2<f32>(1.0, 1.0));
    return mix(mix(a, b, ff.x), mix(c, d, ff.x), ff.y);
}

fn fbm2d(p: vec2<f32>, octaves: i32) -> f32 {
    var value = 0.0;
    var amplitude = 0.5;
    var frequency = 1.0;
    for (var i = 0; i < octaves; i = i + 1) {
        value += amplitude * noise2d(p * frequency);
        amplitude *= 0.5;
        frequency *= 2.0;
    }
    return value;
}

// ── Wood texture ──

fn woodTexture(uv: vec2<f32>, worldXY: vec2<f32>) -> vec3<f32> {
    let p = worldXY * 0.28;

    let angle = atan2(p.y, p.x);
    let ringDist = length(p);

    let ringNoise = fbm2d(p * 3.2 + vec2<f32>(angle * 0.5, 0.0), 4);
    let ringWarp = ringNoise * 0.15;
    let warpedDist = ringDist + ringWarp;

    let ringPhase = warpedDist * 10.5 + fbm2d(p * 1.8, 3) * 1.4;
    var rings = sin(ringPhase) * 0.5 + 0.5;
    rings = pow(rings, 2.2);

    let ringVariation = fbm2d(p * 4.5, 3);
    rings = mix(rings, rings * 0.7, ringVariation * 0.4);

    let densityNoise = fbm2d(p * 6.0, 4);
    let density = mix(0.85, 1.15, densityNoise);
    rings *= density;

    let grainDir = angle + ringDist * 0.3;
    let grainScale = 52.0;
    let grainNoise = fbm2d(vec2<f32>(p.y * grainScale, p.x * 0.25 + grainDir * 2.0), 5);
    var grainPattern = sin(p.y * grainScale + grainNoise * 4.5) * 0.5 + 0.5;
    grainPattern = pow(grainPattern, 5.5);

    let grainIntensity = mix(0.65, 0.95, fbm2d(p * 8.0, 3));
    grainPattern = grainPattern * grainIntensity;

    var medullaryRays = sin(angle * 8.0 + ringDist * 3.0) * 0.5 + 0.5;
    medullaryRays = pow(medullaryRays, 12.0) * 0.3;
    medullaryRays *= smoothstep(0.2, 1.5, ringDist);

    let heartwoodDark = vec3<f32>(0.38, 0.24, 0.16);
    let heartwoodMid = vec3<f32>(0.52, 0.36, 0.24);
    let heartwoodLight = vec3<f32>(0.68, 0.52, 0.38);
    let sapwoodLight = vec3<f32>(0.82, 0.70, 0.56);

    let ringMask = rings;
    var heartwoodColor = mix(heartwoodDark, heartwoodMid, vec3<f32>(ringMask));
    heartwoodColor = mix(heartwoodColor, heartwoodLight, vec3<f32>(smoothstep(0.3, 0.7, rings)));

    let sapwoodMask = smoothstep(0.0, 0.35, ringDist) * smoothstep(1.4, 0.95, ringDist);
    var woodColor = mix(heartwoodColor, sapwoodLight, vec3<f32>(sapwoodMask * 0.65));

    let grainEffect = grainPattern * 0.45;
    woodColor = mix(woodColor, heartwoodLight * 1.15, vec3<f32>(grainEffect));

    woodColor = mix(woodColor, woodColor * 1.12, vec3<f32>(medullaryRays));

    let colorVariation = fbm2d(p * 0.95, 4);
    woodColor *= mix(0.88, 1.12, colorVariation);

    let textureVariation = fbm2d(p * 2.3, 5);
    woodColor *= mix(0.92, 1.08, textureVariation);

    let finalColor = clamp(woodColor, vec3<f32>(0.0), vec3<f32>(1.0));
    return finalColor;
}

fn woodTextureVariant(uv: vec2<f32>, worldXY: vec2<f32>, seed: f32) -> vec3<f32> {
    let seedAngle = seed * 2.399;
    let sa = sin(seedAngle);
    let ca = cos(seedAngle);
    let rotated = vec2<f32>(worldXY.x * ca - worldXY.y * sa,
                            worldXY.x * sa + worldXY.y * ca);
    let offset = vec2<f32>(hash21(vec2<f32>(seed, seed * 7.13)) * 40.0 - 20.0,
                           hash21(vec2<f32>(seed * 3.71, seed)) * 40.0 - 20.0);
    var p = (rotated + offset) * 1.1;
    let scaleVar = mix(0.8, 1.2, hash21(vec2<f32>(seed * 1.23, 0.0)));
    p *= scaleVar;

    let angle = atan2(p.y, p.x);
    let ringDist = length(p);

    let ringNoise = fbm2d(p * 3.2 + vec2<f32>(angle * 0.5, 0.0), 3);
    let ringWarp = ringNoise * 0.12;
    let warpedDist = ringDist + ringWarp;

    let ringDensity = mix(10.0, 18.0, hash21(vec2<f32>(seed * 2.17, 1.0)));
    let ringPhase = warpedDist * ringDensity + fbm2d(p * 2.2, 3) * 1.2;
    var rings = sin(ringPhase) * 0.5 + 0.5;
    let ringSharp = mix(2.5, 4.0, hash21(vec2<f32>(seed * 5.77, 3.0)));
    rings = pow(rings, ringSharp);

    let ringVariation = fbm2d(p * 5.5, 3);
    rings = mix(rings, rings * 0.65, ringVariation * 0.45);

    let densityNoise = fbm2d(p * 8.0, 3);
    let density = mix(0.88, 1.12, densityNoise);
    rings *= density;

    let grainDir = angle + ringDist * 0.3;
    let grainScale = mix(45.0, 80.0, hash21(vec2<f32>(seed * 4.31, 2.0)));
    let grainNoise = fbm2d(vec2<f32>(p.y * grainScale, p.x * 0.25 + grainDir * 2.0), 3);
    var grainPattern = sin(p.y * grainScale + grainNoise * 5.0) * 0.5 + 0.5;
    grainPattern = pow(grainPattern, 6.0);

    let grainIntensity = mix(0.6, 0.95, fbm2d(p * 10.0, 3));
    grainPattern *= grainIntensity;

    let fineGrain = fbm2d(p * 28.0, 2) * 0.08;

    var medullaryRays = sin(angle * 10.0 + ringDist * 4.0) * 0.5 + 0.5;
    medullaryRays = pow(medullaryRays, 14.0) * 0.25;
    medullaryRays *= smoothstep(0.15, 1.2, ringDist);

    let pores = smoothstep(0.92, 0.98, hash21(floor(p * 120.0))) * 0.06;

    let hue = fract(seed * 0.618033988);

    var dark: vec3<f32>;
    var mid: vec3<f32>;
    var light: vec3<f32>;
    var sap: vec3<f32>;

    if (hue < 0.125) {
        dark  = vec3<f32>(0.38, 0.24, 0.16);
        mid   = vec3<f32>(0.52, 0.36, 0.24);
        light = vec3<f32>(0.68, 0.52, 0.38);
        sap   = vec3<f32>(0.82, 0.70, 0.56);
    } else if (hue < 0.25) {
        dark  = vec3<f32>(0.18, 0.12, 0.08);
        mid   = vec3<f32>(0.30, 0.20, 0.14);
        light = vec3<f32>(0.42, 0.30, 0.22);
        sap   = vec3<f32>(0.55, 0.42, 0.32);
    } else if (hue < 0.375) {
        dark  = vec3<f32>(0.62, 0.52, 0.38);
        mid   = vec3<f32>(0.74, 0.64, 0.50);
        light = vec3<f32>(0.84, 0.76, 0.62);
        sap   = vec3<f32>(0.92, 0.86, 0.74);
    } else if (hue < 0.5) {
        dark  = vec3<f32>(0.12, 0.08, 0.06);
        mid   = vec3<f32>(0.22, 0.15, 0.10);
        light = vec3<f32>(0.32, 0.22, 0.16);
        sap   = vec3<f32>(0.44, 0.32, 0.24);
    } else if (hue < 0.625) {
        dark  = vec3<f32>(0.42, 0.20, 0.14);
        mid   = vec3<f32>(0.58, 0.32, 0.22);
        light = vec3<f32>(0.72, 0.46, 0.32);
        sap   = vec3<f32>(0.85, 0.62, 0.48);
    } else if (hue < 0.75) {
        dark  = vec3<f32>(0.06, 0.05, 0.04);
        mid   = vec3<f32>(0.14, 0.11, 0.08);
        light = vec3<f32>(0.22, 0.17, 0.13);
        sap   = vec3<f32>(0.34, 0.26, 0.20);
    } else if (hue < 0.875) {
        dark  = vec3<f32>(0.26, 0.12, 0.08);
        mid   = vec3<f32>(0.40, 0.22, 0.14);
        light = vec3<f32>(0.54, 0.32, 0.22);
        sap   = vec3<f32>(0.68, 0.48, 0.34);
    } else {
        dark  = vec3<f32>(0.52, 0.46, 0.38);
        mid   = vec3<f32>(0.64, 0.58, 0.50);
        light = vec3<f32>(0.76, 0.70, 0.62);
        sap   = vec3<f32>(0.88, 0.84, 0.76);
    }

    let ringMask = rings;
    var heartwoodColor = mix(dark, mid, vec3<f32>(ringMask));
    heartwoodColor = mix(heartwoodColor, light, vec3<f32>(smoothstep(0.3, 0.7, rings)));

    let sapwoodMask = smoothstep(0.0, 0.35, ringDist) * smoothstep(1.4, 0.95, ringDist);
    var woodColor = mix(heartwoodColor, sap, vec3<f32>(sapwoodMask * 0.65));

    let grainEffect = grainPattern * 0.4;
    woodColor = mix(woodColor, light * 1.1, vec3<f32>(grainEffect));
    woodColor += fineGrain;

    woodColor = mix(woodColor, woodColor * 1.1, vec3<f32>(medullaryRays));
    woodColor -= pores;

    let colorVariation = fbm2d(p * 1.2, 3);
    woodColor *= mix(0.9, 1.1, colorVariation);

    let textureVariation = fbm2d(p * 3.0, 3);
    woodColor *= mix(0.93, 1.07, textureVariation);

    return clamp(woodColor, vec3<f32>(0.0), vec3<f32>(1.0));
}

// ── Matte rubber ──

fn matteRubber(baseColor: vec3<f32>, n: vec3<f32>, l: vec3<f32>, v: vec3<f32>, rough: f32, fiber: f32) -> vec3<f32> {
    let nl = clamp(dot(n, l), 0.0, 1.0);
    let nv = clamp(dot(n, v), 0.0, 1.0);
    let h = normalize(l + v);
    let nh = clamp(dot(n, h), 0.0, 1.0);

    let wrap = mix(0.32, 0.68, rough);
    let horizon = pow(1.0 - nv, 4.0);
    let fd90 = 0.4 + 2.4 * nh * nh * rough;
    let lightScatter = mix(1.0, fd90, pow(1.0 - nl, 5.0));
    let viewScatter = mix(1.0, fd90, pow(1.0 - nv, 5.0));
    let wrapTerm = clamp((nl + wrap) / (1.0 + wrap), 0.0, 1.0);
    let microShadow = clamp(nl * nv * 4.0, 0.0, 1.0);
    let coreDark = mix(0.18, 0.10, rough);
    let edgeLift = mix(0.70, 0.88, rough);
    let lobe = clamp(nl * 0.7 + nv * 0.3, 0.0, 1.0);
    var diff = baseColor * mix(coreDark, edgeLift, wrapTerm) * lightScatter * viewScatter;
    diff *= mix(0.82, 1.02, microShadow);
    diff *= mix(1.0, 0.84, horizon);
    let sheenMix = mix(0.25, 0.55, fiber);
    diff = mix(diff, diff * vec3<f32>(1.05, 1.02, 0.98), vec3<f32>(sheenMix * pow(1.0 - nv, 2.5)));

    let sheen = pow(1.0 - nh, 4.2) * (0.06 + 0.24 * fiber);
    let sheenCol = mix(baseColor, vec3<f32>(1.0), vec3<f32>(0.26)) * sheen;

    let alpha = max(0.08, rough * rough);
    let alpha2 = alpha * alpha;
    let denom = nh * nh * (alpha2 - 1.0) + 1.0;
    let d = alpha2 / (3.14159265 * denom * denom + 1e-5);
    let k = alpha * 0.5 + 1e-4;
    let gl = nl / (nl * (1.0 - k) + k);
    let gv = nv / (nv * (1.0 - k) + k);
    let spec = d * gl * gv;
    let fres = pow(1.0 - nv, 5.0);
    let spTint = mix(vec3<f32>(0.05, 0.05, 0.04), baseColor, vec3<f32>(0.35));
    let grazeFres = pow(1.0 - nv, 2.5);
    var sp = spTint * spec * (0.06 + 0.36 * fiber) * (0.08 + 0.90 * fres);
    let forward = pow(clamp(dot(h, v), 0.0, 1.0), 8.0) * (0.08 + 0.12 * fiber);
    sp += spTint * forward;

    let subsurface = (1.0 - nl) * (0.14 + 0.12 * (1.0 - rough));
    let rim = pow(1.0 - nv, 3.2) * (0.08 + 0.16 * fiber);
    let graze = pow(1.0 - nv, 3.6) * (0.06 + 0.12 * (1.0 - rough));
    diff *= 1.0 + subsurface;
    diff += baseColor * rim;
    diff += baseColor * graze;
    return diff + sheenCol + sp;
}

// ── Rubber shading (simple) ──

fn rubberShading(baseColor: vec3<f32>, n: vec3<f32>, l: vec3<f32>, v: vec3<f32>) -> vec3<f32> {
    let ndl = clamp(dot(n, l), 0.0, 1.0);
    let h = normalize(l + v);
    let ndh = clamp(dot(n, h), 0.0, 1.0);

    let wrap = clamp((ndl + 0.48) / 1.48, 0.0, 1.0);
    let diff = baseColor * (0.18 + 0.82 * wrap);

    let nv = clamp(dot(n, v), 0.0, 1.0);
    let fres = pow(1.0 - nv, 6.0);

    let specPow = 2.2;
    let spec = pow(ndh, specPow) * 0.018;
    let sp = vec3<f32>(spec) * (0.15 + 0.85 * fres);

    let subsurface = (1.0 - ndl) * 0.06;
    return diff * (1.0 + subsurface) + sp;
}

// ── SDF Shadow ──

fn segmentDistSq(p: vec2<f32>, a: vec2<f32>, b: vec2<f32>) -> f32 {
    let ab = b - a;
    let ap = p - a;
    let t = clamp(dot(ap, ab) / max(dot(ab, ab), 1e-10), 0.0, 1.0);
    let closest = a + ab * t;
    let d = p - closest;
    return dot(d, d);
}

fn segmentClosestZ(p: vec2<f32>, seg_a: vec3<f32>, seg_b: vec3<f32>) -> f32 {
    let ab = vec2<f32>(seg_b.x - seg_a.x, seg_b.y - seg_a.y);
    let ap = p - vec2<f32>(seg_a.x, seg_a.y);
    let t = clamp(dot(ap, ab) / max(dot(ab, ab), 1e-10), 0.0, 1.0);
    return seg_a.z + (seg_b.z - seg_a.z) * t;
}

fn sdfShadow(worldPos: vec3<f32>, skipRopeId: f32) -> f32 {
    let lightDir = normalize(frame.lightDir_intensity.xyz);
    let invLz = select(0.0, 1.0 / lightDir.z, abs(lightDir.z) > 1e-6);

    let t_to_surface = -worldPos.z * invLz;
    let surfaceHit = vec2<f32>(
        worldPos.x + lightDir.x * t_to_surface,
        worldPos.y + lightDir.y * t_to_surface,
    );

    var visibility = 1.0;
    let segCount = shadow_segments.count;

    for (var i = 0u; i < segCount; i = i + 1u) {
        let seg = shadow_segments.data[i];
        if (abs(seg.rope_id - skipRopeId) < 0.5) { continue; }
        let a = vec3<f32>(seg.ax, seg.ay, seg.az);
        let b = vec3<f32>(seg.bx, seg.by, seg.bz);
        let r = seg.radius;

        let t_a = -a.z * invLz;
        let proj_a = vec2<f32>(a.x + lightDir.x * t_a, a.y + lightDir.y * t_a);
        let t_b = -b.z * invLz;
        let proj_b = vec2<f32>(b.x + lightDir.x * t_b, b.y + lightDir.y * t_b);

        let distSq = segmentDistSq(surfaceHit, proj_a, proj_b);
        let rSq = r * r;

        if (distSq < rSq * 4.0) {
            let dist = sqrt(distSq);
            let casterZ = segmentClosestZ(surfaceHit, a, b);
            let height = casterZ - worldPos.z;
            if (height < r * 0.5) { continue; }
            let softness = 1.0 + height * 3.0;
            let edge = smoothstep(r * softness, r * 0.7, dist);
            visibility = min(visibility, 1.0 - edge * 0.9);
        }
    }

    return visibility;
}

fn shadowVisibility(worldPos: vec3<f32>, worldN: vec3<f32>) -> f32 {
    return sdfShadow(worldPos, -1.0);
}

// ── Vertex/fragment IO structs ──

struct FullscreenVSOut {
    @builtin(position) position: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

struct HoleVSOut {
    @builtin(position) position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) worldPos: vec3<f32>,
    @location(2) highlight: f32,
    @location(3) instanceId: f32,
};

struct RopeVSOut {
    @builtin(position) position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) color: vec3<f32>,
    @location(2) worldPos: vec3<f32>,
    @location(3) uv: vec2<f32>,
    @location(4) params: vec4<f32>,
};

struct ShadowVSOut {
    @builtin(position) position: vec4<f32>,
};

// ── Fullscreen vertex ──

@vertex
fn fullscreen_vertex(@builtin(vertex_index) vid: u32) -> FullscreenVSOut {
    var p: vec2<f32>;
    if (vid == 0u) { p = vec2<f32>(-1.0, -1.0); }
    else if (vid == 1u) { p = vec2<f32>(3.0, -1.0); }
    else { p = vec2<f32>(-1.0, 3.0); }

    var out: FullscreenVSOut;
    out.position = vec4<f32>(p, 0.0, 1.0);
    out.uv = p * 0.5 + 0.5;
    return out;
}

// ── Table vertex ──

struct TableVSOut {
    @builtin(position) position: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) worldXY: vec2<f32>,
};

@vertex
fn table_vertex(@builtin(vertex_index) vid: u32) -> TableVSOut {
    let halfW = frame.orthoHalfSize_shadowBias.x;
    let halfH = frame.orthoHalfSize_shadowBias.y;
    let margin = 1.2;

    var corners = array<vec2<f32>, 4>(
        vec2<f32>(-halfW * margin, -halfH * margin),
        vec2<f32>( halfW * margin, -halfH * margin),
        vec2<f32>(-halfW * margin,  halfH * margin),
        vec2<f32>( halfW * margin,  halfH * margin),
    );
    let idx = array<u32, 6>(0u, 1u, 2u, 2u, 1u, 3u);
    let ci = idx[vid];
    let wxy = corners[ci];
    let worldPos = vec4<f32>(wxy.x, wxy.y, 0.0, 1.0);
    let clip = frame.viewProj * worldPos;

    var out: TableVSOut;
    out.position = clip;
    out.uv = vec2<f32>(
        (wxy.x / halfW) * 0.5 + 0.5,
        (wxy.y / halfH) * 0.5 + 0.5,
    );
    out.worldXY = wxy;
    return out;
}

// ── Table fragment ──

struct TableFSOut {
    @location(0) color: vec4<f32>,
    @builtin(frag_depth) depth: f32,
};

@fragment
fn table_fragment(in: TableVSOut) -> TableFSOut {
    let uv = in.uv;
    let worldXY = in.worldXY;

    let holeCount = arrayLength(&hole_instances.data);
    if (holeCount > 0u) {
        let holeRadius = hole_instances.data[0].position_radius.w;
        for (var i = 0u; i < holeCount; i = i + 1u) {
            let hc = hole_instances.data[i].position_radius.xy;
            let dist = length(worldXY - hc);
            if (dist < holeRadius * 0.76) {
                discard;
            }
        }
    }

    let worldPos = vec3<f32>(worldXY.x, worldXY.y, 0.0);
    let worldN = vec3<f32>(0.0, 0.0, 1.0);

    let levelSeed = frame.timeDrag.z;
    let tableMode = u32(frame.shadowInvSize_unused.w);

    var baseColor: vec3<f32>;
    if (tableMode == 1u) {
        baseColor = vec3<f32>(0.95, 0.95, 0.95);
    } else if (tableMode == 2u) {
        baseColor = vec3<f32>(0.08, 0.08, 0.08);
    } else if (tableMode == 3u) {
        baseColor = vec3<f32>(0.45, 0.45, 0.45);
    } else {
        baseColor = woodTextureVariant(uv, worldXY, levelSeed);
    }

    let l = normalize(frame.lightDir_intensity.xyz);
    let v = normalize(frame.cameraPos.xyz - worldPos);
    let nl = clamp(dot(worldN, l), 0.0, 1.0);
    let nv = clamp(dot(worldN, v), 0.0, 1.0);
    let h = normalize(l + v);
    let nh = clamp(dot(worldN, h), 0.0, 1.0);

    let wrap = 0.4;
    let wrapTerm = clamp((nl + wrap) / (1.0 + wrap), 0.0, 1.0);
    let diff = baseColor * mix(0.25, 0.95, wrapTerm);

    let fresnel = pow(1.0 - nv, 3.0);
    let roughness = 0.75;
    let alpha = roughness * roughness;
    let alpha2 = alpha * alpha;
    let denom = nh * nh * (alpha2 - 1.0) + 1.0;
    let d = alpha2 / (3.14159265 * denom * denom + 1e-5);
    let k = alpha * 0.5 + 1e-4;
    let gl = nl / (nl * (1.0 - k) + k);
    let gv = nv / (nv * (1.0 - k) + k);
    let spec = d * gl * gv;
    let specColor = vec3<f32>(0.9, 0.85, 0.75) * spec * 0.12 * (0.2 + 0.8 * fresnel);

    var c = diff + specColor;

    let shadow = shadowVisibility(worldPos, worldN);
    c *= mix(0.25, 1.0, shadow);

    let ambient = 0.10;
    c += baseColor * ambient;
    c *= 0.85;

    let clipPos = frame.viewProj * vec4<f32>(worldPos, 1.0);
    let tableDepth = clipPos.z / clipPos.w + 0.0004;

    var out: TableFSOut;
    out.color = vec4<f32>(c, 1.0);
    out.depth = tableDepth;
    return out;
}

// ── Hole vertex ──

@vertex
fn hole_vertex(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @builtin(vertex_index) vid: u32,
    @builtin(instance_index) iid: u32,
) -> HoleVSOut {
    let inst = hole_instances.data[iid];
    let radius = inst.position_radius.w;
    let lp = position * radius;
    let wp = vec3<f32>(inst.position_radius.x + lp.x, inst.position_radius.y + lp.y, lp.z);

    let hlIdx = frame.ambientColor.w;
    let isHighlight = select(0.0, 1.0, hlIdx >= 0.0 && abs(f32(iid) - hlIdx) < 0.5);

    var o: HoleVSOut;
    o.worldPos = wp;
    o.position = frame.viewProj * vec4<f32>(wp, 1.0);
    o.normal = normalize(normal);
    o.highlight = isHighlight;
    o.instanceId = f32(iid);
    return o;
}

// ── Hole fragment ──

@fragment
fn hole_fragment(in: HoleVSOut) -> @location(0) vec4<f32> {
    let n = normalize(in.normal);
    let l = normalize(frame.lightDir_intensity.xyz);
    let v = normalize(frame.cameraPos.xyz - in.worldPos);
    let nv = clamp(dot(n, v), 0.0, 1.0);

    let levelSeed = frame.timeDrag.z;
    let matSeed = hash21(vec2<f32>(levelSeed * 1.37, 0.0));
    let holeId = in.instanceId;

    var topCol: vec3<f32>;
    var wallCol: vec3<f32>;
    var specPower: f32;
    var specStrength: f32;

    if (matSeed < 0.2) {
        topCol = vec3<f32>(0.90, 0.92, 0.97);
        wallCol = vec3<f32>(0.70, 0.74, 0.82);
        specPower = 32.0;
        specStrength = 0.18;
    } else if (matSeed < 0.4) {
        topCol = vec3<f32>(0.78, 0.60, 0.42);
        wallCol = vec3<f32>(0.55, 0.40, 0.28);
        specPower = 24.0;
        specStrength = 0.22;
    } else if (matSeed < 0.6) {
        topCol = vec3<f32>(0.35, 0.38, 0.42);
        wallCol = vec3<f32>(0.22, 0.24, 0.28);
        specPower = 48.0;
        specStrength = 0.15;
    } else if (matSeed < 0.8) {
        topCol = vec3<f32>(0.95, 0.82, 0.45);
        wallCol = vec3<f32>(0.75, 0.60, 0.25);
        specPower = 28.0;
        specStrength = 0.25;
    } else {
        topCol = vec3<f32>(0.85, 0.55, 0.35);
        wallCol = vec3<f32>(0.65, 0.38, 0.22);
        specPower = 26.0;
        specStrength = 0.20;
    }

    let perHole = hash21(vec2<f32>(holeId * 3.17, levelSeed * 2.31));
    let perHoleMul = mix(0.92, 1.08, perHole);
    topCol *= perHoleMul;
    wallCol *= perHoleMul;

    let col = mix(wallCol, topCol, vec3<f32>(smoothstep(0.15, 0.65, n.z)));

    let ndl = clamp(dot(n, l), 0.0, 1.0);
    let h = normalize(l + v);
    let ndh = clamp(dot(n, h), 0.0, 1.0);

    let fresnel = pow(1.0 - nv, 4.0);
    let spec = pow(ndh, specPower) * specStrength * (0.3 + 0.7 * fresnel);
    let specCol = mix(col, vec3<f32>(1.0), vec3<f32>(0.5)) * spec;

    var lit = col * (0.25 + 0.75 * ndl) + specCol;

    if (in.highlight > 0.5) {
        lit = mix(lit, vec3<f32>(0.55, 0.85, 1.0), vec3<f32>(0.6));
        lit += vec3<f32>(0.08, 0.15, 0.22);
    }

    let shadow = shadowVisibility(in.worldPos, n);
    lit *= mix(0.25, 1.0, shadow);
    return vec4<f32>(lit, 1.0);
}

// ── Rope vertex ──

@vertex
fn rope_vertex(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec3<f32>,
    @location(3) uv: vec2<f32>,
    @location(4) params: vec4<f32>,
) -> RopeVSOut {
    var o: RopeVSOut;
    let time = frame.timeDrag.x;
    let energy = frame.timeDrag.y;
    let dragActive = frame.timeDrag.w;
    let u = uv.x;
    let pinch = params.y;

    var w = sin(u * 3.14159265);
    w = w * w;
    let amp = (0.002 + 0.010 * pinch) * energy * (0.25 + 0.75 * dragActive) * w;
    let wave = sin(u * 24.0 + time * 16.0) * 0.65 + sin(u * 11.0 - time * 9.0) * 0.35;
    let displaced = position + normalize(normal) * (wave * amp);

    o.worldPos = displaced;
    o.position = frame.viewProj * vec4<f32>(displaced, 1.0);
    o.normal = normal;
    o.color = color;
    o.uv = uv;
    o.params = params;
    return o;
}

// ── Rope fragment ──

@fragment
fn rope_fragment(in: RopeVSOut) -> @location(0) vec4<f32> {
    let l = normalize(frame.lightDir_intensity.xyz);
    let v = normalize(frame.cameraPos.xyz - in.worldPos);
    var n = normalize(in.normal);
    let taut = clamp(in.params.x, 0.0, 1.0);
    let pinch = clamp(in.params.y, 0.0, 1.0);
    let fadeOut = clamp(in.params.w, 0.0, 1.0);

    var base = in.color;
    base = mix(base, vec3<f32>(1.0), vec3<f32>(pinch * 0.18));
    base *= 1.0 + pinch * 0.28;

    let glowT = smoothstep(0.0, 0.6, fadeOut);
    base = mix(base, vec3<f32>(1.0, 1.0, 0.95), vec3<f32>(glowT * 0.7));

    let rough = 0.48 + pinch * 0.06;
    let fiber = 0.15;

    let ao = mix(0.80, 1.0, clamp(taut * 0.5 + (1.0 - pinch) * 0.3, 0.0, 1.0));
    var c = matteRubber(base, n, l, v, rough, fiber);

    let heightLift = clamp(in.worldPos.z / 0.35, 0.0, 1.0);
    c += vec3<f32>(0.06, 0.08, 0.12) * heightLift * 0.5;
    c *= ao;

    let ropeId = in.params.z;
    let shadow = sdfShadow(in.worldPos, ropeId);
    let lift = 0.22 + 0.06 * taut;
    c *= mix(lift, 1.0, shadow);

    c *= 0.65;
    c += c * glowT * 2.5;
    c = mix(c, vec3<f32>(1.2, 1.15, 1.0), vec3<f32>(glowT * glowT * 0.5));

    return vec4<f32>(c, 1.0);
}

// ── Rope shadow vertex ──

@vertex
fn rope_shadow_vertex(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec3<f32>,
    @location(3) uv: vec2<f32>,
    @location(4) params: vec4<f32>,
) -> ShadowVSOut {
    var o: ShadowVSOut;
    let time = frame.timeDrag.x;
    let energy = frame.timeDrag.y;
    let dragActive = frame.timeDrag.w;
    let u = uv.x;
    let pinch = params.y;
    var w = sin(u * 3.14159265);
    w = w * w;
    let amp = (0.002 + 0.010 * pinch) * energy * (0.25 + 0.75 * dragActive) * w;
    let wave = sin(u * 24.0 + time * 16.0) * 0.65 + sin(u * 11.0 - time * 9.0) * 0.35;
    let displaced = position + normalize(normal) * (wave * amp);
    o.position = frame.lightViewProj * vec4<f32>(displaced, 1.0);
    return o;
}

// ── Hole shadow vertex ──

@vertex
fn hole_shadow_vertex(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @builtin(vertex_index) vid: u32,
    @builtin(instance_index) iid: u32,
) -> ShadowVSOut {
    let inst = hole_instances.data[iid];
    let radius = inst.position_radius.w;
    let lp = position * radius;
    let wp = vec3<f32>(inst.position_radius.x + lp.x, inst.position_radius.y + lp.y, lp.z);
    var o: ShadowVSOut;
    o.position = frame.lightViewProj * vec4<f32>(wp, 1.0);
    return o;
}

// ── Post-process fragment ──

fn confettiHash(p: vec2<f32>) -> f32 {
    let n = sin(dot(p, vec2<f32>(41.1, 289.7)));
    return fract(n * 43758.5453);
}

fn confettiColor(id: f32) -> vec3<f32> {
    let i = u32(id * 7.0) % 7u;
    if (i == 0u) { return vec3<f32>(1.0, 0.3, 0.35); }
    if (i == 1u) { return vec3<f32>(0.3, 0.85, 1.0); }
    if (i == 2u) { return vec3<f32>(1.0, 0.85, 0.2); }
    if (i == 3u) { return vec3<f32>(0.4, 1.0, 0.5); }
    if (i == 4u) { return vec3<f32>(0.9, 0.4, 1.0); }
    if (i == 5u) { return vec3<f32>(1.0, 0.6, 0.2); }
    return vec3<f32>(0.3, 0.6, 1.0);
}

fn confettiLayer(uv: vec2<f32>, time: f32, victoryTime: f32, seed: f32) -> vec3<f32> {
    var acc = vec3<f32>(0.0);
    let count = 20;
    let appear = clamp(victoryTime / 0.3, 0.0, 1.0);

    for (var i = 0; i < count; i = i + 1) {
        let fi = f32(i) + seed * 100.0;
        let h1 = confettiHash(vec2<f32>(fi, seed));
        let h2 = confettiHash(vec2<f32>(fi + 0.5, seed + 1.0));
        let h3 = confettiHash(vec2<f32>(fi + 1.0, seed + 2.0));
        let h4 = confettiHash(vec2<f32>(fi + 1.5, seed + 3.0));

        let startX = h1;
        let startY = 1.1 + h2 * 0.3;
        let speed = 0.3 + h3 * 0.5;
        let wobble = sin(time * (2.0 + h4 * 3.0) + fi) * 0.06;

        let px = startX + wobble + sin(time * 0.7 + fi * 2.0) * 0.02;
        let py = startY - victoryTime * speed;

        if (py < -0.1) { continue; }

        let dx = uv.x - px;
        let dy = uv.y - py;
        let angle = time * (3.0 + h2 * 4.0) + fi;
        let ca = cos(angle);
        let sa = sin(angle);
        let rx = dx * ca - dy * sa;
        let ry = dx * sa + dy * ca;

        let w = 0.004 + h3 * 0.004;
        let h = 0.002 + h4 * 0.003;
        let inRect = step(-w, rx) * step(rx, w) * step(-h, ry) * step(ry, h);

        if (inRect > 0.5) {
            let col = confettiColor(h1) * (0.9 + 0.3 * sin(time * 5.0 + fi));
            acc += col * appear;
        }
    }
    return acc;
}

@fragment
fn post_fragment(in: FullscreenVSOut) -> @location(0) vec4<f32> {
    let uv = in.uv;
    let c = textureSample(hdr_texture, linear_sampler, uv).xyz;
    let b = textureSample(bloom_texture, linear_sampler, uv).xyz;
    var combined = c + b * 0.18;
    let exposure = 1.1;
    var mapped = vec3<f32>(1.0) - exp(-combined * exposure);
    mapped = pow(clamp(mapped, vec3<f32>(0.0), vec3<f32>(1.0)), vec3<f32>(1.0 / 2.2));

    let victoryTime = frame.timeDrag.y;
    let time = frame.timeDrag.x;
    if (victoryTime > 0.0) {
        let confetti = confettiLayer(uv, time, victoryTime, 0.0)
                     + confettiLayer(uv, time, victoryTime, 3.7)
                     + confettiLayer(uv, time, victoryTime, 7.3);
        mapped += confetti;
    }

    return vec4<f32>(mapped, 1.0);
}

// ── Bloom threshold compute ──

@compute @workgroup_size(8, 8)
fn bloom_threshold(@builtin(global_invocation_id) gid: vec3<u32>) {
    let dstDims = textureDimensions(bloom_dst);
    if (gid.x >= dstDims.x || gid.y >= dstDims.y) { return; }
    let uv = (vec2<f32>(f32(gid.x), f32(gid.y)) + 0.5) / vec2<f32>(f32(dstDims.x), f32(dstDims.y));
    let srcDims = textureDimensions(bloom_src);
    let suv = uv * vec2<f32>(f32(srcDims.x), f32(srcDims.y));
    var sid = vec2<u32>(u32(suv.x), u32(suv.y));
    sid.x = min(sid.x, srcDims.x - 1u);
    sid.y = min(sid.y, srcDims.y - 1u);
    let c = textureLoad(bloom_src, vec2<i32>(i32(sid.x), i32(sid.y)), 0).xyz;
    let lum = dot(c, vec3<f32>(0.2126, 0.7152, 0.0722));
    let t = smoothstep(0.92, 1.25, lum);
    textureStore(bloom_dst, vec2<i32>(i32(gid.x), i32(gid.y)), vec4<f32>(c * t, 1.0));
}

// ── Bloom horizontal blur compute ──

@compute @workgroup_size(8, 8)
fn bloom_blur_h(@builtin(global_invocation_id) gid: vec3<u32>) {
    let dims = textureDimensions(bloom_dst);
    if (gid.x >= dims.x || gid.y >= dims.y) { return; }
    let p = vec2<i32>(i32(gid.x), i32(gid.y));
    let maxP = vec2<i32>(i32(dims.x) - 1, i32(dims.y) - 1);
    let w0 = 0.227027;
    let w1 = 0.1945946;
    let w2 = 0.1216216;
    let w3 = 0.054054;
    let w4 = 0.016216;
    var acc = textureLoad(bloom_src, p, 0) * w0;
    acc += textureLoad(bloom_src, clamp(p + vec2<i32>(1, 0), vec2<i32>(0), maxP), 0) * w1;
    acc += textureLoad(bloom_src, clamp(p - vec2<i32>(1, 0), vec2<i32>(0), maxP), 0) * w1;
    acc += textureLoad(bloom_src, clamp(p + vec2<i32>(2, 0), vec2<i32>(0), maxP), 0) * w2;
    acc += textureLoad(bloom_src, clamp(p - vec2<i32>(2, 0), vec2<i32>(0), maxP), 0) * w2;
    acc += textureLoad(bloom_src, clamp(p + vec2<i32>(3, 0), vec2<i32>(0), maxP), 0) * w3;
    acc += textureLoad(bloom_src, clamp(p - vec2<i32>(3, 0), vec2<i32>(0), maxP), 0) * w3;
    acc += textureLoad(bloom_src, clamp(p + vec2<i32>(4, 0), vec2<i32>(0), maxP), 0) * w4;
    acc += textureLoad(bloom_src, clamp(p - vec2<i32>(4, 0), vec2<i32>(0), maxP), 0) * w4;
    textureStore(bloom_dst, vec2<i32>(i32(gid.x), i32(gid.y)), acc);
}

// ── Bloom vertical blur compute ──

@compute @workgroup_size(8, 8)
fn bloom_blur_v(@builtin(global_invocation_id) gid: vec3<u32>) {
    let dims = textureDimensions(bloom_dst);
    if (gid.x >= dims.x || gid.y >= dims.y) { return; }
    let p = vec2<i32>(i32(gid.x), i32(gid.y));
    let maxP = vec2<i32>(i32(dims.x) - 1, i32(dims.y) - 1);
    let w0 = 0.227027;
    let w1 = 0.1945946;
    let w2 = 0.1216216;
    let w3 = 0.054054;
    let w4 = 0.016216;
    var acc = textureLoad(bloom_src, p, 0) * w0;
    acc += textureLoad(bloom_src, clamp(p + vec2<i32>(0, 1), vec2<i32>(0), maxP), 0) * w1;
    acc += textureLoad(bloom_src, clamp(p - vec2<i32>(0, 1), vec2<i32>(0), maxP), 0) * w1;
    acc += textureLoad(bloom_src, clamp(p + vec2<i32>(0, 2), vec2<i32>(0), maxP), 0) * w2;
    acc += textureLoad(bloom_src, clamp(p - vec2<i32>(0, 2), vec2<i32>(0), maxP), 0) * w2;
    acc += textureLoad(bloom_src, clamp(p + vec2<i32>(0, 3), vec2<i32>(0), maxP), 0) * w3;
    acc += textureLoad(bloom_src, clamp(p - vec2<i32>(0, 3), vec2<i32>(0), maxP), 0) * w3;
    acc += textureLoad(bloom_src, clamp(p + vec2<i32>(0, 4), vec2<i32>(0), maxP), 0) * w4;
    acc += textureLoad(bloom_src, clamp(p - vec2<i32>(0, 4), vec2<i32>(0), maxP), 0) * w4;
    textureStore(bloom_dst, vec2<i32>(i32(gid.x), i32(gid.y)), acc);
}
