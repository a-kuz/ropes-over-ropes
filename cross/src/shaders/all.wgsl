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
    holeMaskBounds: vec4<f32>,
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
@group(0) @binding(4) var planar_shadow_mask: texture_2d<f32>;
@group(0) @binding(5) var noise_tex: texture_2d<f32>;
@group(0) @binding(6) var noise_sampler: sampler;

@group(1) @binding(0) var<storage, read> hole_instances: HoleInstanceArray;
@group(1) @binding(1) var<storage, read> shadow_segments: ShadowSegmentArray;
@group(1) @binding(2) var hole_mask: texture_2d<f32>;

@group(2) @binding(0) var hdr_texture: texture_2d<f32>;
@group(2) @binding(1) var bloom_texture: texture_2d<f32>;
@group(2) @binding(2) var linear_sampler: sampler;
@group(2) @binding(3) var depth_texture: texture_depth_2d;

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

// (wood texture removed — will be baked to texture on CPU)

// ── Cel shading ──

fn celShading(baseColor: vec3<f32>, n: vec3<f32>, l: vec3<f32>, v: vec3<f32>) -> vec3<f32> {
    let ndl = dot(n, l);
    let lit = select(0.38, 0.72, ndl > 0.1);
    let nv = clamp(dot(n, v), 0.0, 1.0);
    let edge_darken = smoothstep(0.0, 0.45, nv);
    return baseColor * lit * edge_darken;
}

fn celTableShading(baseColor: vec3<f32>, n: vec3<f32>, l: vec3<f32>) -> vec3<f32> {
    return baseColor * 0.62;
}

fn celHoleShading(baseColor: vec3<f32>, n: vec3<f32>, l: vec3<f32>) -> vec3<f32> {
    let ndl = dot(n, l);
    let lit = select(0.25, 0.65, ndl > 0.0);
    return baseColor * lit;
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

fn shadowMapPCF(worldPos: vec3<f32>) -> f32 {
    let lp4 = frame.lightViewProj * vec4<f32>(worldPos, 1.0);
    let ndc3 = lp4.xyz / lp4.w;
    let suv2 = vec2<f32>(ndc3.x * 0.5 + 0.5, 1.0 - (ndc3.y * 0.5 + 0.5));
    let outOfBounds = suv2.x < 0.0 || suv2.x > 1.0 || suv2.y < 0.0 || suv2.y > 1.0;
    let smBias = frame.orthoHalfSize_shadowBias.z;
    let refD = ndc3.z - smBias;
    let smInv = frame.shadowInvSize_unused.x;
    var smVis = 0.0;
    let a = hash21(worldPos.xy * 1.731) * 6.2831853;
    let ca = cos(a);
    let sa = sin(a);
    let radius = smInv * 1.85;
    for (var i = 0u; i < 12u; i = i + 1u) {
        let p = POISSON_DISK[i];
        let rot = vec2<f32>(p.x * ca - p.y * sa, p.x * sa + p.y * ca);
        smVis += textureSampleCompare(shadow_map, shadow_sampler, suv2 + rot * radius, refD);
    }
    return select(smVis / 12.0, 1.0, outOfBounds);
}

fn planarCapsuleShadowAt(worldXY: vec2<f32>) -> f32 {
    let lightDir = normalize(frame.lightDir_intensity.xyz);
    let invLz = select(0.0, 1.0 / lightDir.z, abs(lightDir.z) > 1e-6);
    var visibility = 1.0;
    let segCount = shadow_segments.count;

    for (var i = 0u; i < segCount; i = i + 1u) {
        let seg = shadow_segments.data[i];
        let a = vec3<f32>(seg.ax, seg.ay, seg.az);
        let b = vec3<f32>(seg.bx, seg.by, seg.bz);
        let t_a = -a.z * invLz;
        let t_b = -b.z * invLz;
        let pa = vec2<f32>(a.x + lightDir.x * t_a, a.y + lightDir.y * t_a);
        let pb = vec2<f32>(b.x + lightDir.x * t_b, b.y + lightDir.y * t_b);
        let distSq = segmentDistSq(worldXY, pa, pb);
        let r = seg.radius * 0.95;
        let d = sqrt(distSq);
        let edge = smoothstep(r * 1.04, r * 0.94, d);
        visibility = min(visibility, 1.0 - edge * 0.88);
    }
    return visibility;
}

fn planarMaskShadow(worldXY: vec2<f32>) -> f32 {
    let maskMin = frame.holeMaskBounds.xy;
    let maskMax = frame.holeMaskBounds.zw;
    let maskSize = maskMax - maskMin;
    if (maskSize.x <= 0.0 || maskSize.y <= 0.0) {
        return 1.0;
    }
    let uv = (worldXY - maskMin) / maskSize;
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) {
        return 1.0;
    }
    return textureSampleLevel(planar_shadow_mask, depth_sampler, uv, 0.0).r;
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
    let ext = max(halfW, halfH) * 20.0;

    var corners = array<vec2<f32>, 4>(
        vec2<f32>(-ext, -ext),
        vec2<f32>( ext, -ext),
        vec2<f32>(-ext,  ext),
        vec2<f32>( ext,  ext),
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

    let maskMin = frame.holeMaskBounds.xy;
    let maskMax = frame.holeMaskBounds.zw;
    let maskSize = maskMax - maskMin;
    if (maskSize.x > 0.0 && maskSize.y > 0.0) {
        let maskUV = (worldXY - maskMin) / maskSize;
        if (maskUV.x >= 0.0 && maskUV.x <= 1.0 && maskUV.y >= 0.0 && maskUV.y <= 1.0) {
            let sdf = textureSampleLevel(hole_mask, depth_sampler, maskUV, 0.0).r;
            if (sdf < 0.5) {
                discard;
            }
        }
    }

    let worldPos = vec3<f32>(worldXY.x, worldXY.y, 0.0);
    let worldN = vec3<f32>(0.0, 0.0, 1.0);

    let levelSeed = frame.timeDrag.z;
    let renderMode = u32(frame.shadowInvSize_unused.w);

    let matSeed = hash21(vec2<f32>(levelSeed * 1.37, 0.0));
    var holeLum: f32;
    if (matSeed < 0.2) {
        holeLum = 0.92;
    } else if (matSeed < 0.4) {
        holeLum = 0.58;
    } else if (matSeed < 0.6) {
        holeLum = 0.36;
    } else if (matSeed < 0.8) {
        holeLum = 0.78;
    } else {
        holeLum = 0.58;
    }

    var baseColor: vec3<f32>;
    if (renderMode == 0u) {
        let grayVal = mix(0.35, 0.55, hash21(vec2<f32>(levelSeed * 4.13, 2.0)));
        baseColor = vec3<f32>(grayVal, grayVal, grayVal);
    } else {
        let bgSeed = hash21(vec2<f32>(levelSeed * 2.71, 5.0));
        var tableMode: u32;
        if (holeLum > 0.65) {
            if (bgSeed < 0.6) { tableMode = 2u; } else { tableMode = 3u; }
        } else {
            if (bgSeed < 0.5) { tableMode = 1u; } else if (bgSeed < 0.8) { tableMode = 0u; } else { tableMode = 3u; }
        }

        if (tableMode == 1u) {
            baseColor = vec3<f32>(0.95, 0.95, 0.95);
        } else if (tableMode == 2u) {
            baseColor = vec3<f32>(0.08, 0.08, 0.08);
        } else if (tableMode == 3u) {
            let grayVal = mix(0.35, 0.55, hash21(vec2<f32>(levelSeed * 4.13, 2.0)));
            baseColor = vec3<f32>(grayVal, grayVal, grayVal);
        } else {
            let wSeed = hash21(vec2<f32>(levelSeed * 1.23, 0.0));
            let warmth = hash21(vec2<f32>(levelSeed * 3.71, 1.0));
            let base_lum = mix(0.28, 0.48, wSeed);
            baseColor = vec3<f32>(base_lum * mix(1.0, 1.12, warmth),
                                  base_lum * mix(0.92, 1.0, warmth),
                                  base_lum * mix(0.82, 0.92, warmth));
        }
    }

    let celMode = frame.shadowInvSize_unused.z > 0.5;

    let l = normalize(frame.lightDir_intensity.xyz);
    let v = normalize(frame.cameraPos.xyz - worldPos);

    var c: vec3<f32>;
    if (celMode) {
        c = celTableShading(baseColor, worldN, l);
    } else {
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

        c = diff + specColor;

        let tsMode = frame.ambientColor.x;
        var shadow = 1.0;
        if (tsMode < 0.5) {
            shadow = shadowMapPCF(worldPos);
        } else if (tsMode < 1.5) {
            shadow = planarMaskShadow(worldXY);
        }
        c *= mix(0.25, 1.0, shadow);

        let ambient = 0.10;
        c += baseColor * ambient;
        c *= 0.85;
    }

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

    let celMode = frame.shadowInvSize_unused.z > 0.5;
    let col = mix(wallCol, topCol, vec3<f32>(smoothstep(0.15, 0.65, n.z)));

    var lit: vec3<f32>;
    if (celMode) {
        lit = celHoleShading(col, n, l);
    } else {
        let ndl = clamp(dot(n, l), 0.0, 1.0);
        let h = normalize(l + v);
        let ndh = clamp(dot(n, h), 0.0, 1.0);

        let fresnel = pow(1.0 - nv, 4.0);
        let spec = pow(ndh, specPower) * specStrength * (0.3 + 0.7 * fresnel);
        let specCol = mix(col, vec3<f32>(1.0), vec3<f32>(0.5)) * spec;

        lit = col * (0.25 + 0.75 * ndl) + specCol;
    }

    if (in.highlight > 0.5) {
        lit = mix(lit, vec3<f32>(0.55, 0.85, 1.0), vec3<f32>(0.6));
        lit += vec3<f32>(0.08, 0.15, 0.22);
    }

    let renderMode = u32(frame.shadowInvSize_unused.w);
    if (!celMode && renderMode > 0u) {
        let shadow = shadowVisibility(in.worldPos, n);
        lit *= mix(0.25, 1.0, shadow);
    }
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

    o.worldPos = position;
    o.position = frame.viewProj * vec4<f32>(position, 1.0);
    o.normal = normal;
    o.color = color;
    o.uv = uv;
    o.params = params;
    return o;
}

// ── Rope fragment ──

@fragment
fn rope_fragment(in: RopeVSOut) -> @location(0) vec4<f32> {
    if (in.worldPos.z < -0.01) { discard; }

    let celMode = frame.shadowInvSize_unused.z > 0.5;

    let l = normalize(frame.lightDir_intensity.xyz);
    let v = normalize(frame.cameraPos.xyz - in.worldPos);
    var n = normalize(in.normal);
    let taut = clamp(in.params.x, 0.0, 1.0);
    let pinch = clamp(in.params.y, 0.0, 1.0);
    let fadeOut = clamp(in.params.w, 0.0, 1.0);

    let stretchDamp = 1.0 / (1.0 + taut * 2.5);

    let baseUV = in.uv * vec2<f32>(3.0, 0.8);
    let ns1 = textureSample(noise_tex, noise_sampler, baseUV).rg;
    let ns2 = textureSample(noise_tex, noise_sampler, baseUV * 2.7 + vec2<f32>(0.31, 0.73)).rg;
    let ns3 = textureSample(noise_tex, noise_sampler, baseUV * 5.3 + vec2<f32>(0.67, 0.19)).rg;

    let n0 = ns1.r * 0.5 + ns2.r * 0.3 + ns3.r * 0.2;
    let n1 = ns1.g * 0.5 + ns2.g * 0.3 + ns3.g * 0.2;

    var tVec = cross(n, vec3<f32>(0.0, 0.0, 1.0));
    if (length(tVec) < 1e-3) { tVec = cross(n, vec3<f32>(0.0, 1.0, 0.0)); }
    tVec = normalize(tVec);
    let bVec = normalize(cross(n, tVec));

    let microBump = 0.10 * stretchDamp;
    let microAmp = microBump + pinch * 0.03 * stretchDamp;
    n = normalize(n + (tVec * (n0 - 0.5) + bVec * (n1 - 0.5)) * microAmp);

    var base = in.color;
    let microAO = (n0 + n1 - 1.0) * microBump * 2.0;
    base *= 1.0 + microAO;
    base = mix(base, base * 1.3, vec3<f32>(pinch * 0.15));

    let glowT = smoothstep(0.0, 0.6, fadeOut);
    base = mix(base, vec3<f32>(1.0, 1.0, 0.95), vec3<f32>(glowT * 0.7));

    let roughNoise = textureSample(noise_tex, noise_sampler, baseUV * 1.7 + 0.37).r;
    let rough = roughNoise + pinch * 0.06;

    var c: vec3<f32>;
    if (celMode) {
        c = celShading(base, n, l, v);
    } else {
        let ao = mix(0.80, 1.0, clamp(taut * 0.5 + (1.0 - pinch) * 0.3, 0.0, 1.0));
        let fiber = 0.15;
        c = matteRubber(base, n, l, v, rough, fiber);

        let heightLift = clamp(in.worldPos.z / 0.35, 0.0, 1.0);
        c += vec3<f32>(0.03, 0.04, 0.06) * heightLift * 0.5;
        c *= ao;

        let renderMode = u32(frame.shadowInvSize_unused.w);
        if (renderMode > 0u) {
            let lp4 = frame.lightViewProj * vec4<f32>(in.worldPos, 1.0);
            let ndc3 = lp4.xyz / lp4.w;
            let suv2 = vec2<f32>(ndc3.x * 0.5 + 0.5, 1.0 - (ndc3.y * 0.5 + 0.5));
            let refD = ndc3.z - frame.orthoHalfSize_shadowBias.z;
            let interShadow = textureSampleCompare(shadow_map, shadow_sampler, clamp(suv2, vec2<f32>(0.001), vec2<f32>(0.999)), refD);
            let inBounds = suv2.x >= 0.0 && suv2.x <= 1.0 && suv2.y >= 0.0 && suv2.y <= 1.0;
            c *= mix(0.38, 1.0, select(1.0, interShadow, inBounds));
        } else {
            let lift = 0.22 + 0.06 * taut;
            c *= lift;
        }

        c *= 0.65;
    }

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
    o.position = frame.lightViewProj * vec4<f32>(position, 1.0);
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

const FW_MOD3 = vec3<f32>(0.1031, 0.11369, 0.13787);

fn fw_hash31(p: f32) -> vec3<f32> {
    var p3 = fract(vec3<f32>(p, p, p) * FW_MOD3);
    p3 += dot(p3, p3.yzx + 19.19);
    return fract(vec3<f32>((p3.x + p3.y) * p3.z, (p3.x + p3.z) * p3.y, (p3.y + p3.z) * p3.x));
}

fn fw_explosion(uv: vec2<f32>, p: vec2<f32>, seed: f32, t: f32) -> vec3<f32> {
    var col = vec3<f32>(0.0);
    let en = fw_hash31(seed);
    let baseCol = en;
    let pt = 1.0 - (t - 1.0) * (t - 1.0);
    let fade = clamp(1.0 - t, 0.0, 1.0);
    let fade2 = fade * fade;
    let gravity = t * t * 0.1;

    for (var i = 0; i < 20; i = i + 1) {
        let n = fw_hash31(f32(i)) - 0.5;
        let nlen = max(length(n.xy), 1e-4);
        let dir = n.xy / nlen;
        let endP = p + dir * n.z - vec2<f32>(0.0, gravity);
        let pos = mix(p, endP, pt);

        let d = uv - pos;
        let d2 = dot(d, d);
        if (d2 > 0.01) { continue; }

        var size = 0.008 * fade2;
        let sparkle = sin((pt + n.z) * 80.0) * 0.5 + 0.5;
        size += sparkle * 0.004 * fade;

        col += baseCol * (size * size / max(d2, 1e-6));
    }
    return col;
}

fn fireworks(uv_raw: vec2<f32>, time: f32, victoryTime: f32, aspect: f32) -> vec3<f32> {
    let duration = 3.0;
    if (victoryTime > duration) { return vec3<f32>(0.0); }

    var uv = uv_raw;
    uv.x -= 0.5;
    uv.x *= aspect;

    let appear = clamp(victoryTime / 0.3, 0.0, 1.0);
    let fadeOut = clamp((duration - victoryTime) / 0.6, 0.0, 1.0);
    let t = time * 0.5;
    var c = vec3<f32>(0.0);

    let half_x = aspect * 0.5 * 0.8;
    for (var i = 0; i < 3; i = i + 1) {
        var et = t + f32(i) * 1234.45235;
        let id = floor(et);
        et -= id;
        let h = fw_hash31(id);
        var p = vec2<f32>(
            h.x * 2.0 * half_x - half_x,
            0.15 + h.y * 0.7,
        );
        let d = uv - p;
        if (dot(d, d) > 0.16) { continue; }
        c += fw_explosion(uv, p, id, et);
    }

    return c * appear * fadeOut;
}

fn loadDepth(coord: vec2<i32>, dims: vec2<u32>) -> f32 {
    let cc = clamp(coord, vec2<i32>(0), vec2<i32>(i32(dims.x) - 1, i32(dims.y) - 1));
    return textureLoad(depth_texture, cc, 0);
}

fn edgeDetect(uv: vec2<f32>) -> f32 {
    let dims = textureDimensions(depth_texture);
    let px = vec2<i32>(i32(uv.x * f32(dims.x)), i32(uv.y * f32(dims.y)));
    let s = 3i;

    let d00 = loadDepth(px + vec2<i32>(-s, -s), dims);
    let d10 = loadDepth(px + vec2<i32>( 0, -s), dims);
    let d20 = loadDepth(px + vec2<i32>( s, -s), dims);
    let d01 = loadDepth(px + vec2<i32>(-s,  0), dims);
    let d21 = loadDepth(px + vec2<i32>( s,  0), dims);
    let d02 = loadDepth(px + vec2<i32>(-s,  s), dims);
    let d12 = loadDepth(px + vec2<i32>( 0,  s), dims);
    let d22 = loadDepth(px + vec2<i32>( s,  s), dims);

    let gx = -d00 - 2.0 * d01 - d02 + d20 + 2.0 * d21 + d22;
    let gy = -d00 - 2.0 * d10 - d20 + d02 + 2.0 * d12 + d22;
    let grad = sqrt(gx * gx + gy * gy);

    return smoothstep(0.0004, 0.002, grad);
}

@fragment
fn post_fragment(in: FullscreenVSOut) -> @location(0) vec4<f32> {
    let uv = in.uv;
    let celMode = frame.shadowInvSize_unused.z > 0.5;

    let c = textureSample(hdr_texture, linear_sampler, uv).xyz;
    let b = textureSample(bloom_texture, linear_sampler, uv).xyz;
    var combined: vec3<f32>;
    var exposure: f32;
    if (celMode) {
        combined = c;
        exposure = 0.75;
    } else {
        combined = c + b * 0.18;
        exposure = 1.1;
    }
    var mapped = vec3<f32>(1.0) - exp(-combined * exposure);
    mapped = pow(clamp(mapped, vec3<f32>(0.0), vec3<f32>(1.0)), vec3<f32>(1.0 / 2.2));

    if (celMode) {
        let edge = edgeDetect(uv);
        mapped = mix(mapped, mapped * 0.12, edge);
    }

    let victoryTime = frame.timeDrag.y;
    let time = frame.timeDrag.x;
    if (victoryTime > 0.0) {
        let dims = textureDimensions(hdr_texture);
        let aspect = f32(dims.x) / f32(max(dims.y, 1u));
        let fw = fireworks(uv, time, victoryTime, aspect);
        mapped += fw;
    }

    return vec4<f32>(mapped, 1.0);
}

// ── Bloom threshold compute ──

@compute @workgroup_size(8, 8)
fn planar_shadow_mask_cs(@builtin(global_invocation_id) gid: vec3<u32>) {
    let dstDims = textureDimensions(bloom_dst);
    if (gid.x >= dstDims.x || gid.y >= dstDims.y) { return; }
    let uv = (vec2<f32>(f32(gid.x), f32(gid.y)) + 0.5) / vec2<f32>(f32(dstDims.x), f32(dstDims.y));
    let maskMin = frame.holeMaskBounds.xy;
    let maskMax = frame.holeMaskBounds.zw;
    let worldXY = mix(maskMin, maskMax, uv);
    let vis = planarCapsuleShadowAt(worldXY);
    textureStore(bloom_dst, vec2<i32>(i32(gid.x), i32(gid.y)), vec4<f32>(vis, vis, vis, 1.0));
}

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
