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
    woodBoundsMin: vec4<f32>,
    woodBoundsMax: vec4<f32>,
    holeTint: vec4<f32>,
    visualParams: vec4<f32>,
    lightingParams: vec4<f32>,
    tableParams: vec4<f32>,
    tableParams2: vec4<f32>,
    ropeMatParams: vec4<f32>,
    ropeMatParams2: vec4<f32>,
    ropeMatParams3: vec4<f32>,
    cartoonParams: vec4<f32>,
    wormParams1: vec4<f32>,
    wormParams2: vec4<f32>,
    wormParams3: vec4<f32>,
    wormParams4: vec4<f32>,
    ropeMatParams4: vec4<f32>,
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
@group(0) @binding(7) var wood_baked_tex: texture_2d<f32>;
@group(0) @binding(8) var wood_bake_dst: texture_storage_2d<rgba8unorm, write>;

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

// ── Wood texture (procedural) ──

fn wc_noise3d(p: vec3<f32>) -> f32 {
    let s = vec3<f32>(7.0, 157.0, 113.0);
    let ip = floor(p);
    var fp = fract(p);
    fp = fp * fp * (3.0 - 2.0 * fp);
    let d = dot(ip, s);
    let h0 = vec4<f32>(0.0, s.y, s.z, s.y + s.z) + vec4<f32>(d);
    let h1 = mix(fract(sin(h0) * 43758.545), fract(sin(h0 + vec4<f32>(s.x)) * 43758.545), fp.x);
    let h2 = mix(vec2<f32>(h1.x, h1.z), vec2<f32>(h1.y, h1.w), fp.y);
    return mix(h2.x, h2.y, fp.z);
}

fn wc_fbm3d(p_in: vec3<f32>, octaves: i32, roughness_in: f32) -> f32 {
    var p = p_in;
    var sum = 0.0;
    var amp = 1.0;
    var tot = 0.0;
    let roughness = clamp(roughness_in, 0.0, 1.0);
    for (var i = 0; i < 16; i++) {
        if (i >= octaves) { break; }
        sum += amp * wc_noise3d(p);
        tot += amp;
        amp *= roughness;
        p *= 2.0;
    }
    return sum / tot;
}

fn wc_randomPos3(seed: f32) -> vec3<f32> {
    let s = vec4<f32>(seed, 0.0, 1.0, 2.0);
    return vec3<f32>(hash21(s.xy), hash21(s.xz), hash21(s.xw)) * 100.0 + vec3<f32>(100.0);
}

fn wc_fbmDistorted(p_in: vec3<f32>) -> f32 {
    let d = vec3<f32>(
        wc_noise3d(p_in + wc_randomPos3(0.0)),
        wc_noise3d(p_in + wc_randomPos3(1.0)),
        wc_noise3d(p_in + wc_randomPos3(2.0))
    );
    let p = p_in + (d * 2.0 - vec3<f32>(1.0)) * 1.12;
    return wc_fbm3d(p, 8, 0.5);
}

fn wc_musgraveFbm(p_in: vec3<f32>, octaves: f32, dimension: f32, lacunarity: f32) -> f32 {
    var p = p_in;
    var sum = 0.0;
    var amp = 1.0;
    let m = pow(lacunarity, -dimension);
    for (var i = 0; i < 16; i++) {
        if (f32(i) >= octaves) { break; }
        let n = wc_noise3d(p) * 2.0 - 1.0;
        sum += n * amp;
        amp *= m;
        p *= lacunarity;
    }
    return sum;
}

fn wc_waveFbmX(p: vec3<f32>) -> vec3<f32> {
    let n = p.x * 20.0 + 0.4 * wc_fbm3d(p * 3.0, 3, 3.0);
    return vec3<f32>(sin(n) * 0.5 + 0.5, p.y, p.z);
}

fn wc_remap01(f: f32, in1: f32, in2: f32) -> f32 {
    return clamp((f - in1) / (in2 - in1), 0.0, 1.0);
}

struct WoodPalette {
    dark: vec3<f32>,
    mid: vec3<f32>,
    light: vec3<f32>,
};

fn wc_woodPalette(seed: f32) -> WoodPalette {
    let hue = fract(seed * 0.618033988);
    var pal: WoodPalette;
    if (hue < 0.07) {
        pal.dark  = vec3<f32>(0.04, 0.04, 0.05);
        pal.mid   = vec3<f32>(0.12, 0.12, 0.13);
        pal.light = vec3<f32>(0.24, 0.24, 0.26);
    } else if (hue < 0.14) {
        pal.dark  = vec3<f32>(0.08, 0.09, 0.10);
        pal.mid   = vec3<f32>(0.20, 0.21, 0.23);
        pal.light = vec3<f32>(0.36, 0.37, 0.40);
    } else if (hue < 0.21) {
        pal.dark  = vec3<f32>(0.10, 0.09, 0.08);
        pal.mid   = vec3<f32>(0.24, 0.22, 0.20);
        pal.light = vec3<f32>(0.42, 0.38, 0.35);
    } else if (hue < 0.28) {
        pal.dark  = vec3<f32>(0.06, 0.07, 0.09);
        pal.mid   = vec3<f32>(0.15, 0.16, 0.20);
        pal.light = vec3<f32>(0.28, 0.30, 0.35);
    } else if (hue < 0.35) {
        pal.dark  = vec3<f32>(0.08, 0.07, 0.06);
        pal.mid   = vec3<f32>(0.20, 0.17, 0.14);
        pal.light = vec3<f32>(0.36, 0.30, 0.26);
    } else if (hue < 0.42) {
        pal.dark  = vec3<f32>(0.06, 0.04, 0.03);
        pal.mid   = vec3<f32>(0.18, 0.12, 0.08);
        pal.light = vec3<f32>(0.32, 0.22, 0.16);
    } else if (hue < 0.49) {
        pal.dark  = vec3<f32>(0.03, 0.03, 0.03);
        pal.mid   = vec3<f32>(0.10, 0.09, 0.08);
        pal.light = vec3<f32>(0.20, 0.18, 0.16);
    } else if (hue < 0.56) {
        pal.dark  = vec3<f32>(0.14, 0.10, 0.06);
        pal.mid   = vec3<f32>(0.30, 0.22, 0.14);
        pal.light = vec3<f32>(0.50, 0.38, 0.26);
    } else if (hue < 0.63) {
        pal.dark  = vec3<f32>(0.18, 0.17, 0.16);
        pal.mid   = vec3<f32>(0.34, 0.32, 0.30);
        pal.light = vec3<f32>(0.52, 0.50, 0.47);
    } else if (hue < 0.70) {
        pal.dark  = vec3<f32>(0.06, 0.08, 0.10);
        pal.mid   = vec3<f32>(0.16, 0.19, 0.24);
        pal.light = vec3<f32>(0.30, 0.34, 0.40);
    } else if (hue < 0.77) {
        pal.dark  = vec3<f32>(0.18, 0.08, 0.06);
        pal.mid   = vec3<f32>(0.34, 0.18, 0.12);
        pal.light = vec3<f32>(0.52, 0.30, 0.22);
    } else if (hue < 0.84) {
        pal.dark  = vec3<f32>(0.14, 0.14, 0.12);
        pal.mid   = vec3<f32>(0.30, 0.28, 0.26);
        pal.light = vec3<f32>(0.48, 0.46, 0.42);
    } else if (hue < 0.92) {
        pal.dark  = vec3<f32>(0.05, 0.05, 0.06);
        pal.mid   = vec3<f32>(0.14, 0.14, 0.16);
        pal.light = vec3<f32>(0.26, 0.26, 0.30);
    } else {
        pal.dark  = vec3<f32>(0.12, 0.10, 0.09);
        pal.mid   = vec3<f32>(0.26, 0.22, 0.20);
        pal.light = vec3<f32>(0.44, 0.38, 0.34);
    }
    return pal;
}

fn woodSolidTexture(worldXY: vec2<f32>, seed: f32) -> vec3<f32> {
    let seedAngle = seed * 2.399;
    let sa = sin(seedAngle);
    let ca = cos(seedAngle);
    let rotated = vec2<f32>(worldXY.x * ca - worldXY.y * sa,
                            worldXY.x * sa + worldXY.y * ca);
    let offset = vec2<f32>(hash21(vec2<f32>(seed, seed * 7.13)) * 40.0 - 20.0,
                           hash21(vec2<f32>(seed * 3.71, seed)) * 40.0 - 20.0);
    var p = vec3<f32>((rotated + offset) * 1.1, floor(fract(seed * 0.37) * 8.0));
    let scaleVar = mix(0.8, 1.2, hash21(vec2<f32>(seed * 1.23, 0.0)));
    p = vec3<f32>(p.xy * scaleVar, p.z);

    var n1 = wc_fbmDistorted(p * vec3<f32>(7.8, 1.17, 1.17));
    n1 = mix(n1, 1.0, 0.2);
    let n2_a = wc_musgraveFbm(vec3<f32>(n1 * 4.6), 8.0, 0.0, 2.5);
    var n2 = mix(n2_a, n1, 0.85);
    let dirt = 1.0 - wc_musgraveFbm(wc_waveFbmX(p * vec3<f32>(0.01, 0.15, 0.15)), 15.0, 0.26, 2.4) * 0.4;
    let grain = 1.0 - smoothstep(0.2, 1.0, wc_musgraveFbm(p * vec3<f32>(500.0, 6.0, 1.0), 2.0, 2.0, 2.5)) * 0.2;
    n2 *= dirt * grain;

    let pal = wc_woodPalette(seed);
    let col = mix(mix(pal.dark, pal.mid, wc_remap01(n2, 0.19, 0.56)), pal.light, wc_remap01(n2, 0.56, 1.0));
    return clamp(pow(col, vec3<f32>(0.88)), vec3<f32>(0.0), vec3<f32>(1.0));
}

fn plankWoodTexture(worldXY: vec2<f32>, seed: f32) -> vec3<f32> {
    let seedAngle = seed * 1.73;
    let sa = sin(seedAngle);
    let ca = cos(seedAngle);
    let rotated = vec2<f32>(worldXY.x * ca - worldXY.y * sa,
                            worldXY.x * sa + worldXY.y * ca);
    let globalOffset = vec2<f32>(hash21(vec2<f32>(seed, seed * 5.17)) * 30.0 - 15.0,
                                 hash21(vec2<f32>(seed * 2.91, seed)) * 30.0 - 15.0);
    let wp = rotated + globalOffset;

    let plankWidth = mix(0.28, 0.45, hash21(vec2<f32>(seed * 3.14, 7.0)));
    let gapWidth = 0.006;

    let plankY = wp.y / plankWidth;
    let plankRow = floor(plankY);
    let plankFrac = fract(plankY);

    let rowOffset = hash21(vec2<f32>(plankRow, seed * 11.3)) * 2.0;
    let plankLen = mix(0.8, 1.6, hash21(vec2<f32>(plankRow, seed * 4.7)));
    let plankX = (wp.x + rowOffset) / plankLen;
    let plankCol = floor(plankX);
    let plankFracX = fract(plankX);

    let gapY = smoothstep(0.0, gapWidth / plankWidth, plankFrac) *
               smoothstep(0.0, gapWidth / plankWidth, 1.0 - plankFrac);
    let gapX = smoothstep(0.0, gapWidth / plankLen, plankFracX) *
               smoothstep(0.0, gapWidth / plankLen, 1.0 - plankFracX);
    let gapMask = gapX * gapY;

    let plankId = hash21(vec2<f32>(plankRow * 17.3 + plankCol * 7.1, seed));
    let plankSeed = seed + plankId * 100.0;

    let bendK = (hash21(vec2<f32>(plankSeed, 3.3)) - 0.5) * 0.08;
    let localX = plankFracX * plankLen;
    let localY = plankFrac * plankWidth;
    let bentY = localY + sin(localX * 3.14159 / plankLen) * bendK;

    let p = vec3<f32>(localX * 0.016, bentY * 0.016, plankId * 7.0);

    var n1 = wc_fbmDistorted(p * vec3<f32>(1.0, 0.15, 0.15) * 8.0);
    n1 = mix(n1, 1.0, 0.2);
    let n2 = wc_musgraveFbm(vec3<f32>(n1 * 4.6), 8.0, 0.0, 2.5);
    var n3 = mix(n2, n1, 0.85);
    let q = wc_waveFbmX(p * vec3<f32>(0.01, 0.15, 0.15));
    let dirt = 1.0 - wc_musgraveFbm(q, 15.0, 0.26, 2.4) * 0.4;
    let grain = 1.0 - smoothstep(0.2, 1.0, wc_musgraveFbm(p * vec3<f32>(500.0, 6.0, 1.0), 2.0, 2.0, 2.5)) * 0.2;
    n3 *= dirt * grain;

    let pal = wc_woodPalette(plankSeed);
    let plankBrightness = mix(0.85, 1.15, hash21(vec2<f32>(plankSeed * 1.7, 2.0)));
    var col = mix(mix(pal.dark, pal.mid, wc_remap01(n3, 0.185, 0.565)), pal.light, wc_remap01(n3, 0.565, 1.0));
    col *= plankBrightness;

    let edgeDarken = smoothstep(0.0, 0.04, plankFrac) * smoothstep(0.0, 0.04, 1.0 - plankFrac) *
                     smoothstep(0.0, 0.03, plankFracX) * smoothstep(0.0, 0.03, 1.0 - plankFracX);
    col *= mix(0.7, 1.0, edgeDarken);

    let gapColor = pal.dark * 0.15;
    col = mix(gapColor, col, gapMask);

    return clamp(pow(col, vec3<f32>(0.88)), vec3<f32>(0.0), vec3<f32>(1.0));
}

fn gradientTexture(worldXY: vec2<f32>, seed: f32) -> vec3<f32> {
    let seedAngle = seed * 1.618;
    let sa = sin(seedAngle);
    let ca = cos(seedAngle);
    var rp = vec2<f32>(worldXY.x * ca - worldXY.y * sa,
                       worldXY.x * sa + worldXY.y * ca);
    let off = vec2<f32>(hash21(vec2<f32>(seed * 2.3, 1.0)) * 20.0 - 10.0,
                        hash21(vec2<f32>(1.0, seed * 3.7)) * 20.0 - 10.0);
    rp += off;

    let gradType = fract(seed * 0.7236);

    var c1: vec3<f32>; var c2: vec3<f32>; var c3: vec3<f32>;
    let palIdx = fract(seed * 0.4618);
    if (palIdx < 0.06) {
        c1 = vec3<f32>(0.82, 0.82, 0.84); c2 = vec3<f32>(0.62, 0.63, 0.66); c3 = vec3<f32>(0.44, 0.45, 0.48);
    } else if (palIdx < 0.12) {
        c1 = vec3<f32>(0.90, 0.88, 0.85); c2 = vec3<f32>(0.70, 0.68, 0.64); c3 = vec3<f32>(0.50, 0.48, 0.44);
    } else if (palIdx < 0.18) {
        c1 = vec3<f32>(0.86, 0.86, 0.88); c2 = vec3<f32>(0.56, 0.57, 0.62); c3 = vec3<f32>(0.30, 0.32, 0.38);
    } else if (palIdx < 0.24) {
        c1 = vec3<f32>(0.78, 0.76, 0.72); c2 = vec3<f32>(0.52, 0.50, 0.46); c3 = vec3<f32>(0.30, 0.28, 0.26);
    } else if (palIdx < 0.30) {
        c1 = vec3<f32>(0.88, 0.86, 0.82); c2 = vec3<f32>(0.58, 0.54, 0.48); c3 = vec3<f32>(0.34, 0.30, 0.26);
    } else if (palIdx < 0.36) {
        c1 = vec3<f32>(0.06, 0.06, 0.08); c2 = vec3<f32>(0.18, 0.18, 0.22); c3 = vec3<f32>(0.34, 0.34, 0.40);
    } else if (palIdx < 0.42) {
        c1 = vec3<f32>(0.04, 0.05, 0.07); c2 = vec3<f32>(0.12, 0.14, 0.20); c3 = vec3<f32>(0.24, 0.28, 0.38);
    } else if (palIdx < 0.48) {
        c1 = vec3<f32>(0.08, 0.07, 0.06); c2 = vec3<f32>(0.22, 0.20, 0.18); c3 = vec3<f32>(0.40, 0.36, 0.32);
    } else if (palIdx < 0.54) {
        c1 = vec3<f32>(0.02, 0.04, 0.06); c2 = vec3<f32>(0.08, 0.16, 0.24); c3 = vec3<f32>(0.18, 0.30, 0.42);
    } else if (palIdx < 0.60) {
        c1 = vec3<f32>(0.84, 0.84, 0.82); c2 = vec3<f32>(0.48, 0.48, 0.46); c3 = vec3<f32>(0.20, 0.20, 0.20);
    } else if (palIdx < 0.66) {
        c1 = vec3<f32>(0.04, 0.06, 0.04); c2 = vec3<f32>(0.12, 0.20, 0.14); c3 = vec3<f32>(0.24, 0.38, 0.28);
    } else if (palIdx < 0.72) {
        c1 = vec3<f32>(0.05, 0.03, 0.06); c2 = vec3<f32>(0.16, 0.10, 0.22); c3 = vec3<f32>(0.30, 0.20, 0.40);
    } else if (palIdx < 0.78) {
        c1 = vec3<f32>(0.92, 0.90, 0.86); c2 = vec3<f32>(0.74, 0.72, 0.68); c3 = vec3<f32>(0.56, 0.54, 0.50);
    } else if (palIdx < 0.84) {
        c1 = vec3<f32>(0.10, 0.10, 0.12); c2 = vec3<f32>(0.26, 0.26, 0.30); c3 = vec3<f32>(0.46, 0.46, 0.52);
    } else if (palIdx < 0.92) {
        c1 = vec3<f32>(0.80, 0.82, 0.86); c2 = vec3<f32>(0.54, 0.56, 0.62); c3 = vec3<f32>(0.32, 0.34, 0.40);
    } else {
        c1 = vec3<f32>(0.06, 0.05, 0.04); c2 = vec3<f32>(0.18, 0.16, 0.14); c3 = vec3<f32>(0.36, 0.32, 0.28);
    }

    var t: f32;
    if (gradType < 0.25) {
        let warp = wc_noise3d(vec3<f32>(rp * 0.4, seed)) * 0.15;
        t = clamp(rp.y * 0.25 + 0.5 + warp, 0.0, 1.0);
    } else if (gradType < 0.45) {
        let dist = length(rp) * 0.6;
        let warp = wc_noise3d(vec3<f32>(rp * 1.5, seed)) * 0.2;
        t = clamp((dist + warp) * 0.4, 0.0, 1.0);
    } else if (gradType < 0.60) {
        let diag = (rp.x + rp.y) * 0.3;
        let warp = wc_noise3d(vec3<f32>(rp * 0.6, seed)) * 0.2;
        t = clamp(diag + warp + 0.5, 0.0, 1.0);
    } else if (gradType < 0.75) {
        let angle = atan2(rp.y, rp.x) / 6.28318 + 0.5;
        let warp = wc_noise3d(vec3<f32>(rp * 2.0, seed * 1.3)) * 0.08;
        t = fract(angle + warp);
    } else if (gradType < 0.88) {
        let wave = sin(rp.x * 2.0 + wc_noise3d(vec3<f32>(rp * 0.8, seed)) * 1.5) * 0.5 + 0.5;
        let wave2 = sin(rp.y * 1.8 + wc_noise3d(vec3<f32>(rp * 0.6, seed + 5.0)) * 1.2) * 0.5 + 0.5;
        t = clamp(wave * 0.55 + wave2 * 0.45, 0.0, 1.0);
    } else {
        let n = wc_fbm3d(vec3<f32>(rp * 0.4, seed * 0.7), 5, 0.5);
        t = clamp(n * 0.8 + 0.3, 0.0, 1.0);
    }

    var col: vec3<f32>;
    if (t < 0.5) {
        col = mix(c1, c2, t * 2.0);
    } else {
        col = mix(c2, c3, (t - 0.5) * 2.0);
    }
    let micro = wc_noise3d(vec3<f32>(rp * 8.0, seed * 2.0)) * 0.03 - 0.015;
    col += vec3<f32>(micro);
    return clamp(col, vec3<f32>(0.0), vec3<f32>(1.0));
}

fn noiseGenWood(p: vec3<f32>) -> f32 {
    var w0 = 0.0;
    var w1 = 0.0;
    w0 += sin(dot(p, vec3<f32>(-1.316, 0.918, 1.398))) * 0.0783275458;
    w1 += sin(dot(p, vec3<f32>(0.295, -0.176, 2.167))) * 0.0739931495;
    w0 += sin(dot(p, vec3<f32>(-0.926, 1.445, 1.429))) * 0.0716716966;
    w1 += sin(dot(p, vec3<f32>(-1.878, -0.174, 1.258))) * 0.0697839187;
    w0 += sin(dot(p, vec3<f32>(-1.995, 0.661, -0.908))) * 0.0685409863;
    w1 += sin(dot(p, vec3<f32>(-1.770, 1.350, -0.905))) * 0.0630152419;
    w0 += sin(dot(p, vec3<f32>(2.116, -0.021, 1.161))) * 0.0625361712;
    w1 += sin(dot(p, vec3<f32>(0.405, -1.712, -1.855))) * 0.0567751048;
    w0 += sin(dot(p, vec3<f32>(1.346, 0.945, 1.999))) * 0.0556465603;
    w1 += sin(dot(p, vec3<f32>(-0.397, -0.573, 2.495))) * 0.0555747667;
    w0 += sin(dot(p, vec3<f32>(0.103, -2.457, -1.144))) * 0.0516322279;
    w1 += sin(dot(p, vec3<f32>(-0.483, -1.323, 2.330))) * 0.0513093320;
    w0 += sin(dot(p, vec3<f32>(-1.715, -1.810, -1.164))) * 0.0504567036;
    w1 += sin(dot(p, vec3<f32>(2.529, 0.479, 1.011))) * 0.0500811899;
    w0 += sin(dot(p, vec3<f32>(-1.643, -1.814, -1.437))) * 0.0480875812;
    w1 += sin(dot(p, vec3<f32>(1.495, -1.905, -1.648))) * 0.0458268348;
    w0 += sin(dot(p, vec3<f32>(-1.874, 1.559, 1.762))) * 0.0440084357;
    w1 += sin(dot(p, vec3<f32>(1.068, -2.090, 2.081))) * 0.0413624154;
    w0 += sin(dot(p, vec3<f32>(-0.647, -2.197, -2.237))) * 0.0401592830;
    w1 += sin(dot(p, vec3<f32>(-2.146, -2.171, -1.135))) * 0.0391682940;
    w0 += sin(dot(p, vec3<f32>(2.538, -1.854, -1.604))) * 0.0349588163;
    w1 += sin(dot(p, vec3<f32>(1.687, 2.191, -2.270))) * 0.0342888847;
    w0 += sin(dot(p, vec3<f32>(0.205, 2.617, -2.481))) * 0.0338465332;
    w1 += sin(dot(p, vec3<f32>(3.297, -0.440, -2.317))) * 0.0289423448;
    w0 += sin(dot(p, vec3<f32>(1.068, -1.944, 3.432))) * 0.0286404261;
    w1 += sin(dot(p, vec3<f32>(-3.681, 1.068, 1.789))) * 0.0273625684;
    w0 += sin(dot(p, vec3<f32>(3.116, 2.631, -1.658))) * 0.0259772492;
    w1 += sin(dot(p, vec3<f32>(-1.992, -2.902, -2.954))) * 0.0245830241;
    w0 += sin(dot(p, vec3<f32>(-2.409, -2.374, 3.116))) * 0.0245592756;
    w1 += sin(dot(p, vec3<f32>(0.790, 1.768, 4.196))) * 0.0244078334;
    w0 += sin(dot(p, vec3<f32>(-3.289, 1.007, 3.148))) * 0.0241328015;
    w1 += sin(dot(p, vec3<f32>(3.421, -2.663, 3.262))) * 0.0199736126;
    w0 += sin(dot(p, vec3<f32>(3.062, 2.621, 3.649))) * 0.0199230290;
    w1 += sin(dot(p, vec3<f32>(4.422, -2.206, 2.621))) * 0.0192399437;
    w0 += sin(dot(p, vec3<f32>(2.714, 3.022, 4.200))) * 0.0182510631;
    w1 += sin(dot(p, vec3<f32>(-0.451, 4.143, -4.142))) * 0.0181293526;
    w0 += sin(dot(p, vec3<f32>(-5.838, -0.360, -1.536))) * 0.0175114826;
    w1 += sin(dot(p, vec3<f32>(-0.278, -4.565, 4.149))) * 0.0170799341;
    w0 += sin(dot(p, vec3<f32>(-5.893, -0.163, -2.141))) * 0.0167655258;
    w1 += sin(dot(p, vec3<f32>(4.855, -4.153, 0.606))) * 0.0163155335;
    w0 += sin(dot(p, vec3<f32>(4.498, 0.987, -4.488))) * 0.0162770287;
    w1 += sin(dot(p, vec3<f32>(-1.463, 5.321, -3.315))) * 0.0162569125;
    w0 += sin(dot(p, vec3<f32>(-1.862, 4.386, 4.749))) * 0.0154338176;
    w1 += sin(dot(p, vec3<f32>(0.563, 3.616, -5.751))) * 0.0151952226;
    w0 += sin(dot(p, vec3<f32>(-0.126, 2.569, -6.349))) * 0.0151089405;
    w1 += sin(dot(p, vec3<f32>(-5.094, 4.759, 0.186))) * 0.0147947096;
    w0 += sin(dot(p, vec3<f32>(1.319, 5.713, 3.845))) * 0.0147035221;
    w1 += sin(dot(p, vec3<f32>(7.141, -0.327, 1.420))) * 0.0140573910;
    w0 += sin(dot(p, vec3<f32>(3.888, 6.543, 0.547))) * 0.0133309850;
    w1 += sin(dot(p, vec3<f32>(-1.898, -3.563, -6.483))) * 0.0133171360;
    w0 += sin(dot(p, vec3<f32>(1.719, 7.769, 0.340))) * 0.0126913718;
    w1 += sin(dot(p, vec3<f32>(-2.210, -7.836, 0.102))) * 0.0123746071;
    w0 += sin(dot(p, vec3<f32>(6.248, -5.451, 1.866))) * 0.0117861898;
    w1 += sin(dot(p, vec3<f32>(1.627, -7.066, -4.732))) * 0.0115417453;
    w0 += sin(dot(p, vec3<f32>(4.099, -7.704, 1.474))) * 0.0112591564;
    w1 += sin(dot(p, vec3<f32>(7.357, 3.788, 3.204))) * 0.0112252325;
    w0 += sin(dot(p, vec3<f32>(-2.797, 6.208, 6.253))) * 0.0107206906;
    w1 += sin(dot(p, vec3<f32>(6.130, -5.335, -4.650))) * 0.0105693992;
    w0 += sin(dot(p, vec3<f32>(5.276, -5.576, -5.438))) * 0.0105139072;
    w1 += sin(dot(p, vec3<f32>(9.148, 2.530, -0.383))) * 0.0103996383;
    w0 += sin(dot(p, vec3<f32>(3.894, 2.559, 8.357))) * 0.0103161113;
    w1 += sin(dot(p, vec3<f32>(-6.604, 8.024, -0.289))) * 0.0094066875;
    w0 += sin(dot(p, vec3<f32>(-5.925, 6.505, -6.403))) * 0.0089444733;
    w1 += sin(dot(p, vec3<f32>(9.085, 10.331, -0.451))) * 0.0069245599;
    w0 += sin(dot(p, vec3<f32>(-8.228, 6.323, -9.900))) * 0.0066251015;
    w1 += sin(dot(p, vec3<f32>(10.029, -3.802, 12.151))) * 0.0058122824;
    w0 += sin(dot(p, vec3<f32>(-10.151, -6.513, -11.063))) * 0.0057522358;
    w1 += sin(dot(p, vec3<f32>(-1.773, -16.284, 2.828))) * 0.0056578101;
    w0 += sin(dot(p, vec3<f32>(11.081, 8.687, -9.852))) * 0.0054614334;
    w1 += sin(dot(p, vec3<f32>(-3.941, -4.386, 16.191))) * 0.0054454253;
    w0 += sin(dot(p, vec3<f32>(-6.742, 2.133, -17.268))) * 0.0050050132;
    w1 += sin(dot(p, vec3<f32>(-10.743, 5.698, 14.975))) * 0.0048323955;
    w0 += sin(dot(p, vec3<f32>(-9.603, 12.472, 14.542))) * 0.0043264378;
    w1 += sin(dot(p, vec3<f32>(13.515, 14.345, 8.481))) * 0.0043208884;
    w0 += sin(dot(p, vec3<f32>(-10.330, 16.209, -9.742))) * 0.0043013736;
    w1 += sin(dot(p, vec3<f32>(-8.580, -6.628, 19.191))) * 0.0042005922;
    w0 += sin(dot(p, vec3<f32>(-17.154, 10.620, 11.828))) * 0.0039482427;
    w1 += sin(dot(p, vec3<f32>(16.330, 14.123, -10.420))) * 0.0038474789;
    w0 += sin(dot(p, vec3<f32>(-21.275, 10.768, -3.252))) * 0.0038320501;
    w1 += sin(dot(p, vec3<f32>(1.744, 7.922, 23.152))) * 0.0037560829;
    w0 += sin(dot(p, vec3<f32>(-3.895, 21.321, 12.006))) * 0.0037173885;
    w1 += sin(dot(p, vec3<f32>(-22.705, 2.543, 10.695))) * 0.0036484394;
    w0 += sin(dot(p, vec3<f32>(-13.053, -16.634, -13.993))) * 0.0036291121;
    w1 += sin(dot(p, vec3<f32>(22.697, -11.230, 1.417))) * 0.0036280459;
    w0 += sin(dot(p, vec3<f32>(20.646, 14.602, 3.400))) * 0.0036055008;
    w1 += sin(dot(p, vec3<f32>(5.824, -8.717, -23.680))) * 0.0035501527;
    w0 += sin(dot(p, vec3<f32>(6.691, 15.499, 20.079))) * 0.0035029508;
    w1 += sin(dot(p, vec3<f32>(9.926, -22.778, 9.144))) * 0.0034694278;
    w0 += sin(dot(p, vec3<f32>(-9.552, -27.491, 2.197))) * 0.0031359281;
    w1 += sin(dot(p, vec3<f32>(21.071, -17.991, -11.566))) * 0.0030453280;
    w0 += sin(dot(p, vec3<f32>(9.780, 1.783, 28.536))) * 0.0030251754;
    w1 += sin(dot(p, vec3<f32>(8.738, -18.373, 22.725))) * 0.0029960272;
    w0 += sin(dot(p, vec3<f32>(14.105, 25.703, -8.834))) * 0.0029840058;
    w1 += sin(dot(p, vec3<f32>(-24.926, -17.766, -4.740))) * 0.0029487709;
    w0 += sin(dot(p, vec3<f32>(1.060, -1.570, 32.535))) * 0.0027980099;
    w1 += sin(dot(p, vec3<f32>(-24.532, -19.629, -16.759))) * 0.0025538949;
    w0 += sin(dot(p, vec3<f32>(28.772, -21.183, -9.935))) * 0.0024494819;
    w1 += sin(dot(p, vec3<f32>(-28.413, 22.959, 8.338))) * 0.0024236674;
    w0 += sin(dot(p, vec3<f32>(-27.664, 22.197, 13.301))) * 0.0023965996;
    w1 += sin(dot(p, vec3<f32>(-27.421, 20.643, 18.713))) * 0.0023203498;
    w0 += sin(dot(p, vec3<f32>(18.961, -7.189, 35.907))) * 0.0021967023;
    w1 += sin(dot(p, vec3<f32>(-23.949, 4.885, 33.762))) * 0.0021727461;
    w0 += sin(dot(p, vec3<f32>(35.305, 8.594, 20.564))) * 0.0021689816;
    w1 += sin(dot(p, vec3<f32>(30.364, -11.608, -27.199))) * 0.0021357139;
    w0 += sin(dot(p, vec3<f32>(34.268, 26.742, 0.958))) * 0.0020807976;
    w1 += sin(dot(p, vec3<f32>(-26.376, -17.313, -32.023))) * 0.0020108850;
    w0 += sin(dot(p, vec3<f32>(31.860, -32.181, -2.834))) * 0.0019919601;
    w1 += sin(dot(p, vec3<f32>(25.590, 32.340, 21.381))) * 0.0019446179;
    w0 += sin(dot(p, vec3<f32>(-17.771, -23.941, 37.324))) * 0.0018898258;
    w1 += sin(dot(p, vec3<f32>(-38.699, 19.953, -22.675))) * 0.0018379538;
    w0 += sin(dot(p, vec3<f32>(-46.284, 11.672, -15.411))) * 0.0017980056;
    w1 += sin(dot(p, vec3<f32>(-32.023, -43.976, -7.378))) * 0.0016399251;
    w0 += sin(dot(p, vec3<f32>(-42.390, -21.165, -31.889))) * 0.0015752176;
    w1 += sin(dot(p, vec3<f32>(-18.949, -40.461, 39.107))) * 0.0015141244;
    w0 += sin(dot(p, vec3<f32>(-21.507, -5.939, -58.531))) * 0.0014339601;
    w1 += sin(dot(p, vec3<f32>(-51.745, -43.821, 9.651))) * 0.0013096306;
    w0 += sin(dot(p, vec3<f32>(39.239, 25.971, -52.615))) * 0.0012701774;
    w1 += sin(dot(p, vec3<f32>(-49.669, -35.051, -36.306))) * 0.0012661695;
    w0 += sin(dot(p, vec3<f32>(-49.996, 35.309, 38.460))) * 0.0012398870;
    w1 += sin(dot(p, vec3<f32>(27.000, -65.904, -36.267))) * 0.0011199347;
    w0 += sin(dot(p, vec3<f32>(-52.523, -26.557, 57.693))) * 0.0010856391;
    w1 += sin(dot(p, vec3<f32>(-42.670, 0.269, -71.125))) * 0.0010786551;
    w0 += sin(dot(p, vec3<f32>(-9.377, 64.575, -68.151))) * 0.0009468199;
    w1 += sin(dot(p, vec3<f32>(14.571, -29.160, 106.329))) * 0.0008019719;
    w0 += sin(dot(p, vec3<f32>(-21.549, 103.887, 36.882))) * 0.0007939609;
    w1 += sin(dot(p, vec3<f32>(-42.781, 110.966, -9.070))) * 0.0007473261;
    w0 += sin(dot(p, vec3<f32>(-112.686, 18.296, -37.920))) * 0.0007409259;
    w1 += sin(dot(p, vec3<f32>(71.493, 33.838, -96.931))) * 0.0007121903;
    return w0 + w1;
}

fn reprampWood(x: f32) -> f32 {
    return pow(sin(x) * 0.5 + 0.5, 8.0) + cos(x) * 0.7 + 0.7;
}

fn otavioWoodTexture(worldXY: vec2<f32>, seed: f32) -> vec3<f32> {
    let seedAngle = seed * 2.718;
    let sa = sin(seedAngle);
    let ca = cos(seedAngle);
    let rotated = vec2<f32>(worldXY.x * ca - worldXY.y * sa,
                            worldXY.x * sa + worldXY.y * ca);
    let offset = vec2<f32>(hash21(vec2<f32>(seed, seed * 5.37)) * 40.0 - 20.0,
                           hash21(vec2<f32>(seed * 2.83, seed)) * 40.0 - 20.0);
    var pos = vec3<f32>((rotated + offset) * 1.1, floor(fract(seed * 0.41) * 8.0));
    let scaleVar = mix(0.8, 1.2, hash21(vec2<f32>(seed * 1.47, 0.0)));
    pos = vec3<f32>(pos.xy * scaleVar, pos.z);

    let n_a = noiseGenWood(pos * vec3<f32>(8.0, 1.5, 8.0));
    let n_b = noiseGenWood(-pos * vec3<f32>(8.0, 1.5, 8.0) + vec3<f32>(4.5678));
    let rings = reprampWood(length(pos.xz + vec2<f32>(n_a, n_b) * 0.05) * 64.0) / 1.8
              - noiseGenWood(pos) * 0.75;

    let pal = wc_woodPalette(seed);
    var texColor = mix(pal.dark * 0.95, pal.light * 0.4, clamp(rings, 0.0, 1.0)) * 1.5;
    texColor = max(vec3<f32>(0.0), texColor);
    let rough = noiseGenWood(pos * 64.0 * vec3<f32>(1.0, 0.2, 1.0)) * 0.1 + 0.9;
    texColor *= rough;
    return clamp(texColor, vec3<f32>(0.0), vec3<f32>(1.0));
}

fn woodTexture(worldXY: vec2<f32>, seed: f32) -> vec3<f32> {
    let style = fract(seed * 0.3819);
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

fn toonStep(nl: f32, levels: i32) -> f32 {
    if (levels <= 1) {
        return 1.0;
    }
    let n = f32(levels - 1);
    return floor(nl * n + 0.5) / n;
}

fn celStep(ndl: f32, levels: i32, shadowBright: f32) -> f32 {
    if (levels <= 1) {
        return select(shadowBright, 1.0, ndl > 0.0);
    }
    let n = f32(levels);
    let bucket = clamp(floor(ndl * n), 0.0, n - 1.0);
    let t = bucket / (n - 1.0);
    return mix(shadowBright, 1.0, t);
}

fn celRopeShading(baseColor: vec3<f32>, n: vec3<f32>, l: vec3<f32>, v: vec3<f32>, levels: i32, cp: vec4<f32>) -> vec3<f32> {
    let shadowBright = cp.x;
    let wrap = cp.y;
    let ndl = dot(n, l);
    let lit = celStep(clamp((ndl + wrap) / (1.0 + wrap), 0.0, 1.0), levels, shadowBright);
    return baseColor * lit;
}

// ── Rubber PBR (ported from Metal) ──

fn rubberPBR(baseColor: vec3<f32>, n: vec3<f32>, l: vec3<f32>, v: vec3<f32>,
             roughness: f32, taut: f32, vCoord: f32, cartoonMode: f32, cartoonLevels: i32,
             matP: vec4<f32>, matP2: vec4<f32>, stretchGloss: f32, stretchSpec: f32) -> vec3<f32> {
    let matteAmount = matP.x;
    let glossAmount = matP.y;
    let diffuseWrap = matP.z;
    let subsurface = matP.w;
    let edgeLight = matP2.x;
    let saturation = matP2.y;

    let nl = clamp(dot(n, l), 0.0, 1.0);
    let nv = clamp(dot(n, v), 0.0, 1.0);
    let h = normalize(l + v);
    let nh = clamp(dot(n, h), 0.0, 1.0);
    let vh = clamp(dot(v, h), 0.0, 1.0);

    let radial = abs(vCoord - 0.5) * 2.0;
    let coreDarken = 1.0 - (1.0 - radial) * (1.0 - radial) * mix(0.15, 0.45, matteAmount);
    let edgeBrighten = pow(radial, 2.5) * mix(0.15, 0.02, matteAmount);
    var albedo = baseColor * coreDarken + vec3<f32>(edgeBrighten);

    let grey = dot(albedo, vec3<f32>(0.299, 0.587, 0.114));
    albedo = mix(vec3<f32>(grey), albedo, vec3<f32>(saturation));

    let wrapDiff = clamp((nl + diffuseWrap) / (1.0 + diffuseWrap), 0.0, 1.0);

    let sssBackNL = clamp(dot(-n, l), 0.0, 1.0);
    let sssBackWrap = clamp((sssBackNL + 0.5) / 1.5, 0.0, 1.0);
    let sssForwardWrap = clamp((-nl + 0.8) / 1.8, 0.0, 1.0);
    let sssViewEdge = pow(1.0 - nv, 2.0);
    let sssContrib = (sssBackWrap * 0.5 + sssForwardWrap * 0.35 + sssViewEdge * 0.15) * subsurface;
    let sssTint = albedo * vec3<f32>(1.25, 0.85, 0.7);

    let ambientBase = mix(0.20, 0.45, matteAmount);
    var diff = albedo * (ambientBase + (1.0 - ambientBase) * wrapDiff) + sssTint * sssContrib;

    var rough = mix(0.18, 0.92, matteAmount) + roughness * 0.1;
    let taut2 = taut * taut;
    let roughFloor = mix(0.85, 0.25, taut) * stretchGloss + 0.85 * (1.0 - stretchGloss);
    rough = mix(rough, rough * roughFloor, taut);
    rough = clamp(rough, 0.05, 0.99);
    let alpha = rough * rough;
    let alpha2 = alpha * alpha;
    let denom = nh * nh * (alpha2 - 1.0) + 1.0;
    let D = alpha2 / (3.14159265 * denom * denom + 1e-5);
    let k = (rough + 1.0) * (rough + 1.0) / 8.0;
    let G1l = nl / (nl * (1.0 - k) + k);
    let G1v = nv / (nv * (1.0 - k) + k);
    let G = G1l * G1v;
    let F0base = mix(0.08, 0.01, matteAmount);
    let F0 = F0base + taut2 * 0.12 * stretchSpec;
    let F = F0 + (1.0 - F0) * pow(1.0 - vh, 5.0);
    let spec = D * G * F / max(4.0 * nl * nv, 0.001);
    let specBoost = 1.0 + (taut * 1.5 + taut2 * 3.0) * stretchSpec;
    let specIntensity = glossAmount * mix(1.2, 0.08, matteAmount);
    var specColor = vec3<f32>(1.0) * spec * specIntensity * specBoost;

    let rimPow = max(1.5, mix(3.0, 6.0, matteAmount) - taut * 1.5 * stretchGloss);
    let rim = pow(1.0 - nv, rimPow) * edgeLight * (1.0 + taut2 * 2.0 * stretchSpec);
    diff += albedo * rim;

    return diff + specColor;
}

fn wormShading(
    baseColor: vec3<f32>,
    n: vec3<f32>,
    l: vec3<f32>,
    v: vec3<f32>,
    worldPos: vec3<f32>,
    uv: vec2<f32>,
    time: f32,
) -> vec3<f32> {
    let u = uv.x;
    let vCoord = uv.y;

    let grooveDepth = frame.wormParams1.x;
    let bellyBright = frame.wormParams1.y;
    let backDark = frame.wormParams1.z;
    let skinNoiseAmt = frame.wormParams1.w;
    let sssStr = frame.wormParams2.x;
    let rough = frame.wormParams2.y;
    let specStr = frame.wormParams2.z;
    let rimStr = frame.wormParams2.w;
    let eyeSize = frame.wormParams3.x;
    let pulseSpeed = frame.wormParams3.y;
    let pulseAmp = frame.wormParams3.z;
    let segFreq = frame.wormParams3.w;

    let segPhase = u * segFreq * 3.14159265 * 2.0;
    let segGroove = smoothstep(0.85, 1.0, abs(sin(segPhase)));
    let grooveDarken = 1.0 - segGroove * grooveDepth;

    let bodyGrad = sin(u * 3.14159265);
    let bellyColor = baseColor * vec3<f32>(bellyBright, bellyBright * 0.957, bellyBright * 0.87);
    let backColor = baseColor * vec3<f32>(backDark, backDark * 1.07, backDark);
    let bellySide = smoothstep(0.3, 0.7, vCoord);
    var skinColor = mix(bellyColor, backColor, vec3<f32>(bellySide));

    let skinNoise = hash21(worldPos.xy * 80.0 + vec2<f32>(u * 5.0)) * skinNoiseAmt - skinNoiseAmt * 0.5;
    skinColor += vec3<f32>(skinNoise);
    skinColor *= grooveDarken;

    let ndl = clamp(dot(n, l), 0.0, 1.0);
    let wrap = 0.45;
    let wrapDiff = clamp((ndl + wrap) / (1.0 + wrap), 0.0, 1.0);

    let sssNL = clamp(dot(-n, l), 0.0, 1.0);
    let sss = sssNL * sssStr * bodyGrad;
    let sssColor = baseColor * vec3<f32>(1.3, 0.5, 0.3);

    let diff = skinColor * (0.25 + 0.75 * wrapDiff) + sssColor * sss;

    let nv = clamp(dot(n, v), 0.0, 1.0);
    let h = normalize(l + v);
    let nh = clamp(dot(n, h), 0.0, 1.0);

    let alpha = rough * rough;
    let alpha2 = alpha * alpha;
    let denom = nh * nh * (alpha2 - 1.0) + 1.0;
    let D = alpha2 / (3.14159265 * denom * denom + 1e-5);
    let k = (rough + 1.0) * (rough + 1.0) / 8.0;
    let G1l = ndl / (ndl * (1.0 - k) + k + 1e-4);
    let G1v = nv / (nv * (1.0 - k) + k + 1e-4);
    let G = G1l * G1v;
    let F0 = 0.06;
    let F = F0 + (1.0 - F0) * pow(1.0 - clamp(dot(v, h), 0.0, 1.0), 5.0);
    let spec = D * G * F / max(4.0 * ndl * nv, 0.001);
    let specColor = vec3<f32>(0.95, 0.97, 1.0) * spec * specStr;

    let fresnel = pow(1.0 - nv, 3.5);
    let rim = vec3<f32>(rimStr, rimStr * 1.5, rimStr * 1.25) * fresnel;

    let headDist = min(u, 1.0 - u);
    let eyeZone = smoothstep(0.06, 0.02, headDist);
    let eyeAngle = vCoord * 3.14159265 * 2.0;
    let eyeLeft = smoothstep(eyeSize, eyeSize * 0.33, abs(eyeAngle - 1.2));
    let eyeRight = smoothstep(eyeSize, eyeSize * 0.33, abs(eyeAngle - 5.08));
    let eyeDot = (eyeLeft + eyeRight) * eyeZone;
    let eyeColor = vec3<f32>(0.02, 0.02, 0.02);

    var c = diff + specColor + rim;
    c = mix(c, eyeColor, vec3<f32>(eyeDot * 0.9));

    let pulse = sin(time * pulseSpeed + u * 8.0) * pulseAmp + 1.0;
    c *= pulse;

    return c;
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
    return shadowMapPCF(worldPos, worldN);
}

fn shadowCompareAt(uv: vec2<f32>, refD: f32) -> f32 {
    let smDims = textureDimensions(shadow_map);
    let sampleUV = clamp(uv, vec2<f32>(0.0), vec2<f32>(1.0));
    let maxCoord = vec2<i32>(i32(smDims.x) - 1, i32(smDims.y) - 1);
    let tc = clamp(
        vec2<i32>(sampleUV * vec2<f32>(f32(smDims.x), f32(smDims.y))),
        vec2<i32>(0),
        maxCoord,
    );
    let sampleDepth = textureLoad(shadow_map, tc, 0);
    return select(0.0, 1.0, refD <= sampleDepth);
}

fn shadowMapPCF(worldPos: vec3<f32>, worldN: vec3<f32>) -> f32 {
    if (frame.lightingParams.w < 0.5) { return 1.0; }

    let lp4 = frame.lightViewProj * vec4<f32>(worldPos, 1.0);
    let ndc3 = lp4.xyz / lp4.w;
    let suv2 = vec2<f32>(ndc3.x * 0.5 + 0.5, 1.0 - (ndc3.y * 0.5 + 0.5));
    if (suv2.x < 0.0 || suv2.x > 1.0 || suv2.y < 0.0 || suv2.y > 1.0) { return 1.0; }

    let biasBase = frame.orthoHalfSize_shadowBias.z;
    let ndlBias = clamp(dot(normalize(worldN), normalize(frame.lightDir_intensity.xyz)), 0.0, 1.0);
    let smBias = biasBase + (1.0 - ndlBias) * biasBase * 2.2;
    let refD = ndc3.z - smBias;
    let smInv = frame.shadowInvSize_unused.x;
    let shadowType = frame.orthoHalfSize_shadowBias.w;

    if (shadowType < 0.5) {
        return shadowCompareAt(suv2, refD);
    }

    if (shadowType < 1.5) {
        var smVis = 0.0;
        let radius = smInv * 4.0;
        for (var i = 0u; i < 32u; i = i + 1u) {
            let p = POISSON_DISK[i];
            smVis += shadowCompareAt(suv2 + p * radius, refD);
        }
        return smoothstep(0.0, 1.0, smVis / 32.0);
    }

    // PCSS
    let lightSize = max(0.001, frame.lightingParams.z);
    let nearPlane = 0.01;
    var blockerSearchRadius = lightSize * (refD - nearPlane) / refD;
    blockerSearchRadius *= 0.65;
    blockerSearchRadius = clamp(blockerSearchRadius, smInv * 1.5, smInv * 10.0);

    let smDims = textureDimensions(shadow_map);
    var blockerSum = 0.0;
    var blockerCount = 0.0;
    for (var i = 0u; i < 24u; i = i + 1u) {
        let p = POISSON_DISK[i];
        let sampleUV = clamp(suv2 + p * blockerSearchRadius, vec2<f32>(0.0), vec2<f32>(1.0));
        let tc = vec2<i32>(i32(sampleUV.x * f32(smDims.x)), i32(sampleUV.y * f32(smDims.y)));
        let sampleDepth = textureLoad(shadow_map, tc, 0);
        let isBlocker = select(0.0, 1.0, sampleDepth < refD);
        blockerSum += sampleDepth * isBlocker;
        blockerCount += isBlocker;
    }

    if (blockerCount < 1.0) { return 1.0; }

    let avgBlocker = blockerSum / blockerCount;
    var penumbraRadius = lightSize * (refD - avgBlocker) / avgBlocker;
    penumbraRadius = max(0.0001, penumbraRadius);
    var filterRadius = penumbraRadius * 1.4;
    filterRadius = clamp(filterRadius, smInv * 3.0, smInv * 20.0);

    var smVis = 0.0;
    for (var i = 0u; i < 32u; i = i + 1u) {
        let p = POISSON_DISK[i];
        smVis += shadowCompareAt(suv2 + p * filterRadius, refD);
    }
    let shadow = smoothstep(0.0, 1.0, smVis / 32.0);
    return pow(shadow, 1.2);
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

struct BoardVSOut {
    @builtin(position) position: vec4<f32>,
    @location(0) worldPos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) worldXY: vec2<f32>,
    @location(3) elevation: f32,
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
    let tableStyle = u32(frame.tableParams.x + 0.5);
    let tableColor1 = frame.tableParams.yzw;
    let tableColor2 = frame.tableParams2.xyz;
    var baseColor: vec3<f32>;
    if (tableStyle == 1u) {
        let t = clamp(worldXY.y * 0.35 + 0.5, 0.0, 1.0);
        baseColor = mix(tableColor1, tableColor2, vec3<f32>(t));
    } else if (tableStyle == 2u) {
        baseColor = tableColor1;
    } else {
        let wood_world_min = frame.woodBoundsMin.xy;
        let wood_world_max = frame.woodBoundsMax.xy;
        let wood_uv = (worldXY - wood_world_min) / (wood_world_max - wood_world_min);
        baseColor = textureSample(wood_baked_tex, noise_sampler, wood_uv).rgb;
    }

    let celMode = frame.visualParams.z > 0.5;

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

        let lightI = frame.lightDir_intensity.w;
        let wrap = 0.4;
        let wrapTerm = clamp((nl + wrap) / (1.0 + wrap), 0.0, 1.0);
        let diff = baseColor * mix(0.25, 0.95, wrapTerm) * lightI;

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
        let specColor = vec3<f32>(0.9, 0.85, 0.75) * spec * 0.12 * (0.2 + 0.8 * fresnel) * lightI;

        c = diff + specColor;

        let tsMode = frame.ambientColor.x;
        var shadow = 1.0;
        if (tsMode < 0.5) {
            shadow = shadowMapPCF(worldPos, worldN);
        } else if (tsMode < 1.5) {
            shadow = planarMaskShadow(worldXY);
        }
        shadow = pow(shadow, 2.2);
        let shadowDark = frame.lightingParams.y;
        c *= mix(shadowDark, 1.0, shadow);

        let ambient = frame.lightingParams.x;
        c += baseColor * ambient;
    }

    let clipPos = frame.viewProj * vec4<f32>(worldPos, 1.0);
    let tableDepth = clipPos.z / clipPos.w + 0.0004;

    var out: TableFSOut;
    out.color = vec4<f32>(c, 1.0);
    out.depth = tableDepth;
    return out;
}

// ── Board vertex/fragment ──

@vertex
fn board_vertex(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) worldXY: vec2<f32>,
) -> BoardVSOut {
    var out: BoardVSOut;
    out.worldPos = position;
    out.position = frame.viewProj * vec4<f32>(position, 1.0);
    out.normal = normal;
    out.worldXY = worldXY;
    out.elevation = position.z;
    return out;
}

@fragment
fn board_fragment(in: BoardVSOut) -> @location(0) vec4<f32> {
    let n = normalize(in.normal);
    let l = normalize(frame.lightDir_intensity.xyz);
    let v = normalize(frame.cameraPos.xyz - in.worldPos);
    let h = normalize(l + v);

    let style = u32(frame.tableParams.x + 0.5);
    let color1 = frame.tableParams.yzw;
    let color2 = frame.tableParams2.xyz;
    let wood_scale = max(frame.woodBoundsMin.w, 0.001);
    let wood_brightness = frame.woodBoundsMin.z;

    var baseColor: vec3<f32>;
    if (style == 1u) {
        let t = clamp(in.worldXY.y * 0.35 + 0.5, 0.0, 1.0);
        baseColor = mix(color1, color2, vec3<f32>(t));
    } else if (style == 2u) {
        baseColor = color1;
    } else {
        let uv = in.worldXY * wood_scale * 0.18;
        let ring = sin(uv.x * 8.0 + textureSample(noise_tex, noise_sampler, uv * 0.4 + vec2<f32>(0.17, 0.31)).r * 3.0);
        let grain = textureSample(noise_tex, noise_sampler, uv * vec2<f32>(0.15, 0.7) + vec2<f32>(0.43, 0.19)).r;
        let t = clamp(0.5 + ring * 0.35 + (grain - 0.5) * 0.25, 0.0, 1.0);
        baseColor = mix(color1, color2, vec3<f32>(t)) * wood_brightness;
    }

    let nl = clamp(dot(n, l), 0.0, 1.0);
    let ambient = frame.lightingParams.x;
    let shadowDark = frame.lightingParams.y;
    let shadowEnabled = frame.lightingParams.w > 0.5;
    var shadow = 1.0;
    if (shadowEnabled) {
        shadow = shadowMapPCF(in.worldPos, n);
        shadow = pow(shadow, 2.2);
    }

    var lit = baseColor * (ambient + nl * frame.lightDir_intensity.w * mix(shadowDark, 1.0, shadow));
    let spec = pow(clamp(dot(n, h), 0.0, 1.0), 60.0) * 0.2 * shadow;
    lit += vec3<f32>(spec);

    if (n.z < 0.5) {
        let edgeDarken = 0.65 + 0.35 * clamp(in.worldPos.z / max(in.elevation, 0.01), 0.0, 1.0);
        lit *= edgeDarken;
    }

    return vec4<f32>(lit, 1.0);
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
    let wp = vec3<f32>(inst.position_radius.x + lp.x, inst.position_radius.y + lp.y, inst.position_radius.z + lp.z);

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

    let celMode = frame.visualParams.z > 0.5;
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

    if (!celMode && frame.lightingParams.w > 0.5) {
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
    let time = frame.timeDrag.x;
    let dragActive = frame.timeDrag.w;
    let u = uv.x;
    let pinch = params.y;

    let w = sin(u * 3.14159265) * sin(u * 3.14159265);
    let energy = frame.ropeMatParams4.y;
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
    if (in.worldPos.z < -0.01) { discard; }

    let celMode = frame.visualParams.z > 0.5;

    let l = normalize(frame.lightDir_intensity.xyz);
    let lightI = frame.lightDir_intensity.w;
    let v = normalize(frame.cameraPos.xyz - in.worldPos);
    var n = normalize(in.normal);
    let taut = clamp(in.params.x, 0.0, 1.0);
    let pinch = clamp(in.params.y, 0.0, 1.0);
    let repel = clamp(in.params.z, 0.0, 1.0);
    let isWorm = in.params.w;

    let microBump = frame.ropeMatParams2.z;
    let contactAOStr = frame.ropeMatParams2.w;
    let liftGlowStr = frame.ropeMatParams3.x;
    let bumpScale = max(frame.ropeMatParams3.y, 0.5);

    let stretchDamp = 1.0 / (1.0 + taut * 2.5);

    let baseUV = in.uv * vec2<f32>(bumpScale, bumpScale * 0.27);
    let ns1 = textureSample(noise_tex, noise_sampler, baseUV).rg;
    let ns2 = textureSample(noise_tex, noise_sampler, baseUV * 2.7 + vec2<f32>(0.31, 0.73)).rg;
    let ns3 = textureSample(noise_tex, noise_sampler, baseUV * 5.3 + vec2<f32>(0.67, 0.19)).rg;

    let n0 = ns1.r * 0.5 + ns2.r * 0.3 + ns3.r * 0.2;
    let n1 = ns1.g * 0.5 + ns2.g * 0.3 + ns3.g * 0.2;

    var tVec = cross(n, vec3<f32>(0.0, 0.0, 1.0));
    if (length(tVec) < 1e-3) { tVec = cross(n, vec3<f32>(0.0, 1.0, 0.0)); }
    tVec = normalize(tVec);
    let bVec = normalize(cross(n, tVec));

    let microAmp = (microBump + pinch * 0.12) * stretchDamp;
    n = normalize(n + (tVec * (n0 - 0.5) + bVec * (n1 - 0.5)) * microAmp);

    var base = in.color;
    let microAO = (n0 + n1 - 1.0) * microBump * stretchDamp * 3.0;
    base *= 1.0 + microAO;
    base = mix(base, base * 1.3, vec3<f32>(pinch * 0.15));

    let roughNoise = textureSample(noise_tex, noise_sampler, baseUV * 1.7 + 0.37).r;
    let rough = roughNoise + pinch * 0.06 + repel * 0.04;

    let repelAO = smoothstep(0.0, 0.5, repel) * contactAOStr;
    let pinchAO = smoothstep(0.0, 0.3, pinch) * contactAOStr * 0.6;
    let contactAO = 1.0 - clamp(repelAO + pinchAO, 0.0, 1.0);

    var c: vec3<f32>;
    if (isWorm > 0.5) {
        c = wormShading(base, n, l, v, in.worldPos, in.uv, frame.timeDrag.x);
        let ambient = frame.lightingParams.x + 0.03;
        let shadow = shadowMapPCF(in.worldPos, n);
        c *= mix(ambient, 1.0, pow(shadow, 2.0));
    } else if (celMode) {
        c = celRopeShading(base, n, l, v, i32(frame.visualParams.w), frame.cartoonParams);
    } else {
        c = rubberPBR(base, n, l, v, rough, taut, in.uv.y, frame.visualParams.z, i32(frame.visualParams.w),
                      frame.ropeMatParams, frame.ropeMatParams2,
                      frame.ropeMatParams3.z, frame.ropeMatParams3.w) * lightI;
        c *= contactAO;

        let liftGlow = clamp(in.worldPos.z / 0.35, 0.0, 1.0);
        c += vec3<f32>(0.03, 0.04, 0.06) * liftGlow * liftGlowStr;

        var shadow = shadowMapPCF(in.worldPos, n);
        shadow = pow(shadow, 2.0);
        let ambient = frame.lightingParams.x + 0.04 * taut;
        c *= mix(ambient, 1.0, shadow);
    }

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
    let time = frame.timeDrag.x;
    let dragActive = frame.timeDrag.w;
    let u = uv.x;
    let pinch = params.y;
    let w = sin(u * 3.14159265) * sin(u * 3.14159265);
    let energy = frame.ropeMatParams4.y;
    let amp = (0.002 + 0.010 * pinch) * energy * (0.25 + 0.75 * dragActive) * w;
    let wave = sin(u * 24.0 + time * 16.0) * 0.65 + sin(u * 11.0 - time * 9.0) * 0.35;
    let displaced = position + normalize(normal) * (wave * amp);

    var o: ShadowVSOut;
    o.position = frame.lightViewProj * vec4<f32>(displaced, 1.0);
    return o;
}

@vertex
fn board_shadow_vertex(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) worldXY: vec2<f32>,
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
    let wp = vec3<f32>(inst.position_radius.x + lp.x, inst.position_radius.y + lp.y, inst.position_radius.z + lp.z);
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
    let spread = i32(round(mix(1.0, 3.0, frame.cartoonParams.z)));
    let s = max(spread, 1);

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
    let cartoonMode = frame.visualParams.z > 0.5;

    let c = textureSample(hdr_texture, linear_sampler, uv).xyz;
    let b = textureSample(bloom_texture, linear_sampler, uv).xyz;
    let combined = c + b * frame.visualParams.y;
    let exposure = max(frame.visualParams.x, 0.001);
    var mapped = vec3<f32>(1.0) - exp(-combined * exposure);
    mapped = pow(clamp(mapped, vec3<f32>(0.0), vec3<f32>(1.0)), vec3<f32>(1.0 / 2.2));

    if (cartoonMode) {
        let edge = edgeDetect(uv);
        let darken = 1.0 - frame.cartoonParams.w;
        mapped = floor(mapped * 32.0 + 0.5) / 32.0;
        mapped = mix(mapped, mapped * darken, edge);
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

// ── Bake wood texture ──

@compute @workgroup_size(8, 8)
fn bake_wood_kernel(@builtin(global_invocation_id) gid: vec3<u32>) {
    let dims = textureDimensions(wood_bake_dst);
    if (gid.x >= dims.x || gid.y >= dims.y) { return; }
    let uv = (vec2<f32>(gid.xy) + 0.5) / vec2<f32>(dims);
    let world_min = frame.woodBoundsMin.xy;
    let world_max = frame.woodBoundsMax.xy;
    let worldXY = mix(world_min, world_max, uv);
    let seed = frame.timeDrag.z;
    let col = clamp(woodTexture(worldXY, seed), vec3<f32>(0.0), vec3<f32>(1.0));
    textureStore(wood_bake_dst, vec2<i32>(gid.xy), vec4<f32>(col, 1.0));
}
