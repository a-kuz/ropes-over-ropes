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
    vec4 uRopeMatParams4;
};

// Hole instances passed as a texture or SSBO. For GLES 3.0 we use a UBO or texture.
// We'll use a uniform array for holes.
const int MAX_HOLES = 64;
uniform int uHoleCount;
uniform vec4 uHoles[MAX_HOLES]; // (x, y, elevation, radius)

uniform sampler2DShadow uShadowMap;
uniform sampler2D uWoodTex;

in vec2 vUV;

layout(location = 0) out vec4 fragColor;

// ============================================================
// WoodCommon.h functions
// ============================================================
float wc_hash21(vec2 p) {
    float n = sin(dot(p, vec2(127.1, 311.7)));
    return fract(n * 43758.5453123);
}

float wc_noise3d(vec3 p) {
    const vec3 s = vec3(7.0, 157.0, 113.0);
    vec3 ip = floor(p);
    vec3 fp = fract(p);
    fp = fp * fp * (3.0 - 2.0 * fp);
    vec4 h = vec4(0.0, s.yz, s.y + s.z) + vec4(dot(ip, s));
    h = mix(fract(sin(h) * 43758.545), fract(sin(h + s.x) * 43758.545), fp.x);
    h.xy = mix(h.xz, h.yw, fp.y);
    return mix(h.x, h.y, fp.z);
}

float wc_fbm3d(vec3 p, int octaves, float roughness) {
    float sum = 0.0, amp = 1.0, tot = 0.0;
    roughness = clamp(roughness, 0.0, 1.0);
    for (int i = 0; i < octaves; i++) {
        sum += amp * wc_noise3d(p);
        tot += amp;
        amp *= roughness;
        p *= 2.0;
    }
    return sum / tot;
}

vec3 wc_randomPos3(float seed) {
    vec4 s = vec4(seed, 0.0, 1.0, 2.0);
    return vec3(wc_hash21(s.xy), wc_hash21(s.xz), wc_hash21(s.xw)) * 100.0 + 100.0;
}

float wc_fbmDistorted(vec3 p) {
    p += (vec3(wc_noise3d(p + wc_randomPos3(0.0)),
               wc_noise3d(p + wc_randomPos3(1.0)),
               wc_noise3d(p + wc_randomPos3(2.0))) * 2.0 - 1.0) * 1.12;
    return wc_fbm3d(p, 8, 0.5);
}

float wc_musgraveFbm(vec3 p, float octaves, float dimension, float lacunarity) {
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

vec3 wc_waveFbmX(vec3 p) {
    float n = p.x * 20.0;
    n += 0.4 * wc_fbm3d(p * 3.0, 3, 3.0);
    return vec3(sin(n) * 0.5 + 0.5, p.yz);
}

float wc_remap01(float f, float in1, float in2) {
    return clamp((f - in1) / (in2 - in1), 0.0, 1.0);
}

void wc_woodPalette(float seed, out vec3 colDark, out vec3 colMid, out vec3 colLight) {
    float hue = fract(seed * 0.618033988);
    if (hue < 0.07) {
        colDark  = vec3(0.04, 0.04, 0.05);
        colMid   = vec3(0.12, 0.12, 0.13);
        colLight = vec3(0.24, 0.24, 0.26);
    } else if (hue < 0.14) {
        colDark  = vec3(0.08, 0.09, 0.10);
        colMid   = vec3(0.20, 0.21, 0.23);
        colLight = vec3(0.36, 0.37, 0.40);
    } else if (hue < 0.21) {
        colDark  = vec3(0.10, 0.09, 0.08);
        colMid   = vec3(0.24, 0.22, 0.20);
        colLight = vec3(0.42, 0.38, 0.35);
    } else if (hue < 0.28) {
        colDark  = vec3(0.06, 0.07, 0.09);
        colMid   = vec3(0.15, 0.16, 0.20);
        colLight = vec3(0.28, 0.30, 0.35);
    } else if (hue < 0.35) {
        colDark  = vec3(0.08, 0.07, 0.06);
        colMid   = vec3(0.20, 0.17, 0.14);
        colLight = vec3(0.36, 0.30, 0.26);
    } else if (hue < 0.42) {
        colDark  = vec3(0.06, 0.04, 0.03);
        colMid   = vec3(0.18, 0.12, 0.08);
        colLight = vec3(0.32, 0.22, 0.16);
    } else if (hue < 0.49) {
        colDark  = vec3(0.03, 0.03, 0.03);
        colMid   = vec3(0.10, 0.09, 0.08);
        colLight = vec3(0.20, 0.18, 0.16);
    } else if (hue < 0.56) {
        colDark  = vec3(0.14, 0.10, 0.06);
        colMid   = vec3(0.30, 0.22, 0.14);
        colLight = vec3(0.50, 0.38, 0.26);
    } else if (hue < 0.63) {
        colDark  = vec3(0.18, 0.17, 0.16);
        colMid   = vec3(0.34, 0.32, 0.30);
        colLight = vec3(0.52, 0.50, 0.47);
    } else if (hue < 0.70) {
        colDark  = vec3(0.06, 0.08, 0.10);
        colMid   = vec3(0.16, 0.19, 0.24);
        colLight = vec3(0.30, 0.34, 0.40);
    } else if (hue < 0.77) {
        colDark  = vec3(0.18, 0.08, 0.06);
        colMid   = vec3(0.34, 0.18, 0.12);
        colLight = vec3(0.52, 0.30, 0.22);
    } else if (hue < 0.84) {
        colDark  = vec3(0.14, 0.14, 0.12);
        colMid   = vec3(0.30, 0.28, 0.26);
        colLight = vec3(0.48, 0.46, 0.42);
    } else if (hue < 0.92) {
        colDark  = vec3(0.05, 0.05, 0.06);
        colMid   = vec3(0.14, 0.14, 0.16);
        colLight = vec3(0.26, 0.26, 0.30);
    } else {
        colDark  = vec3(0.12, 0.10, 0.09);
        colMid   = vec3(0.26, 0.22, 0.20);
        colLight = vec3(0.44, 0.38, 0.34);
    }
}

// ============================================================
// WoodOtavio.h functions
// ============================================================
float noiseGenWood(vec3 p) {
    float wave0 = 0.0;
    float wave1 = 0.0;
    wave0 += sin(dot(p, vec3(-1.316, 0.918, 1.398))) * 0.0783275458;
    wave1 += sin(dot(p, vec3(0.295, -0.176, 2.167))) * 0.0739931495;
    wave0 += sin(dot(p, vec3(-0.926, 1.445, 1.429))) * 0.0716716966;
    wave1 += sin(dot(p, vec3(-1.878, -0.174, 1.258))) * 0.0697839187;
    wave0 += sin(dot(p, vec3(-1.995, 0.661, -0.908))) * 0.0685409863;
    wave1 += sin(dot(p, vec3(-1.770, 1.350, -0.905))) * 0.0630152419;
    wave0 += sin(dot(p, vec3(2.116, -0.021, 1.161))) * 0.0625361712;
    wave1 += sin(dot(p, vec3(0.405, -1.712, -1.855))) * 0.0567751048;
    wave0 += sin(dot(p, vec3(1.346, 0.945, 1.999))) * 0.0556465603;
    wave1 += sin(dot(p, vec3(-0.397, -0.573, 2.495))) * 0.0555747667;
    wave0 += sin(dot(p, vec3(0.103, -2.457, -1.144))) * 0.0516322279;
    wave1 += sin(dot(p, vec3(-0.483, -1.323, 2.330))) * 0.0513093320;
    wave0 += sin(dot(p, vec3(-1.715, -1.810, -1.164))) * 0.0504567036;
    wave1 += sin(dot(p, vec3(2.529, 0.479, 1.011))) * 0.0500811899;
    wave0 += sin(dot(p, vec3(-1.643, -1.814, -1.437))) * 0.0480875812;
    wave1 += sin(dot(p, vec3(1.495, -1.905, -1.648))) * 0.0458268348;
    wave0 += sin(dot(p, vec3(-1.874, 1.559, 1.762))) * 0.0440084357;
    wave1 += sin(dot(p, vec3(1.068, -2.090, 2.081))) * 0.0413624154;
    wave0 += sin(dot(p, vec3(-0.647, -2.197, -2.237))) * 0.0401592830;
    wave1 += sin(dot(p, vec3(-2.146, -2.171, -1.135))) * 0.0391682940;
    wave0 += sin(dot(p, vec3(2.538, -1.854, -1.604))) * 0.0349588163;
    wave1 += sin(dot(p, vec3(1.687, 2.191, -2.270))) * 0.0342888847;
    wave0 += sin(dot(p, vec3(0.205, 2.617, -2.481))) * 0.0338465332;
    wave1 += sin(dot(p, vec3(3.297, -0.440, -2.317))) * 0.0289423448;
    wave0 += sin(dot(p, vec3(1.068, -1.944, 3.432))) * 0.0286404261;
    wave1 += sin(dot(p, vec3(-3.681, 1.068, 1.789))) * 0.0273625684;
    wave0 += sin(dot(p, vec3(3.116, 2.631, -1.658))) * 0.0259772492;
    wave1 += sin(dot(p, vec3(-1.992, -2.902, -2.954))) * 0.0245830241;
    wave0 += sin(dot(p, vec3(-2.409, -2.374, 3.116))) * 0.0245592756;
    wave1 += sin(dot(p, vec3(0.790, 1.768, 4.196))) * 0.0244078334;
    wave0 += sin(dot(p, vec3(-3.289, 1.007, 3.148))) * 0.0241328015;
    wave1 += sin(dot(p, vec3(3.421, -2.663, 3.262))) * 0.0199736126;
    wave0 += sin(dot(p, vec3(3.062, 2.621, 3.649))) * 0.0199230290;
    wave1 += sin(dot(p, vec3(4.422, -2.206, 2.621))) * 0.0192399437;
    wave0 += sin(dot(p, vec3(2.714, 3.022, 4.200))) * 0.0182510631;
    wave1 += sin(dot(p, vec3(-0.451, 4.143, -4.142))) * 0.0181293526;
    wave0 += sin(dot(p, vec3(-5.838, -0.360, -1.536))) * 0.0175114826;
    wave1 += sin(dot(p, vec3(-0.278, -4.565, 4.149))) * 0.0170799341;
    wave0 += sin(dot(p, vec3(-5.893, -0.163, -2.141))) * 0.0167655258;
    wave1 += sin(dot(p, vec3(4.855, -4.153, 0.606))) * 0.0163155335;
    wave0 += sin(dot(p, vec3(4.498, 0.987, -4.488))) * 0.0162770287;
    wave1 += sin(dot(p, vec3(-1.463, 5.321, -3.315))) * 0.0162569125;
    wave0 += sin(dot(p, vec3(-1.862, 4.386, 4.749))) * 0.0154338176;
    wave1 += sin(dot(p, vec3(0.563, 3.616, -5.751))) * 0.0151952226;
    wave0 += sin(dot(p, vec3(-0.126, 2.569, -6.349))) * 0.0151089405;
    wave1 += sin(dot(p, vec3(-5.094, 4.759, 0.186))) * 0.0147947096;
    wave0 += sin(dot(p, vec3(1.319, 5.713, 3.845))) * 0.0147035221;
    wave1 += sin(dot(p, vec3(7.141, -0.327, 1.420))) * 0.0140573910;
    wave0 += sin(dot(p, vec3(3.888, 6.543, 0.547))) * 0.0133309850;
    wave1 += sin(dot(p, vec3(-1.898, -3.563, -6.483))) * 0.0133171360;
    wave0 += sin(dot(p, vec3(1.719, 7.769, 0.340))) * 0.0126913718;
    wave1 += sin(dot(p, vec3(-2.210, -7.836, 0.102))) * 0.0123746071;
    wave0 += sin(dot(p, vec3(6.248, -5.451, 1.866))) * 0.0117861898;
    wave1 += sin(dot(p, vec3(1.627, -7.066, -4.732))) * 0.0115417453;
    wave0 += sin(dot(p, vec3(4.099, -7.704, 1.474))) * 0.0112591564;
    wave1 += sin(dot(p, vec3(7.357, 3.788, 3.204))) * 0.0112252325;
    wave0 += sin(dot(p, vec3(-2.797, 6.208, 6.253))) * 0.0107206906;
    wave1 += sin(dot(p, vec3(6.130, -5.335, -4.650))) * 0.0105693992;
    wave0 += sin(dot(p, vec3(5.276, -5.576, -5.438))) * 0.0105139072;
    wave1 += sin(dot(p, vec3(9.148, 2.530, -0.383))) * 0.0103996383;
    wave0 += sin(dot(p, vec3(3.894, 2.559, 8.357))) * 0.0103161113;
    wave1 += sin(dot(p, vec3(-6.604, 8.024, -0.289))) * 0.0094066875;
    wave0 += sin(dot(p, vec3(-5.925, 6.505, -6.403))) * 0.0089444733;
    wave1 += sin(dot(p, vec3(9.085, 10.331, -0.451))) * 0.0069245599;
    wave0 += sin(dot(p, vec3(-8.228, 6.323, -9.900))) * 0.0066251015;
    wave1 += sin(dot(p, vec3(10.029, -3.802, 12.151))) * 0.0058122824;
    wave0 += sin(dot(p, vec3(-10.151, -6.513, -11.063))) * 0.0057522358;
    wave1 += sin(dot(p, vec3(-1.773, -16.284, 2.828))) * 0.0056578101;
    wave0 += sin(dot(p, vec3(11.081, 8.687, -9.852))) * 0.0054614334;
    wave1 += sin(dot(p, vec3(-3.941, -4.386, 16.191))) * 0.0054454253;
    wave0 += sin(dot(p, vec3(-6.742, 2.133, -17.268))) * 0.0050050132;
    wave1 += sin(dot(p, vec3(-10.743, 5.698, 14.975))) * 0.0048323955;
    wave0 += sin(dot(p, vec3(-9.603, 12.472, 14.542))) * 0.0043264378;
    wave1 += sin(dot(p, vec3(13.515, 14.345, 8.481))) * 0.0043208884;
    wave0 += sin(dot(p, vec3(-10.330, 16.209, -9.742))) * 0.0043013736;
    wave1 += sin(dot(p, vec3(-8.580, -6.628, 19.191))) * 0.0042005922;
    wave0 += sin(dot(p, vec3(-17.154, 10.620, 11.828))) * 0.0039482427;
    wave1 += sin(dot(p, vec3(16.330, 14.123, -10.420))) * 0.0038474789;
    wave0 += sin(dot(p, vec3(-21.275, 10.768, -3.252))) * 0.0038320501;
    wave1 += sin(dot(p, vec3(1.744, 7.922, 23.152))) * 0.0037560829;
    wave0 += sin(dot(p, vec3(-3.895, 21.321, 12.006))) * 0.0037173885;
    wave1 += sin(dot(p, vec3(-22.705, 2.543, 10.695))) * 0.0036484394;
    wave0 += sin(dot(p, vec3(-13.053, -16.634, -13.993))) * 0.0036291121;
    wave1 += sin(dot(p, vec3(22.697, -11.230, 1.417))) * 0.0036280459;
    wave0 += sin(dot(p, vec3(20.646, 14.602, 3.400))) * 0.0036055008;
    wave1 += sin(dot(p, vec3(5.824, -8.717, -23.680))) * 0.0035501527;
    wave0 += sin(dot(p, vec3(6.691, 15.499, 20.079))) * 0.0035029508;
    wave1 += sin(dot(p, vec3(9.926, -22.778, 9.144))) * 0.0034694278;
    wave0 += sin(dot(p, vec3(-9.552, -27.491, 2.197))) * 0.0031359281;
    wave1 += sin(dot(p, vec3(21.071, -17.991, -11.566))) * 0.0030453280;
    wave0 += sin(dot(p, vec3(9.780, 1.783, 28.536))) * 0.0030251754;
    wave1 += sin(dot(p, vec3(8.738, -18.373, 22.725))) * 0.0029960272;
    wave0 += sin(dot(p, vec3(14.105, 25.703, -8.834))) * 0.0029840058;
    wave1 += sin(dot(p, vec3(-24.926, -17.766, -4.740))) * 0.0029487709;
    wave0 += sin(dot(p, vec3(1.060, -1.570, 32.535))) * 0.0027980099;
    wave1 += sin(dot(p, vec3(-24.532, -19.629, -16.759))) * 0.0025538949;
    wave0 += sin(dot(p, vec3(28.772, -21.183, -9.935))) * 0.0024494819;
    wave1 += sin(dot(p, vec3(-28.413, 22.959, 8.338))) * 0.0024236674;
    wave0 += sin(dot(p, vec3(-27.664, 22.197, 13.301))) * 0.0023965996;
    wave1 += sin(dot(p, vec3(-27.421, 20.643, 18.713))) * 0.0023203498;
    wave0 += sin(dot(p, vec3(18.961, -7.189, 35.907))) * 0.0021967023;
    wave1 += sin(dot(p, vec3(-23.949, 4.885, 33.762))) * 0.0021727461;
    wave0 += sin(dot(p, vec3(35.305, 8.594, 20.564))) * 0.0021689816;
    wave1 += sin(dot(p, vec3(30.364, -11.608, -27.199))) * 0.0021357139;
    wave0 += sin(dot(p, vec3(34.268, 26.742, 0.958))) * 0.0020807976;
    wave1 += sin(dot(p, vec3(-26.376, -17.313, -32.023))) * 0.0020108850;
    wave0 += sin(dot(p, vec3(31.860, -32.181, -2.834))) * 0.0019919601;
    wave1 += sin(dot(p, vec3(25.590, 32.340, 21.381))) * 0.0019446179;
    wave0 += sin(dot(p, vec3(-17.771, -23.941, 37.324))) * 0.0018898258;
    wave1 += sin(dot(p, vec3(-38.699, 19.953, -22.675))) * 0.0018379538;
    wave0 += sin(dot(p, vec3(-46.284, 11.672, -15.411))) * 0.0017980056;
    wave1 += sin(dot(p, vec3(-32.023, -43.976, -7.378))) * 0.0016399251;
    wave0 += sin(dot(p, vec3(-42.390, -21.165, -31.889))) * 0.0015752176;
    wave1 += sin(dot(p, vec3(-18.949, -40.461, 39.107))) * 0.0015141244;
    wave0 += sin(dot(p, vec3(-21.507, -5.939, -58.531))) * 0.0014339601;
    wave1 += sin(dot(p, vec3(-51.745, -43.821, 9.651))) * 0.0013096306;
    wave0 += sin(dot(p, vec3(39.239, 25.971, -52.615))) * 0.0012701774;
    wave1 += sin(dot(p, vec3(-49.669, -35.051, -36.306))) * 0.0012661695;
    wave0 += sin(dot(p, vec3(-49.996, 35.309, 38.460))) * 0.0012398870;
    wave1 += sin(dot(p, vec3(27.000, -65.904, -36.267))) * 0.0011199347;
    wave0 += sin(dot(p, vec3(-52.523, -26.557, 57.693))) * 0.0010856391;
    wave1 += sin(dot(p, vec3(-42.670, 0.269, -71.125))) * 0.0010786551;
    wave0 += sin(dot(p, vec3(-9.377, 64.575, -68.151))) * 0.0009468199;
    wave1 += sin(dot(p, vec3(14.571, -29.160, 106.329))) * 0.0008019719;
    wave0 += sin(dot(p, vec3(-21.549, 103.887, 36.882))) * 0.0007939609;
    wave1 += sin(dot(p, vec3(-42.781, 110.966, -9.070))) * 0.0007473261;
    wave0 += sin(dot(p, vec3(-112.686, 18.296, -37.920))) * 0.0007409259;
    wave1 += sin(dot(p, vec3(71.493, 33.838, -96.931))) * 0.0007121903;
    return wave0 + wave1;
}

float reprampWood(float x) {
    return pow(sin(x) * 0.5 + 0.5, 8.0) + cos(x) * 0.7 + 0.7;
}

vec3 otavioWoodTexture(vec2 worldXY, float seed) {
    float seedAngle = seed * 2.718;
    float sa = sin(seedAngle), ca = cos(seedAngle);
    vec2 rotated = vec2(worldXY.x * ca - worldXY.y * sa,
                        worldXY.x * sa + worldXY.y * ca);
    vec2 offset = vec2(wc_hash21(vec2(seed, seed * 5.37)) * 40.0 - 20.0,
                       wc_hash21(vec2(seed * 2.83, seed)) * 40.0 - 20.0);
    vec3 pos = vec3((rotated + offset) * 1.1, floor(fract(seed * 0.41) * 8.0));

    float scaleVar = mix(0.8, 1.2, wc_hash21(vec2(seed * 1.47, 0.0)));
    pos.xy *= scaleVar;

    float rings = reprampWood(length(pos.xz + vec2(noiseGenWood(pos * vec3(8.0, 1.5, 8.0)),
                                                     noiseGenWood(-pos * vec3(8.0, 1.5, 8.0) + 4.5678)) * 0.05) * 64.0) / 1.8;
    rings -= noiseGenWood(pos * 1.0) * 0.75;

    vec3 colDark, colMid, colLight;
    wc_woodPalette(seed, colDark, colMid, colLight);

    vec3 texColor = mix(colDark * 0.95, colLight * 0.4, clamp(rings, 0.0, 1.0)) * 1.5;
    texColor = max(vec3(0.0), texColor);
    float rough = noiseGenWood(pos * 64.0 * vec3(1.0, 0.2, 1.0)) * 0.1 + 0.9;
    texColor *= rough;
    return clamp(texColor, 0.0, 1.0);
}

// ============================================================
// woodSolidTexture (from Base.metal)
// ============================================================
vec3 woodSolidTexture(vec2 worldXY, float seed) {
    float seedAngle = seed * 2.399;
    float sa = sin(seedAngle), ca = cos(seedAngle);
    vec2 rotated = vec2(worldXY.x * ca - worldXY.y * sa,
                        worldXY.x * sa + worldXY.y * ca);
    vec2 offset = vec2(wc_hash21(vec2(seed, seed * 7.13)) * 40.0 - 20.0,
                       wc_hash21(vec2(seed * 3.71, seed)) * 40.0 - 20.0);
    vec3 p = vec3((rotated + offset) * 1.1, floor(fract(seed * 0.37) * 8.0));

    float scaleVar = mix(0.8, 1.2, wc_hash21(vec2(seed * 1.23, 0.0)));
    p.xy *= scaleVar;

    float n1 = wc_fbmDistorted(p * vec3(7.8, 1.17, 1.17));
    n1 = mix(n1, 1.0, 0.2);
    float n2 = mix(wc_musgraveFbm(vec3(n1 * 4.6), 8.0, 0.0, 2.5), n1, 0.85);
    float dirt = 1.0 - wc_musgraveFbm(wc_waveFbmX(p * vec3(0.01, 0.15, 0.15)), 15.0, 0.26, 2.4) * 0.4;
    float grain = 1.0 - smoothstep(0.2, 1.0, wc_musgraveFbm(p * vec3(500.0, 6.0, 1.0), 2.0, 2.0, 2.5)) * 0.2;
    n2 *= dirt * grain;

    vec3 colDark, colMid, colLight;
    wc_woodPalette(seed, colDark, colMid, colLight);

    vec3 col = mix(mix(colDark, colMid, wc_remap01(n2, 0.19, 0.56)), colLight, wc_remap01(n2, 0.56, 1.0));
    return clamp(pow(col, vec3(0.88)), 0.0, 1.0);
}

// ============================================================
// plankWoodTexture (from Base.metal)
// ============================================================
vec3 plankWoodTexture(vec2 worldXY, float seed) {
    float seedAngle = seed * 1.73;
    float sa = sin(seedAngle), ca = cos(seedAngle);
    vec2 rotated = vec2(worldXY.x * ca - worldXY.y * sa,
                        worldXY.x * sa + worldXY.y * ca);
    vec2 globalOffset = vec2(wc_hash21(vec2(seed, seed * 5.17)) * 30.0 - 15.0,
                             wc_hash21(vec2(seed * 2.91, seed)) * 30.0 - 15.0);
    vec2 wp = rotated + globalOffset;

    float plankWidth = mix(0.28, 0.45, wc_hash21(vec2(seed * 3.14, 7.0)));
    float gapWidth = 0.006;

    float plankY = wp.y / plankWidth;
    float plankRow = floor(plankY);
    float plankFrac = fract(plankY);

    float rowOffset = wc_hash21(vec2(plankRow, seed * 11.3)) * 2.0;
    float plankLen = mix(0.8, 1.6, wc_hash21(vec2(plankRow, seed * 4.7)));
    float plankX = (wp.x + rowOffset) / plankLen;
    float plankCol = floor(plankX);
    float plankFracX = fract(plankX);

    float gapY = smoothstep(0.0, gapWidth / plankWidth, plankFrac) *
                 smoothstep(0.0, gapWidth / plankWidth, 1.0 - plankFrac);
    float gapX = smoothstep(0.0, gapWidth / plankLen, plankFracX) *
                 smoothstep(0.0, gapWidth / plankLen, 1.0 - plankFracX);
    float gapMask = gapX * gapY;

    float plankId = wc_hash21(vec2(plankRow * 17.3 + plankCol * 7.1, seed));
    float plankSeed = seed + plankId * 100.0;

    float bendK = (wc_hash21(vec2(plankSeed, 3.3)) - 0.5) * 0.08;
    float localX = plankFracX * plankLen;
    float localY = plankFrac * plankWidth;
    float bentY = localY + sin(localX * 3.14159 / plankLen) * bendK;

    vec3 p = vec3(localX * 0.016, bentY * 0.016, plankId * 7.0);

    float n1 = wc_fbmDistorted(p * vec3(1.0, 0.15, 0.15) * 8.0);
    n1 = mix(n1, 1.0, 0.2);
    float n2 = wc_musgraveFbm(vec3(n1 * 4.6), 8.0, 0.0, 2.5);
    float n3 = mix(n2, n1, 0.85);
    vec3 q = wc_waveFbmX(p * vec3(0.01, 0.15, 0.15));
    float dirt = 1.0 - wc_musgraveFbm(q, 15.0, 0.26, 2.4) * 0.4;
    float grain = 1.0 - smoothstep(0.2, 1.0, wc_musgraveFbm(p * vec3(500.0, 6.0, 1.0), 2.0, 2.0, 2.5)) * 0.2;
    n3 *= dirt * grain;

    vec3 colDark, colMid, colLight;
    wc_woodPalette(plankSeed, colDark, colMid, colLight);

    float plankBrightness = mix(0.85, 1.15, wc_hash21(vec2(plankSeed * 1.7, 2.0)));
    vec3 col = mix(mix(colDark, colMid, wc_remap01(n3, 0.185, 0.565)), colLight, wc_remap01(n3, 0.565, 1.0));
    col *= plankBrightness;

    float edgeDarken = smoothstep(0.0, 0.04, plankFrac) * smoothstep(0.0, 0.04, 1.0 - plankFrac) *
                       smoothstep(0.0, 0.03, plankFracX) * smoothstep(0.0, 0.03, 1.0 - plankFracX);
    col *= mix(0.7, 1.0, edgeDarken);

    vec3 gapColor = colDark * 0.15;
    col = mix(gapColor, col, gapMask);

    return clamp(pow(col, vec3(0.88)), 0.0, 1.0);
}

// ============================================================
// gradientTexture (from Base.metal)
// ============================================================
vec3 gradientTexture(vec2 worldXY, float seed) {
    float seedAngle = seed * 1.618;
    float sa = sin(seedAngle), ca = cos(seedAngle);
    vec2 rp = vec2(worldXY.x * ca - worldXY.y * sa,
                    worldXY.x * sa + worldXY.y * ca);
    vec2 off = vec2(wc_hash21(vec2(seed * 2.3, 1.0)) * 20.0 - 10.0,
                    wc_hash21(vec2(1.0, seed * 3.7)) * 20.0 - 10.0);
    rp += off;

    float gradType = fract(seed * 0.7236);

    vec3 c1, c2, c3;
    float palIdx = fract(seed * 0.4618);
    if (palIdx < 0.06) {
        c1 = vec3(0.82, 0.82, 0.84); c2 = vec3(0.62, 0.63, 0.66); c3 = vec3(0.44, 0.45, 0.48);
    } else if (palIdx < 0.12) {
        c1 = vec3(0.90, 0.88, 0.85); c2 = vec3(0.70, 0.68, 0.64); c3 = vec3(0.50, 0.48, 0.44);
    } else if (palIdx < 0.18) {
        c1 = vec3(0.86, 0.86, 0.88); c2 = vec3(0.56, 0.57, 0.62); c3 = vec3(0.30, 0.32, 0.38);
    } else if (palIdx < 0.24) {
        c1 = vec3(0.78, 0.76, 0.72); c2 = vec3(0.52, 0.50, 0.46); c3 = vec3(0.30, 0.28, 0.26);
    } else if (palIdx < 0.30) {
        c1 = vec3(0.88, 0.86, 0.82); c2 = vec3(0.58, 0.54, 0.48); c3 = vec3(0.34, 0.30, 0.26);
    } else if (palIdx < 0.36) {
        c1 = vec3(0.06, 0.06, 0.08); c2 = vec3(0.18, 0.18, 0.22); c3 = vec3(0.34, 0.34, 0.40);
    } else if (palIdx < 0.42) {
        c1 = vec3(0.04, 0.05, 0.07); c2 = vec3(0.12, 0.14, 0.20); c3 = vec3(0.24, 0.28, 0.38);
    } else if (palIdx < 0.48) {
        c1 = vec3(0.08, 0.07, 0.06); c2 = vec3(0.22, 0.20, 0.18); c3 = vec3(0.40, 0.36, 0.32);
    } else if (palIdx < 0.54) {
        c1 = vec3(0.02, 0.04, 0.06); c2 = vec3(0.08, 0.16, 0.24); c3 = vec3(0.18, 0.30, 0.42);
    } else if (palIdx < 0.60) {
        c1 = vec3(0.84, 0.84, 0.82); c2 = vec3(0.48, 0.48, 0.46); c3 = vec3(0.20, 0.20, 0.20);
    } else if (palIdx < 0.66) {
        c1 = vec3(0.04, 0.06, 0.04); c2 = vec3(0.12, 0.20, 0.14); c3 = vec3(0.24, 0.38, 0.28);
    } else if (palIdx < 0.72) {
        c1 = vec3(0.05, 0.03, 0.06); c2 = vec3(0.16, 0.10, 0.22); c3 = vec3(0.30, 0.20, 0.40);
    } else if (palIdx < 0.78) {
        c1 = vec3(0.92, 0.90, 0.86); c2 = vec3(0.74, 0.72, 0.68); c3 = vec3(0.56, 0.54, 0.50);
    } else if (palIdx < 0.84) {
        c1 = vec3(0.10, 0.10, 0.12); c2 = vec3(0.26, 0.26, 0.30); c3 = vec3(0.46, 0.46, 0.52);
    } else if (palIdx < 0.92) {
        c1 = vec3(0.80, 0.82, 0.86); c2 = vec3(0.54, 0.56, 0.62); c3 = vec3(0.32, 0.34, 0.40);
    } else {
        c1 = vec3(0.06, 0.05, 0.04); c2 = vec3(0.18, 0.16, 0.14); c3 = vec3(0.36, 0.32, 0.28);
    }

    float t;
    if (gradType < 0.25) {
        float warp = wc_noise3d(vec3(rp * 0.4, seed)) * 0.15;
        t = clamp(rp.y * 0.25 + 0.5 + warp, 0.0, 1.0);
    } else if (gradType < 0.45) {
        float dist = length(rp) * 0.6;
        float warp = wc_noise3d(vec3(rp * 1.5, seed)) * 0.2;
        t = clamp((dist + warp) * 0.4, 0.0, 1.0);
    } else if (gradType < 0.60) {
        float diag = (rp.x + rp.y) * 0.3;
        float warp = wc_noise3d(vec3(rp * 0.6, seed)) * 0.2;
        t = clamp(diag + warp + 0.5, 0.0, 1.0);
    } else if (gradType < 0.75) {
        float angle = atan(rp.y, rp.x) / 6.28318 + 0.5;
        float warp = wc_noise3d(vec3(rp * 2.0, seed * 1.3)) * 0.08;
        t = fract(angle + warp);
    } else if (gradType < 0.88) {
        float wave  = sin(rp.x * 2.0 + wc_noise3d(vec3(rp * 0.8, seed)) * 1.5) * 0.5 + 0.5;
        float wave2 = sin(rp.y * 1.8 + wc_noise3d(vec3(rp * 0.6, seed + 5.0)) * 1.2) * 0.5 + 0.5;
        t = clamp(wave * 0.55 + wave2 * 0.45, 0.0, 1.0);
    } else {
        float n = wc_fbm3d(vec3(rp * 0.4, seed * 0.7), 5, 0.5);
        t = clamp(n * 0.8 + 0.3, 0.0, 1.0);
    }

    vec3 col;
    if (t < 0.5) {
        col = mix(c1, c2, t * 2.0);
    } else {
        col = mix(c2, c3, (t - 0.5) * 2.0);
    }

    float micro = wc_noise3d(vec3(rp * 8.0, seed * 2.0)) * 0.03 - 0.015;
    col += micro;

    return clamp(col, 0.0, 1.0);
}

// ============================================================
// woodTexture dispatcher
// ============================================================
vec3 woodTexture(vec2 uv, vec2 worldXY, float seed) {
    float style = fract(seed * 0.3819);
    if (style < 0.25) {
        return woodSolidTexture(worldXY, seed);
    } else if (style < 0.50) {
        return gradientTexture(worldXY, seed);
    } else {
        return otavioWoodTexture(worldXY, seed);
    }
}

// ============================================================
// Shadow (same PCSS pipeline)
// ============================================================
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
    vec2 suv = vec2(ndc.x * 0.5 + 0.5, ndc.y * 0.5 + 0.5);
    if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) return 1.0;

    float biasBase = uOrthoHalfSizeShadowBias.z;
    float ndl = clamp(dot(normalize(worldN), normalize(uLightDirIntensity.xyz)), 0.0, 1.0);
    float bias = biasBase + (1.0 - ndl) * biasBase * 2.2;
    vec2 invSize = uShadowInvSize.xy;
    float depthRef = ndc.z * 0.5 + 0.5 - bias;
    float shadowType = uOrthoHalfSizeShadowBias.w;

    if (shadowType < 0.5) {
        return texture(uShadowMap, vec3(suv, depthRef));
    }
    if (shadowType < 1.5) {
        float filterRadius = invSize.x * 4.0;
        float shadow = pcssFilter(suv, depthRef, filterRadius);
        return smoothstep(0.0, 1.0, shadow);
    }

    float lightSize = max(0.001, uLightingParams.z);
    float nearPlane = 0.01;
    float blockerSearchRadius = lightSize * (depthRef - nearPlane) / depthRef;
    blockerSearchRadius *= 0.65;
    blockerSearchRadius = clamp(blockerSearchRadius, invSize.x * 1.5, invSize.x * 10.0);
    float avgBlockerDepth = findBlocker(suv, depthRef, blockerSearchRadius);
    if (avgBlockerDepth < 0.0) return 1.0;
    float penumbraRadius = lightSize * (depthRef - avgBlockerDepth) / avgBlockerDepth;
    penumbraRadius = max(0.0001, penumbraRadius);
    float filterRadius = penumbraRadius * 1.4;
    filterRadius = clamp(filterRadius, invSize.x * 3.0, invSize.x * 20.0);
    float shadow = pcssFilter(suv, depthRef, filterRadius);
    shadow = smoothstep(0.0, 1.0, shadow);
    shadow = pow(shadow, 1.2);
    return shadow;
}

// ============================================================
// Cel shading for table
// ============================================================
float celStep(float ndl, int levels, float shadowBright) {
    if (levels <= 1) return ndl > 0.0 ? 1.0 : shadowBright;
    float n = float(levels);
    float bucket = floor(ndl * n);
    bucket = clamp(bucket, 0.0, n - 1.0);
    float t = bucket / (n - 1.0);
    return mix(shadowBright, 1.0, t);
}

vec3 celTableShading(vec3 baseColor, vec3 n, vec3 l, int levels, vec4 cp) {
    float shadowBright = cp.x;
    float wrap = cp.y;
    float ndl = dot(n, l);
    float lit = celStep(clamp((ndl + wrap * 2.0) / (1.0 + wrap * 2.0), 0.0, 1.0), levels, shadowBright);
    return baseColor * lit;
}

// ============================================================
// Main
// ============================================================
void main() {
    vec2 uv = vUV;
    vec2 ndc = uv * 2.0 - 1.0;

    // OpenGL NDC near = -1, far = +1 (Metal: near = 0, far = 1)
    vec4 nearW = uInvViewProj * vec4(ndc.x, ndc.y, -1.0, 1.0);
    vec4 farW  = uInvViewProj * vec4(ndc.x, ndc.y, 1.0, 1.0);
    vec3 nearP = nearW.xyz / nearW.w;
    vec3 farP  = farW.xyz / farW.w;
    vec3 dir = farP - nearP;
    float t = -nearP.z / dir.z;
    vec2 worldXY = nearP.xy + dir.xy * t;

    vec3 worldPos = vec3(worldXY, 0.0);

    vec4 clipPos = uViewProj * vec4(worldPos, 1.0);
    // Metal NDC Z is [0,1], OpenGL NDC Z is [-1,1]. gl_FragDepth is [0,1] in both.
    // Convert OpenGL NDC to depth buffer range:
    float tableDepth = (clipPos.z / clipPos.w) * 0.5 + 0.5 + 0.0004;

    float squareHoles = uTableParams2.w;
    for (int i = 0; i < uHoleCount; i++) {
        vec2 holeCenter = uHoles[i].xy;
        float holeR = uHoles[i].w;
        vec2 d = abs(worldXY - holeCenter);
        float dist = squareHoles > 0.5 ? max(d.x, d.y) : length(worldXY - holeCenter);
        if (dist < holeR) {
            tableDepth = 1.0;
            break;
        }
    }

    vec3 worldN = vec3(0.0, 0.0, 1.0);

    float tableStyle = uTableParams.x;
    float woodBrightness = uWoodBoundsMin.z;
    float woodPatternScale = uWoodBoundsMin.w;
    if (woodBrightness <= 0.0) woodBrightness = 1.0;
    if (woodPatternScale <= 0.0) woodPatternScale = 1.0;
    vec3 baseColor;
    if (tableStyle < 0.5) {
        float levelSeed = uTimeDrag.z;
        if (uWoodBoundsMax.w > 0.5) {
            vec2 woodUV = (worldXY - uWoodBoundsMin.xy) / (uWoodBoundsMax.xy - uWoodBoundsMin.xy);
            baseColor = texture(uWoodTex, woodUV).rgb;
        } else {
            vec2 scaledXY = worldXY * woodPatternScale;
            baseColor = woodTexture(uv, scaledXY, levelSeed);
        }
        baseColor = clamp(baseColor * woodBrightness, 0.0, 1.0);
    } else if (tableStyle < 1.5) {
        vec3 c1 = uTableParams.yzw;
        vec3 c2 = uTableParams2.xyz;
        baseColor = mix(c1, c2, uv.y);
    } else {
        baseColor = uTableParams.yzw;
    }

    vec3 l = normalize(uLightDirIntensity.xyz);
    float lightI = uLightDirIntensity.w;
    vec3 v = normalize(uCameraPos.xyz - worldPos);
    float nl = clamp(dot(worldN, l), 0.0, 1.0);
    float nv = clamp(dot(worldN, v), 0.0, 1.0);
    vec3 h = normalize(l + v);
    float nh = clamp(dot(worldN, h), 0.0, 1.0);

    float cartoonMode   = uVisualParams.z;
    int   cartoonLevels = int(uVisualParams.w);

    vec3 c;
    if (cartoonMode > 0.5) {
        vec3 flatColor = vec3(0.88, 0.88, 0.88);
        c = celTableShading(flatColor, worldN, l, cartoonLevels, uCartoonParams);
    } else {
        float wrap = 0.4;
        float wrapTerm = clamp((nl + wrap) / (1.0 + wrap), 0.0, 1.0);
        vec3 diff = baseColor * mix(0.25, 0.95, wrapTerm) * lightI;

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
        vec3 specColor = vec3(0.9, 0.85, 0.75) * spec * 0.12 * (0.2 + 0.8 * fresnel) * lightI;

        c = diff + specColor;

        float shadow = shadowVisibility(worldPos, worldN);
        shadow = pow(shadow, 2.2);
        float shadowDark = uLightingParams.y;
        c *= mix(shadowDark, 1.0, shadow);

        float ambient = uLightingParams.x;
        c += baseColor * ambient;
    }

    fragColor = vec4(c, 1.0);
    gl_FragDepth = tableDepth;
}
