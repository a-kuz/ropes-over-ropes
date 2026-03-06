#version 300 es
precision highp float;
precision highp sampler2D;
precision highp sampler2DShadow;
precision highp sampler3D;

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

const int MAX_HOLES = 64;
uniform int uHoleCount;
uniform vec4 uHoles[MAX_HOLES];

uniform sampler2DShadow uShadowMap;
uniform sampler3D uBoardWoodVolumeTex;

in vec3 vWorldPos;
in vec3 vNormal;
in vec2 vWorldXY;
in float vElevation;

layout(location = 0) out vec4 fragColor;

// ============================================================
// WoodCommon noise
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

void wc_woodPalette(float seed, out vec3 colDark, out vec3 colMid, out vec3 colLight) {
    float hue = fract(seed * 0.618033988);
    if (hue < 0.07) { colDark = vec3(0.04,0.04,0.05); colMid = vec3(0.12,0.12,0.13); colLight = vec3(0.24,0.24,0.26); }
    else if (hue < 0.14) { colDark = vec3(0.08,0.09,0.10); colMid = vec3(0.20,0.21,0.23); colLight = vec3(0.36,0.37,0.40); }
    else if (hue < 0.21) { colDark = vec3(0.10,0.09,0.08); colMid = vec3(0.24,0.22,0.20); colLight = vec3(0.42,0.38,0.35); }
    else if (hue < 0.28) { colDark = vec3(0.06,0.07,0.09); colMid = vec3(0.15,0.16,0.20); colLight = vec3(0.28,0.30,0.35); }
    else if (hue < 0.35) { colDark = vec3(0.08,0.07,0.06); colMid = vec3(0.20,0.17,0.14); colLight = vec3(0.36,0.30,0.26); }
    else if (hue < 0.42) { colDark = vec3(0.06,0.04,0.03); colMid = vec3(0.18,0.12,0.08); colLight = vec3(0.32,0.22,0.16); }
    else if (hue < 0.49) { colDark = vec3(0.03,0.03,0.03); colMid = vec3(0.10,0.09,0.08); colLight = vec3(0.20,0.18,0.16); }
    else if (hue < 0.56) { colDark = vec3(0.14,0.10,0.06); colMid = vec3(0.30,0.22,0.14); colLight = vec3(0.50,0.38,0.26); }
    else if (hue < 0.63) { colDark = vec3(0.18,0.17,0.16); colMid = vec3(0.34,0.32,0.30); colLight = vec3(0.52,0.50,0.47); }
    else if (hue < 0.70) { colDark = vec3(0.06,0.08,0.10); colMid = vec3(0.16,0.19,0.24); colLight = vec3(0.30,0.34,0.40); }
    else if (hue < 0.77) { colDark = vec3(0.18,0.08,0.06); colMid = vec3(0.34,0.18,0.12); colLight = vec3(0.52,0.30,0.22); }
    else if (hue < 0.84) { colDark = vec3(0.14,0.14,0.12); colMid = vec3(0.30,0.28,0.26); colLight = vec3(0.48,0.46,0.42); }
    else if (hue < 0.92) { colDark = vec3(0.05,0.05,0.06); colMid = vec3(0.14,0.14,0.16); colLight = vec3(0.26,0.26,0.30); }
    else { colDark = vec3(0.12,0.10,0.09); colMid = vec3(0.26,0.22,0.20); colLight = vec3(0.44,0.38,0.34); }
}

// ============================================================
// WoodOtavio noiseGenWood (full 128-wave)
// ============================================================
float noiseGenWood(vec3 p) {
    float wave0 = 0.0, wave1 = 0.0;
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

vec3 otavioBoardWoodTexture(vec2 worldXY, float elevation, float seed) {
    float seedAngle = seed * 2.718;
    float sa = sin(seedAngle), ca = cos(seedAngle);
    vec2 rotated = vec2(worldXY.x * ca - worldXY.y * sa,
                        worldXY.x * sa + worldXY.y * ca);
    vec2 offset = vec2(wc_hash21(vec2(seed * 3.17, 1.0)) * 20.0 - 10.0,
                       wc_hash21(vec2(1.0, seed * 4.31)) * 20.0 - 10.0);
    vec3 pos = vec3((rotated + offset) * 0.5, elevation * 2.0 + seed * 7.0);
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

// GenisSole approximation (as_type bitcast not available in GLSL ES)
float gsHash21_approx(vec2 p) { return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453); }
float gsHash11_approx(float p) { return fract(sin(p * 127.1) * 43758.5453); }
vec2 gsHash22_approx(vec2 p) {
    return vec2(fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453) * 2.0 - 1.0,
                fract(sin(dot(p, vec2(269.5, 183.3))) * 43758.5453) * 2.0 - 1.0);
}

float gsVnoise2(vec2 p) {
    vec2 i = floor(p); vec2 f = fract(p);
    float a = gsHash21_approx(i);
    float b = gsHash21_approx(i + vec2(1.0, 0.0));
    float c = gsHash21_approx(i + vec2(0.0, 1.0));
    float d = gsHash21_approx(i + vec2(1.0, 1.0));
    vec2 u = f * f * (3.0 - 2.0 * f);
    return a + u.x * (b - a) + u.y * (c - a) + u.x * u.y * (d - c - b + a);
}
float gsVnoise1(float p) {
    float i = floor(p); float f = fract(p);
    float a = gsHash11_approx(i); float b = gsHash11_approx(i + 1.0);
    float u = f * f * (3.0 - 2.0 * f);
    return mix(a, b, u);
}
float gsFbm2(vec2 p) { return gsVnoise2(p) * 0.5 + gsVnoise2(p * 2.0) * 0.25 + gsVnoise2(p * 4.0) * 0.125; }
float gsFbm1(float p) { return gsVnoise1(p) * 0.5 + gsVnoise1(p * 2.0) * 0.25 + gsVnoise1(p * 4.0) * 0.125; }
float gsGnoise2(vec2 p) {
    vec2 i = floor(p); vec2 f = fract(p);
    float a = dot(gsHash22_approx(i), f);
    float b = dot(gsHash22_approx(i + vec2(1.0, 0.0)), f - vec2(1.0, 0.0));
    float c = dot(gsHash22_approx(i + vec2(0.0, 1.0)), f - vec2(0.0, 1.0));
    float d = dot(gsHash22_approx(i + vec2(1.0)), f - vec2(1.0));
    vec2 q = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    return a + q.x * (b - a) + q.y * (c - a) + q.x * q.y * (d - c - b + a);
}

vec3 gsWood3D(vec3 p) {
    float sn = sin(0.25); float cn = cos(0.25);
    vec3 axis = normalize(vec3(1.0, 1.0, 1.0));
    p += 2.0 * cross(axis * sn, cross(axis * sn, p) + cn * p);
    vec2 U = 40.0 * p.xz + 15.0;
    float w = length(U);
    w += gsFbm1(w * 0.5);
    w *= log(max(0.001, w * 0.2)) * 0.1;
    w += gsFbm2(U * 0.05 + vec2(p.y * 0.5 + 500.0));
    float ringFloor = floor(w); float ringFrac = fract(w);
    float rings = ringFrac * smoothstep(0.0, 1.0, 1.0 - ringFrac) * 2.7;
    float grain = 0.5 * (1.0 + gsGnoise2(U * 8.0 + vec2(100.0 + p.y * 5.0))) * 0.5 +
                  gsGnoise2(vec2((ringFrac + ringFloor + 100.0 + gsFbm2(U + vec2(200.0)) * 0.06) * 20.0)) * 0.5;
    float combined = mix(rings, grain, 0.5);
    return clamp((combined * 1.5 + 0.3) * vec3(0.7, 0.4, 0.2), 0.0, 1.0);
}

vec3 gsBoardWoodTexture(vec2 worldXY, float elevation, float seed) {
    float seedAngle = seed * 1.93;
    float sa = sin(seedAngle), ca = cos(seedAngle);
    vec2 rotated = vec2(worldXY.x * ca - worldXY.y * sa, worldXY.x * sa + worldXY.y * ca);
    vec2 offset = vec2(wc_hash21(vec2(seed * 3.17, 1.0)) * 20.0 - 10.0,
                       wc_hash21(vec2(1.0, seed * 4.31)) * 20.0 - 10.0);
    vec3 p = vec3((rotated + offset) * 0.5, elevation * 2.0 + seed * 7.0);
    vec3 raw = gsWood3D(p);
    vec3 colDark, colMid, colLight;
    wc_woodPalette(seed, colDark, colMid, colLight);
    float lum = dot(raw, vec3(0.299, 0.587, 0.114));
    vec3 col = mix(colDark, mix(colMid, colLight, clamp(lum * 2.0 - 1.0, 0.0, 1.0)), clamp(lum * 2.0, 0.0, 1.0));
    return clamp(col, 0.0, 1.0);
}

vec3 boardWoodTexture(vec2 worldXY, float elevation, float seed) {
    float style = fract(seed * 0.5173);
    if (style < 0.5) return otavioBoardWoodTexture(worldXY, elevation, seed);
    else return gsBoardWoodTexture(worldXY, elevation, seed);
}

// ============================================================
// Shadow (PCSS)
// ============================================================
const vec2 poissonDisk[32] = vec2[32](
    vec2(-0.613392, 0.617481), vec2( 0.170019,-0.040254),
    vec2(-0.299417, 0.791925), vec2( 0.645680, 0.493210),
    vec2(-0.651784, 0.717887), vec2( 0.421003, 0.027070),
    vec2(-0.817194,-0.271096), vec2(-0.705374,-0.668203),
    vec2( 0.977050,-0.108615), vec2( 0.063326, 0.142369),
    vec2( 0.203528, 0.214331), vec2(-0.667531, 0.326090),
    vec2(-0.098422,-0.295755), vec2(-0.885922, 0.215369),
    vec2( 0.566637, 0.605213), vec2( 0.039766,-0.396100),
    vec2( 0.308439,-0.723416), vec2(-0.345912,-0.938257),
    vec2( 0.854412, 0.263352), vec2(-0.367833, 0.440661),
    vec2( 0.234208, 0.887153), vec2(-0.951050,-0.240556),
    vec2( 0.587940,-0.598885), vec2(-0.102601, 0.515472),
    vec2( 0.798181,-0.179661), vec2(-0.435220,-0.589435),
    vec2( 0.142256,-0.897236), vec2( 0.468750, 0.750000),
    vec2(-0.750000, 0.468750), vec2( 0.750000,-0.468750),
    vec2(-0.468750,-0.750000), vec2( 0.250000, 0.866025)
);

float pcssFilter(vec2 uv, float depthRef, float filterRadius) {
    float sum = 0.0;
    for (int i = 0; i < 32; i++) sum += texture(uShadowMap, vec3(uv + poissonDisk[i] * filterRadius, depthRef));
    return sum / 32.0;
}
float findBlocker(vec2 uv, float depthRef, float searchRadius) {
    float blockerSum = 0.0, blockerCount = 0.0;
    for (int i = 0; i < 24; i++) {
        float cmp = texture(uShadowMap, vec3(uv + poissonDisk[i] * searchRadius, depthRef));
        if (cmp < 0.5) { blockerSum += depthRef - 0.005; blockerCount += 1.0; }
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
    if (shadowType < 0.5) return texture(uShadowMap, vec3(suv, depthRef));
    if (shadowType < 1.5) return smoothstep(0.0, 1.0, pcssFilter(suv, depthRef, invSize.x * 4.0));
    float lightSize = max(0.001, uLightingParams.z);
    float bsr = lightSize * (depthRef - 0.01) / depthRef * 0.65;
    bsr = clamp(bsr, invSize.x * 1.5, invSize.x * 10.0);
    float abd = findBlocker(suv, depthRef, bsr);
    if (abd < 0.0) return 1.0;
    float pr = max(0.0001, lightSize * (depthRef - abd) / abd);
    float fr = clamp(pr * 1.4, invSize.x * 3.0, invSize.x * 20.0);
    return pow(smoothstep(0.0, 1.0, pcssFilter(suv, depthRef, fr)), 1.2);
}

// ============================================================
// Main
// ============================================================
void main() {
    vec3 n = normalize(vNormal);
    bool isTopFace = n.z > 0.5;

    vec4 clipPos = uViewProj * vec4(vWorldPos, 1.0);
    // Metal NDC Z [0,1] → OpenGL NDC Z [-1,1]. gl_FragDepth needs [0,1].
    float outDepth = clipPos.z / clipPos.w * 0.5 + 0.5;

    if (isTopFace) {
        float sqHoles = uTableParams2.w;
        for (int i = 0; i < uHoleCount; i++) {
            float holeElev = uHoles[i].z;
            if (abs(holeElev - vElevation) > 0.05) continue;
            vec2 holeCenter = uHoles[i].xy;
            float holeR = uHoles[i].w;
            vec2 dd = abs(vWorldXY - holeCenter);
            float dist = sqHoles > 0.5 ? max(dd.x, dd.y) : length(vWorldXY - holeCenter);
            if (dist < holeR) {
                gl_FragDepth = 1.0;
                fragColor = vec4(0.0);
                return;
            }
        }
    }

    vec3 l = normalize(uLightDirIntensity.xyz);
    float lightI = uLightDirIntensity.w;

    float levelSeed = uTimeDrag.z;
    vec3 baseColor;
    // uWoodBoundsMax.w > 0.5 means a baked 3D wood volume texture is available
    // (Metal checks boardWoodVolumeTex.get_width() > 1)
    if (uWoodBoundsMax.w > 0.5) {
        vec3 woodUVW = (vWorldPos - uWoodBoundsMin.xyz) / (uWoodBoundsMax.xyz - uWoodBoundsMin.xyz);
        baseColor = texture(uBoardWoodVolumeTex, woodUVW).rgb;
    } else {
        baseColor = boardWoodTexture(vWorldXY, vWorldPos.z, levelSeed);
    }

    float ndl = clamp(dot(n, l), 0.0, 1.0);
    float ambient = uLightingParams.x;
    float shadow = shadowVisibility(vWorldPos, n);
    vec3 lit = baseColor * (ambient + ndl * lightI * shadow);

    vec3 v = normalize(uCameraPos.xyz - vWorldPos);
    vec3 h = normalize(l + v);
    float spec = pow(clamp(dot(n, h), 0.0, 1.0), 60.0) * 0.2 * shadow;
    lit += spec;

    float edgeDarken = 1.0;
    if (!isTopFace) {
        edgeDarken = 0.65 + 0.35 * clamp(vWorldPos.z / max(vElevation, 0.01), 0.0, 1.0);
    }
    lit *= edgeDarken;

    fragColor = vec4(lit, 1.0);
    gl_FragDepth = outDepth;
}
