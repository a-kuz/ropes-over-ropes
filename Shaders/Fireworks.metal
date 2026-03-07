#include <metal_stdlib>
using namespace metal;

// MARK: - Vertex (shared)

struct FWVertexOut {
    float4 position [[position]];
};

vertex FWVertexOut fireworks_vertex(uint vid [[vertex_id]]) {
    const float2 pos[] = { float2(-1,-1), float2(3,-1), float2(-1,3) };
    FWVertexOut o;
    o.position = float4(pos[vid], 0, 1);
    return o;
}

// MARK: - Uniforms (shared)

struct FWUniforms {
    float2 resolution;
    float  time;
};

// ============================================================
// MARK: - SHADER 1  (night sky + water reflection)
// Original: https://www.shadertoy.com/view/...
// ============================================================

static float2 hash21(float p) {
    float3 p3 = fract(float3(p) * float3(.1031f, .1030f, .0973f));
    p3 += dot(p3, p3.yzx + 33.33f);
    return fract((p3.xx + p3.yz) * p3.zy);
}

static float3 hash31_s1(float p) {
    float3 p2 = fract(p * float3(5.3983f, 5.4427f, 6.9371f));
    p2 += dot(p2.zxy, p2.xyz + float3(21.5351f, 14.3137f, 15.3219f));
    return fract(float3(p2.x*p2.y*95.4337f, p2.y*p2.z*97.597f, p2.z*p2.x*93.8365f));
}

static float2 fw_dir(float id) {
    float2 h = hash21(id);
    h.y *= 2.0f * M_PI_F;
    return h.x * float2(cos(h.y), sin(h.y));
}

#define PARTICLES_MIN 20.0f
#define PARTICLES_MAX 200.0f
#define FW_EXT        0.25f
#define FW_DURATION   2.2f
#define NUM_ROCKETS   3.0f

static float bang_s1(float2 uv, float t, float id) {
    if (t <= 0.0f) return 0.04f / dot(uv, uv);
    float s          = (sqrt(t) + t * exp2(-t / 0.125f) * 0.8f) * 10.0f;
    float brightness = sqrt(1.0f - t) * 0.015f * (step(0.0001f, t) * 0.9f + 0.1f);
    float blinkI     = exp2(-t / 0.125f);
    float PARTICLES  = PARTICLES_MIN + (PARTICLES_MAX - PARTICLES_MIN) * fract(cos(id) * 45241.45f);
    float o = 0.0f;
    for (int ii = 0; ii < int(PARTICLES); ii++) {
        float i  = float(ii);
        float2 d = fw_dir(i + 0.012f * id);
        float2 p = d * s;
        float2 h = hash21(5.33345f * i + 0.015f * id);
        float blink = mix(
            cos((t + h.x) * 10.0f * (2.0f + h.y) + h.x * h.y * 10.0f) * 0.3f + 0.7f,
            1.0f, blinkI);
        float2 diff = uv - p;
        o += blink * brightness / dot(diff, diff);
    }
    return o;
}

static float firework_s1(float2 uv, float t, float id) {
    if (id < 1.0f) return 0.0f;
    float2 h      = hash21(id * 5.645f) * 2.0f - 1.0f;
    float2 offset = float2(h.x * 0.1f, 0.0f);
    h.y = h.y * 0.95f; h.y *= abs(h.y);
    float2 di    = float2(h.y, sqrt(1.0f - h.y * h.y));
    float thrust = sqrt(min(t, FW_EXT) / FW_EXT) * 25.0f;
    float2 p     = offset + FW_DURATION * (di * thrust + float2(0.0f, -9.81f) * t) * t;
    return sqrt(1.0f - t) * bang_s1(uv - p, max(0.0f, (t - FW_EXT) / (1.0f - FW_EXT)), id);
}

fragment float4 fireworks_fragment(FWVertexOut in [[stage_in]],
                                   constant FWUniforms& u [[buffer(0)]])
{
    float2 fragCoord = float2(in.position.x, u.resolution.y - in.position.y);
    float2 uv = (2.0f * fragCoord - u.resolution * float2(1.0f, 0.0f)) / u.resolution.y;
    float3 col = float3(0.0f);
    float time = 0.75f * u.time;
    float t    = time / FW_DURATION;
    uv.y -= 0.65f; uv *= 35.0f;
    float m = 1.0f;
    if (uv.y < 0.0f) {
        const float h0 = 5.0f, dcam = 1000.5f;
        float y = uv.y - h0, z = dcam * h0 / y, x = uv.x * z / dcam;
        uv += float2(sin((x*1.5f+z*.75f)*.0005f-t*1.5f), cos((z*2.0f-x*.5f)*.0005f-t*2.69f))
            * (sin(x*.07f+z*.09f+sin(x*.2f-t)-t*15.0f)
             + cos(z*.1f-x*(.08f+.001f*sin(x*.01f-t))-t*16.0f)*.7f
             + cos(z*.01f+x*.004f-t*10.0f)*1.7f) * .15f * dcam / z;
        float ndv = -uv.y / sqrt(dcam*dcam + uv.y*uv.y);
        m = mix(1.0f, 0.98f, pow(1.0f - ndv, 5.0f));
        uv.y = -uv.y;
    }
    col += (exp2(-abs(uv.y)*float3(1.f,2.f,3.f)-.5f) + exp2(-abs(uv.y)*float3(1.f,.2f,.1f)-4.0f))*.5f;
    if (uv.y*1.5f < (uv.x-20.f)*.01f*(-uv.x+90.f)+sin(uv.x)*cos(uv.y*1.1f)*.75f) col *= 0.0f;
    for (int i = 0; i < int(ceil(NUM_ROCKETS)); i++) {
        float fi = float(i);
        float T  = 1.0f + t + fi / NUM_ROCKETS;
        float id = floor(T) - fi / NUM_ROCKETS;
        float3 color = hash31_s1(id * 0.75645f);
        color /= max(color.r, max(color.g, color.b));
        col += firework_s1(uv, fract(T), id) * color;
    }
    return float4(m * col, 1.0f);
}

// ============================================================
// MARK: - SHADER 2  (city skyline fireworks)
// Original: https://www.shadertoy.com/view/WtjyzR
// ============================================================

static float hash11_s2(float p) {
    uint2 n = uint(int(p)) * uint2(1597334673u, 3812015801u);
    uint q = (n.x ^ n.y) * 1597334673u;
    return float(q) * (1.0f / float(0xffffffffu));
}

static float3 hash31_s2(float p) {
    uint3 n = uint(int(p)) * uint3(1597334673u, 3812015801u, 2798796415u);
    n = (n.x ^ n.y ^ n.z) * uint3(1597334673u, 3812015801u, 2798796415u);
    return float3(n) * (1.0f / float(0xffffffffu));
}

static float remap_s2(float x, float a, float b, float c, float d) {
    return ((x - a) / (b - a)) * (d - c) + c;
}

// Value noise replacing iChannel0 texture lookup
static float vnoise3_s2(float3 p) {
    float3 i = floor(p), f = fract(p);
    f = f * f * (3.0f - 2.0f * f);
    float n000 = hash21(i.x + i.y * 157.0f + i.z * 113.0f).x;
    float n100 = hash21((i.x+1)+i.y*157.0f+i.z*113.0f).x;
    float n010 = hash21(i.x+(i.y+1)*157.0f+i.z*113.0f).x;
    float n110 = hash21((i.x+1)+(i.y+1)*157.0f+i.z*113.0f).x;
    float n001 = hash21(i.x+i.y*157.0f+(i.z+1)*113.0f).x;
    float n101 = hash21((i.x+1)+i.y*157.0f+(i.z+1)*113.0f).x;
    float n011 = hash21(i.x+(i.y+1)*157.0f+(i.z+1)*113.0f).x;
    float n111 = hash21((i.x+1)+(i.y+1)*157.0f+(i.z+1)*113.0f).x;
    return mix(mix(mix(n000,n100,f.x), mix(n010,n110,f.x), f.y),
               mix(mix(n001,n101,f.x), mix(n011,n111,f.x), f.y), f.z);
}

static float fbm3_s2(float3 p) {
    return vnoise3_s2(p) + vnoise3_s2(p*2.0f)/2.0f + vnoise3_s2(p*4.0f)/4.0f;
}

static float windows_s2(float2 uv, float offset, float iTime) {
    float2 grid = float2(20.0f, 1.0f);
    uv.x += offset;
    float2 tmp1 = float2(int2(floor(uv * grid))) + 0.5f;
    float n1 = fbm3_s2(tmp1.xxx);
    uv.x *= n1 * 6.0f;
    float2 id = float2(int2(floor(uv * grid))) + 0.5f;
    float n = fbm3_s2(id.xxx);
    float2 lightGrid = float2(79.0f*(n+0.5f), 250.0f*n);
    float2 tmp2 = float2(int2(floor(uv * lightGrid + floor(iTime*0.4f)*0.2f))) + 0.5f;
    float n2 = fbm3_s2(tmp2.xyx);
    float2 lPos = fract(uv * lightGrid);
    n2 = (lPos.y < 0.2f || lPos.y > 0.7f) ? 0.0f : n2;
    n2 = (lPos.x < 0.5f || lPos.y > 0.7f) ? 0.0f : n2;
    n2 = smoothstep(0.225f, 0.5f, n2);
    return (uv.y < n - 0.01f) ? n2 : 0.0f;
}

static float Ff(float x, float f) { return floor(x * f) / f; }

static float buildings_s2(float2 st) {
    float b = 0.1f * Ff(cos(st.x*4.0f+1.7f), 1.0f);
    b += (b+0.3f)*0.3f * Ff(cos(st.x*4.0f-0.1f), 2.0f);
    b += (b-0.01f)*0.1f * Ff(cos(st.x*12.0f), 4.0f);
    b += (b-0.05f)*0.3f * Ff(cos(st.x*24.0f), 1.0f);
    return clamp((st.y + b - 0.1f) * 100.0f, 0.0f, 1.0f);
}

static float stars_s2(float2 st, float2 fragCoord, float2 resolution, float iTime) {
    float2 uv = (2.0f * fragCoord - resolution) / resolution.y;
    uv.y += 0.3f; uv.y = abs(uv.y);
    float t = iTime * 0.1f;
    float2 h = pow(hash21(uv.x * resolution.y + uv.y), float2(50.0f));
    float twinkle = sin((st.x + t + cos(st.y*50.0f+t)) * 25.0f);
    twinkle *= cos((st.y*.187f - t*4.16f + sin(st.x*11.8f+t*.347f)) * 6.57f);
    return h.x * h.y * (twinkle * 0.5f + 0.5f) * 1.5f;
}

#define FW2_COUNT      6
#define FW2_DURATION   8.5f
#define FW2_LOW        0.75f
#define FW2_HIGH       1.05f
#define FW2_ROCKET_N   16
#define FW2_ROCKET_DUR 1.5f
#define FW2_FLASH_DUR  1.7f    // ROCKET_DURATION + 0.2
#define FW2_THRUST_SPD 0.25f
#define FW2_EXP_STR    0.025f
#define FW2_EXP_N      128

static float3 fireworks_s2(float2 st, float iTime) {
    float3 finalCol = float3(0.0f);
    for (int j = 0; j < FW2_COUNT; ++j) {
        float fj = float(j);
        float timeHash   = hash11_s2((fj+1.0f)*9.6144f + 78.6118f);
        float timeOffset = (fj+1.0f) + (fj+1.0f)*timeHash;
        float3 fwHash = hash31_s2(471.5277f*fj + 1226.9146f
            + float(int((iTime+timeOffset) / FW2_DURATION))) * 2.0f - 1.0f;
        float3 fwCol  = fwHash * 0.5f + 0.5f;
        fwHash.y = remap_s2(fwHash.y, -1.0f, 1.0f, FW2_LOW, FW2_HIGH);
        fwHash.x = ((fj + 0.5f + fwHash.x*0.25f) / float(FW2_COUNT)) * 2.0f - 1.0f;
        float time = fmod(iTime + timeOffset, FW2_DURATION);
        float2 fwPos, partPos;
        if (time > FW2_ROCKET_DUR) {
            fwPos = float2(fwHash.x, fwHash.y);
            for (int i = 0; i < FW2_EXP_N; ++i) {
                float fi = float(i);
                float3 ph = hash31_s2(fj*1291.1978f + fi*1619.8196f + 469.7119f);
                float theta       = remap_s2(ph.x, 0.0f, 1.0f, 0.0f, 6.283185f);
                float radiusScale = ph.y * FW2_EXP_STR;
                float radius      = radiusScale * time * time;
                partPos = float2(radius*cos(theta), radius*sin(theta));
                partPos.y -= max(1e-4f, pow(length(partPos)-0.05f, 2.0f)*1.25f);
                partPos.y -= pow(radius/radiusScale, 3.0f) * 4e-5f;
                float spark = 3e-4f / pow(length(st - partPos - fwPos), 1.7f);
                float sdist = 2.0f * length(fwPos - partPos);
                float shimmer = max(0.0f, sqrt(sdist)
                    * sin((iTime*max(1.3f,fwHash.z*2.0f) + ph.y*6.283185f) * 18.0f));
                float shimThr = FW2_DURATION * 0.9f;
                float fade = clamp(FW2_DURATION*2.0f * radiusScale - radius, 0.0f, 1.0f);
                finalCol += mix(spark, spark*shimmer,
                    smoothstep(shimThr*radiusScale, (shimThr+1.0f)*radiusScale, radius))
                    * fade * fwCol;
            }
            if (time < FW2_FLASH_DUR) {
                float flashSpark = 3e-4f / pow(length(st - fwPos), 1.7f);
                finalCol += flashSpark / (0.01f + fmod(time, FW2_ROCKET_DUR));
            }
        } else {
            float rp = fmod(time, FW2_ROCKET_DUR) / FW2_ROCKET_DUR;
            rp = sin(rp / (FW2_ROCKET_DUR * 0.75f) * M_PI_F * 0.5f);
            fwPos = float2(fwHash.x, rp * fwHash.y);
            fwPos.x += sin(st.y*50.0f + time) * fwCol.z * 0.0035f;
            for (int i = 0; i < FW2_ROCKET_N; ++i) {
                float3 ph = hash31_s2(float(i)*603.6837f + 1472.3486f);
                float radius = fmod(time+ph.y, FW2_THRUST_SPD)/FW2_THRUST_SPD * ph.z * 0.1f;
                float theta  = remap_s2(ph.x, 0.0f, 1.0f, 0.0f, M_PI_F*0.1f) + M_PI_F*1.45f;
                partPos = float2(radius*cos(theta), radius*sin(theta));
                finalCol += 8e-5f / pow(length(st-partPos-fwPos), 1.1f)
                    * mix(float3(1.4f,0.7f,0.2f), float3(1.4f), radius*16.0f);
            }
        }
    }
    return finalCol;
}

fragment float4 fireworks2_fragment(FWVertexOut in [[stage_in]],
                                    constant FWUniforms& u [[buffer(0)]])
{
    float2 fragCoord = float2(in.position.x, u.resolution.y - in.position.y);
    float2 uv = (2.0f * fragCoord - u.resolution) / min(u.resolution.y, u.resolution.x);
    uv.y += 0.3f;
    float reflection = 0.0f;
    if (uv.y < 0.0f) {
        reflection = 1.0f;
        uv.x += cos(uv.y*192.0f - u.time*0.6f) * sin(uv.y*96.0f + u.time*0.75f) * 0.042f;
    }
    float2 st = float2(uv.x, abs(uv.y));
    float3 col = float3(0.0f);
    float mountain = sin(1.69f*st.x*1.38f*cos(2.74f*st.x) + 4.87f*sin(1.17f*st.x))*0.1f - 0.18f + st.y;
    mountain = clamp(smoothstep(-0.005f, 0.005f, mountain), 0.0f, 1.0f);
    float building = buildings_s2(st);
    col += float3(0.18f-st.y*0.1f, 0.18f-st.y*0.1f, 0.1f+st.y*0.03f);
    col  = col*mountain + float3(0.1f-st.y*0.1f, 0.1f-st.y*0.1f, 0.08f)*(1.0f-mountain);
    col *= building;
    col += windows_s2(st*0.8f, 2.0f, u.time) * (1.0f-building) * float3(1.2f,1.0f,0.8f);
    float moon = smoothstep(0.3f, 0.29f, length(st - float2(1.0f, 0.8f)));
    col += stars_s2(st, fragCoord, u.resolution, u.time) * mountain * building * (1.0f-moon);
    moon *= smoothstep(0.32f, 0.48f, length(st - float2(0.92f, 0.88f))) * 1.25f;
    col += moon * float3(1.2f, 1.18f, 1.0f);
    col += clamp(fireworks_s2(st, u.time), 0.0f, 1.0f) * (building + moon);
    col.r -= reflection * 0.05f;
    col.gb += reflection * 0.01f;
    col -= hash11_s2(fragCoord.x * fragCoord.y * 0.2f * (u.time + 50.0f)) * 0.008f;
    return float4(col, 1.0f);
}

// ============================================================
// MARK: - SHADER 3  (dr2 "Fireworks 3d")
// Original: https://www.shadertoy.com/view/lscGRl
// ============================================================

static float hashff_d(float p) {
    return fract(sin(p) * 43758.54f);
}

static float hashfv3_d(float3 p) {
    return fract(sin(dot(p, float3(37.0f, 39.0f, 41.0f))) * 43758.54f);
}

static float4 hashv4v3_d(float3 p) {
    float3 va = float3(37.0f, 39.0f, 41.0f);
    float3 e1 = float3(1.0f, 0.0f, 0.0f);
    float3 e2 = float3(0.0f, 1.0f, 0.0f);
    float3 e3 = float3(0.0f, 0.0f, 1.0f);
    return fract(sin(float4(
        dot(p,          va),
        dot(p + e1,     va),
        dot(p + e2,     va),
        dot(p + e1+e2,  va))) * 43758.54f);
}

static float noisefv3_d(float3 p) {
    float3 ip = floor(p), fp = fract(p);
    fp = fp * fp * (3.0f - 2.0f * fp);
    float4 t = mix(hashv4v3_d(ip), hashv4v3_d(ip + float3(0.0f,0.0f,1.0f)), fp.z);
    return mix(mix(t.x, t.y, fp.x), mix(t.z, t.w, fp.x), fp.y);
}

static float2 rot2d_d(float2 q, float a) {
    float2 cs = sin(a + float2(0.5f * M_PI_F, 0.0f));
    return float2(dot(q, float2(cs.x, -cs.y)), dot(q.yx, cs));
}

static float3 hsvtorgb_d(float3 c) {
    return c.z * mix(float3(1.0f),
        clamp(abs(fract(c.xxx + float3(1.0f, 2.0f/3.0f, 1.0f/3.0f)) * 6.0f - 3.0f) - 1.0f, 0.0f, 1.0f),
        c.y);
}

static float2 ballHit_d(float3 ro, float3 rd, float3 p, float s, float dstFar) {
    float3 v = ro - p;
    float b = dot(rd, v);
    float d = b*b + s*s - dot(v,v);
    if (d > 0.0f) {
        d = sqrt(d);
        return float2(-b-d, -b+d);
    }
    return float2(dstFar, dstFar);
}

static float4 sphFib_d(float3 v, float n) {
    const float phi = 1.618034f;
    float ni = 1.0f / n;
    float fk = pow(phi, max(2.0f, floor(log(n * M_PI_F * sqrt(5.0f) * dot(v.xy, v.xy)) /
        log(phi + 1.0f)))) / sqrt(5.0f);
    float2 ff = float2(floor(fk + 0.5f), floor(fk * phi + 0.5f));
    float4 b  = float4(ff * ni, M_PI_F * (fract((ff + 1.0f) * phi) - (phi - 1.0f)));
    float denom = b.y * b.z - b.x * b.w;
    float2x2 m  = float2x2(float2(b.y, -b.x), float2(b.w, -b.z));
    float2 c = floor((0.5f / denom) * (m * float2(atan2(v.y, v.x), v.z - (1.0f - ni))));
    float ddMin = 4.1f;
    float3 vfMin = float3(0.0f);
    for (int j = 0; j < 4; j++) {
        float a = dot(ff, float2(float(j - 2*(j/2)), float(j/2)) + c);
        float z = 1.0f - (2.0f * a + 1.0f) * ni;
        float3 vf = float3(sin(2.0f*M_PI_F*fract(phi*a) + float2(0.5f*M_PI_F, 0.0f)) * sqrt(1.0f-z*z), z);
        float dd = dot(vf-v, vf-v);
        if (dd < ddMin) { ddMin = dd; vfMin = vf; }
    }
    return float4(sqrt(ddMin), vfMin);
}

static float3 skyCol_d(float3 ro, float3 rd, float tCur) {
    float3 rdRot = rd;
    rdRot.xz = rot2d_d(rdRot.xz, 0.001f * tCur);
    float3 mDir = normalize(float3(0.0f, 1.0f, 1.0f));
    float mRad  = 0.02f;
    float3 col  = float3(0.02f, 0.02f, 0.04f)
        + float3(0.06f, 0.04f, 0.02f) * pow(clamp(dot(rdRot, mDir), 0.0f, 1.0f), 16.0f);
    float bs = dot(rdRot, mDir);
    float ts = bs*bs - dot(mDir, mDir) + mRad*mRad;
    if (ts > 0.0f) {
        ts = bs - sqrt(ts);
        if (ts > 0.0f) {
            float3 vn = normalize((ts * rdRot - mDir) / mRad);
            col += 0.8f * float3(1.0f, 0.9f, 0.5f)
                * clamp(dot(float3(-0.77f,0.4f,0.5f), vn) * (1.0f - 0.3f*noisefv3_d(4.0f*vn)), 0.0f, 1.0f);
        }
        col *= 1.3f;
    } else {
        float3 rds = floor(2000.0f * rdRot);
        rds = 0.00015f * rds + 0.1f * noisefv3_d(0.0005f * rds.yzx);
        for (int j = 0; j < 19; j++) rds = abs(rds)/dot(rds,rds) - 0.9f;
        col += 0.5f * smoothstep(0.01f, 0.04f, rdRot.y) * float3(0.8f, 0.8f, 0.6f)
            * min(1.0f, 0.5e-3f * pow(min(6.0f, length(rds)), 5.0f));
    }
    return col;
}

static float3 showScene_d(float3 ro, float3 rd, float tCur, float dstFar) {
    const float tCyc = 3.0f;
    const float phi  = 1.618034f;
    float nCyc = floor(tCur / tCyc) + 1.0f;
    float hm   = 0.2f * max(hashff_d(17.0f * nCyc) - 0.2f, 0.0f);
    float hr   = 0.8f * min(2.0f * hashff_d(27.0f * nCyc), 1.0f);
    float iFib = 500.0f + floor(6000.0f * hashff_d(37.0f * nCyc));
    float3 col = skyCol_d(ro, rd, tCur);
    bool isBg  = true;
    for (int k = 0; k < 40; k++) {
        float fk  = float(k);
        float phs = fract(tCur / tCyc) - 0.005f * fk;
        float3 bPos = float3(0.0f);
        if (phs > 0.1f) {
            float a = smoothstep(0.1f, 0.15f, phs) - 0.7f * smoothstep(0.3f, 1.0f, phs);
            float h = hm + hr * max(phs - 0.2f, 0.0f);
            float2 dSph = ballHit_d(ro, rd, bPos, 0.5f + 8.5f * sqrt(phs - 0.1f), dstFar);
            if (dSph.x < dstFar) {
                if (k == 0) col = mix(col, float3(1.0f, 1.0f, 0.0f), 0.03f * a);
                float r = 0.015f * (0.5f + 0.5f * a);
                float3 roHit = ro + dSph.x * rd;
                float4 f4 = sphFib_d(normalize(roHit), iFib);
                float s = hashfv3_d(73.0f * f4.yzw + 87.0f * nCyc);
                if (s > 0.5f && f4.x < r * s) {
                    col = mix(col, hsvtorgb_d(float3(h, 1.0f, a)), a);
                    isBg = false;
                } else {
                    roHit = ro + dSph.y * rd;
                    f4 = sphFib_d(normalize(roHit), iFib);
                    s = hashfv3_d(73.0f * f4.yzw + 87.0f * nCyc);
                    if (s > 0.5f && f4.x < r * s) {
                        col = mix(col, hsvtorgb_d(float3(h, 1.0f, 0.7f * a)), a);
                        isBg = false;
                    }
                }
            }
            if (!isBg) break;
        }
    }
    return clamp(col, 0.0f, 1.0f);
}

fragment float4 fireworks3_fragment(FWVertexOut in [[stage_in]],
                                    constant FWUniforms& u [[buffer(0)]])
{
    float2 fragCoord = float2(in.position.x, u.resolution.y - in.position.y);
    float2 uv = 2.0f * fragCoord / u.resolution - 1.0f;
    uv.x *= u.resolution.x / u.resolution.y;
    float tCur  = u.time;
    float dstFar = 100.0f;
    const float phi = 1.618034f;
    // Camera: slow auto-rotation, no mouse
    float az  = -0.1f * M_PI_F + 0.001f * M_PI_F * tCur;
    float el  =  0.2f * M_PI_F;
    float2 ca = cos(float2(el, az));
    float2 sa = sin(float2(el, az));
    float3x3 vuMat = float3x3(
        float3( ca.y, 0.0f, -sa.y),
        float3( 0.0f, 1.0f,  0.0f),
        float3( sa.y, 0.0f,  ca.y)
    ) * float3x3(
        float3(1.0f,  0.0f,  0.0f),
        float3(0.0f,  ca.x, -sa.x),
        float3(0.0f,  sa.x,  ca.x)
    );
    float3 ro = vuMat * float3(0.0f, 0.0f, -40.0f);
    float3 rd = vuMat * normalize(float3(uv, 4.5f));
    float3 col = showScene_d(ro, rd, tCur, dstFar);
    return float4(col, 1.0f);
}
