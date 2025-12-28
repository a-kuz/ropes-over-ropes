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
};

static float shadowVisibility(float3 worldPos, float3 worldN, constant FrameUniforms& frame, depth2d<float> shadowMap);

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
    float2 p = uv * 2.0 - 1.0;
    float v = smoothstep(1.35, 0.0, dot(p, p));
    float3 base = mix(float3(0.92, 0.93, 0.96), float3(0.86, 0.88, 0.94), uv.y);
    float3 c = base * (0.86 + 0.14 * v);
    c += frame.ambientColor.xyz * 0.12;

    float halfW = frame.orthoHalfSize_shadowBias.x;
    float halfH = frame.orthoHalfSize_shadowBias.y;
    float2 worldXY = float2((uv.x * 2.0 - 1.0) * halfW, (uv.y * 2.0 - 1.0) * halfH);
    float3 worldPos = float3(worldXY.x, worldXY.y, 0.0);
    float3 worldN = float3(0.0, 0.0, 1.0);

    float shadow = 1.0;
    if (shadowMap.get_width() > 0) {
        shadow = shadowVisibility(worldPos, worldN, frame, shadowMap);
    }
    c *= mix(0.62, 1.0, shadow);
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
    lit += frame.ambientColor.xyz * col * 0.35;

    float shadow = 1.0;
    if (shadowMap.get_width() > 0) {
        shadow = shadowVisibility(in.worldPos, n, frame, shadowMap);
    }
    lit *= mix(0.55, 1.0, shadow);
    return float4(lit, 1.0);
}

struct RopeIn {
    float3 position [[attribute(0)]];
    float3 normal [[attribute(1)]];
    float3 color [[attribute(2)]];
    float2 uv [[attribute(3)]];
};

struct RopeOut {
    float4 position [[position]];
    float3 normal;
    float3 color;
    float3 worldPos;
};

vertex RopeOut ropeVertex(RopeIn in [[stage_in]],
                          constant FrameUniforms& frame [[buffer(1)]]) {
    RopeOut o;
    o.worldPos = in.position;
    o.position = frame.viewProj * float4(in.position, 1.0);
    o.normal = in.normal;
    o.color = in.color;
    return o;
}

static float3 rubberShading(float3 baseColor, float3 n, float3 l, float3 v) {
    float ndl = saturate(dot(n, l));
    float3 h = normalize(l + v);
    float ndh = saturate(dot(n, h));

    float rough = 0.72;
    float specPow = mix(16.0, 2.0, rough);
    float spec = pow(ndh, specPow) * 0.22;
    float wrap = saturate((ndl + 0.35) / 1.35);

    float3 diff = baseColor * (0.35 + 0.65 * wrap);
    float3 sp = float3(spec);
    float fres = pow(1.0 - saturate(dot(n, v)), 5.0);
    return diff + sp * (0.35 + 0.65 * fres);
}

fragment float4 ropeFragment(RopeOut in [[stage_in]],
                             constant FrameUniforms& frame [[buffer(1)]],
                             depth2d<float> shadowMap [[texture(2)]]) {
    float3 l = normalize(frame.lightDir_intensity.xyz);
    float3 v = normalize(frame.cameraPos.xyz - in.worldPos);
    float3 n = normalize(in.normal);

    float3 c = rubberShading(in.color, n, l, v);
    c += frame.ambientColor.xyz * in.color * 0.55;

    float h = saturate(in.worldPos.z / 0.35);
    c += float3(0.08, 0.10, 0.16) * h * 0.6;

    float shadow = 1.0;
    if (shadowMap.get_width() > 0) {
        shadow = shadowVisibility(in.worldPos, n, frame, shadowMap);
    }
    c *= mix(0.55, 1.0, shadow);
    return float4(c, 1.0);
}

struct ShadowOut {
    float4 position [[position]];
};

vertex ShadowOut ropeShadowVertex(RopeIn in [[stage_in]],
                                  constant FrameUniforms& frame [[buffer(1)]]) {
    ShadowOut o;
    o.position = frame.lightViewProj * float4(in.position, 1.0);
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

static float shadowVisibility(float3 worldPos, float3 worldN, constant FrameUniforms& frame, depth2d<float> shadowMap) {
    float4 lp = frame.lightViewProj * float4(worldPos, 1.0);
    float3 ndc = lp.xyz / max(1e-6, lp.w);
    float2 uv = float2(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) return 1.0;

    float biasBase = frame.orthoHalfSize_shadowBias.z;
    float ndl = saturate(dot(normalize(worldN), normalize(frame.lightDir_intensity.xyz)));
    float bias = biasBase + (1.0 - ndl) * biasBase * 2.2;

    float2 invSize = frame.shadowInvSize_unused.xy;
    constexpr sampler shadowSampler(coord::normalized, address::clamp_to_edge, filter::linear, compare_func::less_equal);

    float depthRef = ndc.z - bias;
    float sum = 0.0;
    float2 offsets[9] = {
        float2(-1,-1), float2(0,-1), float2(1,-1),
        float2(-1, 0), float2(0, 0), float2(1, 0),
        float2(-1, 1), float2(0, 1), float2(1, 1)
    };
    for (int i = 0; i < 9; i++) {
        float2 suv = uv + offsets[i] * invSize;
        sum += shadowMap.sample_compare(shadowSampler, suv, depthRef);
    }
    return sum / 9.0;
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
    c += b * 1.05;
    float3 mapped = c / (c + float3(1.0));
    mapped = pow(mapped, float3(1.0 / 2.2));
    return float4(mapped, 1.0);
}

