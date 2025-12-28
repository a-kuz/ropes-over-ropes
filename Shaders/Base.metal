#include <metal_stdlib>
using namespace metal;

struct VSOut {
    float4 position [[position]];
    float2 uv;
};

struct FrameUniforms {
    float4x4 viewProj;
    float4 lightDir_intensity;
    float4 ambientColor;
    float4 cameraPos;
};

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
                              constant FrameUniforms& frame [[buffer(1)]]) {
    float2 uv = in.uv;
    float2 p = uv * 2.0 - 1.0;
    float v = smoothstep(1.35, 0.0, dot(p, p));
    float3 base = mix(float3(0.92, 0.93, 0.96), float3(0.86, 0.88, 0.94), uv.y);
    float3 c = base * (0.86 + 0.14 * v);
    c += frame.ambientColor.xyz * 0.12;
    return float4(c, 1.0);
}

vertex SceneVSOut sceneVertex(uint vid [[vertex_id]],
                              uint iid [[instance_id]],
                              constant FrameUniforms& frame [[buffer(1)]],
                              const device HoleInstance* holes [[buffer(2)]]) {
    float2 quad[6] = { float2(-1,-1), float2(1,-1), float2(1,1), float2(-1,-1), float2(1,1), float2(-1,1) };
    float2 lp = quad[vid];
    HoleInstance h = holes[iid];
    float2 c = h.position_radius.xy;
    float r = h.position_radius.w;
    float2 wp = c + lp * r;

    SceneVSOut o;
    o.position = frame.viewProj * float4(wp.x, wp.y, 0.0, 1.0);
    o.local = lp;
    o.worldXY = wp;
    return o;
}

fragment float4 sceneFragment(SceneVSOut in [[stage_in]]) {
    float d = length(in.local);
    float ring = smoothstep(1.0, 0.92, d) * (1.0 - smoothstep(0.78, 0.70, d));
    float hole = 1.0 - smoothstep(0.78, 0.72, d);
    float rim = smoothstep(0.92, 0.84, d) - smoothstep(0.98, 0.92, d);

    float2 uv = in.worldXY * 0.55 + 0.5;
    float3 base = mix(float3(0.86, 0.88, 0.93), float3(0.82, 0.84, 0.90), uv.y);

    float3 holeCol = float3(0.70, 0.73, 0.80);
    float3 ringCol = float3(0.92, 0.94, 0.98);
    float3 rimCol = float3(0.72, 0.76, 0.86);
    float3 c = base;
    c = mix(c, holeCol, hole);
    c = mix(c, ringCol, ring);
    c = mix(c, rimCol, rim * 0.85);
    return float4(c, 1.0);
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
                             constant FrameUniforms& frame [[buffer(1)]]) {
    float3 l = normalize(frame.lightDir_intensity.xyz);
    float3 v = normalize(frame.cameraPos.xyz - in.worldPos);
    float3 n = normalize(in.normal);

    float3 c = rubberShading(in.color, n, l, v);
    c += frame.ambientColor.xyz * in.color * 0.55;

    float h = saturate(in.worldPos.z / 0.35);
    c += float3(0.08, 0.10, 0.16) * h * 0.6;
    return float4(c, 1.0);
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

