#include <metal_stdlib>
using namespace metal;

struct RopeSimParams {
    float dt;
    float3 gravity;
    float stretchStiffness;
    uint iterations;
    float particleRadius;
    float damping;
};

struct RopeMetaGPU {
    uint offset;
    uint count;
    uint restLengthBits;
    uint pad1;
    float4 pinA;
    float4 pinB;
};

kernel void ropeProjectToTargets(device float4* posBuf [[buffer(0)]],
                                 device float4* prevBuf [[buffer(1)]],
                                 device const float4* targetBuf [[buffer(2)]],
                                 device const RopeMetaGPU* metaBuf [[buffer(3)]],
                                 constant float& alpha [[buffer(4)]],
                                 constant uint& particlesPerRope [[buffer(5)]],
                                 uint pid [[thread_position_in_grid]]) {
    uint ropeIndex = pid / particlesPerRope;
    if (metaBuf[ropeIndex].count == 0) return;

    float3 p = posBuf[pid].xyz;
    float3 t = targetBuf[pid].xyz;
    p = mix(p, t, alpha);
    posBuf[pid] = float4(p, 1.0);
    prevBuf[pid] = float4(p, 1.0);
}

struct GridParams {
    float2 origin;
    float cellSize;
    uint gridWidth;
    uint gridHeight;
    uint particleCount;
    uint particlesPerRope;
};

static inline float asFloat(uint bits) {
    return as_type<float>(bits);
}

kernel void ropeSimStep(device float4* posBuf [[buffer(0)]],
                        device float4* prevBuf [[buffer(1)]],
                        device const RopeMetaGPU* metaBuf [[buffer(2)]],
                        constant RopeSimParams& params [[buffer(3)]],
                        uint tid [[thread_index_in_threadgroup]],
                        uint3 tgp [[threadgroup_position_in_grid]]) {
    uint ropeIndex = tgp.x;
    RopeMetaGPU meta = metaBuf[ropeIndex];
    uint count = meta.count;
    uint off = meta.offset;
    float restLen = asFloat(meta.restLengthBits);

    threadgroup float3 p[128];
    threadgroup float3 prev[128];

    if (tid < count) {
        float4 pv = posBuf[off + tid];
        float4 qv = prevBuf[off + tid];
        p[tid] = pv.xyz;
        prev[tid] = qv.xyz;
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    if (tid < count) {
        float3 x = p[tid];
        float3 xp = prev[tid];
        float3 v = (x - xp) * (1.0 - params.damping);
        prev[tid] = x;
        x += v + params.gravity * (params.dt * params.dt);
        x.z = max(0.0, x.z);
        p[tid] = x;
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    if (tid == 0) {
        p[0] = meta.pinA.xyz;
        prev[0] = meta.pinA.xyz;
    }
    if (tid == count - 1) {
        p[count - 1] = meta.pinB.xyz;
        prev[count - 1] = meta.pinB.xyz;
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    for (uint it = 0; it < params.iterations; it++) {
        for (uint phase = 0; phase < 2; phase++) {
            uint i0 = tid * 2 + phase;
            uint i1 = i0 + 1;
            if (i1 < count) {
                float3 x0 = p[i0];
                float3 x1 = p[i1];
                float3 d = x1 - x0;
                float len = length(d);
                if (len > 1e-6) {
                    float diff = (len - restLen) / len;
                    float3 corr = d * (diff * 0.5 * params.stretchStiffness);
                    bool pin0 = (i0 == 0);
                    bool pin1 = (i1 == count - 1);
                    if (pin0 && !pin1) {
                        x1 -= corr * 2.0;
                    } else if (!pin0 && pin1) {
                        x0 += corr * 2.0;
                    } else if (!pin0 && !pin1) {
                        x0 += corr;
                        x1 -= corr;
                    }
                    x0.z = max(0.0, x0.z);
                    x1.z = max(0.0, x1.z);
                    p[i0] = x0;
                    p[i1] = x1;
                }
            }
            threadgroup_barrier(mem_flags::mem_threadgroup);
        }
    }

    bool bothOnTable = (meta.pinA.z == 0.0 && meta.pinB.z == 0.0);
    if (bothOnTable && tid < count) {
        p[tid].z = 0.0;
        prev[tid].z = 0.0;
    }

    if (tid < count) {
        posBuf[off + tid] = float4(p[tid], 1.0);
        prevBuf[off + tid] = float4(prev[tid], 1.0);
    }
}

kernel void gridClear(device atomic_int* gridHeads [[buffer(0)]],
                      constant GridParams& grid [[buffer(1)]],
                      uint gid [[thread_position_in_grid]]) {
    uint cellCount = grid.gridWidth * grid.gridHeight;
    if (gid >= cellCount) return;
    atomic_store_explicit(&gridHeads[gid], -1, memory_order_relaxed);
}

kernel void gridBuild(device const float4* posBuf [[buffer(0)]],
                      device atomic_int* gridHeads [[buffer(1)]],
                      device int* nextBuf [[buffer(2)]],
                      constant GridParams& grid [[buffer(3)]],
                      device const RopeMetaGPU* metaBuf [[buffer(4)]],
                      uint pid [[thread_position_in_grid]]) {
    if (pid >= grid.particleCount) return;
    uint ropeIndex = pid / grid.particlesPerRope;
    if (metaBuf[ropeIndex].count == 0) return;

    float2 p = posBuf[pid].xy;
    float2 rel = (p - grid.origin) / grid.cellSize;
    int cx = clamp((int)floor(rel.x), 0, (int)grid.gridWidth - 1);
    int cy = clamp((int)floor(rel.y), 0, (int)grid.gridHeight - 1);
    uint cellIndex = (uint)cy * grid.gridWidth + (uint)cx;

    int old = atomic_exchange_explicit(&gridHeads[cellIndex], (int)pid, memory_order_relaxed);
    nextBuf[pid] = old;
}

kernel void gridCollide(device float4* posBuf [[buffer(0)]],
                        device const atomic_int* gridHeads [[buffer(1)]],
                        device const int* nextBuf [[buffer(2)]],
                        constant GridParams& grid [[buffer(3)]],
                        constant RopeSimParams& params [[buffer(4)]],
                        device const RopeMetaGPU* metaBuf [[buffer(5)]],
                        device float4* prevBuf [[buffer(6)]],
                        uint pid [[thread_position_in_grid]]) {
    if (pid >= grid.particleCount) return;

    uint ropeIndex = pid / grid.particlesPerRope;
    uint localIndex = pid - ropeIndex * grid.particlesPerRope;
    if (metaBuf[ropeIndex].count == 0) return;
    if (localIndex == 0 || localIndex == grid.particlesPerRope - 1) return;

    float3 pos = posBuf[pid].xyz;
    float2 rel = (pos.xy - grid.origin) / grid.cellSize;
    int cx = clamp((int)floor(rel.x), 0, (int)grid.gridWidth - 1);
    int cy = clamp((int)floor(rel.y), 0, (int)grid.gridHeight - 1);

    float radius = params.particleRadius;
    float radiusSum = radius + radius;

    for (int oy = -1; oy <= 1; oy++) {
        for (int ox = -1; ox <= 1; ox++) {
            int nx = clamp(cx + ox, 0, (int)grid.gridWidth - 1);
            int ny = clamp(cy + oy, 0, (int)grid.gridHeight - 1);
            uint cellIndex = (uint)ny * grid.gridWidth + (uint)nx;

            int other = atomic_load_explicit(&gridHeads[cellIndex], memory_order_relaxed);
            while (other != -1) {
                uint oid = (uint)other;
                if (oid != pid) {
                    uint otherRope = oid / grid.particlesPerRope;
                    if (otherRope != ropeIndex) {
                        if (metaBuf[otherRope].count == 0) {
                            other = nextBuf[oid];
                            continue;
                        }
                        float3 op = posBuf[oid].xyz;
                        float3 d = pos - op;
                        float dist2 = dot(d, d);
                        if (dist2 > 1e-10) {
                            float dist = sqrt(dist2);
                            float overlap = radiusSum - dist;
                            if (overlap > 0.0) {
                                float3 n = d / dist;
                                pos += n * (overlap * 0.18);
                                pos.z = max(0.0, pos.z);
                            }
                        }
                    }
                }
                other = nextBuf[oid];
            }
        }
    }

    posBuf[pid] = float4(pos, 1.0);
    prevBuf[pid] = float4(pos, 1.0);
}

