import simd

struct RopeMesh {
    var vertices: [RopeVertex]
    var indices: [UInt32]
}

enum RopeMeshBuilder {
    struct TwistEvent {
        let dist: Float
        let angle: Float
        let window: Float
    }

    static func buildRect(points: UnsafeBufferPointer<SIMD3<Float>>, width: Float, height: Float, color: SIMD3<Float>, twistEvents: [TwistEvent]) -> RopeMesh {
        let pointCount = points.count
        if pointCount < 2 { return RopeMesh(vertices: [], indices: []) }

        let bandWidth = max(0.0005, width)
        let bandHeight = max(0.0005, height)
        let profile = beveledProfile(width: bandWidth, height: bandHeight, radiusFactor: 0.22, cornerSegments: 5)
        let profileCount = profile.positions.count

        var totalLen: Float = 0
        for pointIndex in 1..<pointCount {
            totalLen += simd_length(points[pointIndex] - points[pointIndex - 1])
        }
        totalLen = max(1e-6, totalLen)

        var vertices: [RopeVertex] = []
        vertices.reserveCapacity(pointCount * profileCount)

        var indices: [UInt32] = []
        indices.reserveCapacity((pointCount - 1) * profileCount * 6)

        let up = SIMD3<Float>(0, 0, 1)

        var distanceAlong: Float = 0
        var tPrev = simd_normalize(points[1] - points[0])
        var nrmPrev: SIMD3<Float> = {
            var nrm = simd_cross(up, tPrev)
            if simd_length_squared(nrm) < 1e-8 { nrm = SIMD3<Float>(1, 0, 0) }
            return simd_normalize(nrm)
        }()

        for pointIndex in 0..<pointCount {
            let position = points[pointIndex]
            if pointIndex > 0 {
                distanceAlong += simd_length(points[pointIndex] - points[pointIndex - 1])
            }

            let tangent: SIMD3<Float>
            if pointIndex == 0 {
                tangent = simd_normalize(points[1] - points[0])
            } else if pointIndex == pointCount - 1 {
                tangent = simd_normalize(points[pointCount - 1] - points[pointCount - 2])
            } else {
                tangent = simd_normalize(points[pointIndex + 1] - points[pointIndex - 1])
            }

            var nrm = nrmPrev
            if pointIndex > 0 {
                let axis = simd_cross(tPrev, tangent)
                let axisLen = simd_length(axis)
                if axisLen > 1e-6 {
                    let axisN = axis / axisLen
                    let dotClamped = max(-1.0 as Float, min(1.0 as Float, simd_dot(tPrev, tangent)))
                    let angle = atan2(axisLen, dotClamped)
                    nrm = rotate(vector: nrmPrev, axis: axisN, angle: angle)
                    let proj = nrm - tangent * simd_dot(nrm, tangent)
                    if simd_length_squared(proj) > 1e-10 {
                        nrm = simd_normalize(proj)
                    }
                }
            }

            var bin = simd_cross(tangent, nrm)
            if simd_length_squared(bin) < 1e-8 {
                var fallback = simd_cross(up, tangent)
                if simd_length_squared(fallback) < 1e-8 { fallback = SIMD3<Float>(1, 0, 0) }
                nrm = simd_normalize(fallback)
                bin = simd_cross(tangent, nrm)
            }
            bin = simd_normalize(bin)

            let twist = twistAngle(at: distanceAlong, events: twistEvents)
            if abs(twist) > 1e-6 {
                let n2 = nrm * cos(twist) + bin * sin(twist)
                let b2 = -nrm * sin(twist) + bin * cos(twist)
                nrm = n2
                bin = b2
            }

            let uCoord = distanceAlong / totalLen
            for k in 0..<profileCount {
                let localPos = profile.positions[k]
                let localN = profile.normals[k]
                let worldPos = position + nrm * localPos.x + bin * localPos.y
                let worldN = simd_normalize(nrm * localN.x + bin * localN.y)
                vertices.append(RopeVertex(position: worldPos, normal: worldN, color: color, texCoord: SIMD2<Float>(uCoord, profile.v[k])))
            }

            if pointIndex < pointCount - 1 {
                let baseA = UInt32(pointIndex * profileCount)
                let baseB = UInt32((pointIndex + 1) * profileCount)
                for k in 0..<profileCount {
                    let k0 = UInt32(k)
                    let k1 = UInt32((k + 1) % profileCount)
                    indices.append(baseA + k0)
                    indices.append(baseB + k0)
                    indices.append(baseB + k1)
                    indices.append(baseA + k0)
                    indices.append(baseB + k1)
                    indices.append(baseA + k1)
                }
            }

            tPrev = tangent
            nrmPrev = nrm
        }

        return RopeMesh(vertices: vertices, indices: indices)
    }

    private static func rotate(vector: SIMD3<Float>, axis: SIMD3<Float>, angle: Float) -> SIMD3<Float> {
        let cosAngle = cos(angle)
        let sinAngle = sin(angle)
        return vector * cosAngle
            + simd_cross(axis, vector) * sinAngle
            + axis * simd_dot(axis, vector) * (1 - cosAngle)
    }

    private static func twistAngle(at distanceAlong: Float, events: [TwistEvent]) -> Float {
        if events.isEmpty { return 0 }
        var accumulated: Float = 0
        for event in events {
            let distance = abs(distanceAlong - event.dist)
            let norm = min(1, distance / max(1e-6, event.window))
            let weight = smoothstep(edge0: 1, edge1: 0, value: norm)
            accumulated += event.angle * weight
        }
        return accumulated
    }

    private static func smoothstep(edge0: Float, edge1: Float, value: Float) -> Float {
        let normalized = max(0, min(1, (value - edge0) / (edge1 - edge0)))
        return normalized * normalized * (3 - 2 * normalized)
    }

    private struct Profile2D {
        let positions: [SIMD2<Float>]
        let normals: [SIMD2<Float>]
        let v: [Float]
    }

    private static func beveledProfile(width: Float, height: Float, radiusFactor: Float, cornerSegments: Int) -> Profile2D {
        let halfW = width * 0.5
        let halfH = height * 0.5
        let rMax = max(0.0005, min(halfW, halfH))
        let r = min(rMax * radiusFactor, rMax * 0.92)
        let seg = max(2, min(12, cornerSegments))

        var pos: [SIMD2<Float>] = []
        var nrm: [SIMD2<Float>] = []
        var v: [Float] = []

        pos.reserveCapacity(seg * 4 + 8)
        nrm.reserveCapacity(seg * 4 + 8)
        v.reserveCapacity(seg * 4 + 8)

        func addArc(cx: Float, cy: Float, a0: Float, a1: Float) {
            for i in 0...seg {
                let t = Float(i) / Float(seg)
                let a = a0 + (a1 - a0) * t
                let ca = cos(a)
                let sa = sin(a)
                pos.append(SIMD2<Float>(cx + ca * r, cy + sa * r))
                nrm.append(simd_normalize(SIMD2<Float>(ca, sa)))
            }
        }

        addArc(cx: halfW - r, cy: halfH - r, a0: 0, a1: Float.pi * 0.5)
        addArc(cx: -halfW + r, cy: halfH - r, a0: Float.pi * 0.5, a1: Float.pi)
        addArc(cx: -halfW + r, cy: -halfH + r, a0: Float.pi, a1: Float.pi * 1.5)
        addArc(cx: halfW - r, cy: -halfH + r, a0: Float.pi * 1.5, a1: Float.pi * 2.0)

        if pos.count > 1 {
            pos.removeLast()
            nrm.removeLast()
        }

        let count = pos.count
        for i in 0..<count {
            v.append(Float(i) / Float(count))
        }

        return Profile2D(positions: pos, normals: nrm, v: v)
    }
}

