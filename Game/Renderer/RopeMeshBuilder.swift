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

        var totalLen: Float = 0
        for pointIndex in 1..<pointCount {
            totalLen += simd_length(points[pointIndex] - points[pointIndex - 1])
        }
        totalLen = max(1e-6, totalLen)

        var vertices: [RopeVertex] = []
        vertices.reserveCapacity(pointCount * 8)

        var indices: [UInt32] = []
        indices.reserveCapacity((pointCount - 1) * 24)

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
            let halfWidth = bandWidth * 0.5
            let halfHeight = bandHeight * 0.5

            let c00 = position + (-nrm * halfWidth) + (-bin * halfHeight)
            let c01 = position + (-nrm * halfWidth) + ( bin * halfHeight)
            let c10 = position + ( nrm * halfWidth) + (-bin * halfHeight)
            let c11 = position + ( nrm * halfWidth) + ( bin * halfHeight)

            vertices.append(RopeVertex(position: c00, normal: -nrm, color: color, texCoord: SIMD2<Float>(uCoord, 0)))
            vertices.append(RopeVertex(position: c01, normal: -nrm, color: color, texCoord: SIMD2<Float>(uCoord, 1)))
            vertices.append(RopeVertex(position: c10, normal: nrm, color: color, texCoord: SIMD2<Float>(uCoord, 0)))
            vertices.append(RopeVertex(position: c11, normal: nrm, color: color, texCoord: SIMD2<Float>(uCoord, 1)))

            vertices.append(RopeVertex(position: c00, normal: -bin, color: color, texCoord: SIMD2<Float>(uCoord, 0)))
            vertices.append(RopeVertex(position: c10, normal: -bin, color: color, texCoord: SIMD2<Float>(uCoord, 1)))
            vertices.append(RopeVertex(position: c01, normal: bin, color: color, texCoord: SIMD2<Float>(uCoord, 0)))
            vertices.append(RopeVertex(position: c11, normal: bin, color: color, texCoord: SIMD2<Float>(uCoord, 1)))

            if pointIndex < pointCount - 1 {
                let baseA = UInt32(pointIndex * 8)
                let baseB = UInt32((pointIndex + 1) * 8)
                appendQuad(indices: &indices, a0: baseA + 0, a1: baseA + 1, b0: baseB + 0, b1: baseB + 1)
                appendQuad(indices: &indices, a0: baseA + 2, a1: baseA + 3, b0: baseB + 2, b1: baseB + 3)
                appendQuad(indices: &indices, a0: baseA + 4, a1: baseA + 5, b0: baseB + 4, b1: baseB + 5)
                appendQuad(indices: &indices, a0: baseA + 6, a1: baseA + 7, b0: baseB + 6, b1: baseB + 7)
            }

            tPrev = tangent
            nrmPrev = nrm
        }

        return RopeMesh(vertices: vertices, indices: indices)
    }

    private static func appendQuad(indices: inout [UInt32], a0: UInt32, a1: UInt32, b0: UInt32, b1: UInt32) {
        indices.append(a0)
        indices.append(b0)
        indices.append(b1)
        indices.append(a0)
        indices.append(b1)
        indices.append(a1)
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
}

