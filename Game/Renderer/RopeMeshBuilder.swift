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

    static func buildRect(points: UnsafeBufferPointer<SIMD3<Float>>, radius: Float, color: SIMD3<Float>, twistEvents: [TwistEvent], tautness: Float, repulsors: [SIMD4<Float>], stretchRatio: Float = 1.0, oscillation: Float = 0.0, segmentStarts: [Int] = [], restLength: Float = 0) -> RopeMesh {
        let pointCount = points.count
        if pointCount < 2 { return RopeMesh(vertices: [], indices: []) }
        
        let debugColors: [SIMD3<Float>] = [
            SIMD3<Float>(1.0, 0.3, 0.3),
            SIMD3<Float>(0.3, 1.0, 0.3),
            SIMD3<Float>(0.3, 0.3, 1.0),
            SIMD3<Float>(1.0, 1.0, 0.3),
            SIMD3<Float>(1.0, 0.3, 1.0),
            SIMD3<Float>(0.3, 1.0, 1.0),
        ]
        
        func segmentIndex(for pointIndex: Int) -> Int {
            if segmentStarts.isEmpty { return 0 }
            var seg = 0
            for (idx, start) in segmentStarts.enumerated() {
                if pointIndex >= start {
                    seg = idx
                } else {
                    break
                }
            }
            return seg
        }

        let r = max(0.0005, radius)
        let profile = circularProfile(radius: r, segments: 16)
        let profileCount = profile.positions.count

        var totalLen: Float = 0
        for pointIndex in 1..<pointCount {
            totalLen += simd_length(points[pointIndex] - points[pointIndex - 1])
        }
        totalLen = max(1e-6, totalLen)
        
        let effectiveRestLength = restLength > 0 ? restLength : totalLen
        let globalStretchFactor = totalLen / max(1e-6, effectiveRestLength)

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
            var position = points[pointIndex]
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
            let center = sin(uCoord * Float.pi)
            let centerMask = center * center
            let centerMaskStrong = centerMask * centerMask
            
            let baseLatexScale: Float = 0.90
            
            let stretchFromRest = max(0, globalStretchFactor - 1.0)
            
            let stretchEffect = stretchRatio - 1.0
            let dragTension = max(0, stretchEffect)
            
            let totalTension = stretchFromRest + dragTension * 0.8
            
            let latexDeform = totalTension * 0.35 * centerMaskStrong
            
            let stretchRelax = stretchEffect < 0 ? abs(stretchEffect) * 0.2 * centerMask : 0.0
            
            let pinch = latexDeform - stretchRelax * 0.3
            
            let adjustedTautness = max(0.0, tautness - stretchRelax * 0.3)
            
            let endFade = smoothstep(edge0: 0.04, edge1: 0.14, value: uCoord) * smoothstep(edge0: 0.04, edge1: 0.14, value: 1 - uCoord)

            var repelMagTotal: Float = 0
            if !repulsors.isEmpty && endFade > 1e-4 {
                let p2 = SIMD2<Float>(position.x, position.y)
                var repel = SIMD2<Float>(0, 0)
                for r in repulsors {
                    let c = SIMD2<Float>(r.x, r.y)
                    let radius = r.z
                    let strength = r.w
                    let d = p2 - c
                    let d2 = simd_length_squared(d)
                    if d2 < 1e-12 { continue }
                    let dist = sqrt(d2)
                    let falloff = max(1e-4, radius * 0.85)
                    let w = smoothstep(edge0: radius + falloff, edge1: radius, value: dist)
                    if w <= 0 { continue }
                    let dir = d / dist
                    repel += dir * (w * strength)
                    repelMagTotal += w * strength
                }
                let repelScale = endFade * (0.35 + 0.65 * (1 - pinch))
                position.x += repel.x * repelScale
                position.y += repel.y * repelScale
                position.z += min(0.02, repelMagTotal * 0.22) * endFade
            }

            let params = SIMD4<Float>(adjustedTautness, pinch, min(1, repelMagTotal / max(1e-4, radius)), 0)
            
            let latexThinning = 1.0 / sqrt(max(1.0, 1.0 + totalTension * 1.5 * centerMaskStrong))
            let relaxThickening = 1.0 + stretchRelax * 0.15
            let scale = max(0.45, baseLatexScale * latexThinning * relaxThickening)
            
            let lightenAmount = totalTension * centerMaskStrong * 0.35
            let baseColor: SIMD3<Float>
            if !segmentStarts.isEmpty {
                let segIdx = segmentIndex(for: pointIndex)
                baseColor = debugColors[segIdx % debugColors.count]
            } else {
                baseColor = color
            }
            let adjustedColor = baseColor * (1.0 + lightenAmount) + SIMD3<Float>(lightenAmount * 0.15, lightenAmount * 0.15, lightenAmount * 0.15)
            
            let oscWave = sin(uCoord * Float.pi * 3.0 + oscillation * 6.0)
            let oscAmplitude = abs(oscillation) * 0.12
            let oscOffset = oscWave * oscAmplitude
            let oscDir = SIMD2<Float>(nrm.x, nrm.y)
            let oscDirLen = simd_length(oscDir)
            if oscDirLen > 1e-6 {
                let oscDirNorm = oscDir / oscDirLen
                position.x += oscDirNorm.x * oscOffset
                position.y += oscDirNorm.y * oscOffset
            } else {
                let fallbackDir = SIMD2<Float>(bin.x, bin.y)
                let fallbackLen = simd_length(fallbackDir)
                if fallbackLen > 1e-6 {
                    let fallbackNorm = fallbackDir / fallbackLen
                    position.x += fallbackNorm.x * oscOffset
                    position.y += fallbackNorm.y * oscOffset
                }
            }
            
            for k in 0..<profileCount {
                let localPos = profile.positions[k]
                let localN = profile.normals[k]
                let worldPos = position + nrm * (localPos.x * scale) + bin * (localPos.y * scale)
                let worldN = simd_normalize(nrm * localN.x + bin * localN.y)
                vertices.append(RopeVertex(position: worldPos, normal: worldN, color: adjustedColor, texCoord: SIMD2<Float>(uCoord, profile.v[k]), params: params))
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

    private static func circularProfile(radius: Float, segments: Int) -> Profile2D {
        let r = max(0.0005, radius)
        let seg = max(8, min(32, segments))

        var pos: [SIMD2<Float>] = []
        var nrm: [SIMD2<Float>] = []
        var v: [Float] = []

        pos.reserveCapacity(seg)
        nrm.reserveCapacity(seg)
        v.reserveCapacity(seg)

        for i in 0..<seg {
            let angle = Float(i) / Float(seg) * Float.pi * 2.0
            let ca = cos(angle)
            let sa = sin(angle)
            pos.append(SIMD2<Float>(ca * r, sa * r))
            nrm.append(simd_normalize(SIMD2<Float>(ca, sa)))
            v.append(Float(i) / Float(seg))
        }

        return Profile2D(positions: pos, normals: nrm, v: v)
    }
}

