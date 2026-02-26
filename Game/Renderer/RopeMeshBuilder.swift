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

    static func buildRect(points: UnsafeBufferPointer<SIMD3<Float>>, radius: Float, color: SIMD3<Float>, twistEvents: [TwistEvent], tautness: Float, repulsors: [SIMD4<Float>], stretchRatio: Float = 1.0, oscillation: Float = 0.0, segmentStarts: [Int] = [], restLength: Float = 0, crossSection: CrossSection = .circular(radius: 0), materialFrames: [MaterialFrame]? = nil) -> RopeMesh {
        let pointCount = points.count
        if pointCount < 2 { return RopeMesh(vertices: [], indices: []) }

        let usePhysicsFrames = crossSection.isRectangular && materialFrames != nil && materialFrames!.count == pointCount
        
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
        let profile: Profile2D
        switch crossSection {
        case .rectangular(let w, let h):
            profile = rectangularProfile(width: w, height: h)
        default:
            profile = circularProfile(radius: r, segments: 16)
        }
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

            var nrm: SIMD3<Float>
            var bin: SIMD3<Float>

            if usePhysicsFrames {
                let frame = materialFrames![pointIndex]
                nrm = frame.d1
                bin = frame.d2
            } else {
                let tangent: SIMD3<Float>
                if pointIndex == 0 {
                    tangent = simd_normalize(points[1] - points[0])
                } else if pointIndex == pointCount - 1 {
                    tangent = simd_normalize(points[pointCount - 1] - points[pointCount - 2])
                } else {
                    tangent = simd_normalize(points[pointIndex + 1] - points[pointIndex - 1])
                }

                nrm = nrmPrev
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

                bin = simd_cross(tangent, nrm)
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

                tPrev = {
                    if pointIndex == 0 {
                        return simd_normalize(points[1] - points[0])
                    } else if pointIndex == pointCount - 1 {
                        return simd_normalize(points[pointCount - 1] - points[pointCount - 2])
                    } else {
                        return simd_normalize(points[pointIndex + 1] - points[pointIndex - 1])
                    }
                }()
                nrmPrev = nrm
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
                for rep in repulsors {
                    let c = SIMD2<Float>(rep.x, rep.y)
                    let repRadius = rep.z
                    let strength = rep.w
                    let d = p2 - c
                    let d2 = simd_length_squared(d)
                    if d2 < 1e-12 { continue }
                    let dist = sqrt(d2)
                    let falloff = max(1e-4, repRadius * 0.85)
                    let w = smoothstep(edge0: repRadius + falloff, edge1: repRadius, value: dist)
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
            let scale = latexThinning * relaxThickening
            
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
        }

        return RopeMesh(vertices: vertices, indices: indices)
    }

    static func buildHemisphere(center: SIMD3<Float>, radius: Float, facing: SIMD3<Float>, color: SIMD3<Float>, segments: Int = 12, rings: Int = 6, darken: Float = 0.7) -> RopeMesh {
        let r = max(0.001, radius)
        let seg = max(6, segments)
        let rng = max(3, rings)

        var up = SIMD3<Float>(0, 0, 1)
        if abs(simd_dot(up, facing)) > 0.95 { up = SIMD3<Float>(0, 1, 0) }
        let right = simd_normalize(simd_cross(up, facing))
        let forward = simd_normalize(simd_cross(facing, right))

        let darkColor = color * (1.0 - darken)

        var vertices: [RopeVertex] = []
        var indices: [UInt32] = []
        vertices.reserveCapacity((rng + 1) * seg + 1)
        indices.reserveCapacity(rng * seg * 6)

        for ring in 0...rng {
            let phi = (Float(ring) / Float(rng)) * (Float.pi * 0.5)
            let t = Float(ring) / Float(rng)
            let blend = t * t
            let ringColor = color * (1.0 - blend) + darkColor * blend
            let ringR = r * cos(phi)
            let ringZ = r * sin(phi)
            for s in 0..<seg {
                let theta = (Float(s) / Float(seg)) * Float.pi * 2
                let localX = cos(theta) * ringR
                let localY = sin(theta) * ringR
                let pos = center + right * localX + forward * localY + facing * ringZ
                let nrm = simd_normalize(right * (cos(theta) * cos(phi)) + forward * (sin(theta) * cos(phi)) + facing * sin(phi))
                vertices.append(RopeVertex(position: pos, normal: nrm, color: ringColor, texCoord: SIMD2<Float>(0.5, 0.5), params: .zero))
            }
        }

        for ring in 0..<rng {
            for s in 0..<seg {
                let curr = UInt32(ring * seg + s)
                let next = UInt32(ring * seg + (s + 1) % seg)
                let currUp = UInt32((ring + 1) * seg + s)
                let nextUp = UInt32((ring + 1) * seg + (s + 1) % seg)
                indices.append(contentsOf: [curr, currUp, next, next, currUp, nextUp])
            }
        }

        let tipIdx = UInt32(vertices.count)
        vertices.append(RopeVertex(position: center + facing * r, normal: facing, color: darkColor, texCoord: SIMD2<Float>(0.5, 0.5), params: .zero))
        let topRing = rng * seg
        for s in 0..<seg {
            let curr = UInt32(topRing + s)
            let next = UInt32(topRing + (s + 1) % seg)
            indices.append(contentsOf: [curr, tipIdx, next])
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

    private static func rectangularProfile(width: Float, height: Float) -> Profile2D {
        let hw = max(0.0005, width * 0.5)
        let hh = max(0.0005, height * 0.5)
        let cornerR = min(hw, hh) * 0.15
        let cornerSegs = 3

        var pos: [SIMD2<Float>] = []
        var nrm: [SIMD2<Float>] = []
        var v: [Float] = []

        let corners: [(SIMD2<Float>, Float, Float)] = [
            (SIMD2<Float>( hw - cornerR,  hh - cornerR), 0, Float.pi * 0.5),
            (SIMD2<Float>(-hw + cornerR,  hh - cornerR), Float.pi * 0.5, Float.pi),
            (SIMD2<Float>(-hw + cornerR, -hh + cornerR), Float.pi, Float.pi * 1.5),
            (SIMD2<Float>( hw - cornerR, -hh + cornerR), Float.pi * 1.5, Float.pi * 2.0),
        ]

        let totalVerts = corners.count * (cornerSegs + 1)
        pos.reserveCapacity(totalVerts)
        nrm.reserveCapacity(totalVerts)
        v.reserveCapacity(totalVerts)

        var idx = 0
        for (center, startAngle, endAngle) in corners {
            for s in 0...cornerSegs {
                let t = Float(s) / Float(cornerSegs)
                let angle = startAngle + (endAngle - startAngle) * t
                let ca = cos(angle)
                let sa = sin(angle)
                pos.append(center + SIMD2<Float>(ca * cornerR, sa * cornerR))
                nrm.append(simd_normalize(SIMD2<Float>(ca, sa)))
                v.append(Float(idx) / Float(totalVerts))
                idx += 1
            }
        }

        return Profile2D(positions: pos, normals: nrm, v: v)
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

