import simd

extension VerletSimulator {
    static func computeFrames(
        positions: ContiguousArray<SIMD3<Float>>,
        twistAngles: ContiguousArray<Float>
    ) -> [MaterialFrame] {
        let n = positions.count
        guard n >= 2 else { return [] }

        var frames = [MaterialFrame](repeating: MaterialFrame(tangent: .zero, d1: .zero, d2: .zero), count: n)
        let up = SIMD3<Float>(0, 0, 1)

        var tPrev = simd_normalize(positions[1] - positions[0])
        var uPrev: SIMD3<Float> = {
            var u = simd_cross(up, tPrev)
            if simd_length_squared(u) < 1e-8 { u = SIMD3<Float>(1, 0, 0) }
            return simd_normalize(u)
        }()

        for i in 0..<n {
            let tangent: SIMD3<Float>
            if i == 0 {
                tangent = simd_normalize(positions[1] - positions[0])
            } else if i == n - 1 {
                tangent = simd_normalize(positions[n - 1] - positions[n - 2])
            } else {
                tangent = simd_normalize(positions[i + 1] - positions[i - 1])
            }

            var u = uPrev
            if i > 0 {
                let axis = simd_cross(tPrev, tangent)
                let axisLen = simd_length(axis)
                if axisLen > 1e-6 {
                    let axisN = axis / axisLen
                    let dotClamped = max(-1.0 as Float, min(1.0 as Float, simd_dot(tPrev, tangent)))
                    let angle = atan2(axisLen, dotClamped)
                    u = rotateVector(u, axis: axisN, angle: angle)
                    let proj = u - tangent * simd_dot(u, tangent)
                    if simd_length_squared(proj) > 1e-10 {
                        u = simd_normalize(proj)
                    }
                }
            }

            var v = simd_cross(tangent, u)
            if simd_length_squared(v) < 1e-8 {
                var fallback = simd_cross(up, tangent)
                if simd_length_squared(fallback) < 1e-8 { fallback = SIMD3<Float>(1, 0, 0) }
                u = simd_normalize(fallback)
                v = simd_cross(tangent, u)
            }
            v = simd_normalize(v)

            let twist = twistAngles[i]
            let cosT = cos(twist)
            let sinT = sin(twist)
            let d1 = u * cosT + v * sinT
            let d2 = -u * sinT + v * cosT

            frames[i] = MaterialFrame(tangent: tangent, d1: d1, d2: d2)

            tPrev = tangent
            uPrev = u
        }

        return frames
    }

    static func rotateVector(_ vector: SIMD3<Float>, axis: SIMD3<Float>, angle: Float) -> SIMD3<Float> {
        let cosA = cos(angle)
        let sinA = sin(angle)
        return vector * cosA
            + simd_cross(axis, vector) * sinA
            + axis * simd_dot(axis, vector) * (1 - cosA)
    }

    func recomputeFrames() {
        if cachedFrames.count != bands.count {
            cachedFrames = Array(repeating: [], count: bands.count)
        }
        for bi in bands.indices {
            guard bands[bi].active && bands[bi].fadeOut == 0 else {
                cachedFrames[bi] = []
                continue
            }
            if bands[bi].crossSection.isRectangular {
                cachedFrames[bi] = Self.computeFrames(
                    positions: bands[bi].positions,
                    twistAngles: bands[bi].twistAngles
                )
            } else if squareCrossSection {
                let zeroTwist = ContiguousArray<Float>(repeating: 0, count: bands[bi].positions.count)
                cachedFrames[bi] = Self.computeFrames(
                    positions: bands[bi].positions,
                    twistAngles: zeroTwist
                )
            } else {
                cachedFrames[bi] = []
            }
        }
    }
}
