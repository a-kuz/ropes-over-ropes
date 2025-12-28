import simd

enum SegmentIntersection {
    static func intersect(a0: SIMD2<Float>, a1: SIMD2<Float>, b0: SIMD2<Float>, b1: SIMD2<Float>) -> (t: Float, u: Float, p: SIMD2<Float>)? {
        let r = a1 - a0
        let s = b1 - b0
        let rxs = cross(r, s)
        let qpxr = cross(b0 - a0, r)
        let eps: Float = 1e-6

        if abs(rxs) < eps {
            return nil
        }

        let t = cross((b0 - a0), s) / rxs
        let u = cross((b0 - a0), r) / rxs

        if t > eps && t < 1 - eps && u > eps && u < 1 - eps {
            let p = a0 + r * t
            return (t, u, p)
        }

        _ = qpxr
        return nil
    }

    private static func cross(_ a: SIMD2<Float>, _ b: SIMD2<Float>) -> Float {
        a.x * b.y - a.y * b.x
    }
}

