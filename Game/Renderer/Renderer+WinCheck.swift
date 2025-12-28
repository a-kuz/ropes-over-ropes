import simd

extension Renderer {
    func removeUntangledRopes() {
        var removed = true
        while removed {
            removed = false
            for ropeIndex in ropes.indices {
                if ropes[ropeIndex].startHole == -1 { continue }
                if isRopeUntangled(ropeIndex: ropeIndex) {
                    deactivateRope(ropeIndex: ropeIndex)
                    removed = true
                    break
                }
            }
        }
    }

    private func deactivateRope(ropeIndex: Int) {
        let startHoleIndex = ropes[ropeIndex].startHole
        let endHoleIndex = ropes[ropeIndex].endHole
        if startHoleIndex >= 0 && startHoleIndex < holeOccupied.count {
            holeOccupied[startHoleIndex] = false
        }
        if endHoleIndex >= 0 && endHoleIndex < holeOccupied.count {
            holeOccupied[endHoleIndex] = false
        }
        ropes[ropeIndex].startHole = -1
        ropes[ropeIndex].endHole = -1
        simulation.deactivateRope(ropeIndex: ropeIndex)
        topology?.deactivateRope(ropeIndex: ropeIndex)
    }

    private func isRopeUntangled(ropeIndex: Int) -> Bool {
        guard let topology else { return false }
        if ropeIndex >= topology.ropes.count { return false }
        if !topology.ropes[ropeIndex].active { return false }
        if topology.ropes[ropeIndex].nodes.count < 2 { return false }

        for crossing in topology.crossings.values {
            if crossing.ropeA == ropeIndex || crossing.ropeB == ropeIndex {
                return false
            }
        }

        return true
    }

    private func segmentsIntersect(a0: SIMD2<Float>, a1: SIMD2<Float>, b0: SIMD2<Float>, b1: SIMD2<Float>) -> Bool {
        let o1 = orient(a: a0, b: a1, c: b0)
        let o2 = orient(a: a0, b: a1, c: b1)
        let o3 = orient(a: b0, b: b1, c: a0)
        let o4 = orient(a: b0, b: b1, c: a1)

        if o1 == 0 && onSegment(a: a0, b: a1, p: b0) { return true }
        if o2 == 0 && onSegment(a: a0, b: a1, p: b1) { return true }
        if o3 == 0 && onSegment(a: b0, b: b1, p: a0) { return true }
        if o4 == 0 && onSegment(a: b0, b: b1, p: a1) { return true }

        return (o1 != o2) && (o3 != o4)
    }

    private func orient(a: SIMD2<Float>, b: SIMD2<Float>, c: SIMD2<Float>) -> Int {
        let v = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y)
        let eps: Float = 1e-6
        if abs(v) < eps { return 0 }
        return v > 0 ? 1 : 2
    }

    private func onSegment(a: SIMD2<Float>, b: SIMD2<Float>, p: SIMD2<Float>) -> Bool {
        min(a.x, b.x) - 1e-6 <= p.x && p.x <= max(a.x, b.x) + 1e-6 &&
        min(a.y, b.y) - 1e-6 <= p.y && p.y <= max(a.y, b.y) + 1e-6
    }
}

