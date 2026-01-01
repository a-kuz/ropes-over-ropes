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

    private func segmentsIntersect(
        segmentAStart: SIMD2<Float>,
        segmentAEnd: SIMD2<Float>,
        segmentBStart: SIMD2<Float>,
        segmentBEnd: SIMD2<Float>
    ) -> Bool {
        let orient1 = orient(pointA: segmentAStart, pointB: segmentAEnd, pointC: segmentBStart)
        let orient2 = orient(pointA: segmentAStart, pointB: segmentAEnd, pointC: segmentBEnd)
        let orient3 = orient(pointA: segmentBStart, pointB: segmentBEnd, pointC: segmentAStart)
        let orient4 = orient(pointA: segmentBStart, pointB: segmentBEnd, pointC: segmentAEnd)

        if orient1 == 0 && onSegment(start: segmentAStart, end: segmentAEnd, point: segmentBStart) { return true }
        if orient2 == 0 && onSegment(start: segmentAStart, end: segmentAEnd, point: segmentBEnd) { return true }
        if orient3 == 0 && onSegment(start: segmentBStart, end: segmentBEnd, point: segmentAStart) { return true }
        if orient4 == 0 && onSegment(start: segmentBStart, end: segmentBEnd, point: segmentAEnd) { return true }

        return (orient1 != orient2) && (orient3 != orient4)
    }

    private func orient(pointA: SIMD2<Float>, pointB: SIMD2<Float>, pointC: SIMD2<Float>) -> Int {
        let value = (pointB.y - pointA.y) * (pointC.x - pointB.x) - (pointB.x - pointA.x) * (pointC.y - pointB.y)
        let eps: Float = 1e-6
        if abs(value) < eps { return 0 }
        return value > 0 ? 1 : 2
    }

    private func onSegment(start: SIMD2<Float>, end: SIMD2<Float>, point: SIMD2<Float>) -> Bool {
        min(start.x, end.x) - 1e-6 <= point.x && point.x <= max(start.x, end.x) + 1e-6 &&
        min(start.y, end.y) - 1e-6 <= point.y && point.y <= max(start.y, end.y) + 1e-6
    }
}
