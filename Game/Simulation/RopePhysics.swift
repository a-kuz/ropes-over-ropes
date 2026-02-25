import simd
import os.log

final class RopePhysics {
    private static let logger = Logger(subsystem: "com.uzls.four", category: "RopePhysics")

    private struct LinkState: Equatable {
        let ropeA: Int
        let ropeB: Int
        let linking: Int  // signed linking number — topological invariant
    }

    private var lastLinkStates: [LinkState] = []
    private var lastLogTime: Double = 0

    func logStateIfNeeded(
        time: Double,
        ropes: [(index: Int, points: ContiguousArray<SIMD3<Float>>)]
    ) {
        // Compute linking number for each pair
        var currentLinks: [LinkState] = []
        for i in 0..<ropes.count {
            for j in (i + 1)..<ropes.count {
                let lk = linkingNumber(ropes[i].points, ropes[j].points)
                currentLinks.append(LinkState(ropeA: ropes[i].index, ropeB: ropes[j].index, linking: lk))
            }
        }

        let changed = currentLinks != lastLinkStates
        let timeSinceLastLog = time - lastLogTime

        if changed && timeSinceLastLog >= 0.5 {
            // Detect pass-throughs: linking number changed
            for link in currentLinks {
                if let prev = lastLinkStates.first(where: { $0.ropeA == link.ropeA && $0.ropeB == link.ropeB }) {
                    if prev.linking != link.linking {
                        Self.logger.error("[PASSTHROUGH] R\(link.ropeA)-R\(link.ropeB) linking \(prev.linking)→\(link.linking)")
                    }
                }
            }

            lastLogTime = time
            lastLinkStates = currentLinks

            let linkStr = currentLinks.map { "\($0.ropeA)-\($0.ropeB):\($0.linking)" }.joined(separator: " ")
            Self.logger.info("[STATE] t=\(String(format: "%.1f", time)) active=\(ropes.count) links: \(linkStr)")
        }
    }

    /// Signed linking number: sum of (crossing_orientation * over_under_sign) for all 2D crossings.
    /// Topological invariant — only changes when a rope physically passes through another.
    /// Each pass-through changes it by ±2.
    func linkingNumber<C1: RandomAccessCollection, C2: RandomAccessCollection>(
        _ ptsA: C1, _ ptsB: C2
    ) -> Int where C1.Element == SIMD3<Float>, C2.Element == SIMD3<Float>, C1.Index == Int, C2.Index == Int {
        var sum = 0
        for i in ptsA.startIndex..<(ptsA.endIndex - 1) {
            let a0 = SIMD2<Float>(ptsA[i].x, ptsA[i].y)
            let a1 = SIMD2<Float>(ptsA[i + 1].x, ptsA[i + 1].y)
            let d1 = a1 - a0
            for j in ptsB.startIndex..<(ptsB.endIndex - 1) {
                let b0 = SIMD2<Float>(ptsB[j].x, ptsB[j].y)
                let b1 = SIMD2<Float>(ptsB[j + 1].x, ptsB[j + 1].y)
                let d2 = b1 - b0
                let cross = d1.x * d2.y - d1.y * d2.x
                if abs(cross) < 1e-9 { continue }
                let d = b0 - a0
                let t = (d.x * d2.y - d.y * d2.x) / cross
                let u = (d.x * d1.y - d.y * d1.x) / cross
                if t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99 {
                    let zA = ptsA[i].z * (1 - t) + ptsA[i + 1].z * t
                    let zB = ptsB[j].z * (1 - u) + ptsB[j + 1].z * u
                    let orientation = cross > 0 ? 1 : -1    // crossing direction
                    let over = zA > zB ? 1 : -1              // who's on top
                    sum += orientation * over
                }
            }
        }
        return sum
    }
}
