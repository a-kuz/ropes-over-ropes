import Foundation
import simd

enum LevelGenerator {
    // MARK: - Pastel colors

    private static let colors: [LevelDefinition.Color] = [
        .init(redChannel: 0.95, greenChannel: 0.90, blueChannel: 0.35),  // yellow
        .init(redChannel: 0.95, greenChannel: 0.55, blueChannel: 0.65),  // pink
        .init(redChannel: 0.55, greenChannel: 0.85, blueChannel: 0.95),  // light blue
        .init(redChannel: 0.75, greenChannel: 0.60, blueChannel: 0.90),  // purple
        .init(redChannel: 0.55, greenChannel: 0.92, blueChannel: 0.60),  // green
    ]

    // MARK: - Difficulty

    private struct Difficulty {
        let ropeCount: Int
        let totalDrags: Int  // total number of individual drag operations
    }

    private static func difficulty(for levelId: Int) -> Difficulty {
        switch levelId {
        case 1...2:   return Difficulty(ropeCount: 2, totalDrags: 2)
        case 3...5:   return Difficulty(ropeCount: 3, totalDrags: 3)
        case 6...9:   return Difficulty(ropeCount: 3, totalDrags: 4)
        case 10...15: return Difficulty(ropeCount: 4, totalDrags: 5)
        case 16...25: return Difficulty(ropeCount: 4, totalDrags: 6)
        case 26...50: return Difficulty(ropeCount: 5, totalDrags: 7 + (levelId - 25) / 5)
        default:      return Difficulty(ropeCount: 5, totalDrags: min(15, 12 + (levelId - 50) / 20))
        }
    }

    // MARK: - Hole layouts (deterministic, not random)

    private enum HoleLayout: CaseIterable {
        case grid4x5
        case circle12
        case hexagon
        case diamond
        case cross
        case twoRings
        case triangle
        case star

        func generate() -> [LevelDefinition.Vec2] {
            switch self {
            case .grid4x5:
                return grid(cols: 5, rows: 4, spacingX: 0.42, spacingY: 0.45)
            case .circle12:
                return circle(count: 12, radius: 0.72)
            case .hexagon:
                return hexagonLayout()
            case .diamond:
                return diamondLayout()
            case .cross:
                return crossLayout()
            case .twoRings:
                return twoRingsLayout()
            case .triangle:
                return triangleLayout()
            case .star:
                return starLayout()
            }
        }

        private func grid(cols: Int, rows: Int, spacingX: Float, spacingY: Float) -> [LevelDefinition.Vec2] {
            let ox = -Float(cols - 1) / 2 * spacingX
            let oy = -Float(rows - 1) / 2 * spacingY
            var pts: [LevelDefinition.Vec2] = []
            for row in 0..<rows {
                for col in 0..<cols {
                    pts.append(.init(xPosition: ox + Float(col) * spacingX,
                                     yPosition: oy + Float(row) * spacingY))
                }
            }
            return pts
        }

        private func circle(count: Int, radius: Float) -> [LevelDefinition.Vec2] {
            (0..<count).map { i in
                let angle = Float(i) / Float(count) * 2 * .pi - .pi / 2
                return .init(xPosition: radius * cos(angle), yPosition: radius * sin(angle))
            }
        }

        private func hexagonLayout() -> [LevelDefinition.Vec2] {
            // Hexagonal grid: center + 6 + 12
            var pts: [LevelDefinition.Vec2] = [.init(xPosition: 0, yPosition: 0)]
            let r1: Float = 0.45
            for i in 0..<6 {
                let a = Float(i) / 6 * 2 * .pi - .pi / 6
                pts.append(.init(xPosition: r1 * cos(a), yPosition: r1 * sin(a)))
            }
            let r2: Float = 0.85
            for i in 0..<12 {
                let a = Float(i) / 12 * 2 * .pi
                pts.append(.init(xPosition: r2 * cos(a), yPosition: r2 * sin(a)))
            }
            return pts
        }

        private func diamondLayout() -> [LevelDefinition.Vec2] {
            let s: Float = 0.38
            var pts: [LevelDefinition.Vec2] = []
            // Diamond shape: rows of 1,3,5,3,1
            let rows: [Int] = [1, 3, 5, 3, 1]
            for (rowIdx, count) in rows.enumerated() {
                let y = (Float(rowIdx) - 2) * s
                let ox = -Float(count - 1) / 2 * s
                for col in 0..<count {
                    pts.append(.init(xPosition: ox + Float(col) * s, yPosition: y))
                }
            }
            return pts  // 13 holes
        }

        private func crossLayout() -> [LevelDefinition.Vec2] {
            let s: Float = 0.40
            var pts: [LevelDefinition.Vec2] = []
            // Vertical bar: 5 holes
            for i in -2...2 {
                pts.append(.init(xPosition: 0, yPosition: Float(i) * s))
            }
            // Horizontal arms: 2 on each side (skip center already added)
            for i in [-2, -1, 1, 2] {
                pts.append(.init(xPosition: Float(i) * s, yPosition: 0))
            }
            // Corner accents
            for dx: Float in [-1, 1] {
                for dy: Float in [-1, 1] {
                    pts.append(.init(xPosition: dx * s, yPosition: dy * s))
                }
            }
            return pts  // 13 holes
        }

        private func twoRingsLayout() -> [LevelDefinition.Vec2] {
            var pts: [LevelDefinition.Vec2] = []
            // Inner ring
            for i in 0..<6 {
                let a = Float(i) / 6 * 2 * .pi
                pts.append(.init(xPosition: 0.35 * cos(a), yPosition: 0.35 * sin(a)))
            }
            // Outer ring
            for i in 0..<10 {
                let a = Float(i) / 10 * 2 * .pi + .pi / 10
                pts.append(.init(xPosition: 0.78 * cos(a), yPosition: 0.78 * sin(a)))
            }
            return pts  // 16 holes
        }

        private func triangleLayout() -> [LevelDefinition.Vec2] {
            let s: Float = 0.38
            var pts: [LevelDefinition.Vec2] = []
            // Rows of 1,2,3,4,5 — equilateral triangle
            for row in 0..<5 {
                let count = row + 1
                let ox = -Float(count - 1) / 2 * s
                let y = (Float(row) - 2) * s * 0.866  // sin(60°)
                for col in 0..<count {
                    pts.append(.init(xPosition: ox + Float(col) * s, yPosition: y))
                }
            }
            return pts  // 15 holes
        }

        private func starLayout() -> [LevelDefinition.Vec2] {
            var pts: [LevelDefinition.Vec2] = [.init(xPosition: 0, yPosition: 0)]
            // 5-pointed star: outer tips + inner vertices
            for i in 0..<5 {
                let outerAngle = Float(i) / 5 * 2 * .pi - .pi / 2
                pts.append(.init(xPosition: 0.82 * cos(outerAngle), yPosition: 0.82 * sin(outerAngle)))
                let innerAngle = outerAngle + .pi / 5
                pts.append(.init(xPosition: 0.38 * cos(innerAngle), yPosition: 0.38 * sin(innerAngle)))
            }
            // Extra ring for more endpoints
            for i in 0..<5 {
                let a = Float(i) / 5 * 2 * .pi - .pi / 2 + .pi / 5
                pts.append(.init(xPosition: 0.60 * cos(a), yPosition: 0.60 * sin(a)))
            }
            return pts  // 16 holes
        }
    }

    // MARK: - Generation

    static func generate(levelId: Int) -> LevelDefinition {
        var rng = SeededRNG(seed: UInt64(levelId) &* 2654435761)

        // Pick layout based on levelId (cycle through all layouts)
        let layouts = HoleLayout.allCases
        let layout = layouts[levelId % layouts.count]
        let holes = layout.generate()

        let diff = difficulty(for: levelId)
        let ropeCount = diff.ropeCount

        // Build structured rope pairs: connect roughly-opposite holes through center
        // This creates star/cross patterns instead of random spaghetti
        let ropePairs = pickStructuredPairs(
            holes: holes, count: ropeCount, rng: &rng
        )
        var usedHoles = Set(ropePairs.flatMap { [$0.0, $0.1] })

        let ropes = ropePairs.enumerated().map { i, pair in
            LevelDefinition.Rope(
                startHole: pair.0,
                endHole: pair.1,
                color: colors[i % colors.count],
                radius: 0.038
            )
        }

        // Generate actions: pin all, then drag to create tangles
        var actions: [LevelDefinition.Action] = []

        for i in 0..<ropes.count {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: ropes[i].startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: ropes[i].endHole))
        }

        // Generate drags that create real tangling.
        // Strategy: for each drag, pick a TARGET rope to cross, then move our end
        // to the opposite side of that rope. This guarantees the drag path crosses
        // the target rope, creating an actual interleaving in physics.
        var currentEndpoints = ropes.map { ($0.startHole, $0.endHole) }
        let holeSimd = holes.map { $0.simd }

        for d in 0..<diff.totalDrags {
            let ropeIdx = d % ropes.count
            let endIdx = 0  // always drag start end — consistent direction builds linking

            let currentHole = endIdx == 0 ? currentEndpoints[ropeIdx].0 : currentEndpoints[ropeIdx].1
            let currentPos = holeSimd[currentHole]

            // Pick a target rope to cross (cycle through others)
            let targetRopeIdx = (ropeIdx + 1 + d / ropes.count) % ropes.count
            if targetRopeIdx == ropeIdx { continue }

            let targetStart = holeSimd[currentEndpoints[targetRopeIdx].0]
            let targetEnd = holeSimd[currentEndpoints[targetRopeIdx].1]
            let targetMid = (targetStart + targetEnd) * 0.5
            let targetDir = simd_normalize(targetEnd - targetStart)
            // Normal to target rope (perpendicular)
            let targetNormal = SIMD2<Float>(-targetDir.y, targetDir.x)

            // Which side of target rope is our current position?
            let currentSide = simd_dot(currentPos - targetMid, targetNormal)

            // We want to go to the OPPOSITE side → guaranteed crossing
            let otherUsed = Set(
                (0..<ropes.count).filter { $0 != ropeIdx }.flatMap {
                    [currentEndpoints[$0].0, currentEndpoints[$0].1]
                }
            )
            let anchorHole = endIdx == 0 ? currentEndpoints[ropeIdx].1 : currentEndpoints[ropeIdx].0

            var bestHole: Int?
            var bestScore: Float = -.greatestFiniteMagnitude

            for candidate in 0..<holes.count {
                if candidate == currentHole || candidate == anchorHole { continue }
                if otherUsed.contains(candidate) { continue }

                let candidatePos = holeSimd[candidate]
                let candidateSide = simd_dot(candidatePos - targetMid, targetNormal)

                // Must be on opposite side (sign differs)
                guard currentSide * candidateSide < 0 else { continue }

                // Score: prefer candidates far from current (big crossing), close to target mid
                let distToMid = simd_length(candidatePos - targetMid)
                let score = -distToMid  // closer to mid = better crossing
                if score > bestScore {
                    bestScore = score
                    bestHole = candidate
                }
            }

            guard let targetHole = bestHole else { continue }

            usedHoles.remove(currentHole)
            usedHoles.insert(targetHole)

            if endIdx == 0 {
                currentEndpoints[ropeIdx].0 = targetHole
            } else {
                currentEndpoints[ropeIdx].1 = targetHole
            }

            actions.append(.init(type: "drag", ropeIndex: ropeIdx, endIndex: endIdx, holeIndex: targetHole))
        }

        return LevelDefinition(
            id: levelId,
            holeRadius: 0.08,
            particlesPerRope: 32,
            holes: holes,
            ropes: ropes,
            hooks: nil,
            actions: actions
        )
    }

    // MARK: - Structured rope selection

    /// Pick hole pairs that form a star/cross pattern through the center.
    /// Strategy: sort holes by angle from center, pair roughly-opposite holes.
    /// Offset by rng to get variation between levels with same layout.
    private static func pickStructuredPairs(
        holes: [LevelDefinition.Vec2], count: Int, rng: inout SeededRNG
    ) -> [(Int, Int)] {
        guard holes.count >= 4 else { return [] }

        // Center of all holes
        let cx = holes.map { $0.simd.x }.reduce(0, +) / Float(holes.count)
        let cy = holes.map { $0.simd.y }.reduce(0, +) / Float(holes.count)
        let center = SIMD2<Float>(cx, cy)

        // Sort holes by angle from center
        let indexed = holes.enumerated().map { (idx: $0.offset, angle: atan2($0.element.simd.y - cy, $0.element.simd.x - cx)) }
        let sorted = indexed.sorted { $0.angle < $1.angle }

        // Offset starting position for variety
        let offset = rng.next(bound: sorted.count)

        var pairs: [(Int, Int)] = []
        var usedHoles = Set<Int>()
        let n = sorted.count

        // Pair holes that are roughly opposite but offset by 1 position,
        // so ropes cross through the center instead of being parallel.
        // Pattern: hole[i] → hole[i + n/2 + shift], where shift varies per rope.
        let half = n / 2
        for i in 0..<count {
            guard pairs.count < count else { break }
            let aIdx = (i * 2 + offset) % n  // spread starts evenly
            let a = sorted[aIdx].idx
            if usedHoles.contains(a) { continue }

            // Opposite with offset: shift by (i+1) to create crossing
            let shift = (i % 2 == 0) ? 1 : -1
            let bIdx = (aIdx + half + shift + n) % n
            let b = sorted[bIdx].idx

            if b != a && !usedHoles.contains(b) {
                pairs.append((a, b))
                usedHoles.insert(a)
                usedHoles.insert(b)
            }
        }

        // Fallback: fill remaining with any far-apart unused pairs
        if pairs.count < count {
            for i in 0..<n {
                guard pairs.count < count else { break }
                let a = sorted[(i + offset) % n].idx
                if usedHoles.contains(a) { continue }
                for j in (i+1)..<n {
                    let b = sorted[(j + offset) % n].idx
                    if usedHoles.contains(b) { continue }
                    let dist = simd_length(holes[a].simd - holes[b].simd)
                    if dist > 0.5 {
                        pairs.append((a, b))
                        usedHoles.insert(a)
                        usedHoles.insert(b)
                        break
                    }
                }
            }
        }

        return pairs
    }

    // MARK: - Helpers

    private static func segmentsCross(
        _ a0: SIMD2<Float>, _ a1: SIMD2<Float>,
        _ b0: SIMD2<Float>, _ b1: SIMD2<Float>
    ) -> Bool {
        let d1 = a1 - a0
        let d2 = b1 - b0
        let cross = d1.x * d2.y - d1.y * d2.x
        if abs(cross) < 1e-9 { return false }
        let d = b0 - a0
        let t = (d.x * d2.y - d.y * d2.x) / cross
        let u = (d.x * d1.y - d.y * d1.x) / cross
        return t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99
    }
}

// MARK: - Seeded RNG (deterministic per level)

private struct SeededRNG {
    private var state: UInt64

    init(seed: UInt64) {
        state = seed == 0 ? 1 : seed
    }

    mutating func nextUInt64() -> UInt64 {
        state ^= state << 13
        state ^= state >> 7
        state ^= state << 17
        return state
    }

    mutating func next(bound: Int) -> Int {
        guard bound > 0 else { return 0 }
        return Int(nextUInt64() % UInt64(bound))
    }
}
