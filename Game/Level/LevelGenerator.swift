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
        .init(redChannel: 0.95, greenChannel: 0.70, blueChannel: 0.40),  // orange
        .init(redChannel: 0.85, greenChannel: 0.45, blueChannel: 0.85),  // magenta
        .init(redChannel: 0.45, greenChannel: 0.90, blueChannel: 0.85),  // teal
        .init(redChannel: 0.90, greenChannel: 0.80, blueChannel: 0.60),  // sand
        .init(redChannel: 0.65, greenChannel: 0.75, blueChannel: 0.95),  // periwinkle
    ]

    // MARK: - Difficulty

    private struct Difficulty {
        let ropeCount: Int
        let totalDrags: Int  // total number of individual drag operations
    }

    private static func difficulty(for levelId: Int) -> Difficulty {
        switch levelId {
        case 1...2:   return Difficulty(ropeCount: 3, totalDrags: 3)
        case 3...5:   return Difficulty(ropeCount: 4, totalDrags: 4)
        case 6...9:   return Difficulty(ropeCount: 5, totalDrags: 6)
        case 10...15: return Difficulty(ropeCount: 6, totalDrags: 8)
        case 16...25: return Difficulty(ropeCount: 7, totalDrags: 10)
        case 26...50: return Difficulty(ropeCount: 8, totalDrags: 12 + (levelId - 25) / 5)
        default:      return Difficulty(ropeCount: min(10, 8 + (levelId - 50) / 25), totalDrags: min(22, 16 + (levelId - 50) / 10))
        }
    }

    // MARK: - Hole layouts (deterministic, not random)

    private enum HoleLayout: CaseIterable {
        case grid4x5
        case grid5x6
        case circle12
        case circle16
        case hexagon
        case hexagonLarge
        case diamond
        case diamondWide
        case cross
        case crossLarge
        case twoRings
        case threeRings
        case triangle
        case triangleLarge
        case star
        case starLarge
        case honeycomb
        case spiral
        case doubleGrid
        case scattered

        func generate() -> [LevelDefinition.Vec2] {
            switch self {
            case .grid4x5:       return grid(cols: 5, rows: 4, spacingX: 0.42, spacingY: 0.45)
            case .grid5x6:       return grid(cols: 6, rows: 5, spacingX: 0.38, spacingY: 0.40)
            case .circle12:      return circle(count: 12, radius: 0.72)
            case .circle16:      return circle(count: 16, radius: 0.85)
            case .hexagon:       return hexagonLayout()
            case .hexagonLarge:  return hexagonLargeLayout()
            case .diamond:       return diamondLayout()
            case .diamondWide:   return diamondWideLayout()
            case .cross:         return crossLayout()
            case .crossLarge:    return crossLargeLayout()
            case .twoRings:      return twoRingsLayout()
            case .threeRings:    return threeRingsLayout()
            case .triangle:      return triangleLayout()
            case .triangleLarge: return triangleLargeLayout()
            case .star:          return starLayout()
            case .starLarge:     return starLargeLayout()
            case .honeycomb:     return honeycombLayout()
            case .spiral:        return spiralLayout()
            case .doubleGrid:    return doubleGridLayout()
            case .scattered:     return scatteredLayout()
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
            return pts  // 19
        }

        private func hexagonLargeLayout() -> [LevelDefinition.Vec2] {
            var pts: [LevelDefinition.Vec2] = [.init(xPosition: 0, yPosition: 0)]
            for ring in 1...3 {
                let r = Float(ring) * 0.35
                let count = ring * 6
                for i in 0..<count {
                    let a = Float(i) / Float(count) * 2 * .pi + Float(ring) * 0.15
                    pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
                }
            }
            return pts  // 1+6+12+18 = 37
        }

        private func diamondLayout() -> [LevelDefinition.Vec2] {
            let s: Float = 0.38
            var pts: [LevelDefinition.Vec2] = []
            let rows: [Int] = [1, 3, 5, 3, 1]
            for (rowIdx, count) in rows.enumerated() {
                let y = (Float(rowIdx) - 2) * s
                let ox = -Float(count - 1) / 2 * s
                for col in 0..<count {
                    pts.append(.init(xPosition: ox + Float(col) * s, yPosition: y))
                }
            }
            return pts  // 13
        }

        private func diamondWideLayout() -> [LevelDefinition.Vec2] {
            let s: Float = 0.34
            var pts: [LevelDefinition.Vec2] = []
            let rows: [Int] = [2, 4, 6, 8, 6, 4, 2]
            for (rowIdx, count) in rows.enumerated() {
                let y = (Float(rowIdx) - 3) * s
                let ox = -Float(count - 1) / 2 * s
                for col in 0..<count {
                    pts.append(.init(xPosition: ox + Float(col) * s, yPosition: y))
                }
            }
            return pts  // 32
        }

        private func crossLayout() -> [LevelDefinition.Vec2] {
            let s: Float = 0.40
            var pts: [LevelDefinition.Vec2] = []
            for i in -2...2 {
                pts.append(.init(xPosition: 0, yPosition: Float(i) * s))
            }
            for i in [-2, -1, 1, 2] {
                pts.append(.init(xPosition: Float(i) * s, yPosition: 0))
            }
            for dx: Float in [-1, 1] {
                for dy: Float in [-1, 1] {
                    pts.append(.init(xPosition: dx * s, yPosition: dy * s))
                }
            }
            return pts  // 13
        }

        private func crossLargeLayout() -> [LevelDefinition.Vec2] {
            let s: Float = 0.32
            var pts: [LevelDefinition.Vec2] = []
            for i in -3...3 {
                pts.append(.init(xPosition: 0, yPosition: Float(i) * s))
            }
            for i in [-3, -2, -1, 1, 2, 3] {
                pts.append(.init(xPosition: Float(i) * s, yPosition: 0))
            }
            for dx: Float in [-2, -1, 1, 2] {
                for dy: Float in [-1, 1] {
                    pts.append(.init(xPosition: dx * s, yPosition: dy * s))
                }
            }
            for dx: Float in [-1, 1] {
                for dy: Float in [-2, 2] {
                    pts.append(.init(xPosition: dx * s, yPosition: dy * s))
                }
            }
            return pts  // 25
        }

        private func twoRingsLayout() -> [LevelDefinition.Vec2] {
            var pts: [LevelDefinition.Vec2] = []
            for i in 0..<6 {
                let a = Float(i) / 6 * 2 * .pi
                pts.append(.init(xPosition: 0.35 * cos(a), yPosition: 0.35 * sin(a)))
            }
            for i in 0..<10 {
                let a = Float(i) / 10 * 2 * .pi + .pi / 10
                pts.append(.init(xPosition: 0.78 * cos(a), yPosition: 0.78 * sin(a)))
            }
            return pts  // 16
        }

        private func threeRingsLayout() -> [LevelDefinition.Vec2] {
            var pts: [LevelDefinition.Vec2] = [.init(xPosition: 0, yPosition: 0)]
            for i in 0..<5 {
                let a = Float(i) / 5 * 2 * .pi
                pts.append(.init(xPosition: 0.30 * cos(a), yPosition: 0.30 * sin(a)))
            }
            for i in 0..<9 {
                let a = Float(i) / 9 * 2 * .pi + .pi / 9
                pts.append(.init(xPosition: 0.60 * cos(a), yPosition: 0.60 * sin(a)))
            }
            for i in 0..<13 {
                let a = Float(i) / 13 * 2 * .pi
                pts.append(.init(xPosition: 0.92 * cos(a), yPosition: 0.92 * sin(a)))
            }
            return pts  // 28
        }

        private func triangleLayout() -> [LevelDefinition.Vec2] {
            let s: Float = 0.38
            var pts: [LevelDefinition.Vec2] = []
            for row in 0..<5 {
                let count = row + 1
                let ox = -Float(count - 1) / 2 * s
                let y = (Float(row) - 2) * s * 0.866
                for col in 0..<count {
                    pts.append(.init(xPosition: ox + Float(col) * s, yPosition: y))
                }
            }
            return pts  // 15
        }

        private func triangleLargeLayout() -> [LevelDefinition.Vec2] {
            let s: Float = 0.30
            var pts: [LevelDefinition.Vec2] = []
            for row in 0..<7 {
                let count = row + 1
                let ox = -Float(count - 1) / 2 * s
                let y = (Float(row) - 3) * s * 0.866
                for col in 0..<count {
                    pts.append(.init(xPosition: ox + Float(col) * s, yPosition: y))
                }
            }
            return pts  // 28
        }

        private func starLayout() -> [LevelDefinition.Vec2] {
            var pts: [LevelDefinition.Vec2] = [.init(xPosition: 0, yPosition: 0)]
            for i in 0..<5 {
                let outerAngle = Float(i) / 5 * 2 * .pi - .pi / 2
                pts.append(.init(xPosition: 0.82 * cos(outerAngle), yPosition: 0.82 * sin(outerAngle)))
                let innerAngle = outerAngle + .pi / 5
                pts.append(.init(xPosition: 0.38 * cos(innerAngle), yPosition: 0.38 * sin(innerAngle)))
            }
            for i in 0..<5 {
                let a = Float(i) / 5 * 2 * .pi - .pi / 2 + .pi / 5
                pts.append(.init(xPosition: 0.60 * cos(a), yPosition: 0.60 * sin(a)))
            }
            return pts  // 16
        }

        private func starLargeLayout() -> [LevelDefinition.Vec2] {
            var pts: [LevelDefinition.Vec2] = [.init(xPosition: 0, yPosition: 0)]
            for i in 0..<6 {
                let outerAngle = Float(i) / 6 * 2 * .pi - .pi / 2
                pts.append(.init(xPosition: 0.95 * cos(outerAngle), yPosition: 0.95 * sin(outerAngle)))
                let innerAngle = outerAngle + .pi / 6
                pts.append(.init(xPosition: 0.42 * cos(innerAngle), yPosition: 0.42 * sin(innerAngle)))
            }
            for i in 0..<6 {
                let a = Float(i) / 6 * 2 * .pi - .pi / 2 + .pi / 6
                pts.append(.init(xPosition: 0.68 * cos(a), yPosition: 0.68 * sin(a)))
            }
            for i in 0..<6 {
                let a = Float(i) / 6 * 2 * .pi - .pi / 2
                pts.append(.init(xPosition: 0.68 * cos(a), yPosition: 0.68 * sin(a)))
            }
            return pts  // 25
        }

        private func honeycombLayout() -> [LevelDefinition.Vec2] {
            let s: Float = 0.36
            let h = s * 0.866
            var pts: [LevelDefinition.Vec2] = []
            for row in -2...2 {
                let cols = (row % 2 == 0) ? 5 : 4
                let offset: Float = (row % 2 == 0) ? 0 : s * 0.5
                let ox = -Float(cols - 1) / 2 * s + offset
                for col in 0..<cols {
                    pts.append(.init(xPosition: ox + Float(col) * s,
                                     yPosition: Float(row) * h))
                }
            }
            return pts  // 22
        }

        private func spiralLayout() -> [LevelDefinition.Vec2] {
            var pts: [LevelDefinition.Vec2] = [.init(xPosition: 0, yPosition: 0)]
            let count = 20
            for i in 1...count {
                let t = Float(i) / Float(count)
                let r = 0.15 + t * 0.75
                let a = t * 4.5 * .pi
                pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
            }
            return pts  // 21
        }

        private func doubleGridLayout() -> [LevelDefinition.Vec2] {
            var pts: [LevelDefinition.Vec2] = []
            let s: Float = 0.38
            let gap: Float = 0.22
            for gy: Float in [-1, 1] {
                let cy = gy * (s * 1.5 + gap)
                for row in 0..<3 {
                    for col in 0..<4 {
                        let x = -Float(3) / 2 * s + Float(col) * s
                        let y = cy + (Float(row) - 1) * s
                        pts.append(.init(xPosition: x, yPosition: y))
                    }
                }
            }
            return pts  // 24
        }

        private func scatteredLayout() -> [LevelDefinition.Vec2] {
            var pts: [LevelDefinition.Vec2] = []
            let positions: [(Float, Float)] = [
                (-0.75, -0.70), (-0.30, -0.80), (0.20, -0.75), (0.70, -0.65),
                (-0.85, -0.25), (-0.40, -0.30), (0.05, -0.35), (0.50, -0.20), (0.88, -0.30),
                (-0.70, 0.15), (-0.25, 0.10), (0.25, 0.18), (0.72, 0.10),
                (-0.80, 0.60), (-0.35, 0.55), (0.10, 0.65), (0.55, 0.58), (0.85, 0.55),
                (-0.50, 0.90), (0.00, 0.92), (0.50, 0.88),
            ]
            for (x, y) in positions {
                pts.append(.init(xPosition: x, yPosition: y))
            }
            return pts  // 21
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
        let maxRopes = max(1, (holes.count - 3) / 2)
        let ropeCount = min(diff.ropeCount, maxRopes)

        // Build structured rope pairs: connect roughly-opposite holes through center
        // This creates star/cross patterns instead of random spaghetti
        let ropePairs = pickStructuredPairs(
            holes: holes, count: ropeCount, rng: &rng
        )
        var usedHoles = Set(ropePairs.flatMap { [$0.0, $0.1] })

        let ropes = ropePairs.enumerated().map { i, pair -> LevelDefinition.Rope in
            // TODO: прямоугольный профиль ведет себя неадекватно
            let useRect = false
            let csDef: LevelDefinition.CrossSectionDef? = useRect
                ? LevelDefinition.CrossSectionDef(type: "rectangular", width: 0.070, height: 0.025)
                : nil
            return LevelDefinition.Rope(
                startHole: pair.0,
                endHole: pair.1,
                color: colors[i % colors.count],
                radius: useRect ? 0.035 : 0.038,
                crossSectionDef: csDef
            )
        }

        // Generate actions: pin all, then drag to create tangles
        var actions: [LevelDefinition.Action] = []

        for i in 0..<ropes.count {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: ropes[i].startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: ropes[i].endHole))
        }

        var currentEndpoints = ropes.map { ($0.startHole, $0.endHole) }
        let holeSimd = holes.map { $0.simd }
        var tangleSet = Set<Int>()

        func tryDrag(ropeIdx: Int, endIdx: Int, targetRopeIdx: Int) -> Bool {
            let currentHole = endIdx == 0 ? currentEndpoints[ropeIdx].0 : currentEndpoints[ropeIdx].1
            let anchorHole = endIdx == 0 ? currentEndpoints[ropeIdx].1 : currentEndpoints[ropeIdx].0
            let anchorPos = holeSimd[anchorHole]

            let tS = holeSimd[currentEndpoints[targetRopeIdx].0]
            let tE = holeSimd[currentEndpoints[targetRopeIdx].1]

            let otherUsed = Set(
                (0..<ropes.count).filter { $0 != ropeIdx }.flatMap {
                    [currentEndpoints[$0].0, currentEndpoints[$0].1]
                }
            )

            var bestHole: Int?
            var bestScore: Float = -.greatestFiniteMagnitude

            for candidate in 0..<holes.count {
                if candidate == currentHole || candidate == anchorHole { continue }
                if otherUsed.contains(candidate) { continue }
                let cPos = holeSimd[candidate]
                if segmentsCross(anchorPos, cPos, tS, tE) {
                    let dist = simd_length(cPos - (tS + tE) * 0.5)
                    let score = -dist
                    if score > bestScore {
                        bestScore = score
                        bestHole = candidate
                    }
                }
            }

            guard let targetHole = bestHole else { return false }

            usedHoles.remove(currentHole)
            usedHoles.insert(targetHole)
            if endIdx == 0 {
                currentEndpoints[ropeIdx].0 = targetHole
            } else {
                currentEndpoints[ropeIdx].1 = targetHole
            }
            actions.append(.init(type: "drag", ropeIndex: ropeIdx, endIndex: endIdx, holeIndex: targetHole))
            tangleSet.insert(ropeIdx)
            tangleSet.insert(targetRopeIdx)
            return true
        }

        for d in 0..<diff.totalDrags {
            let ropeIdx = d % ropes.count
            let targetRopeIdx = (ropeIdx + 1 + d / ropes.count) % ropes.count
            if targetRopeIdx == ropeIdx { continue }
            let endIdx = d / ropes.count % 2
            if !tryDrag(ropeIdx: ropeIdx, endIdx: endIdx, targetRopeIdx: targetRopeIdx) {
                let _ = tryDrag(ropeIdx: ropeIdx, endIdx: 1 - endIdx, targetRopeIdx: targetRopeIdx)
            }
        }

        for attempt in 0..<ropes.count * 4 {
            let crossings = ropeCrossings(endpoints: currentEndpoints, holes: holeSimd)
            let isolated = (0..<ropes.count).filter { crossings[$0] == 0 }
            if isolated.isEmpty { break }

            let ropeIdx = isolated[attempt % isolated.count]

            var fixed = false
            for other in 0..<ropes.count where other != ropeIdx && !fixed {
                for endIdx in 0...1 where !fixed {
                    fixed = tryDrag(ropeIdx: ropeIdx, endIdx: endIdx, targetRopeIdx: other)
                }
            }
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

    private static func ropeCrossings(endpoints: [(Int, Int)], holes: [SIMD2<Float>]) -> [Int] {
        let n = endpoints.count
        var counts = Array(repeating: 0, count: n)
        for i in 0..<n {
            let a0 = holes[endpoints[i].0]
            let a1 = holes[endpoints[i].1]
            for j in (i+1)..<n {
                let b0 = holes[endpoints[j].0]
                let b1 = holes[endpoints[j].1]
                if segmentsCross(a0, a1, b0, b1) {
                    counts[i] += 1
                    counts[j] += 1
                }
            }
        }
        return counts
    }

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
