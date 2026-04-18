import Foundation
import simd

enum LevelGenerator {

    // MARK: - Public

    /// Whether this level uses braid-style structured generation
    static func isBraidUntangleLevel(_ levelId: Int) -> Bool {
        guard levelId >= 8 else { return false }
        // Every 4th level starting from 8: 8, 12, 16, 20, 24, ...
        return levelId % 4 == 0
    }

    static func generate(levelId: Int, boardElevation: Float = 0.12, particleCount: Int = 55) -> LevelDefinition {
        // Braid-style levels: structured crossing patterns
        if isBraidUntangleLevel(levelId) {
            return generateBraidUntangle(levelId: levelId, particleCount: particleCount)
        }

        var rng = SeededRNG(seed: UInt64(levelId) &* 2654435761)

        let diff = difficulty(for: levelId)
        let use3D = levelId >= 10 && levelId % 3 == 0
        let n = minHoles(for: levelId, ropeCount: diff.ropeCount)

        let holes: [LevelDefinition.Vec2]
        let boards: [LevelDefinition.Board]?

        if use3D {
            let bl = pickBoardLayout(for: levelId)
            let result = generateBoardLayout(bl, n: n, elevation: boardElevation)
            holes = result.holes
            boards = result.boards
        } else {
            let layout = pickLayout(for: levelId)
            holes = layout.generate(n: n)
            boards = nil
        }

        let maxRopes = max(1, (holes.count - 3) / 2)
        let ropeCount = min(diff.ropeCount, maxRopes)
        let shortCount = min(diff.shortRopeCount, max(0, ropeCount - 1))
        let ropePairs = pickStructuredPairs(holes: holes, count: ropeCount, shortCount: shortCount, rng: &rng)
        let ropes = ropePairs.enumerated().map { i, pair in
            LevelDefinition.Rope(startHole: pair.0, endHole: pair.1,
                                 color: colors[i % colors.count], radius: 0.038)
        }
        let actions = buildActions(ropes: ropes, holes: holes,
                                   totalDrags: diff.totalDrags, shortCount: shortCount,
                                   swapPercent: diff.swapPercent)

        return LevelDefinition(
            mode: nil,
            id: levelId,
            holeRadius: 0.08,
            particlesPerRope: particleCount,
            holes: holes,
            ropes: ropes,
            hooks: nil,
            actions: actions,
            boards: boards,
            weights: nil,
            targets: nil,
            rails: nil,
            carts: nil,
            stations: nil
        )
    }

    // MARK: - Difficulty

    struct Difficulty {
        let ropeCount: Int
        let totalDrags: Int
        let shortRopeCount: Int
        let swapPercent: Int
    }

    /// Cyclic swap percent: repeats every 15 levels
    private static let swapCycle: [Int] = [0, 0, 0, 15, 25, 35, 50, 60, 75, 85, 100, 100, 50, 25, 0]

    static func difficulty(for levelId: Int) -> Difficulty {
        let swap = swapCycle[levelId % swapCycle.count]
        switch levelId {
        case 1...2:   return Difficulty(ropeCount: 3, totalDrags: 3, shortRopeCount: 0, swapPercent: swap)
        case 3...5:   return Difficulty(ropeCount: 4, totalDrags: 4, shortRopeCount: levelId >= 4 ? 2 : 0, swapPercent: swap)
        case 6...9:   return Difficulty(ropeCount: 5, totalDrags: 6, shortRopeCount: 3, swapPercent: swap)
        case 10...15: return Difficulty(ropeCount: 6, totalDrags: 8, shortRopeCount: 4, swapPercent: swap)
        case 16...25: return Difficulty(ropeCount: 7, totalDrags: 10, shortRopeCount: 5, swapPercent: swap)
        case 26...50: return Difficulty(ropeCount: 8, totalDrags: 12 + (levelId - 25) / 3, shortRopeCount: 6, swapPercent: swap)
        case 51...100: return Difficulty(ropeCount: min(12, 9 + (levelId - 50) / 15),
                                         totalDrags: min(35, 20 + (levelId - 50) / 5),
                                         shortRopeCount: min(8, 6 + (levelId - 50) / 20),
                                         swapPercent: swap)
        case 101...200: return Difficulty(ropeCount: min(14, 12 + (levelId - 100) / 30),
                                          totalDrags: min(50, 35 + (levelId - 100) / 5),
                                          shortRopeCount: min(10, 8 + (levelId - 100) / 25),
                                          swapPercent: swap)
        default:       return Difficulty(ropeCount: min(16, 14 + (levelId - 200) / 50),
                                         totalDrags: min(70, 50 + (levelId - 200) / 5),
                                         shortRopeCount: min(12, 10 + (levelId - 200) / 40),
                                         swapPercent: swap)
        }
    }

    static func minHoles(for levelId: Int, ropeCount: Int) -> Int {
        let base = ropeCount * 3 + 6
        switch levelId {
        case 1...5:    return max(base, 12)
        case 6...15:   return max(base, 16)
        case 16...30:  return max(base, 22)
        case 31...60:  return max(base, 28)
        case 61...100: return max(base, 36)
        case 101...200: return max(base, 48)
        case 201...500: return max(base, 64)
        default:       return max(base, 80)
        }
    }

    // MARK: - Layout selection

    static func pickLayout(for levelId: Int) -> HoleLayout {
        if levelId > 60 {
            let scalableLayouts: [HoleLayout] = [
                .circle, .rings, .spiral, .honeycomb, .star,
                .clusters, .clover, .ringWithSpokes, .hexagon,
                .centerPlusRing, .triangleInCircle, .grid,
            ]
            return scalableLayouts[levelId % scalableLayouts.count]
        }
        let layouts = HoleLayout.allCases
        return layouts[levelId % layouts.count]
    }

    // MARK: - Colors

    static let colors: [LevelDefinition.Color] = [
        .init(redChannel: 0.95, greenChannel: 0.30, blueChannel: 0.05),
        .init(redChannel: 0.10, greenChannel: 0.35, blueChannel: 0.92),
        .init(redChannel: 0.90, greenChannel: 0.12, blueChannel: 0.25),
        .init(redChannel: 0.15, greenChannel: 0.75, blueChannel: 0.30),
        .init(redChannel: 0.92, greenChannel: 0.78, blueChannel: 0.05),
        .init(redChannel: 0.60, greenChannel: 0.10, blueChannel: 0.72),
        .init(redChannel: 0.05, greenChannel: 0.65, blueChannel: 0.72),
        .init(redChannel: 0.85, greenChannel: 0.15, blueChannel: 0.55),
        .init(redChannel: 0.20, greenChannel: 0.55, blueChannel: 0.90),
        .init(redChannel: 0.80, greenChannel: 0.50, blueChannel: 0.05),
    ]

    // MARK: - Rope pair selection

    static func pickStructuredPairs(
        holes: [LevelDefinition.Vec2], count: Int, shortCount: Int, rng: inout SeededRNG
    ) -> [(Int, Int)] {
        guard holes.count >= 4 else { return [] }

        let cx = holes.map { $0.simd.x }.reduce(0, +) / Float(holes.count)
        let cy = holes.map { $0.simd.y }.reduce(0, +) / Float(holes.count)

        let sorted = holes.enumerated()
            .map { (idx: $0.offset, angle: atan2($0.element.simd.y - cy, $0.element.simd.x - cx)) }
            .sorted { $0.angle < $1.angle }

        let offset = rng.next(bound: sorted.count)
        let n = sorted.count
        let half = n / 2
        let quarter = max(1, n / 4)

        var pairs: [(Int, Int)] = []
        var usedHoles = Set<Int>()

        if shortCount > 0 {
            let sectorStart = offset % n
            let sectorLen = min(quarter + shortCount, n / 2)

            var sectorHoles: [Int] = []
            for k in 0..<sectorLen {
                sectorHoles.append(sorted[(sectorStart + k) % n].idx)
            }

            let minShortDist: Float = 0.3
            for i in 0..<sectorHoles.count {
                guard pairs.count < shortCount else { break }
                let a = sectorHoles[i]
                if usedHoles.contains(a) { continue }
                let shift = max(2, min(quarter / 2, sectorHoles.count / 3))
                for delta in stride(from: shift, to: sectorHoles.count, by: 1) {
                    let b = sectorHoles[(i + delta) % sectorHoles.count]
                    if b == a || usedHoles.contains(b) { continue }
                    if simd_length(holes[a].simd - holes[b].simd) < minShortDist { continue }
                    pairs.append((a, b))
                    usedHoles.insert(a)
                    usedHoles.insert(b)
                    break
                }
            }
        }

        for i in 0..<count {
            guard pairs.count < count else { break }
            let aIdx = (i * 2 + offset) % n
            let a = sorted[aIdx].idx
            if usedHoles.contains(a) { continue }

            let shift = (i % 2 == 0) ? 1 : -1
            let bIdx = (aIdx + half + shift + n) % n
            let b = sorted[bIdx].idx

            if b != a && !usedHoles.contains(b) {
                pairs.append((a, b))
                usedHoles.insert(a)
                usedHoles.insert(b)
            }
        }

        if pairs.count < count {
            for i in 0..<n {
                guard pairs.count < count else { break }
                let a = sorted[(i + offset) % n].idx
                if usedHoles.contains(a) { continue }
                for j in (i+1)..<n {
                    let b = sorted[(j + offset) % n].idx
                    if usedHoles.contains(b) { continue }
                    if simd_length(holes[a].simd - holes[b].simd) > 0.5 {
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
}
