import Foundation
import simd

enum LevelGenerator {

    // MARK: - Public

    static func generate(levelId: Int, boardElevation: Float = 0.12) -> LevelDefinition {
        var rng = SeededRNG(seed: UInt64(levelId) &* 2654435761)

        let diff = difficulty(for: levelId)
        let use3D = levelId >= 10 && levelId % 3 == 0
        let boardLayout: BoardLayout? = use3D ? pickBoardLayout(for: levelId) : nil

        let holes: [LevelDefinition.Vec2]
        let boards: [LevelDefinition.Board]?

        if let bl = boardLayout {
            let result = generateBoardLayout(bl, n: minHoles(for: levelId, ropeCount: diff.ropeCount), elevation: boardElevation)
            holes = result.holes
            boards = result.boards
        } else {
            let layout = pickLayout(for: levelId)
            holes = layout.generate(n: minHoles(for: levelId, ropeCount: diff.ropeCount))
            boards = nil
        }

        let maxRopes = max(1, (holes.count - 3) / 2)
        let ropeCount = min(diff.ropeCount, maxRopes)

        let ropePairs = pickStructuredPairs(holes: holes, count: ropeCount, rng: &rng)
        let ropes = ropePairs.enumerated().map { i, pair in
            LevelDefinition.Rope(
                startHole: pair.0,
                endHole: pair.1,
                color: colors[i % colors.count],
                radius: 0.038
            )
        }

        let actions = buildActions(ropes: ropes, holes: holes, totalDrags: diff.totalDrags)

        return LevelDefinition(
            id: levelId,
            holeRadius: 0.08,
            particlesPerRope: 32,
            holes: holes,
            ropes: ropes,
            hooks: nil,
            actions: actions,
            boards: boards
        )
    }

    // MARK: - 3D Board layouts

    enum BoardLayout: CaseIterable {
        case twoSides
        case bridge
        case staircase
        case platform
        case valley
    }

    struct BoardLayoutResult {
        let holes: [LevelDefinition.Vec2]
        let boards: [LevelDefinition.Board]
    }

    private static func pickBoardLayout(for levelId: Int) -> BoardLayout {
        let layouts = BoardLayout.allCases
        return layouts[(levelId / 3) % layouts.count]
    }

    // MARK: - Difficulty

    private struct Difficulty {
        let ropeCount: Int
        let totalDrags: Int
    }

    private static func difficulty(for levelId: Int) -> Difficulty {
        switch levelId {
        case 1...2:   return Difficulty(ropeCount: 3, totalDrags: 3)
        case 3...5:   return Difficulty(ropeCount: 4, totalDrags: 4)
        case 6...9:   return Difficulty(ropeCount: 5, totalDrags: 6)
        case 10...15: return Difficulty(ropeCount: 6, totalDrags: 8)
        case 16...25: return Difficulty(ropeCount: 7, totalDrags: 10)
        case 26...50: return Difficulty(ropeCount: 8, totalDrags: 12 + (levelId - 25) / 5)
        default:      return Difficulty(ropeCount: min(10, 8 + (levelId - 50) / 25),
                                        totalDrags: min(22, 16 + (levelId - 50) / 10))
        }
    }

    private static func minHoles(for levelId: Int, ropeCount: Int) -> Int {
        let base = ropeCount * 2 + 4
        switch levelId {
        case 1...5:    return max(base, 10)
        case 6...15:   return max(base, 14)
        case 16...30:  return max(base, 18)
        case 31...60:  return max(base, 20)
        case 61...100: return max(base, 22)
        default:       return max(base, 24)
        }
    }

    // MARK: - Layout selection

    private static func pickLayout(for levelId: Int) -> HoleLayout {
        let layouts = HoleLayout.allCases
        return layouts[levelId % layouts.count]
    }

    // MARK: - Colors

    private static let colors: [LevelDefinition.Color] = [
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

    private static func pickStructuredPairs(
        holes: [LevelDefinition.Vec2], count: Int, rng: inout SeededRNG
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

        var pairs: [(Int, Int)] = []
        var usedHoles = Set<Int>()

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

    // MARK: - Tangle generation

    private static func buildActions(
        ropes: [LevelDefinition.Rope],
        holes: [LevelDefinition.Vec2],
        totalDrags: Int
    ) -> [LevelDefinition.Action] {
        var actions: [LevelDefinition.Action] = []
        let holeSimd = holes.map { $0.simd }

        for i in 0..<ropes.count {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: ropes[i].startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: ropes[i].endHole))
        }

        var endpoints = ropes.map { ($0.startHole, $0.endHole) }
        var usedHoles = Set(endpoints.flatMap { [$0.0, $0.1] })

        @discardableResult
        func tryDrag(ropeIdx: Int, endIdx: Int, targetRopeIdx: Int) -> Bool {
            let currentHole = endIdx == 0 ? endpoints[ropeIdx].0 : endpoints[ropeIdx].1
            let anchorHole = endIdx == 0 ? endpoints[ropeIdx].1 : endpoints[ropeIdx].0
            let anchorPos = holeSimd[anchorHole]
            let tS = holeSimd[endpoints[targetRopeIdx].0]
            let tE = holeSimd[endpoints[targetRopeIdx].1]

            let otherUsed = Set(
                (0..<ropes.count).filter { $0 != ropeIdx }
                    .flatMap { [endpoints[$0].0, endpoints[$0].1] }
            )

            var bestHole: Int?
            var bestScore: Float = -.greatestFiniteMagnitude

            for candidate in 0..<holes.count {
                if candidate == currentHole || candidate == anchorHole { continue }
                if otherUsed.contains(candidate) { continue }
                let cPos = holeSimd[candidate]
                if segmentsCross(anchorPos, cPos, tS, tE) {
                    let score = -simd_length(cPos - (tS + tE) * 0.5)
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
                endpoints[ropeIdx].0 = targetHole
            } else {
                endpoints[ropeIdx].1 = targetHole
            }
            actions.append(.init(type: "drag", ropeIndex: ropeIdx, endIndex: endIdx, holeIndex: targetHole))
            return true
        }

        for d in 0..<totalDrags {
            let ropeIdx = d % ropes.count
            let targetRopeIdx = (ropeIdx + 1 + d / ropes.count) % ropes.count
            if targetRopeIdx == ropeIdx { continue }
            let endIdx = d / ropes.count % 2
            if !tryDrag(ropeIdx: ropeIdx, endIdx: endIdx, targetRopeIdx: targetRopeIdx) {
                tryDrag(ropeIdx: ropeIdx, endIdx: 1 - endIdx, targetRopeIdx: targetRopeIdx)
            }
        }

        for attempt in 0..<ropes.count * 4 {
            let crossings = ropeCrossings(endpoints: endpoints, holes: holeSimd)
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

        return actions
    }

    // MARK: - Geometry helpers

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

    // MARK: - 3D Board layout implementations

    static func generateBoardLayout(_ layout: BoardLayout, n: Int, elevation: Float = 0.12) -> BoardLayoutResult {
        switch layout {
        case .twoSides:     return twoSidesLayout(n: n, elevation: elevation)
        case .bridge:       return bridgeLayout(n: n, elevation: elevation)
        case .staircase:    return staircaseLayout(n: n, elevation: elevation)
        case .platform:     return platformLayout(n: n, elevation: elevation)
        case .valley:       return valleyLayout(n: n, elevation: elevation)
        }
    }

    private static func twoSidesLayout(n: Int, elevation: Float) -> BoardLayoutResult {
        let boardW: Float = 0.55
        let boardH: Float = 1.4
        let gap: Float = 0.35

        let leftBoard = LevelDefinition.Board(centerX: -(gap + boardW * 0.5), centerY: 0, width: boardW, height: boardH, elevation: elevation)
        let rightBoard = LevelDefinition.Board(centerX: gap + boardW * 0.5, centerY: 0, width: boardW, height: boardH, elevation: elevation)

        let centerHoles = max(4, n / 3)
        let sideHoles = max(3, (n - centerHoles) / 2)
        let s: Float = 0.35

        var holes: [LevelDefinition.Vec2] = []

        let centerCols = max(2, Int(ceil(sqrt(Float(centerHoles) * 1.5))))
        let centerRows = max(2, (centerHoles + centerCols - 1) / centerCols)
        let cs = min(s, gap * 1.5 / Float(centerCols))
        for row in 0..<centerRows {
            for col in 0..<centerCols {
                let x = (Float(col) - Float(centerCols - 1) / 2) * cs
                let y = (Float(row) - Float(centerRows - 1) / 2) * cs
                holes.append(.init(xPosition: x, yPosition: y))
            }
        }

        let sideRows = max(2, sideHoles)
        let ss = min(s, boardH * 0.8 / Float(sideRows - 1))
        for i in 0..<sideRows {
            let y = (Float(i) - Float(sideRows - 1) / 2) * ss
            holes.append(.init(xPosition: leftBoard.centerX, yPosition: y, zPosition: elevation))
        }
        for i in 0..<sideRows {
            let y = (Float(i) - Float(sideRows - 1) / 2) * ss
            holes.append(.init(xPosition: rightBoard.centerX, yPosition: y, zPosition: elevation))
        }

        return BoardLayoutResult(holes: holes, boards: [leftBoard, rightBoard])
    }

    private static func bridgeLayout(n: Int, elevation: Float) -> BoardLayoutResult {
        let boardW: Float = 1.8
        let boardH: Float = 0.45

        let bridge = LevelDefinition.Board(centerX: 0, centerY: 0, width: boardW, height: boardH, elevation: elevation)

        let topHoles = max(3, n / 3)
        let bottomHoles = max(3, n / 3)
        let bridgeHoles = max(3, n - topHoles - bottomHoles)
        let s: Float = 0.35

        var holes: [LevelDefinition.Vec2] = []

        let cols = max(2, Int(ceil(sqrt(Float(topHoles) * 2))))
        let rows = max(2, (topHoles + cols - 1) / cols)
        let ts = min(s, 1.4 / Float(cols))
        for row in 0..<rows {
            for col in 0..<cols {
                let x = (Float(col) - Float(cols - 1) / 2) * ts
                let y = boardH * 0.5 + 0.25 + Float(row) * ts
                holes.append(.init(xPosition: x, yPosition: y))
            }
        }

        for row in 0..<rows {
            for col in 0..<cols {
                guard holes.count < topHoles + bottomHoles else { break }
                let x = (Float(col) - Float(cols - 1) / 2) * ts
                let y = -(boardH * 0.5 + 0.25 + Float(row) * ts)
                holes.append(.init(xPosition: x, yPosition: y))
            }
        }

        let bCols = max(2, bridgeHoles)
        let bs = min(s, boardW * 0.7 / Float(bCols - 1))
        for i in 0..<bCols {
            let x = (Float(i) - Float(bCols - 1) / 2) * bs
            holes.append(.init(xPosition: x, yPosition: 0, zPosition: elevation))
        }

        return BoardLayoutResult(holes: holes, boards: [bridge])
    }

    private static func staircaseLayout(n: Int, elevation: Float) -> BoardLayoutResult {
        let steps = 3
        let stepW: Float = 0.55
        let stepH: Float = 1.2
        let stepGap: Float = 0.08

        var boards: [LevelDefinition.Board] = []
        var holes: [LevelDefinition.Vec2] = []
        let holesPerStep = max(3, n / (steps + 1))
        let s: Float = 0.32

        let floorHoles = max(3, n - holesPerStep * steps)
        let floorX: Float = -(Float(steps) * (stepW + stepGap)) * 0.5 - 0.3
        let floorRows = max(2, floorHoles)
        let fs = min(s, stepH * 0.8 / Float(floorRows - 1))
        for i in 0..<floorRows {
            let y = (Float(i) - Float(floorRows - 1) / 2) * fs
            holes.append(.init(xPosition: floorX, yPosition: y))
        }

        for step in 0..<steps {
            let elev: Float = Float(step + 1) * elevation
            let cx = Float(step) * (stepW + stepGap) - Float(steps - 1) * (stepW + stepGap) * 0.5 + stepW * 0.5
            let board = LevelDefinition.Board(centerX: cx, centerY: 0, width: stepW, height: stepH, elevation: elev)
            boards.append(board)

            let stepRows = max(2, holesPerStep)
            let ss = min(s, stepH * 0.7 / Float(stepRows - 1))
            for i in 0..<stepRows {
                let y = (Float(i) - Float(stepRows - 1) / 2) * ss
                holes.append(.init(xPosition: cx, yPosition: y, zPosition: elev))
            }
        }

        return BoardLayoutResult(holes: holes, boards: boards)
    }

    private static func platformLayout(n: Int, elevation: Float) -> BoardLayoutResult {
        let boardW: Float = 0.8
        let boardH: Float = 0.8

        let platform = LevelDefinition.Board(centerX: 0, centerY: 0, width: boardW, height: boardH, elevation: elevation)

        let platformHoles = max(4, n / 3)
        let groundHoles = n - platformHoles
        let s: Float = 0.35

        var holes: [LevelDefinition.Vec2] = []

        let pCols = max(2, Int(ceil(sqrt(Float(platformHoles)))))
        let pRows = max(2, (platformHoles + pCols - 1) / pCols)
        let ps = min(s, boardW * 0.7 / Float(max(pCols, pRows) - 1))
        for row in 0..<pRows {
            for col in 0..<pCols {
                let x = (Float(col) - Float(pCols - 1) / 2) * ps
                let y = (Float(row) - Float(pRows - 1) / 2) * ps
                holes.append(.init(xPosition: x, yPosition: y, zPosition: elevation))
            }
        }

        let ringR: Float = boardW * 0.5 + 0.35
        let ringCount = max(8, groundHoles)
        for i in 0..<ringCount {
            let a = Float(i) / Float(ringCount) * 2 * .pi - .pi / 2
            holes.append(.init(xPosition: ringR * cos(a), yPosition: ringR * sin(a)))
        }

        return BoardLayoutResult(holes: holes, boards: [platform])
    }

    private static func valleyLayout(n: Int, elevation: Float) -> BoardLayoutResult {
        let boardW: Float = 0.50
        let boardH: Float = 1.6

        let leftBoard = LevelDefinition.Board(centerX: -0.65, centerY: 0, width: boardW, height: boardH, elevation: elevation)
        let rightBoard = LevelDefinition.Board(centerX: 0.65, centerY: 0, width: boardW, height: boardH, elevation: elevation)

        let sideHoles = max(3, n / 3)
        let valleyHoles = max(4, n - sideHoles * 2)
        let s: Float = 0.32

        var holes: [LevelDefinition.Vec2] = []

        let sRows = max(2, sideHoles)
        let ss = min(s, boardH * 0.7 / Float(sRows - 1))
        for i in 0..<sRows {
            let y = (Float(i) - Float(sRows - 1) / 2) * ss
            holes.append(.init(xPosition: leftBoard.centerX, yPosition: y, zPosition: elevation))
        }
        for i in 0..<sRows {
            let y = (Float(i) - Float(sRows - 1) / 2) * ss
            holes.append(.init(xPosition: rightBoard.centerX, yPosition: y, zPosition: elevation))
        }

        let vCols = max(2, Int(ceil(sqrt(Float(valleyHoles) * 2))))
        let vRows = max(2, (valleyHoles + vCols - 1) / vCols)
        let vs = min(s, 0.5 / Float(max(vCols, vRows) - 1))
        for row in 0..<vRows {
            for col in 0..<vCols {
                let x = (Float(col) - Float(vCols - 1) / 2) * vs
                let y = (Float(row) - Float(vRows - 1) / 2) * vs
                holes.append(.init(xPosition: x, yPosition: y))
            }
        }

        return BoardLayoutResult(holes: holes, boards: [leftBoard, rightBoard])
    }
}
