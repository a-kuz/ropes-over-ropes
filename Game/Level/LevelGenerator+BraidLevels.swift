import Foundation
import simd

extension LevelGenerator {

    // MARK: - Braid mode levels

    static func generateBraidLevel(levelId: Int, particleCount: Int = 55) -> LevelDefinition {
        let localId = levelId - 3000
        let strandCount: Int
        let swapCount: Int
        switch localId {
        case 1:  strandCount = 3; swapCount = 100
        case 2:  strandCount = 3; swapCount = 4
        case 3:  strandCount = 3; swapCount = 6
        case 4:  strandCount = 4; swapCount = 4
        case 5:  strandCount = 4; swapCount = 6
        case 6:  strandCount = 4; swapCount = 8
        case 7:  strandCount = 5; swapCount = 6
        case 8:  strandCount = 5; swapCount = 8
        case 9:  strandCount = 5; swapCount = 10
        case 10: strandCount = 6; swapCount = 10
        default:
            let sc = min(8, 3 + (localId - 1) / 3)
            let sw = min(16, 2 + localId)
            strandCount = sc; swapCount = sw
        }
        let halfHeight: Float = localId == 1 ? 2.2 : 0.55
        return buildBraidModeLevel(levelId: levelId, strandCount: strandCount, swapCount: swapCount, halfHeight: halfHeight, particleCount: particleCount)
    }

    /// Build a braid puzzle level:
    /// - N holes at top (anchors), N holes at bottom (moveable ends)
    /// - N strands connecting top[i] to bottom[i]
    /// - Target: a permutation achieved by `swapCount` adjacent swaps (classic braid)
    private static func buildBraidModeLevel(levelId: Int, strandCount: Int, swapCount: Int, halfHeight: Float = 0.55, particleCount: Int) -> LevelDefinition {
        let n = strandCount
        let spacing: Float = 0.28
        let totalWidth = Float(n - 1) * spacing
        let startX = -totalWidth / 2

        // Top holes: indices 0..<n, Bottom holes: indices n..<2n
        var holes: [LevelDefinition.Vec2] = []
        for i in 0..<n {
            let x = startX + Float(i) * spacing
            holes.append(.init(xPosition: x, yPosition: halfHeight))  // top
        }
        for i in 0..<n {
            let x = startX + Float(i) * spacing
            holes.append(.init(xPosition: x, yPosition: -halfHeight)) // bottom
        }

        // Ropes: strand i goes from top[i] to bottom[i]
        let ropes = (0..<n).map { i in
            LevelDefinition.Rope(startHole: i, endHole: n + i,
                                 color: colors[i % colors.count], radius: 0.038)
        }

        // Pin actions
        var actions: [LevelDefinition.Action] = []
        for i in 0..<n {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: i))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: n + i))
        }

        // Simulate braid swaps to compute target permutation
        // Classic braid: alternate swap(0,1), swap(n-2,n-1), swap(0,1), ...
        // For 3 strands: swap(0,1), swap(1,2), swap(0,1), swap(1,2), ...
        var slots = Array(0..<n) // slots[position] = strandIndex
        for step in 0..<swapCount {
            let swapPos: Int
            if n == 3 {
                swapPos = (step % 2 == 0) ? 0 : 1
            } else {
                // For N strands: cycle through swap positions
                swapPos = step % (n - 1)
            }
            slots.swapAt(swapPos, swapPos + 1)
        }

        // braidTargets[strandIndex] = target bottom hole index
        // slots[position] = strandIndex, so we need the inverse:
        // strandIndex -> which position it ended up in -> bottom hole = n + position
        var braidTargets = Array(repeating: 0, count: n)
        for pos in 0..<n {
            braidTargets[slots[pos]] = n + pos
        }

        return LevelDefinition(
            mode: "braid", id: levelId, holeRadius: 0.08, particlesPerRope: particleCount,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil,
            rails: nil, carts: nil, stations: nil,
            braidTargets: braidTargets, braidMinCrossings: max(1, swapCount / 2)
        )
    }

    // MARK: - Braid-style untangle levels (spiral/braid via zigzag drags)

    /// Generates a level where ropes spiral around each other.
    ///
    /// Key insight: to create a spiral, drag one rope's end back and forth between
    /// two fixed holes on opposite sides of another rope. Each physical drag lifts
    /// the end, carries it ACROSS the other rope, and lowers it. The rope physically
    /// wraps one more time with each crossing.
    ///
    /// Pattern for 2-rope spiral:
    ///   Pin A at (a0, a1), Pin B at (b0, b1)  — A and B cross
    ///   Find holes P (side1 of B) and Q (side2 of B) near crossing
    ///   Drag A end1 → P (crosses B, wraps once)
    ///   Drag A end1 → Q (crosses B back, wraps again)
    ///   Drag A end1 → P (wraps again)
    ///   ...
    static func generateBraidUntangle(
        levelId: Int,
        particleCount: Int = 55
    ) -> LevelDefinition {
        var rng = SeededRNG(seed: UInt64(levelId) &* 2654435761)

        let layout = pickLayout(for: levelId)
        let wraps: Int  // number of zigzag crossings (each pair of drags = 1 crossing)
        switch levelId {
        case 1...15:  wraps = 8
        case 16...30: wraps = 10
        case 31...60: wraps = 12
        default:      wraps = 14
        }

        let holeCount = max(24, wraps + 10)
        let holes = layout.generate(n: holeCount)
        let holeSimd = holes.map { $0.simd }
        let center = holeSimd.reduce(.zero, +) / Float(holes.count)

        // Sort holes by angle from center
        let byAngle = holeSimd.enumerated()
            .map { (idx: $0.offset, angle: atan2($0.element.y - center.y, $0.element.x - center.x)) }
            .sorted { $0.angle < $1.angle }
        let n = byAngle.count

        // Pick 2 crossing ropes: roughly perpendicular through center
        let offset = rng.next(bound: n)
        let a0 = byAngle[offset % n].idx
        let a1 = byAngle[(offset + n / 2) % n].idx
        let b0 = byAngle[(offset + n / 4) % n].idx
        let b1 = byAngle[(offset + n / 4 + n / 2) % n].idx

        // Verify they cross; if not, adjust b1
        var finalB1 = b1
        if !segmentsCross(holeSimd[a0], holeSimd[a1], holeSimd[b0], holeSimd[b1]) {
            // Try nearby holes for b1
            for delta in 1..<n/2 {
                for sign in [-1, 1] {
                    let idx = byAngle[((offset + n / 4 + n / 2) + sign * delta + n) % n].idx
                    if idx == a0 || idx == a1 || idx == b0 { continue }
                    if segmentsCross(holeSimd[a0], holeSimd[a1], holeSimd[b0], holeSimd[idx]) {
                        finalB1 = idx
                        break
                    }
                }
                if finalB1 != b1 { break }
            }
        }

        let ropes: [LevelDefinition.Rope] = [
            .init(startHole: a0, endHole: a1, color: colors[0], radius: 0.038),
            .init(startHole: b0, endHole: finalB1, color: colors[1], radius: 0.038),
        ]

        let occupied = Set([a0, a1, b0, finalB1])

        // Find crossing point
        let xPt: SIMD2<Float> = {
            let d1 = holeSimd[a1] - holeSimd[a0]
            let d2 = holeSimd[finalB1] - holeSimd[b0]
            let cross = d1.x * d2.y - d1.y * d2.x
            guard abs(cross) > 1e-9 else { return center }
            let d = holeSimd[b0] - holeSimd[a0]
            let t = (d.x * d2.y - d.y * d2.x) / cross
            return holeSimd[a0] + d1 * t
        }()

        // Find perpendicular to rope B (defines "sides")
        let bDir = simd_normalize(holeSimd[finalB1] - holeSimd[b0])
        let perp = SIMD2<Float>(-bDir.y, bDir.x)

        // Find 2 holes: P on side+, Q on side- of B.
        // They must be far enough apart (≥0.3) for the drag to create real wrapping.
        // Prefer holes at moderate distance from crossing — not too close (no amplitude),
        // not too far (drag path misses the other rope).
        let minSep: Float = 0.25  // min perpendicular separation from crossing
        var sidePos: [(idx: Int, perpDist: Float)] = []
        var sideNeg: [(idx: Int, perpDist: Float)] = []
        for e in byAngle {
            if occupied.contains(e.idx) { continue }
            let offset = holeSimd[e.idx] - xPt
            let perpDist = simd_dot(perp, offset)  // signed distance from crossing along perp
            if perpDist > minSep { sidePos.append((e.idx, perpDist)) }
            else if perpDist < -minSep { sideNeg.append((e.idx, -perpDist)) }
        }
        // Sort by perpendicular distance — pick ones with good separation
        sidePos.sort { $0.perpDist < $1.perpDist }
        sideNeg.sort { $0.perpDist < $1.perpDist }

        // Pick P and Q that are far enough from each other
        guard let holeP = sidePos.first(where: { $0.perpDist > minSep })?.idx,
              let holeQ = sideNeg.first(where: { $0.perpDist > minSep })?.idx,
              simd_length(holeSimd[holeP] - holeSimd[holeQ]) > 0.3
        else {
            // Fallback to regular generation
            return generate(levelId: levelId + 1, particleCount: particleCount)
        }

        // Build actions: pin both ropes, then zigzag rope A end1 between P and Q
        var actions: [LevelDefinition.Action] = []
        actions.append(.init(type: "pin", ropeIndex: 0, endIndex: 0, holeIndex: a0))
        actions.append(.init(type: "pin", ropeIndex: 0, endIndex: 1, holeIndex: a1))
        actions.append(.init(type: "pin", ropeIndex: 1, endIndex: 0, holeIndex: b0))
        actions.append(.init(type: "pin", ropeIndex: 1, endIndex: 1, holeIndex: finalB1))

        // Alternate which rope's end gets dragged:
        //   drag rope 0 end 1 → P (rope 0 goes over rope 1)
        //   drag rope 1 end 1 → Q (rope 1 goes over rope 0)
        //   drag rope 0 end 1 → P (rope 0 goes over again)
        //   drag rope 1 end 1 → Q (rope 1 goes over again)
        // Each drag lifts above → the dragged rope is on top.
        // Alternating ropes → alternating over/under → spiral.
        // Crossings accumulate because each rope's crossing is near ITS end,
        // not near the other rope's end — so the next drag doesn't undo it.
        // Alternating drags using DIFFERENT holes each time to avoid collisions.
        // Rope 0 goes to sidePos holes: P0, P1, P2, ...
        // Rope 1 goes to sideNeg holes: Q0, Q1, Q2, ...
        // Sequence: rope0→P0, rope1→Q0, rope0→Q1, rope1→P1, rope0→P2, rope1→Q2, ...
        // Each rope zigzags between sides, each drag to a NEW hole.
        let allPos = sidePos.map { $0.idx }
        let allNeg = sideNeg.map { $0.idx }
        var posIdx = 0, negIdx = 0
        for i in 0..<wraps {
            let rope = i % 2
            // Each rope alternates sides. Rope 0 starts positive, rope 1 starts negative.
            let goPositive: Bool
            if rope == 0 {
                goPositive = (i / 2) % 2 == 0
            } else {
                goPositive = (i / 2) % 2 != 0
            }

            let target: Int
            if goPositive {
                guard posIdx < allPos.count else { break }
                target = allPos[posIdx]; posIdx += 1
            } else {
                guard negIdx < allNeg.count else { break }
                target = allNeg[negIdx]; negIdx += 1
            }
            actions.append(.init(type: "drag", ropeIndex: rope, endIndex: 1, holeIndex: target))
        }

        return LevelDefinition(
            mode: nil, id: levelId, holeRadius: 0.08, particlesPerRope: particleCount,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil, rails: nil, carts: nil, stations: nil
        )
    }
}
