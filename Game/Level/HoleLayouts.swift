import Foundation

enum HoleLayout: CaseIterable {
    case grid
    case circle
    case hexagon
    case diamond
    case cross
    case rings
    case triangle
    case star
    case honeycomb
    case spiral
    case doubleGrid
    case scattered
    case columns
    case centerPlusRing
    case rotatedSquare
    case squareInCircle
    case arc
    case concentricSquares
    case triangleInCircle
    case crescent
    case letterH
    case letterT
    case arrow
    case zigzag
    case clover
    case clusters
    case threeColumns
    case ringWithSpokes
    case diamondOutline
    case bowTie

    func generate(n: Int) -> [LevelDefinition.Vec2] {
        switch self {
        case .grid:              return gridLayout(n: n)
        case .circle:            return circleLayout(n: n)
        case .hexagon:           return hexagonLayout(n: n)
        case .diamond:           return diamondLayout(n: n)
        case .cross:             return crossLayout(n: n)
        case .rings:             return ringsLayout(n: n)
        case .triangle:          return triangleLayout(n: n)
        case .star:              return starLayout(n: n)
        case .honeycomb:         return honeycombLayout(n: n)
        case .spiral:            return spiralLayout(n: n)
        case .doubleGrid:        return doubleGridLayout(n: n)
        case .scattered:         return scatteredLayout(n: n)
        case .columns:           return columnsLayout(n: n)
        case .centerPlusRing:    return centerPlusRingLayout(n: n)
        case .rotatedSquare:     return rotatedSquareLayout(n: n)
        case .squareInCircle:    return squareInCircleLayout(n: n)
        case .arc:               return arcLayout(n: n)
        case .concentricSquares: return concentricSquaresLayout(n: n)
        case .triangleInCircle:  return triangleInCircleLayout(n: n)
        case .crescent:          return crescentLayout(n: n)
        case .letterH:           return letterHLayout(n: n)
        case .letterT:           return letterTLayout(n: n)
        case .arrow:             return arrowLayout(n: n)
        case .zigzag:            return zigzagLayout(n: n)
        case .clover:            return cloverLayout(n: n)
        case .clusters:          return clustersLayout(n: n)
        case .threeColumns:      return threeColumnsLayout(n: n)
        case .ringWithSpokes:    return ringWithSpokesLayout(n: n)
        case .diamondOutline:    return diamondOutlineLayout(n: n)
        case .bowTie:            return bowTieLayout(n: n)
        }
    }
}

// MARK: - Layout implementations

private extension HoleLayout {

    func gridLayout(n: Int) -> [LevelDefinition.Vec2] {
        let cols = max(3, Int(ceil(sqrt(Float(n) * 1.25))))
        let rows = max(3, (n + cols - 1) / cols)
        let s = min(0.42, 1.6 / Float(max(cols, rows) - 1))
        let ox = -Float(cols - 1) / 2 * s
        let oy = -Float(rows - 1) / 2 * s
        var pts: [LevelDefinition.Vec2] = []
        for row in 0..<rows {
            for col in 0..<cols {
                pts.append(.init(xPosition: ox + Float(col) * s,
                                 yPosition: oy + Float(row) * s))
            }
        }
        return pts
    }

    func circleLayout(n: Int) -> [LevelDefinition.Vec2] {
        let count = max(8, n)
        let r: Float = min(0.90, 0.40 + Float(count) * 0.035)
        return (0..<count).map { i in
            let a = Float(i) / Float(count) * 2 * .pi - .pi / 2
            return .init(xPosition: r * cos(a), yPosition: r * sin(a))
        }
    }

    func hexagonLayout(n: Int) -> [LevelDefinition.Vec2] {
        var pts: [LevelDefinition.Vec2] = [.init(xPosition: 0, yPosition: 0)]
        var ring = 1
        while pts.count < n {
            let count = ring * 6
            let r = Float(ring) * min(0.35, 0.95 / Float(ring + 1))
            for i in 0..<count {
                let a = Float(i) / Float(count) * 2 * .pi + Float(ring) * 0.15
                pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
            }
            ring += 1
        }
        return pts
    }

    func diamondLayout(n: Int) -> [LevelDefinition.Vec2] {
        var half = max(2, Int(ceil(sqrt(Float(n)))))
        if half % 2 == 0 { half += 1 }
        let s = min(0.38, 1.6 / Float(half))
        var pts: [LevelDefinition.Vec2] = []
        for rowIdx in 0..<(half * 2 - 1) {
            let dist = abs(rowIdx - (half - 1))
            let count = half - dist
            let y = (Float(rowIdx) - Float(half - 1)) * s
            let ox = -Float(count - 1) / 2 * s
            for col in 0..<count {
                pts.append(.init(xPosition: ox + Float(col) * s, yPosition: y))
            }
        }
        return pts
    }

    func crossLayout(n: Int) -> [LevelDefinition.Vec2] {
        let arm = max(2, n / 5)
        let s = min(0.40, 1.6 / Float(arm * 2))
        var pts: [LevelDefinition.Vec2] = []
        for i in -arm...arm {
            pts.append(.init(xPosition: 0, yPosition: Float(i) * s))
        }
        for i in -arm...arm where i != 0 {
            pts.append(.init(xPosition: Float(i) * s, yPosition: 0))
        }
        let fill = max(1, arm - 1)
        for dx in 1...fill {
            for dy in 1...fill {
                for sx: Float in [-1, 1] {
                    for sy: Float in [-1, 1] {
                        pts.append(.init(xPosition: sx * Float(dx) * s, yPosition: sy * Float(dy) * s))
                    }
                }
            }
        }
        return pts
    }

    func ringsLayout(n: Int) -> [LevelDefinition.Vec2] {
        let ringCount = n <= 14 ? 2 : (n <= 22 ? 3 : 4)
        var pts: [LevelDefinition.Vec2] = []
        let perRing = max(4, (n - (ringCount > 2 ? 1 : 0)) / ringCount)
        if ringCount > 2 {
            pts.append(.init(xPosition: 0, yPosition: 0))
        }
        for ring in 0..<ringCount {
            let r = 0.25 + Float(ring) * (0.65 / Float(ringCount - 1))
            let count = max(4, perRing + ring * 2)
            let offset = Float(ring) * .pi / Float(count)
            for i in 0..<count {
                let a = Float(i) / Float(count) * 2 * .pi + offset
                pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
            }
        }
        return pts
    }

    func triangleLayout(n: Int) -> [LevelDefinition.Vec2] {
        var rows = 4
        while rows * (rows + 1) / 2 < n { rows += 1 }
        let s = min(0.38, 1.6 / Float(rows))
        var pts: [LevelDefinition.Vec2] = []
        let cy = Float(rows - 1) / 2
        for row in 0..<rows {
            let count = row + 1
            let ox = -Float(count - 1) / 2 * s
            let y = (Float(row) - cy) * s * 0.866
            for col in 0..<count {
                pts.append(.init(xPosition: ox + Float(col) * s, yPosition: y))
            }
        }
        return pts
    }

    func starLayout(n: Int) -> [LevelDefinition.Vec2] {
        let points = n <= 14 ? 5 : (n <= 22 ? 6 : 8)
        var pts: [LevelDefinition.Vec2] = [.init(xPosition: 0, yPosition: 0)]
        for i in 0..<points {
            let outerAngle = Float(i) / Float(points) * 2 * .pi - .pi / 2
            pts.append(.init(xPosition: 0.85 * cos(outerAngle), yPosition: 0.85 * sin(outerAngle)))
            let innerAngle = outerAngle + .pi / Float(points)
            pts.append(.init(xPosition: 0.40 * cos(innerAngle), yPosition: 0.40 * sin(innerAngle)))
        }
        if pts.count < n {
            let extra = n - pts.count
            for i in 0..<extra {
                let a = Float(i) / Float(extra) * 2 * .pi - .pi / 2 + 0.3
                pts.append(.init(xPosition: 0.62 * cos(a), yPosition: 0.62 * sin(a)))
            }
        }
        return pts
    }

    func honeycombLayout(n: Int) -> [LevelDefinition.Vec2] {
        var halfRows = 2
        while (2 * halfRows + 1) * (halfRows + 2) < n { halfRows += 1 }
        let s = min(0.36, 1.6 / Float(2 * halfRows + 1))
        let h = s * 0.866
        var pts: [LevelDefinition.Vec2] = []
        for row in -halfRows...halfRows {
            let cols = (abs(row) % 2 == 0) ? (halfRows + 2) : (halfRows + 1)
            let offset: Float = (abs(row) % 2 == 0) ? 0 : s * 0.5
            let ox = -Float(cols - 1) / 2 * s + offset
            for col in 0..<cols {
                pts.append(.init(xPosition: ox + Float(col) * s,
                                 yPosition: Float(row) * h))
            }
        }
        return pts
    }

    func spiralLayout(n: Int) -> [LevelDefinition.Vec2] {
        let count = max(10, n - 1)
        var pts: [LevelDefinition.Vec2] = [.init(xPosition: 0, yPosition: 0)]
        let turns: Float = 2.0 + Float(count) * 0.12
        for i in 1...count {
            let t = Float(i) / Float(count)
            let r = 0.12 + t * 0.78
            let a = t * turns * .pi
            pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
        }
        return pts
    }

    func doubleGridLayout(n: Int) -> [LevelDefinition.Vec2] {
        let half = max(6, n / 2)
        let cols = max(3, Int(ceil(sqrt(Float(half) * 1.5))))
        let rows = max(2, (half + cols - 1) / cols)
        let s = min(0.38, 1.5 / Float(max(cols, rows * 2 + 1)))
        let gap: Float = s * 0.6
        var pts: [LevelDefinition.Vec2] = []
        for gy: Float in [-1, 1] {
            let cy = gy * (Float(rows - 1) / 2 * s + gap)
            for row in 0..<rows {
                for col in 0..<cols {
                    let x = -Float(cols - 1) / 2 * s + Float(col) * s
                    let y = cy + (Float(row) - Float(rows - 1) / 2) * s
                    pts.append(.init(xPosition: x, yPosition: y))
                }
            }
        }
        return pts
    }

    func scatteredLayout(n: Int) -> [LevelDefinition.Vec2] {
        var pts: [LevelDefinition.Vec2] = []
        let cols = max(3, Int(ceil(sqrt(Float(n) * 1.3))))
        let rows = max(3, (n + cols - 1) / cols)
        let sx: Float = 1.6 / Float(cols)
        let sy: Float = 1.6 / Float(rows)
        var idx = 0
        for row in 0..<rows {
            for col in 0..<cols {
                guard idx < n else { break }
                let jitterX: Float = (Float((idx * 7 + 13) % 17) / 17.0 - 0.5) * sx * 0.35
                let jitterY: Float = (Float((idx * 11 + 3) % 13) / 13.0 - 0.5) * sy * 0.35
                let x = -0.8 + Float(col) * sx + jitterX
                let y = -0.8 + Float(row) * sy + jitterY
                pts.append(.init(xPosition: x, yPosition: y))
                idx += 1
            }
        }
        return pts
    }

    func columnsLayout(n: Int) -> [LevelDefinition.Vec2] {
        let colCount = n <= 12 ? 2 : (n <= 20 ? 3 : 4)
        let rowCount = max(3, (n + colCount - 1) / colCount)
        let s = min(0.35, 1.5 / Float(rowCount - 1))
        let xSpread: Float = min(0.55, 0.30 + Float(colCount) * 0.12)
        var pts: [LevelDefinition.Vec2] = []
        for col in 0..<colCount {
            let x = -xSpread + Float(col) * (2 * xSpread / Float(colCount - 1))
            for row in 0..<rowCount {
                let y = (Float(row) - Float(rowCount - 1) / 2) * s
                pts.append(.init(xPosition: x, yPosition: y))
            }
        }
        return pts
    }

    func centerPlusRingLayout(n: Int) -> [LevelDefinition.Vec2] {
        let centerCount = n <= 14 ? 1 : (n <= 20 ? 2 : 3)
        var pts: [LevelDefinition.Vec2] = []
        if centerCount == 1 {
            pts.append(.init(xPosition: 0, yPosition: 0))
        } else {
            let r: Float = 0.12
            for i in 0..<centerCount {
                let a = Float(i) / Float(centerCount) * 2 * .pi - .pi / 2
                pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
            }
        }
        let ringCount = n - centerCount
        let r: Float = min(0.85, 0.55 + Float(ringCount) * 0.02)
        for i in 0..<ringCount {
            let a = Float(i) / Float(ringCount) * 2 * .pi - .pi / 2
            pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
        }
        return pts
    }

    func rotatedSquareLayout(n: Int) -> [LevelDefinition.Vec2] {
        let rings = max(2, (n + 3) / 4)
        var pts: [LevelDefinition.Vec2] = []
        for ring in 1...rings {
            let r = Float(ring) / Float(rings) * 0.85
            let perSide = max(1, ring)
            let count = perSide * 4
            for i in 0..<count {
                let a = Float(i) / Float(count) * 2 * .pi + .pi / 4
                pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
            }
        }
        return pts
    }

    func squareInCircleLayout(n: Int) -> [LevelDefinition.Vec2] {
        let sqSide = max(2, n / 4)
        let ringCount = max(8, n - sqSide * 4)
        let sq: Float = min(0.50, 0.30 + Float(sqSide) * 0.05)
        var pts: [LevelDefinition.Vec2] = []
        let s = 2 * sq / Float(sqSide - 1)
        for row in 0..<sqSide {
            for col in 0..<sqSide {
                if row == 0 || row == sqSide - 1 || col == 0 || col == sqSide - 1 {
                    pts.append(.init(xPosition: -sq + Float(col) * s,
                                     yPosition: -sq + Float(row) * s))
                }
            }
        }
        let r: Float = min(0.90, sq + 0.35)
        for i in 0..<ringCount {
            let a = Float(i) / Float(ringCount) * 2 * .pi - .pi / 2
            pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
        }
        return pts
    }

    func arcLayout(n: Int) -> [LevelDefinition.Vec2] {
        let count = max(8, n)
        let r: Float = min(0.85, 0.55 + Float(count) * 0.02)
        var pts: [LevelDefinition.Vec2] = []
        for i in 0..<count {
            let a = Float(i) / Float(count - 1) * .pi - .pi / 2
            pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
        }
        return pts
    }

    func concentricSquaresLayout(n: Int) -> [LevelDefinition.Vec2] {
        let rings = max(2, n / 6)
        var pts: [LevelDefinition.Vec2] = []
        for ring in 1...rings {
            let r = Float(ring) / Float(rings) * 0.85
            let count = max(4, 4 + (ring - 1) * 4)
            for i in 0..<count {
                let a = Float(i) / Float(count) * 2 * .pi + .pi / 4
                pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
            }
        }
        return pts
    }

    func triangleInCircleLayout(n: Int) -> [LevelDefinition.Vec2] {
        let innerCount = max(3, n / 4)
        let outerCount = max(6, n - innerCount)
        let rInner: Float = min(0.40, 0.20 + Float(innerCount) * 0.04)
        var pts: [LevelDefinition.Vec2] = []
        for i in 0..<innerCount {
            let a = Float(i) / Float(innerCount) * 2 * .pi - .pi / 2
            pts.append(.init(xPosition: rInner * cos(a), yPosition: rInner * sin(a)))
        }
        let rOuter: Float = min(0.85, rInner + 0.35)
        for i in 0..<outerCount {
            let a = Float(i) / Float(outerCount) * 2 * .pi - .pi / 2
            pts.append(.init(xPosition: rOuter * cos(a), yPosition: rOuter * sin(a)))
        }
        return pts
    }

    func crescentLayout(n: Int) -> [LevelDefinition.Vec2] {
        let outerCount = max(5, (n * 3 + 2) / 5)
        let innerCount = max(4, n - outerCount)
        let rOut: Float = 0.80
        let rIn: Float = 0.50
        var pts: [LevelDefinition.Vec2] = []
        for i in 0..<outerCount {
            let a = Float(i) / Float(outerCount - 1) * .pi * 0.9 - .pi / 2 - .pi / 8
            pts.append(.init(xPosition: rOut * cos(a), yPosition: rOut * sin(a)))
        }
        for i in 0..<innerCount {
            let a = Float(i) / Float(innerCount - 1) * .pi * 0.75 - .pi / 2 + .pi / 10
            pts.append(.init(xPosition: rIn * cos(a) + 0.22, yPosition: rIn * sin(a)))
        }
        return pts
    }

    func letterHLayout(n: Int) -> [LevelDefinition.Vec2] {
        let colRows = max(3, (n - 1) / 2)
        let s = min(0.32, 1.5 / Float(colRows - 1))
        let xL: Float = -0.45
        let xR: Float = 0.45
        var pts: [LevelDefinition.Vec2] = []
        for row in 0..<colRows {
            let y = (Float(row) - Float(colRows - 1) / 2) * s
            pts.append(.init(xPosition: xL, yPosition: y))
            pts.append(.init(xPosition: xR, yPosition: y))
        }
        let barCount = max(1, n - colRows * 2)
        let barS = 0.9 / Float(barCount + 1)
        for i in 1...barCount {
            pts.append(.init(xPosition: xL + Float(i) * barS, yPosition: 0))
        }
        return pts
    }

    func letterTLayout(n: Int) -> [LevelDefinition.Vec2] {
        let topCount = max(3, n / 3)
        let stemCount = max(3, n - topCount)
        let topS = min(0.30, 1.5 / Float(topCount - 1))
        let stemS = min(0.30, 1.3 / Float(stemCount))
        var pts: [LevelDefinition.Vec2] = []
        for col in 0..<topCount {
            let x = (Float(col) - Float(topCount - 1) / 2) * topS
            pts.append(.init(xPosition: x, yPosition: 0.65))
        }
        for row in 0..<stemCount {
            pts.append(.init(xPosition: 0, yPosition: 0.65 - Float(row + 1) * stemS))
        }
        return pts
    }

    func arrowLayout(n: Int) -> [LevelDefinition.Vec2] {
        let perSide = max(3, n / 2)
        var pts: [LevelDefinition.Vec2] = []
        for i in 0..<perSide {
            let t = Float(i) / Float(perSide - 1)
            pts.append(.init(xPosition: -0.60 * (1 - t), yPosition: -0.60 + t * 1.2))
            pts.append(.init(xPosition: 0.60 * (1 - t), yPosition: -0.60 + t * 1.2))
        }
        return pts
    }

    func zigzagLayout(n: Int) -> [LevelDefinition.Vec2] {
        let count = max(8, n)
        let s = min(0.28, 1.5 / Float(count - 1))
        var pts: [LevelDefinition.Vec2] = []
        for i in 0..<count {
            let x = (Float(i % 2) - 0.5) * 0.65
            let y = (Float(i) - Float(count - 1) / 2) * s
            pts.append(.init(xPosition: x, yPosition: y))
        }
        return pts
    }

    func cloverLayout(n: Int) -> [LevelDefinition.Vec2] {
        let petals = n <= 14 ? 4 : (n <= 22 ? 5 : 6)
        let perPetal = max(3, n / petals)
        let r: Float = 0.22
        let R: Float = 0.55
        var pts: [LevelDefinition.Vec2] = []
        for petal in 0..<petals {
            let ca = Float(petal) / Float(petals) * 2 * .pi
            let cx = R * cos(ca)
            let cy = R * sin(ca)
            for i in 0..<perPetal {
                let a = Float(i) / Float(perPetal) * 2 * .pi + ca
                pts.append(.init(xPosition: cx + r * cos(a), yPosition: cy + r * sin(a)))
            }
        }
        return pts
    }

    func clustersLayout(n: Int) -> [LevelDefinition.Vec2] {
        let clusterCount = n <= 14 ? 3 : (n <= 22 ? 4 : 5)
        let perCluster = max(3, n / clusterCount)
        let R: Float = 0.65
        let r: Float = min(0.18, 0.10 + Float(perCluster) * 0.02)
        var pts: [LevelDefinition.Vec2] = []
        for c in 0..<clusterCount {
            let ca = Float(c) / Float(clusterCount) * 2 * .pi - .pi / 2
            let cx = R * cos(ca)
            let cy = R * sin(ca)
            for i in 0..<perCluster {
                let a = Float(i) / Float(perCluster) * 2 * .pi
                pts.append(.init(xPosition: cx + r * cos(a), yPosition: cy + r * sin(a)))
            }
        }
        return pts
    }

    func threeColumnsLayout(n: Int) -> [LevelDefinition.Vec2] {
        let rowCount = max(3, (n + 2) / 3)
        let s = min(0.35, 1.5 / Float(rowCount - 1))
        let xPos: [Float] = [-0.55, 0, 0.55]
        var pts: [LevelDefinition.Vec2] = []
        for x in xPos {
            for row in 0..<rowCount {
                let y = (Float(row) - Float(rowCount - 1) / 2) * s
                pts.append(.init(xPosition: x, yPosition: y))
            }
        }
        return pts
    }

    func ringWithSpokesLayout(n: Int) -> [LevelDefinition.Vec2] {
        let spokes = max(3, n / 4)
        let ringCount = max(6, n - spokes - 1)
        var pts: [LevelDefinition.Vec2] = [.init(xPosition: 0, yPosition: 0)]
        for i in 0..<spokes {
            let a = Float(i) / Float(spokes) * 2 * .pi
            pts.append(.init(xPosition: 0.35 * cos(a), yPosition: 0.35 * sin(a)))
        }
        for i in 0..<ringCount {
            let a = Float(i) / Float(ringCount) * 2 * .pi - .pi / 2
            pts.append(.init(xPosition: 0.78 * cos(a), yPosition: 0.78 * sin(a)))
        }
        return pts
    }

    func diamondOutlineLayout(n: Int) -> [LevelDefinition.Vec2] {
        let rings = max(2, n / 4)
        var pts: [LevelDefinition.Vec2] = []
        for ring in 1...rings {
            let r = Float(ring) / Float(rings) * 0.78
            let count = max(4, ring * 4)
            for i in 0..<count {
                let a = Float(i) / Float(count) * 2 * .pi + .pi / 4
                pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
            }
        }
        return pts
    }

    func bowTieLayout(n: Int) -> [LevelDefinition.Vec2] {
        let perSide = max(3, (n - 1) / 2)
        var pts: [LevelDefinition.Vec2] = [.init(xPosition: 0, yPosition: 0)]
        let r: Float = 0.70
        for i in 0..<perSide {
            let a = Float(i) / Float(perSide) * .pi - .pi / 2
            pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
        }
        for i in 0..<perSide {
            let a = Float(i) / Float(perSide) * .pi + .pi / 2
            pts.append(.init(xPosition: r * cos(a), yPosition: r * sin(a)))
        }
        return pts
    }
}
