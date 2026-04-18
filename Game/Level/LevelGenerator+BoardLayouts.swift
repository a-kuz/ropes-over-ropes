import Foundation
import simd

extension LevelGenerator {

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

    static func pickBoardLayout(for levelId: Int) -> BoardLayout {
        let layouts = BoardLayout.allCases
        return layouts[(levelId / 3) % layouts.count]
    }

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
