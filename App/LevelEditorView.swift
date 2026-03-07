import SwiftUI
import simd

enum EditorTool: String, CaseIterable {
    case hole = "Hole"
    case railPoint = "Rail"
    case rope = "Rope"
    case cart = "Cart"
    case station = "Station"
    case move = "Move"
    case delete = "Delete"

    var icon: String {
        switch self {
        case .hole: return "circle"
        case .railPoint: return "point.topleft.down.to.point.bottomright.curvepath"
        case .rope: return "line.diagonal"
        case .cart: return "shippingbox"
        case .station: return "flag"
        case .move: return "arrow.up.and.down.and.arrow.left.and.right"
        case .delete: return "trash"
        }
    }
}

struct EditorRail {
    var points: [SIMD2<Float>] = []
}

struct EditorRope {
    var startHole: Int
    var endHole: Int
    var colorIndex: Int
}

struct EditorCart {
    var railIndex: Int
    var t: Float
    var radius: Float = 0.12
    var mass: Float = 0.5
}

struct EditorStation {
    var railIndex: Int
    var t: Float
    var radius: Float = 0.15
    var cartIndex: Int
}

class EditorState: ObservableObject {
    @Published var holes: [SIMD2<Float>] = []
    @Published var rails: [EditorRail] = []
    @Published var ropes: [EditorRope] = []
    @Published var carts: [EditorCart] = []
    @Published var stations: [EditorStation] = []

    @Published var currentTool: EditorTool = .hole
    @Published var currentRailIndex: Int? = nil
    @Published var ropeStartHole: Int? = nil

    @Published var holeRadius: Float = 0.08
    @Published var particlesPerRope: Int = 32

    // For rope color cycling
    var nextColorIndex: Int = 0

    func reset() {
        holes.removeAll()
        rails.removeAll()
        ropes.removeAll()
        carts.removeAll()
        stations.removeAll()
        currentRailIndex = nil
        ropeStartHole = nil
        nextColorIndex = 0
    }

    func toLevelDefinition(levelId: Int) -> LevelDefinition {
        let levelHoles = holes.map {
            LevelDefinition.Vec2(xPosition: $0.x, yPosition: $0.y)
        }

        let colors: [LevelDefinition.Color] = [
            .init(redChannel: 0.95, greenChannel: 0.30, blueChannel: 0.05),
            .init(redChannel: 0.10, greenChannel: 0.35, blueChannel: 0.92),
            .init(redChannel: 0.90, greenChannel: 0.12, blueChannel: 0.25),
            .init(redChannel: 0.15, greenChannel: 0.75, blueChannel: 0.30),
            .init(redChannel: 0.92, greenChannel: 0.78, blueChannel: 0.05),
            .init(redChannel: 0.60, greenChannel: 0.10, blueChannel: 0.72),
        ]

        let levelRopes = ropes.map { rope in
            LevelDefinition.Rope(
                startHole: rope.startHole,
                endHole: rope.endHole,
                color: colors[rope.colorIndex % colors.count],
                radius: 0.038
            )
        }

        var actions: [LevelDefinition.Action] = []
        for (i, rope) in ropes.enumerated() {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: rope.startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: rope.endHole))
        }

        let levelRails: [LevelDefinition.RailDef]? = rails.isEmpty ? nil : rails.map { rail in
            LevelDefinition.RailDef(points: rail.points.map {
                LevelDefinition.Vec2(xPosition: $0.x, yPosition: $0.y)
            })
        }

        let levelCarts: [LevelDefinition.CartDef]? = carts.isEmpty ? nil : carts.map { cart in
            LevelDefinition.CartDef(railIndex: cart.railIndex, startT: cart.t, radius: cart.radius, mass: cart.mass)
        }

        let levelStations: [LevelDefinition.StationDef]? = stations.isEmpty ? nil : stations.map { station in
            LevelDefinition.StationDef(railIndex: station.railIndex, t: station.t, radius: station.radius, cartIndex: station.cartIndex)
        }

        let mode = rails.isEmpty ? "untangle" : "rail"

        return LevelDefinition(
            mode: mode,
            id: levelId,
            holeRadius: holeRadius,
            particlesPerRope: particlesPerRope,
            holes: levelHoles,
            ropes: levelRopes,
            hooks: nil,
            actions: actions,
            boards: nil,
            weights: nil,
            targets: nil,
            rails: levelRails,
            carts: levelCarts,
            stations: levelStations
        )
    }

    // Find nearest hole to a world position
    func nearestHole(to pos: SIMD2<Float>, maxDist: Float = 0.12) -> Int? {
        var best: Int?
        var bestDist: Float = maxDist
        for (i, h) in holes.enumerated() {
            let d = simd_length(h - pos)
            if d < bestDist {
                bestDist = d
                best = i
            }
        }
        return best
    }

    // Find nearest rail and parameter t
    func nearestRailT(to pos: SIMD2<Float>, maxDist: Float = 0.15) -> (railIndex: Int, t: Float)? {
        var bestRail: Int?
        var bestT: Float = 0
        var bestDist: Float = maxDist

        for (ri, rail) in rails.enumerated() {
            guard rail.points.count >= 2 else { continue }
            var arcLen: Float = 0
            var totalLen: Float = 0
            for i in 1..<rail.points.count {
                totalLen += simd_length(rail.points[i] - rail.points[i-1])
            }
            guard totalLen > 0 else { continue }

            arcLen = 0
            for i in 1..<rail.points.count {
                let a = rail.points[i-1]
                let b = rail.points[i]
                let segLen = simd_length(b - a)
                let dir = (b - a) / max(segLen, 1e-6)
                let toPos = pos - a
                let proj = max(0, min(segLen, simd_dot(toPos, dir)))
                let closest = a + dir * proj
                let dist = simd_length(pos - closest)
                if dist < bestDist {
                    bestDist = dist
                    bestRail = ri
                    bestT = (arcLen + proj) / totalLen
                }
                arcLen += segLen
            }
        }

        if let ri = bestRail {
            return (ri, bestT)
        }
        return nil
    }

    func railPosition(railIndex: Int, t: Float) -> SIMD2<Float>? {
        let rail = rails[railIndex]
        guard rail.points.count >= 2 else { return nil }

        var totalLen: Float = 0
        for i in 1..<rail.points.count {
            totalLen += simd_length(rail.points[i] - rail.points[i-1])
        }
        guard totalLen > 0 else { return rail.points[0] }

        let targetLen = t * totalLen
        var accum: Float = 0
        for i in 1..<rail.points.count {
            let segLen = simd_length(rail.points[i] - rail.points[i-1])
            if accum + segLen >= targetLen {
                let localT = (targetLen - accum) / max(segLen, 1e-6)
                return rail.points[i-1] * (1 - localT) + rail.points[i] * localT
                }
            accum += segLen
        }
        return rail.points.last
    }
}

struct LevelEditorView: View {
    @ObservedObject var editor: EditorState
    @State private var showExport = false
    @State private var showImport = false
    @State private var importJSON = ""
    @State private var importError: String?
    @State private var exportedJSON = ""
    @State private var levelId = 2001
    var onPlay: ((LevelDefinition) -> Void)?

    // Coordinate system: world space roughly -1..1
    private let worldExtent: Float = 1.2

    var body: some View {
        VStack(spacing: 0) {
            // Toolbar
            HStack(spacing: 4) {
                ForEach(EditorTool.allCases, id: \.self) { tool in
                    Button {
                        if tool == .railPoint && editor.currentTool != .railPoint {
                            // Start new rail
                            editor.rails.append(EditorRail())
                            editor.currentRailIndex = editor.rails.count - 1
                        }
                        if editor.currentTool == .railPoint && tool != .railPoint {
                            // Finish current rail
                            if let ri = editor.currentRailIndex, editor.rails[ri].points.count < 2 {
                                editor.rails.remove(at: ri)
                            }
                            editor.currentRailIndex = nil
                        }
                        if tool == .rope {
                            editor.ropeStartHole = nil
                        }
                        editor.currentTool = tool
                    } label: {
                        VStack(spacing: 2) {
                            Image(systemName: tool.icon)
                                .font(.system(size: 16))
                            Text(tool.rawValue)
                                .font(.system(size: 9))
                        }
                        .frame(width: 48, height: 44)
                        .background(editor.currentTool == tool ? Color.blue.opacity(0.3) : Color.clear)
                        .cornerRadius(8)
                    }
                    .foregroundColor(editor.currentTool == tool ? .blue : .white)
                }

                Spacer()

                Button("Clear") { editor.reset() }
                    .foregroundColor(.red)
                    .padding(.horizontal, 8)

                Button("Import") {
                    // Try clipboard first
                    #if os(iOS)
                    importJSON = UIPasteboard.general.string ?? ""
                    #elseif os(macOS)
                    importJSON = NSPasteboard.general.string(forType: .string) ?? ""
                    #endif
                    importError = nil
                    showImport = true
                }
                .foregroundColor(.cyan)
                .padding(.horizontal, 4)

                Button("Export") {
                    let def = editor.toLevelDefinition(levelId: levelId)
                    let enc = JSONEncoder()
                    enc.outputFormatting = [.prettyPrinted, .sortedKeys]
                    if let data = try? enc.encode(def), let str = String(data: data, encoding: .utf8) {
                        exportedJSON = str
                        showExport = true
                    }
                }
                .foregroundColor(.green)
                .padding(.horizontal, 8)

                if let onPlay = onPlay {
                    Button("Play") {
                        let def = editor.toLevelDefinition(levelId: levelId)
                        onPlay(def)
                    }
                    .foregroundColor(.yellow)
                    .padding(.horizontal, 8)
                }
            }
            .padding(.horizontal, 8)
            .padding(.vertical, 4)
            .background(Color.black.opacity(0.8))

            // Canvas
            GeometryReader { geo in
                let size = geo.size
                ZStack {
                    Color(white: 0.15)

                    Canvas { context, canvasSize in
                        let scale = min(canvasSize.width, canvasSize.height) / CGFloat(worldExtent * 2)
                        let cx = canvasSize.width / 2
                        let cy = canvasSize.height / 2

                        func toScreen(_ p: SIMD2<Float>) -> CGPoint {
                            CGPoint(x: cx + CGFloat(p.x) * scale, y: cy - CGFloat(p.y) * scale)
                        }

                        // Grid
                        context.withCGContext { cg in
                            cg.setStrokeColor(CGColor(gray: 0.25, alpha: 1))
                            cg.setLineWidth(0.5)
                            let step: Float = 0.2
                            var g: Float = -worldExtent
                            while g <= worldExtent {
                                let p1 = toScreen(SIMD2(g, -worldExtent))
                                let p2 = toScreen(SIMD2(g, worldExtent))
                                cg.move(to: p1); cg.addLine(to: p2); cg.strokePath()
                                let p3 = toScreen(SIMD2(-worldExtent, g))
                                let p4 = toScreen(SIMD2(worldExtent, g))
                                cg.move(to: p3); cg.addLine(to: p4); cg.strokePath()
                                g += step
                            }
                        }

                        // Rails
                        for (ri, rail) in editor.rails.enumerated() {
                            guard rail.points.count >= 2 else { continue }
                            var path = Path()
                            path.move(to: toScreen(rail.points[0]))
                            for i in 1..<rail.points.count {
                                path.addLine(to: toScreen(rail.points[i]))
                            }
                            let isActive = ri == editor.currentRailIndex
                            context.stroke(path, with: .color(isActive ? .yellow : .gray), lineWidth: isActive ? 4 : 3)

                            // Rail points
                            for pt in rail.points {
                                let sp = toScreen(pt)
                                let r: CGFloat = 4
                                context.fill(Path(ellipseIn: CGRect(x: sp.x - r, y: sp.y - r, width: r*2, height: r*2)),
                                             with: .color(.yellow.opacity(0.6)))
                            }
                        }

                        // Ropes
                        let ropeColors: [Color] = [.orange, .blue, .red, .green, .yellow, .purple]
                        for rope in editor.ropes {
                            guard editor.holes.indices.contains(rope.startHole),
                                  editor.holes.indices.contains(rope.endHole) else { continue }
                            let a = toScreen(editor.holes[rope.startHole])
                            let b = toScreen(editor.holes[rope.endHole])
                            var path = Path()
                            path.move(to: a)
                            path.addLine(to: b)
                            context.stroke(path, with: .color(ropeColors[rope.colorIndex % ropeColors.count]), lineWidth: 3)
                        }

                        // Stations
                        for station in editor.stations {
                            guard editor.rails.indices.contains(station.railIndex) else { continue }
                            if let pos = editor.railPosition(railIndex: station.railIndex, t: station.t) {
                                let sp = toScreen(pos)
                                let r = CGFloat(station.radius) * scale
                                context.stroke(Path(ellipseIn: CGRect(x: sp.x - r, y: sp.y - r, width: r*2, height: r*2)),
                                               with: .color(.green), lineWidth: 2)
                                // Flag icon
                                let flagRect = CGRect(x: sp.x - 6, y: sp.y - 6, width: 12, height: 12)
                                context.draw(Text("🚩").font(.system(size: 10)), in: flagRect)
                            }
                        }

                        // Carts
                        for cart in editor.carts {
                            guard editor.rails.indices.contains(cart.railIndex) else { continue }
                            if let pos = editor.railPosition(railIndex: cart.railIndex, t: cart.t) {
                                let sp = toScreen(pos)
                                let r = CGFloat(cart.radius) * scale
                                context.fill(Path(ellipseIn: CGRect(x: sp.x - r, y: sp.y - r, width: r*2, height: r*2)),
                                             with: .color(.orange.opacity(0.7)))
                                context.stroke(Path(ellipseIn: CGRect(x: sp.x - r, y: sp.y - r, width: r*2, height: r*2)),
                                               with: .color(.orange), lineWidth: 2)
                            }
                        }

                        // Holes
                        for (i, hole) in editor.holes.enumerated() {
                            let sp = toScreen(hole)
                            let r = CGFloat(editor.holeRadius) * scale
                            context.fill(Path(ellipseIn: CGRect(x: sp.x - r, y: sp.y - r, width: r*2, height: r*2)),
                                         with: .color(.white.opacity(0.15)))
                            context.stroke(Path(ellipseIn: CGRect(x: sp.x - r, y: sp.y - r, width: r*2, height: r*2)),
                                           with: .color(editor.ropeStartHole == i ? .cyan : .white), lineWidth: 2)
                            // Index label
                            let labelRect = CGRect(x: sp.x - 8, y: sp.y - 6, width: 16, height: 12)
                            context.draw(Text("\(i)").font(.system(size: 8)).foregroundColor(.white), in: labelRect)
                        }
                    }
                }
                .contentShape(Rectangle())
                .gesture(
                    DragGesture(minimumDistance: 0)
                        .onEnded { value in
                            let scale = min(size.width, size.height) / CGFloat(worldExtent * 2)
                            let cx = size.width / 2
                            let cy = size.height / 2
                            let worldX = Float((value.location.x - cx) / scale)
                            let worldY = Float(-(value.location.y - cy) / scale)
                            let pos = SIMD2<Float>(worldX, worldY)

                            handleTap(pos: pos)
                        }
                )
            }

            // Status bar
            HStack {
                Text("Holes: \(editor.holes.count)")
                Text("Rails: \(editor.rails.count)")
                Text("Ropes: \(editor.ropes.count)")
                Text("Carts: \(editor.carts.count)")
                Text("Stations: \(editor.stations.count)")
                Spacer()
                Text("ID: \(levelId)")
                Stepper("", value: $levelId, in: 2001...9999)
                    .labelsHidden()
                    .frame(width: 80)
            }
            .font(.system(size: 11, design: .monospaced))
            .foregroundColor(.white)
            .padding(.horizontal, 12)
            .padding(.vertical, 4)
            .background(Color.black.opacity(0.8))
        }
        .sheet(isPresented: $showExport) {
            ExportSheet(json: exportedJSON)
        }
        .sheet(isPresented: $showImport) {
            ImportSheet(json: $importJSON, error: $importError) {
                loadFromJSON(importJSON)
            }
        }
    }

    private func loadFromJSON(_ json: String) {
        guard let data = json.data(using: .utf8) else {
            importError = "Invalid text"
            return
        }
        do {
            let def = try JSONDecoder().decode(LevelDefinition.self, from: data)
            editor.reset()
            editor.holeRadius = def.holeRadius
            editor.particlesPerRope = def.particlesPerRope
            editor.holes = def.holes.map { SIMD2<Float>($0.xPosition, $0.yPosition) }
            for rope in def.ropes {
                editor.ropes.append(EditorRope(startHole: rope.startHole, endHole: rope.endHole, colorIndex: editor.nextColorIndex))
                editor.nextColorIndex += 1
            }
            if let rails = def.rails {
                editor.rails = rails.map { r in
                    EditorRail(points: r.points.map { SIMD2<Float>($0.xPosition, $0.yPosition) })
                }
            }
            if let carts = def.carts {
                editor.carts = carts.map { c in
                    EditorCart(railIndex: c.railIndex, t: c.startT, radius: c.radius ?? 0.12, mass: c.mass ?? 0.5)
                }
            }
            if let stations = def.stations {
                editor.stations = stations.map { s in
                    EditorStation(railIndex: s.railIndex, t: s.t, radius: s.radius ?? 0.15, cartIndex: s.cartIndex)
                }
            }
            levelId = def.id
            showImport = false
        } catch {
            importError = error.localizedDescription
        }
    }

    private func handleTap(pos: SIMD2<Float>) {
        switch editor.currentTool {
        case .hole:
            editor.holes.append(pos)

        case .railPoint:
            if let ri = editor.currentRailIndex {
                editor.rails[ri].points.append(pos)
            }

        case .rope:
            if let startHole = editor.ropeStartHole {
                // Second tap — find nearest hole for end
                if let endHole = editor.nearestHole(to: pos, maxDist: 0.2), endHole != startHole {
                    editor.ropes.append(EditorRope(startHole: startHole, endHole: endHole, colorIndex: editor.nextColorIndex))
                    editor.nextColorIndex += 1
                }
                editor.ropeStartHole = nil
            } else {
                // First tap — find nearest hole for start
                if let hole = editor.nearestHole(to: pos, maxDist: 0.2) {
                    editor.ropeStartHole = hole
                }
            }

        case .cart:
            if let hit = editor.nearestRailT(to: pos) {
                editor.carts.append(EditorCart(railIndex: hit.railIndex, t: hit.t))
            }

        case .station:
            if let hit = editor.nearestRailT(to: pos) {
                let cartIdx = editor.carts.isEmpty ? 0 : editor.carts.count - 1
                editor.stations.append(EditorStation(railIndex: hit.railIndex, t: hit.t, cartIndex: cartIdx))
            }

        case .move:
            break // TODO: drag to move

        case .delete:
            // Delete nearest hole
            if let hole = editor.nearestHole(to: pos, maxDist: 0.15) {
                // Remove ropes referencing this hole
                editor.ropes.removeAll { $0.startHole == hole || $0.endHole == hole }
                // Fix rope indices
                for i in editor.ropes.indices {
                    if editor.ropes[i].startHole > hole { editor.ropes[i].startHole -= 1 }
                    if editor.ropes[i].endHole > hole { editor.ropes[i].endHole -= 1 }
                }
                editor.holes.remove(at: hole)
            }
        }
    }
}

struct ImportSheet: View {
    @Binding var json: String
    @Binding var error: String?
    @Environment(\.dismiss) var dismiss
    var onImport: () -> Void

    var body: some View {
        VStack(spacing: 12) {
            HStack {
                Text("Import JSON")
                    .font(.headline)
                Spacer()
                Button("Cancel") { dismiss() }
            }

            if let error = error {
                Text(error)
                    .foregroundColor(.red)
                    .font(.caption)
            }

            TextEditor(text: $json)
                .font(.system(size: 11, design: .monospaced))
                .border(Color.gray.opacity(0.3))
                .frame(minHeight: 200)

            HStack {
                Button("Paste from Clipboard") {
                    #if os(iOS)
                    json = UIPasteboard.general.string ?? ""
                    #elseif os(macOS)
                    json = NSPasteboard.general.string(forType: .string) ?? ""
                    #endif
                }
                Spacer()
                Button("Load") { onImport() }
                    .buttonStyle(.borderedProminent)
            }
        }
        .padding()
        .frame(minWidth: 500, minHeight: 350)
    }
}

struct ExportSheet: View {
    let json: String
    @Environment(\.dismiss) var dismiss

    var body: some View {
        NavigationView {
            ScrollView {
                Text(json)
                    .font(.system(size: 10, design: .monospaced))
                    .padding()
                    .textSelection(.enabled)
            }
            .navigationTitle("Level JSON")
            .toolbar {
                ToolbarItem(placement: .confirmationAction) {
                    Button("Copy") {
                        #if os(iOS)
                        UIPasteboard.general.string = json
                        #elseif os(macOS)
                        NSPasteboard.general.clearContents()
                        NSPasteboard.general.setString(json, forType: .string)
                        #endif
                        dismiss()
                    }
                }
                ToolbarItem(placement: .cancellationAction) {
                    Button("Close") { dismiss() }
                }
            }
        }
    }
}
