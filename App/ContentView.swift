import SwiftUI

enum GameMode: String, CaseIterable {
    case untangle = "Untangle"
    case tension = "Tension"
    case rail = "Rail"
}

struct ContentView: View {
    @StateObject private var gameController = GameController()
    @State private var showControls = false
    @State private var selectedTab = 0
    @State private var showLevelPicker = false
    @State private var levelInput = ""
    @State private var dumpMessage: String?
    @State private var settingsCopied = false
    @State private var settingsImported: Bool? = nil
    @State private var shareURL: URL?
    @State private var usernameInput = ""
    @State private var loginAsInput = ""
    @State private var gameMode: GameMode = .untangle
    @State private var showEditor = false

    @StateObject private var editorState = EditorState()

    var body: some View {
        ZStack {
            if showEditor {
                GeometryReader { geo in
                    let isLandscape = geo.size.width > geo.size.height
                    let editorFraction: CGFloat = isLandscape ? 0.4 : 0.45
                    Group {
                        if isLandscape {
                            HStack(spacing: 0) {
                                LevelEditorView(editor: editorState) { def in
                                    gameController.loadLevelDefinition(def)
                                }
                                .frame(width: geo.size.width * editorFraction)

                                gameFieldWithHUD
                            }
                        } else {
                            VStack(spacing: 0) {
                                LevelEditorView(editor: editorState) { def in
                                    gameController.loadLevelDefinition(def)
                                }
                                .frame(height: geo.size.height * 0.5)

                                gameFieldWithHUD
                            }
                        }
                    }
                }
                .ignoresSafeArea()
            } else {
                GameView(controller: gameController)
                    .ignoresSafeArea()
            }

            if !gameController.showLevelComplete {
            VStack(spacing: 0) {
                HStack(spacing: 8) {
                    Button(action: {
                        gameController.restartLevel()
                    }) {
                        Image(systemName: "arrow.counterclockwise")
                            .font(.system(size: 18, weight: .semibold))
                            .foregroundColor(.white)
                            .frame(width: 36, height: 36)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    .padding(.leading, 16)

                    Button(action: {
                        gameController.resetCamera()
                    }) {
                        Image(systemName: "scope")
                            .font(.system(size: 18, weight: .semibold))
                            .foregroundColor(.white)
                            .frame(width: 36, height: 36)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }

                    Button(action: {
                        if gameController.currentLevel > 1 {
                            gameController.loadLevel(gameController.currentLevel - 1)
                        }
                    }) {
                        Image(systemName: "chevron.left")
                            .font(.system(size: 18, weight: .semibold))
                            .foregroundColor(gameController.currentLevel > 1 ? .white : .white.opacity(0.3))
                            .frame(width: 36, height: 36)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    .disabled(gameController.currentLevel <= 1)

                    Button(action: {
                        levelInput = "\(gameController.currentLevel)"
                        showLevelPicker = true
                    }) {
                        VStack(alignment: .leading, spacing: 2) {
                            Text("\(gameMode == .rail ? "R" : gameMode == .tension ? "T" : "L") \(gameController.currentLevel)")
                                .font(.system(size: 18, weight: .bold, design: .rounded))
                                .foregroundColor(.white)
                            Text("\(Int(gameController.fps)) fps")
                                .font(.system(size: 11, design: .monospaced))
                                .foregroundColor(.white.opacity(0.4))
                        }
                    }

                    Menu {
                        ForEach(GameMode.allCases, id: \.self) { mode in
                            Button(action: {
                                gameMode = mode
                                switch mode {
                                case .tension: gameController.loadLevel(1001)
                                case .rail: gameController.loadLevel(2001)
                                case .untangle: gameController.loadLevel(1)
                                }
                            }) {
                                Label(mode.rawValue, systemImage: mode == gameMode ? "checkmark.circle.fill" : "circle")
                            }
                        }
                        Divider()
                        Button(action: { showEditor = true }) {
                            Label("Level Editor", systemImage: "pencil.and.ruler")
                        }
                    } label: {
                        Image(systemName: gameMode == .rail ? "tram" : gameMode == .tension ? "arrow.up.and.down.and.sparkles" : "puzzlepiece")
                            .font(.system(size: 16, weight: .semibold))
                            .foregroundColor(.white)
                            .frame(width: 36, height: 36)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }

                    Button(action: {
                        if gameController.currentLevel < 200 {
                            gameController.loadLevel(gameController.currentLevel + 1)
                        }
                    }) {
                        Image(systemName: "chevron.right")
                            .font(.system(size: 18, weight: .semibold))
                            .foregroundColor(gameController.currentLevel < 200 ? .white : .white.opacity(0.3))
                            .frame(width: 36, height: 36)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    .disabled(gameController.currentLevel >= 200)

                    Spacer()

                    Button(action: {
                        withAnimation(.easeInOut(duration: 0.2)) { showControls.toggle() }
                    }) {
                        Image(systemName: "slider.horizontal.3")
                            .font(.system(size: 20, weight: .semibold))
                            .foregroundColor(.white)
                            .frame(width: 44, height: 44)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    .padding(.trailing, 8)

                    Button(action: {
                        gameController.profilerActive.toggle()
                    }) {
                        Image(systemName: gameController.profilerActive ? "gauge.open.with.lines.needle.84percent.exclamation" : "gauge.open.with.lines.needle.33percent")
                            .font(.system(size: 20, weight: .semibold))
                            .foregroundColor(gameController.profilerActive ? .red : .white)
                            .frame(width: 44, height: 44)
                            .background(gameController.profilerActive ? Color.red.opacity(0.3) : Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    .padding(.trailing, 16)
                }
                .padding(.top, 8)

                if showControls {
                    VStack(spacing: 0) {
                        HStack {
                            Text("Physics")
                                .font(.system(size: 14, weight: .semibold))
                                .foregroundColor(.white)
                            Spacer()
                            Button(action: {
                                if gameController.dumpSettingsToClipboard() {
                                    settingsCopied = true
                                    DispatchQueue.main.asyncAfter(deadline: .now() + 1.5) { settingsCopied = false }
                                }
                            }) {
                                Image(systemName: "doc.on.clipboard")
                                    .font(.system(size: 13, weight: .bold))
                                    .foregroundColor(settingsCopied ? .green : .white.opacity(0.8))
                                    .frame(width: 30, height: 30)
                                    .background(Color.white.opacity(0.12))
                                    .clipShape(Circle())
                            }
                            Button(action: {
                                let ok = gameController.importSettingsFromClipboard()
                                settingsImported = ok
                                DispatchQueue.main.asyncAfter(deadline: .now() + 1.5) { settingsImported = nil }
                            }) {
                                Image(systemName: "clipboard.fill")
                                    .font(.system(size: 13, weight: .bold))
                                    .foregroundColor(settingsImported == true ? .green : settingsImported == false ? .red : .white.opacity(0.8))
                                    .frame(width: 30, height: 30)
                                    .background(Color.white.opacity(0.12))
                                    .clipShape(Circle())
                            }
                            Button(action: { gameController.resetToDefaults() }) {
                                Image(systemName: "arrow.uturn.backward")
                                    .font(.system(size: 13, weight: .bold))
                                    .foregroundColor(.white.opacity(0.8))
                                    .frame(width: 30, height: 30)
                                    .background(Color.white.opacity(0.12))
                                    .clipShape(Circle())
                            }
                        }
                        .padding(.horizontal, 10)
                        .padding(.top, 8)
                        .padding(.bottom, 4)

                        VStack(spacing: 2) {
                            HStack(spacing: 0) {
                                TabButton(title: "Rope", index: 0, selected: $selectedTab)
                                TabButton(title: "Solver", index: 1, selected: $selectedTab)
                                TabButton(title: "Drag", index: 2, selected: $selectedTab)
                                TabButton(title: "Visual", index: 3, selected: $selectedTab)
                                TabButton(title: "Cartoon", index: 4, selected: $selectedTab)
                            }
                            HStack(spacing: 0) {
                                TabButton(title: "Light", index: 5, selected: $selectedTab)
                                TabButton(title: "Table", index: 6, selected: $selectedTab)
                                TabButton(title: "Matte", index: 7, selected: $selectedTab)
                                TabButton(title: "Cap", index: 8, selected: $selectedTab)
                                TabButton(title: "Worm", index: 9, selected: $selectedTab)
                            }
                            HStack(spacing: 0) {
                                TabButton(title: "Player", index: 10, selected: $selectedTab)
                            }
                        }
                        .padding(.horizontal, 10)
                        .padding(.bottom, 6)

                        // Tab content
                        Group {
                            switch selectedTab {
                            case 0: ropeTab
                            case 1: solverTab
                            case 2: dragTab
                            case 3: visualTab
                            case 4: cartoonTab
                            case 5: lightTab
                            case 6: tableTab
                            case 7: matteTab
                            case 8: capTab
                            case 9: wormTab
                            case 10: playerTab
                            default: ropeTab
                            }
                        }
                        .padding(.horizontal, 10)
                        .padding(.bottom, 10)
                    }
                    .frame(maxHeight: 320)
                    .background(Color.black.opacity(0.5))
                    .cornerRadius(14)
                    .padding(.horizontal, 12)
                    .padding(.top, 6)
                }

                Spacer()
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .top)
            .ignoresSafeArea(edges: .horizontal)

            VStack {
                Spacer()
                HStack(alignment: .bottom) {
                    Button(action: {
                        gameController.undo()
                    }) {
                        Image(systemName: "arrow.uturn.backward")
                            .font(.system(size: 20, weight: .semibold))
                            .foregroundColor(gameController.canUndo ? .white : .white.opacity(0.3))
                            .frame(width: 52, height: 52)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    .disabled(!gameController.canUndo)
                    #if os(macOS)
                    .keyboardShortcut("z", modifiers: .command)
                    #endif
                    .padding(.leading, 20)
                    .padding(.bottom, 40)

                    Spacer()

                    Text("\(gameController.moveCount)")
                        .font(.system(size: 15, weight: .medium, design: .monospaced))
                        .foregroundColor(.white.opacity(0.35))
                        .padding(.trailing, 20)
                        .padding(.bottom, 48)
                }
            }

            if gameController.profilerActive {
                VStack {
                    Spacer()
                    Text(gameController.profilerSummary)
                        .font(.system(size: 10, design: .monospaced))
                        .foregroundColor(.green)
                        .padding(8)
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .background(Color.black.opacity(0.75))
                }
                .ignoresSafeArea()
            }
            } // end if !showLevelComplete

            if gameController.showLevelComplete {
                Color.black.opacity(0.4)
                    .ignoresSafeArea()
                    .transition(.opacity)

                VictoryOverlay(
                    level: gameController.currentLevel,
                    starCount: gameController.starCount,
                    percentile: gameController.percentile
                ) {
                    let next = gameController.currentLevel + 1
                    gameController.showLevelComplete = false
                    gameController.loadLevel(next)
                }
                .transition(.scale(scale: 0.7).combined(with: .opacity))
            }
        }
        .animation(.spring(response: 0.5, dampingFraction: 0.75), value: gameController.showLevelComplete)
        .overlay(alignment: .bottom) {
            if let msg = dumpMessage {
                Text("Saved: \(msg)")
                    .font(.system(size: 13, weight: .medium, design: .monospaced))
                    .foregroundColor(.white)
                    .padding(.horizontal, 16)
                    .padding(.vertical, 10)
                    .background(Color.black.opacity(0.7))
                    .cornerRadius(10)
                    .padding(.bottom, 60)
                    .transition(.move(edge: .bottom).combined(with: .opacity))
                    .onAppear {
                        DispatchQueue.main.asyncAfter(deadline: .now() + 2) {
                            withAnimation { dumpMessage = nil }
                        }
                    }
            }
        }
        .animation(.easeInOut(duration: 0.3), value: dumpMessage)
        .alert("Go to Level", isPresented: $showLevelPicker) {
            TextField("Level number", text: $levelInput)
                #if os(iOS)
                .keyboardType(.numberPad)
                #endif
            Button("Go") {
                if let id = Int(levelInput), id >= 1 {
                    gameController.loadLevel(id)
                }
            }
            Button("Cancel", role: .cancel) {}
        }
    }

    /// Game view + compact HUD for editor split mode
    private var gameFieldWithHUD: some View {
        ZStack {
            GameView(controller: gameController)

            VStack {
                HStack(spacing: 6) {
                    Button { gameController.restartLevel() } label: {
                        Image(systemName: "arrow.counterclockwise")
                            .font(.system(size: 14, weight: .semibold))
                            .foregroundColor(.white)
                            .frame(width: 30, height: 30)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    Button { gameController.resetCamera() } label: {
                        Image(systemName: "scope")
                            .font(.system(size: 14, weight: .semibold))
                            .foregroundColor(.white)
                            .frame(width: 30, height: 30)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    Button { gameController.undo() } label: {
                        Image(systemName: "arrow.uturn.backward")
                            .font(.system(size: 14, weight: .semibold))
                            .foregroundColor(gameController.canUndo ? .white : .white.opacity(0.3))
                            .frame(width: 30, height: 30)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    .disabled(!gameController.canUndo)

                    Spacer()

                    Text("\(gameController.moveCount)")
                        .font(.system(size: 13, weight: .medium, design: .monospaced))
                        .foregroundColor(.white.opacity(0.5))
                }
                .padding(.horizontal, 8)
                .padding(.top, 6)

                Spacer()
            }
        }
    }

    private var ropeTab: some View {
        VStack(spacing: 3) {
            ParamRowInt(label: "Particles", value: $gameController.particleCount, range: 6...200, defaultValue: 60)
            ParamRow(label: "Gravity", value: $gameController.gravity, range: -20.0...0.0, format: "%.1f", defaultValue: -5.0)
            ParamRow(label: "Damping", value: $gameController.damping, range: 0.8...1.0, format: "%.3f", defaultValue: 0.97)
            ParamRow(label: "Tension", value: $gameController.ropeTension, range: 0.50...1.0, format: "%.3f", defaultValue: 0.98)
        }
    }

    private var solverTab: some View {
        VStack(spacing: 3) {
            ParamRowInt(label: "Constr Iter", value: $gameController.constraintIterations, range: 1...60, defaultValue: 8)
            ParamRowInt(label: "Settle Steps", value: $gameController.settleSteps, range: 1...100, defaultValue: 5)
            ParamRow(label: "Bend Comp", value: $gameController.bendCompliance, range: 0.0...0.01, format: "%.4f", defaultValue: 0.0015)
            ParamRow(label: "Bend Damp", value: $gameController.bendVelocityCoupling, range: 0.0...1.0, format: "%.2f", defaultValue: 0.45)
        }
    }

    private var dragTab: some View {
        VStack(spacing: 3) {
            ParamRow(label: "Drag H", value: $gameController.dragHeight, range: 0.05...1.5, format: "%.3f", defaultValue: 0.35)
            ParamRow(label: "Lift H", value: $gameController.liftHeight, range: 0.05...1.5, format: "%.3f", defaultValue: 0.30)
            ParamRow(label: "Board Z", value: $gameController.boardElevation, range: 0.02...0.5, format: "%.3f", defaultValue: 0.12)
        }
    }

    private var visualTab: some View {
        ScrollView(.vertical, showsIndicators: false) {
            VStack(spacing: 3) {
                HStack(spacing: 6) {
                    Text("Square")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                        .frame(width: 70, alignment: .leading)
                    Toggle("", isOn: $gameController.squareCrossSection)
                        .labelsHidden()
                    ResetButton { gameController.squareCrossSection = false }
                }
                ParamRowInt(label: "Profile", value: $gameController.profileSegments, range: 3...32, defaultValue: 16)
                ParamRow(label: "Hole Size", value: $gameController.holeRadiusScale, range: 0.5...2.0, format: "%.2f", defaultValue: 1.0)
                ParamRowInt(label: "Hole Seg", value: $gameController.holeSegments, range: 12...96, defaultValue: 48)
                ParamRow(label: "Rope Scale", value: $gameController.ropeRadiusScale, range: 0.5...2.0, format: "%.2f", defaultValue: 1.0)
                ParamRow(label: "Stretch Thin", value: $gameController.stretchThinning, range: 0.0...1.0, format: "%.2f", defaultValue: 0.5)
                ParamRow(label: "Hole R", value: $gameController.holeTintR, range: 0...1, format: "%.2f", defaultValue: 1.0)
                ParamRow(label: "Hole G", value: $gameController.holeTintG, range: 0...1, format: "%.2f", defaultValue: 1.0)
                ParamRow(label: "Hole B", value: $gameController.holeTintB, range: 0...1, format: "%.2f", defaultValue: 1.0)
                ParamRow(label: "Tint Amt", value: $gameController.holeTintAmount, range: 0...1, format: "%.2f", defaultValue: 0)
                ParamRow(label: "Exposure", value: $gameController.exposure, range: 0.5...2.5, format: "%.2f", defaultValue: 1.05)
                ParamRow(label: "Bloom", value: $gameController.bloomStrength, range: 0...2.0, format: "%.2f", defaultValue: 0.35)
                ParamRow(label: "Render Scale", value: $gameController.renderScale, range: 0.25...2.0, format: "%.2f", defaultValue: 1.0)
            }
        }
        .frame(maxHeight: 220)
    }

    private var cartoonTab: some View {
        ScrollView(.vertical, showsIndicators: false) {
            VStack(spacing: 3) {
                HStack(spacing: 6) {
                    Text("Enabled")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                        .frame(width: 70, alignment: .leading)
                    Toggle("", isOn: $gameController.cartoonShaderEnabled)
                        .labelsHidden()
                    ResetButton { gameController.cartoonShaderEnabled = false }
                }
                ParamRow(label: "Exposure", value: $gameController.cartoonExposure, range: 0.5...1.5, format: "%.2f", defaultValue: 0.75)
                ParamRow(label: "Bloom", value: $gameController.cartoonBloom, range: 0...0.3, format: "%.2f", defaultValue: 0)
                ParamRow(label: "Edge", value: $gameController.cartoonEdgeStrength, range: 0...1, format: "%.2f", defaultValue: 0.88)
                ParamRowInt(label: "Levels", value: $gameController.cartoonLevels, range: 2...6, defaultValue: 4)
                ParamRow(label: "Shadow", value: $gameController.cartoonShadowBright, range: 0.1...0.8, format: "%.2f", defaultValue: 0.38)
                ParamRow(label: "Wrap", value: $gameController.cartoonWrap, range: 0...0.5, format: "%.2f", defaultValue: 0.15)
                ParamRow(label: "Edge Smooth", value: $gameController.cartoonEdgeSmooth, range: 0...1, format: "%.2f", defaultValue: 0.5)
            }
        }
        .frame(maxHeight: 280)
    }

    private var lightTab: some View {
        ScrollView(.vertical, showsIndicators: false) {
            VStack(spacing: 3) {
                HStack(spacing: 6) {
                    Text("Shadow")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                        .frame(width: 70, alignment: .leading)
                    Picker("", selection: $gameController.shadowType) {
                        ForEach(GameController.ShadowType.allCases, id: \.rawValue) { t in
                            Text(t.label).foregroundColor(.primary).tag(t)
                        }
                    }
                    .pickerStyle(.menu)
                    .tint(.white)
                    ResetButton { gameController.shadowType = .pcss }
                }
                HStack(spacing: 6) {
                    Text("Shadows")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                        .frame(width: 70, alignment: .leading)
                    Toggle("", isOn: $gameController.shadowsEnabled)
                        .labelsHidden()
                    ResetButton { gameController.shadowsEnabled = true }
                }
                ParamRow(label: "Intensity", value: $gameController.lightIntensity, range: 0.1...3.0, format: "%.2f", defaultValue: 1.0)
                ParamRow(label: "Light X", value: $gameController.lightDirX, range: -1...1, format: "%.2f", defaultValue: -0.65)
                ParamRow(label: "Light Y", value: $gameController.lightDirY, range: -1...1, format: "%.2f", defaultValue: -0.35)
                ParamRow(label: "Light Z", value: $gameController.lightDirZ, range: -1...1, format: "%.2f", defaultValue: 0.67)
                ParamRow(label: "Ambient", value: $gameController.ambient, range: 0...0.6, format: "%.2f", defaultValue: 0.08)
                ParamRow(label: "Shadow Bias", value: $gameController.shadowBias, range: 0.0001...0.005, format: "%.4f", defaultValue: 0.0012)
                Picker("Shadow Dbg", selection: $gameController.shadowDebugMode) {
                    Text("Off").tag(0)
                    Text("Shadow").tag(1)
                    Text("StoredZ").tag(2)
                    Text("FragZ").tag(3)
                    Text("Diff").tag(4)
                }.pickerStyle(.menu).font(.caption2)
                ParamRow(label: "Shadow Dark", value: $gameController.shadowDarkness, range: 0...0.5, format: "%.2f", defaultValue: 0.12)
                ParamRow(label: "Light Size", value: $gameController.lightSize, range: 0.002...0.08, format: "%.3f", defaultValue: 0.012)
            }
        }
        .frame(maxHeight: 260)
    }

    private var tableTab: some View {
        ScrollView(.vertical, showsIndicators: false) {
            VStack(spacing: 3) {
                HStack(spacing: 6) {
                    Text("Style")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                        .frame(width: 70, alignment: .leading)
                    Picker("", selection: $gameController.tableStyle) {
                        ForEach(GameController.TableStyle.allCases, id: \.rawValue) { s in
                            Text(s.label).foregroundColor(.primary).tag(s)
                        }
                    }
                    .pickerStyle(.menu)
                    .tint(.white)
                    ResetButton { gameController.tableStyle = .wood }
                }
                if gameController.tableStyle != .wood {
                    ParamRow(label: "Color1 R", value: $gameController.tableColor1R, range: 0...1, format: "%.2f", defaultValue: 0.08)
                    ParamRow(label: "Color1 G", value: $gameController.tableColor1G, range: 0...1, format: "%.2f", defaultValue: 0.09)
                    ParamRow(label: "Color1 B", value: $gameController.tableColor1B, range: 0...1, format: "%.2f", defaultValue: 0.13)
                    if gameController.tableStyle == .gradient {
                        ParamRow(label: "Color2 R", value: $gameController.tableColor2R, range: 0...1, format: "%.2f", defaultValue: 0.12)
                        ParamRow(label: "Color2 G", value: $gameController.tableColor2G, range: 0...1, format: "%.2f", defaultValue: 0.13)
                        ParamRow(label: "Color2 B", value: $gameController.tableColor2B, range: 0...1, format: "%.2f", defaultValue: 0.20)
                    }
                }
                ParamRow(label: "Wood Seed", value: $gameController.woodSeed, range: 0...1, format: "%.2f", defaultValue: 0)
                ParamRow(label: "Brightness", value: $gameController.woodBrightness, range: 0.5...1.5, format: "%.2f", defaultValue: 1.0)
                ParamRow(label: "Pattern Scale", value: $gameController.woodPatternScale, range: 0.5...7.0, format: "%.1f", defaultValue: 3.0)
            }
        }
        .frame(maxHeight: 220)
    }

    private var matteTab: some View {
        ScrollView(.vertical, showsIndicators: false) {
            VStack(spacing: 3) {
                ParamRow(label: "Matte", value: $gameController.ropeMatte, range: 0...1, format: "%.2f", defaultValue: 0.6)
                ParamRow(label: "Gloss", value: $gameController.ropeGloss, range: 0...2, format: "%.2f", defaultValue: 0.5)
                ParamRow(label: "Diff Wrap", value: $gameController.ropeDiffuseWrap, range: 0...1, format: "%.2f", defaultValue: 0.3)
                ParamRow(label: "Subsurface", value: $gameController.ropeSubsurface, range: 0...1, format: "%.2f", defaultValue: 0.4)
                ParamRow(label: "Edge Light", value: $gameController.ropeEdgeLight, range: 0...0.5, format: "%.2f", defaultValue: 0.15)
                ParamRow(label: "Saturation", value: $gameController.ropeSaturation, range: 0...2, format: "%.2f", defaultValue: 1.0)
                ParamRow(label: "Micro Bump", value: $gameController.ropeMicroBump, range: 0...1.5, format: "%.3f", defaultValue: 0.14)
                ParamRow(label: "Bump Scale", value: $gameController.ropeBumpScale, range: 0.5...20.0, format: "%.1f", defaultValue: 3.0)
                ParamRow(label: "Contact AO", value: $gameController.ropeContactAO, range: 0...1, format: "%.2f", defaultValue: 0.35)
                ParamRow(label: "Lift Glow", value: $gameController.ropeLiftGlow, range: 0...1, format: "%.2f", defaultValue: 0.25)
                ParamRow(label: "Str Gloss", value: $gameController.ropeStretchGloss, range: 0...1, format: "%.2f", defaultValue: 0.7)
                ParamRow(label: "Str Spec", value: $gameController.ropeStretchSpec, range: 0...2, format: "%.2f", defaultValue: 1.0)
                ParamRow(label: "Reflection", value: $gameController.ropeEnvReflect, range: 0...3, format: "%.2f", defaultValue: 0.15)
                ParamRow(label: "Refl Spread", value: $gameController.ropeEnvSpread, range: 0.01...0.5, format: "%.3f", defaultValue: 0.15)
                Toggle("Env Debug", isOn: $gameController.ropeEnvDebug).font(.caption2)
            }
        }
        .frame(maxHeight: 260)
    }

    private var capTab: some View {
        ScrollView(.vertical, showsIndicators: false) {
            VStack(spacing: 3) {
                ParamRow(label: "Radius Scale", value: $gameController.capRadiusScale, range: 0.3...2.5, format: "%.2f", defaultValue: 1.0)
                ParamRowInt(label: "Segments", value: $gameController.capSegments, range: 4...48, defaultValue: 12)
                ParamRowInt(label: "Rings", value: $gameController.capRings, range: 2...16, defaultValue: 6)
                ParamRow(label: "Darken", value: $gameController.capDarken, range: 0...1, format: "%.2f", defaultValue: 0.7)
            }
        }
        .frame(maxHeight: 180)
    }

    private var wormTab: some View {
        ScrollView(.vertical, showsIndicators: false) {
            VStack(spacing: 3) {
                HStack(spacing: 6) {
                    Text("Enabled")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                        .frame(width: 70, alignment: .leading)
                    Toggle("", isOn: $gameController.wormMode)
                        .labelsHidden()
                    ResetButton { gameController.wormMode = false }
                }
                ParamRow(label: "Seg Freq", value: $gameController.wormSegFreq, range: 4...80, format: "%.0f", defaultValue: 28)
                ParamRow(label: "Seg Bulge", value: $gameController.wormSegBulge, range: 0...0.5, format: "%.3f", defaultValue: 0.12)
                ParamRow(label: "Thickness", value: $gameController.wormThickness, range: 0.5...3.0, format: "%.2f", defaultValue: 1.35)
                ParamRow(label: "Taper Len", value: $gameController.wormTaperLen, range: 0.02...0.4, format: "%.3f", defaultValue: 0.12)
                ParamRow(label: "Groove", value: $gameController.wormGrooveDepth, range: 0...1, format: "%.2f", defaultValue: 0.35)
                ParamRow(label: "Belly", value: $gameController.wormBellyBright, range: 0.5...2.0, format: "%.2f", defaultValue: 1.15)
                ParamRow(label: "Back Dark", value: $gameController.wormBackDark, range: 0.2...1.5, format: "%.2f", defaultValue: 0.7)
                ParamRow(label: "Skin Noise", value: $gameController.wormSkinNoise, range: 0...0.3, format: "%.3f", defaultValue: 0.08)
                ParamRow(label: "SSS", value: $gameController.wormSSS, range: 0...1, format: "%.2f", defaultValue: 0.25)
                ParamRow(label: "Roughness", value: $gameController.wormRoughness, range: 0.05...1, format: "%.2f", defaultValue: 0.25)
                ParamRow(label: "Specular", value: $gameController.wormSpecular, range: 0...2, format: "%.2f", defaultValue: 0.8)
                ParamRow(label: "Rim", value: $gameController.wormRimStrength, range: 0...0.5, format: "%.3f", defaultValue: 0.08)
                ParamRow(label: "Eye Size", value: $gameController.wormEyeSize, range: 0...0.06, format: "%.4f", defaultValue: 0.015)
                ParamRow(label: "Pulse Spd", value: $gameController.wormPulseSpeed, range: 0...10, format: "%.1f", defaultValue: 2.5)
                ParamRow(label: "Pulse Amp", value: $gameController.wormPulseAmp, range: 0...0.1, format: "%.3f", defaultValue: 0.02)
                ParamRow(label: "Crawl Spd", value: $gameController.wormCrawlSpeed, range: 0...15, format: "%.1f", defaultValue: 3.5)
                ParamRow(label: "Crawl Amp", value: $gameController.wormCrawlAmp, range: 0...0.05, format: "%.4f", defaultValue: 0.012)
                ParamRow(label: "Side Amp", value: $gameController.wormSideAmp, range: 0...0.03, format: "%.4f", defaultValue: 0.008)
            }
        }
        .frame(maxHeight: 280)
    }

    private var playerTab: some View {
        VStack(spacing: 8) {
            HStack(spacing: 6) {
                Text("Name")
                    .font(.system(size: 12))
                    .foregroundColor(.white.opacity(0.8))
                    .frame(width: 50, alignment: .leading)
                TextField("username", text: $usernameInput)
                    .textFieldStyle(.roundedBorder)
                    .font(.system(size: 13, design: .monospaced))
                    .frame(maxWidth: .infinity)
                    .onAppear { usernameInput = gameController.leaderboardUsername }
                Button("Save") {
                    let name = usernameInput.trimmingCharacters(in: .whitespaces)
                    guard !name.isEmpty else { return }
                    gameController.updateLeaderboardUsername(name)
                }
                .font(.system(size: 12, weight: .semibold))
                .foregroundColor(.white)
                .padding(.horizontal, 10)
                .padding(.vertical, 5)
                .background(Color.white.opacity(0.2))
                .cornerRadius(6)
            }

            HStack(spacing: 6) {
                Text("Login")
                    .font(.system(size: 12))
                    .foregroundColor(.white.opacity(0.8))
                    .frame(width: 50, alignment: .leading)
                TextField("play as...", text: $loginAsInput)
                    .textFieldStyle(.roundedBorder)
                    .font(.system(size: 13, design: .monospaced))
                    .frame(maxWidth: .infinity)
                Button("Go") {
                    let name = loginAsInput.trimmingCharacters(in: .whitespaces)
                    guard !name.isEmpty else { return }
                    gameController.loginAsPlayer(name)
                    usernameInput = name
                    loginAsInput = ""
                }
                .font(.system(size: 12, weight: .semibold))
                .foregroundColor(.white)
                .padding(.horizontal, 10)
                .padding(.vertical, 5)
                .background(Color.white.opacity(0.2))
                .cornerRadius(6)
            }

            if !gameController.leaderboardUsername.isEmpty {
                Text("Playing as: \(gameController.leaderboardUsername)")
                    .font(.system(size: 12, design: .rounded))
                    .foregroundColor(.white.opacity(0.5))
            }
        }
    }
}

private struct ResetButton: View {
    let action: () -> Void

    var body: some View {
        Button(action: action) {
            Image(systemName: "arrow.counterclockwise")
                .font(.system(size: 9, weight: .bold))
                .foregroundColor(.white.opacity(0.7))
                .frame(width: 22, height: 22)
                .background(Color.white.opacity(0.12))
                .clipShape(Circle())
        }
    }
}

private struct TabButton: View {
    let title: String
    let index: Int
    @Binding var selected: Int

    var body: some View {
        Button(action: { selected = index }) {
            Text(title)
                .font(.system(size: 12, weight: selected == index ? .bold : .regular))
                .foregroundColor(selected == index ? .white : .white.opacity(0.5))
                .frame(maxWidth: .infinity)
                .padding(.vertical, 6)
                .background(selected == index ? Color.white.opacity(0.15) : Color.clear)
                .cornerRadius(8)
        }
    }
}

private struct ParamRow: View {
    let label: String
    @Binding var value: Float
    let range: ClosedRange<Float>
    let format: String
    var defaultValue: Float? = nil

    @State private var textValue: String = ""
    @FocusState private var isEditing: Bool

    var body: some View {
        HStack(spacing: 6) {
            Text(label)
                .font(.system(size: 12))
                .foregroundColor(.white.opacity(0.8))
                .frame(width: 70, alignment: .leading)

            Slider(value: $value, in: range)
                .onChange(of: value) { _, newVal in
                    if !isEditing {
                        textValue = String(format: format, newVal)
                    }
                }

            TextField("", text: $textValue)
                .focused($isEditing)
                .frame(width: 58)
                .textFieldStyle(.roundedBorder)
                .font(.system(size: 10, design: .monospaced))
                #if os(iOS)
                .keyboardType(.decimalPad)
                #endif
                .onAppear { textValue = String(format: format, value) }
                .onSubmit { applyText() }
                .onChange(of: isEditing) { _, editing in
                    if !editing { applyText() }
                }

            if let def = defaultValue {
                Button(action: { value = def; textValue = String(format: format, def) }) {
                    Image(systemName: "arrow.counterclockwise")
                        .font(.system(size: 9, weight: .bold))
                        .foregroundColor(.white.opacity(0.7))
                        .frame(width: 22, height: 22)
                        .background(Color.white.opacity(0.12))
                        .clipShape(Circle())
                }
            }
        }
    }

    private func applyText() {
        if let parsed = Float(textValue.replacingOccurrences(of: ",", with: ".")) {
            value = min(max(parsed, range.lowerBound), range.upperBound)
        }
        textValue = String(format: format, value)
    }
}

private struct ParamRowInt: View {
    let label: String
    @Binding var value: Float
    let range: ClosedRange<Int>
    var defaultValue: Float? = nil

    @State private var textValue: String = ""
    @FocusState private var isEditing: Bool

    var body: some View {
        HStack(spacing: 6) {
            Text(label)
                .font(.system(size: 12))
                .foregroundColor(.white.opacity(0.8))
                .frame(width: 70, alignment: .leading)

            Slider(value: $value, in: Float(range.lowerBound)...Float(range.upperBound), step: 1)
                .onChange(of: value) { _, newVal in
                    if !isEditing {
                        textValue = "\(Int(newVal))"
                    }
                }

            TextField("", text: $textValue)
                .focused($isEditing)
                .frame(width: 42)
                .textFieldStyle(.roundedBorder)
                .font(.system(size: 10, design: .monospaced))
                #if os(iOS)
                .keyboardType(.numberPad)
                #endif
                .onAppear { textValue = "\(Int(value))" }
                .onSubmit { applyText() }
                .onChange(of: isEditing) { _, editing in
                    if !editing { applyText() }
                }

            if let def = defaultValue {
                Button(action: { value = def; textValue = "\(Int(def))" }) {
                    Image(systemName: "arrow.counterclockwise")
                        .font(.system(size: 9, weight: .bold))
                        .foregroundColor(.white.opacity(0.7))
                        .frame(width: 22, height: 22)
                        .background(Color.white.opacity(0.12))
                        .clipShape(Circle())
                }
            }
        }
    }

    private func applyText() {
        if let parsed = Int(textValue) {
            value = Float(min(max(parsed, range.lowerBound), range.upperBound))
        }
        textValue = "\(Int(value))"
    }
}

private struct VictoryOverlay: View {
    let level: Int
    var starCount: Int = 3
    var percentile: Int? = nil
    let onNext: () -> Void

    @State private var titleScale: CGFloat = 0.3
    @State private var titleOpacity: Double = 0
    @State private var buttonOffset: CGFloat = 30
    @State private var buttonOpacity: Double = 0
    private let fireworksVariant: Int = Int.random(in: 0...2)

    var body: some View {
        ZStack {
            // Fireworks background (random variant each time)
            FireworksView(variant: fireworksVariant)
                .allowsHitTesting(false)

            VStack(spacing: 20) {
                Text("Level \(level)")
                    .font(.system(size: 42, weight: .heavy, design: .rounded))
                    .foregroundColor(.white)
                    .shadow(color: .white.opacity(0.4), radius: 16)
                    .scaleEffect(titleScale)
                    .opacity(titleOpacity)

                Text("completed!")
                    .font(.system(size: 24, weight: .medium, design: .rounded))
                    .foregroundColor(.white.opacity(0.8))
                    .scaleEffect(titleScale)
                    .opacity(titleOpacity)

                HStack(spacing: 12) {
                    ForEach(0..<3, id: \.self) { i in
                        Image(systemName: i < starCount ? "star.fill" : "star")
                            .font(.system(size: 40, weight: .bold))
                            .foregroundColor(i < starCount ? Color(red: 1, green: 0.85, blue: 0.2) : .white.opacity(0.2))
                    }
                }
                .scaleEffect(titleScale)
                .opacity(titleOpacity)

                if let pct = percentile, pct >= 50 {
                    Text("Better than \(pct)% of players")
                        .font(.system(size: 15, weight: .medium, design: .rounded))
                        .foregroundColor(.white.opacity(0.7))
                        .scaleEffect(titleScale)
                        .opacity(titleOpacity)
                }

                Button(action: onNext) {
                    HStack(spacing: 10) {
                        Text("Level \(level + 1)")
                            .font(.system(size: 21, weight: .bold, design: .rounded))
                        Image(systemName: "arrow.right")
                            .font(.system(size: 17, weight: .bold))
                    }
                    .foregroundColor(.white)
                    .padding(.horizontal, 40)
                    .padding(.vertical, 15)
                    .background(
                        Capsule()
                            .fill(
                                LinearGradient(
                                    colors: [Color(red: 0.35, green: 0.6, blue: 1.0), Color(red: 0.5, green: 0.35, blue: 0.95)],
                                    startPoint: .leading,
                                    endPoint: .trailing
                                )
                            )
                    )
                    .shadow(color: Color(red: 0.4, green: 0.4, blue: 1.0).opacity(0.5), radius: 12, y: 4)
                }
                .offset(y: buttonOffset)
                .opacity(buttonOpacity)
            }
        }
        .onAppear {
            withAnimation(.spring(response: 0.45, dampingFraction: 0.6)) {
                titleScale = 1.0
                titleOpacity = 1.0
            }
            withAnimation(.easeOut(duration: 0.45).delay(0.2)) {
                buttonOffset = 0
                buttonOpacity = 1.0
            }
        }
    }
}
