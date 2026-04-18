import SwiftUI
#if os(iOS)
import UIKit
#endif

struct ContentView: View {
    @StateObject private var gameController = GameController()
    @State private var showControls = false
    @State private var showMenu = false
    @State private var selectedTab = 0
    @State private var showLevelPicker = false
    @State private var levelInput = ""
    @State private var dumpMessage: String?
    @State private var settingsCopied = false
    @State private var geometryCopied = false
    @State private var settingsImported: Bool? = nil
    @State private var usernameInput = ""
    @State private var loginAsInput = ""
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
                #if os(iOS)
                OrientationLockedGameView(controller: gameController)
                    .ignoresSafeArea()
                #else
                GameView(controller: gameController)
                    .ignoresSafeArea()
                #endif
            }


            if !gameController.showLevelComplete {
            VStack(spacing: 0) {
                HStack(spacing: 8) {
                    Button(action: {
                        gameController.undo()
                    }) {
                        Image(systemName: "arrow.uturn.backward")
                            .font(.system(size: 20, weight: .semibold))
                            .foregroundColor(gameController.canUndo ? .white : .white.opacity(0.3))
                    }
                    .disabled(!gameController.canUndo)
                    #if os(macOS)
                    .keyboardShortcut("z", modifiers: .command)
                    #endif
                    .padding(.leading, 16)
                    .padding(.top, 8)

                    Spacer()

                    Button(action: {
                        withAnimation(.easeInOut(duration: 0.2)) { showMenu.toggle() }
                    }) {
                        Image(systemName: showMenu ? "pause.fill" : "pause")
                            .font(.system(size: 12, weight: .light))
                            .foregroundColor(.white.opacity(0.15))
                            .frame(width: 24, height: 24)
                    }
                    .padding(.trailing, 16)
                    .padding(.top, 8)
                }

                if showMenu {
                    HStack(spacing: 8) {
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
                        .padding(.leading, 16)

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
                                Text("L \(gameController.currentLevel)")
                                    .font(.system(size: 18, weight: .bold, design: .rounded))
                                    .foregroundColor(.white)
                                Text("\(Int(gameController.fps)) fps")
                                    .font(.system(size: 11, design: .monospaced))
                                    .foregroundColor(.white.opacity(0.4))
                            }
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
                        .padding(.trailing, 16)
                    }
                    .padding(.top, 8)
                }

                if showControls {
                    VStack(spacing: 0) {
                        HStack {
                            Text("Physics")
                                .font(.system(size: 14, weight: .semibold))
                                .foregroundColor(.white)
                            Spacer()
                            Button(action: {
                                if gameController.dumpGeometryToClipboard() {
                                    geometryCopied = true
                                    DispatchQueue.main.asyncAfter(deadline: .now() + 1.5) { geometryCopied = false }
                                }
                            }) {
                                Image(systemName: "point.3.connected.trianglepath.dotted")
                                    .font(.system(size: 13, weight: .bold))
                                    .foregroundColor(geometryCopied ? .green : .white.opacity(0.8))
                                    .frame(width: 30, height: 30)
                                    .background(Color.white.opacity(0.12))
                                    .clipShape(Circle())
                            }
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
                                TabButton(title: "Physics", index: 0, selected: $selectedTab)
                                TabButton(title: "Rope", index: 1, selected: $selectedTab)
                                TabButton(title: "Holes", index: 2, selected: $selectedTab)
                                TabButton(title: "Light", index: 3, selected: $selectedTab)
                            }
                            HStack(spacing: 0) {
                                TabButton(title: "Table", index: 4, selected: $selectedTab)
                                TabButton(title: "Cartoon", index: 5, selected: $selectedTab)
                                TabButton(title: "Worm", index: 6, selected: $selectedTab)
                                TabButton(title: "Player", index: 7, selected: $selectedTab)
                                TabButton(title: "Presets", index: 8, selected: $selectedTab)
                            }
                        }
                        .padding(.horizontal, 10)
                        .padding(.bottom, 6)

                        // Tab content
                        Group {
                            switch selectedTab {
                            case 0: physicsTab
                            case 1: ropeTab
                            case 2: holesTab
                            case 3: lightTab
                            case 4: tableTab
                            case 5: cartoonTab
                            case 6: wormTab
                            case 7: playerTab
                            case 8: presetsTab
                            default: physicsTab
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
                    Spacer()

                    Text("\(gameController.moveCount)")
                        .font(.system(size: 15, weight: .medium, design: .monospaced))
                        .foregroundColor(.white.opacity(0.35))
                        .padding(.trailing, 20)
                        .padding(.bottom, 48)
                }
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
            #if os(iOS)
            OrientationLockedGameView(controller: gameController)
            #else
            GameView(controller: gameController)
            #endif

            VStack {
                HStack(spacing: 6) {
                    Button { gameController.undo() } label: {
                        Image(systemName: "arrow.uturn.backward")
                            .font(.system(size: 14, weight: .semibold))
                            .foregroundColor(gameController.canUndo ? .white : .white.opacity(0.3))
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

    private var physicsTab: some View {
        ScrollView(.vertical, showsIndicators: false) {
            VStack(spacing: 3) {
                ParamRowInt(label: "Particles", value: $gameController.particleCount, range: 6...200, defaultValue: 60)
                HStack(spacing: 4) {
                    MiniParam(label: "Gravity", value: $gameController.gravity, range: -20.0...0.0, format: "%.1f")
                    MiniParam(label: "Damping", value: $gameController.damping, range: 0.8...1.0, format: "%.3f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Tension", value: $gameController.ropeTension, range: 0.50...1.0, format: "%.3f")
                    MiniParam(label: "Bend C", value: $gameController.bendCompliance, range: 0.0...0.01, format: "%.4f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Bend D", value: $gameController.bendVelocityCoupling, range: 0.0...1.0, format: "%.2f")
                    MiniParamInt(label: "Constr", value: $gameController.constraintIterations, range: 1...60)
                }
                HStack(spacing: 4) {
                    MiniParamInt(label: "Settle", value: $gameController.settleSteps, range: 1...100)
                    MiniParamInt(label: "Broad", value: $gameController.broadphaseRebuildInterval, range: 0...10)
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Drag H", value: $gameController.dragHeight, range: 0.05...1.5, format: "%.3f")
                    MiniParam(label: "Lift H", value: $gameController.liftHeight, range: 0.05...1.5, format: "%.3f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Frict", value: $gameController.frictionCoefficient, range: 0.0...2.0, format: "%.2f")
                    MiniParam(label: "ColRs", value: $gameController.collisionResponse, range: 0.0...1.0, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "FrDmp", value: $gameController.frictionDampingRatio, range: 0.0...1.0, format: "%.2f")
                    MiniParam(label: "FrCap", value: $gameController.maxFrictionCap, range: 0.0...1.0, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "BrdFr", value: $gameController.boardFrictionRatio, range: 0.0...1.0, format: "%.2f")
                }
                ParamRow(label: "Board Z", value: $gameController.boardElevation, range: 0.02...0.5, format: "%.3f", defaultValue: 0.257)
                HStack(spacing: 4) {
                    MiniParam(label: "TwStf", value: $gameController.twistStiffness, range: 0.0...1.0, format: "%.3f")
                    MiniParam(label: "TwDmp", value: $gameController.twistDamping, range: 0.0...1.0, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "GrvTq", value: $gameController.gravityTorque, range: 0.0...2.0, format: "%.2f")
                    MiniParam(label: "DrgPk", value: $gameController.dragPickupDuration, range: 0.01...0.5, format: "%.3f")
                }
                HStack(spacing: 4) {
                    MiniParamInt(label: "DrgSb", value: $gameController.dragMinSubsteps, range: 1...10)
                    MiniParam(label: "FadeS", value: $gameController.fadeOutSpeed, range: 0.25...100.0, format: "%.1f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "LwrDr", value: $gameController.lowerAnimDuration, range: 0.1...2.0, format: "%.2f")
                    MiniParam(label: "Idle", value: $gameController.idleTimeout, range: 0.5...10.0, format: "%.1f")
                }
            }
        }
        .frame(maxHeight: 340)
    }

    private var ropeTab: some View {
        ScrollView(.vertical, showsIndicators: false) {
            VStack(spacing: 3) {
                HStack(spacing: 3) {
                    Text("Palette")
                        .font(.system(size: 11))
                        .foregroundColor(.white.opacity(0.8))
                        .frame(width: 42, alignment: .leading)
                    ForEach(GameController.RopePalette.allCases, id: \.rawValue) { palette in
                        let selected = gameController.ropePalette == palette
                        Button(action: { gameController.ropePalette = palette }) {
                            HStack(spacing: 2) {
                                ForEach(0..<3, id: \.self) { i in
                                    let c = palette.colors[min(i, palette.colors.count - 1)]
                                    Circle()
                                        .fill(Color(red: Double(c.x), green: Double(c.y), blue: Double(c.z)))
                                        .frame(width: 7, height: 7)
                                }
                            }
                            .padding(.horizontal, 3)
                            .padding(.vertical, 3)
                            .background(selected ? Color.white.opacity(0.3) : Color.white.opacity(0.08))
                            .cornerRadius(5)
                            .overlay(RoundedRectangle(cornerRadius: 5).stroke(selected ? Color.white.opacity(0.5) : Color.clear, lineWidth: 1))
                        }
                    }
                }
                HStack(spacing: 6) {
                    Text("Square")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                        .frame(width: 70, alignment: .leading)
                    Toggle("", isOn: $gameController.squareCrossSection)
                        .labelsHidden()
                    Text("Chain")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                    Toggle("", isOn: $gameController.chainMode)
                        .labelsHidden()
                    Text("Flat N")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                    Toggle("", isOn: $gameController.ropeFlatNormals)
                        .labelsHidden()
                    Text("Pad")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                    Toggle("", isOn: $gameController.padMode)
                        .labelsHidden()
                }
                if gameController.padMode {
                    HStack(spacing: 4) {
                        Text("Pad H").font(.system(size: 10)).foregroundColor(.white.opacity(0.6)).frame(width: 38, alignment: .leading)
                        Slider(value: $gameController.padHeight, in: 0.05...0.5)
                        Text("Met").font(.system(size: 10)).foregroundColor(.white.opacity(0.6))
                        Slider(value: $gameController.padMetallic, in: 0...1)
                        Text("Rgh").font(.system(size: 10)).foregroundColor(.white.opacity(0.6))
                        Slider(value: $gameController.padRoughness, in: 0...1)
                    }
                    HStack(spacing: 4) {
                        Text("Pad R").font(.system(size: 10)).foregroundColor(.white.opacity(0.6)).frame(width: 38, alignment: .leading)
                        Slider(value: $gameController.padColorR, in: 0...1)
                        Text("G").font(.system(size: 10)).foregroundColor(.white.opacity(0.6))
                        Slider(value: $gameController.padColorG, in: 0...1)
                        Text("B").font(.system(size: 10)).foregroundColor(.white.opacity(0.6))
                        Slider(value: $gameController.padColorB, in: 0...1)
                        Text("Tint").font(.system(size: 10)).foregroundColor(.white.opacity(0.6))
                        Slider(value: $gameController.padRopeTint, in: 0...1)
                    }
                }
                HStack(spacing: 4) {
                    Toggle("Env Dbg", isOn: $gameController.ropeEnvDebug)
                        .font(.system(size: 11))
                        .foregroundColor(.white.opacity(0.7))
                }
                HStack(spacing: 6) {
                    Text("Seam")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                        .frame(width: 70, alignment: .leading)
                    Toggle("", isOn: $gameController.ropeSeamEnabled)
                        .labelsHidden()
                }
                if gameController.ropeSeamEnabled {
                    HStack(spacing: 4) {
                        MiniParam(label: "SmW", value: $gameController.ropeSeamWidth, range: 0.005...0.25, format: "%.3f")
                        MiniParam(label: "SmDp", value: $gameController.ropeSeamDepth, range: 0.0...1.0, format: "%.2f")
                    }
                    HStack(spacing: 4) {
                        MiniParam(label: "SmDk", value: $gameController.ropeSeamDarkness, range: 0.0...3.0, format: "%.2f")
                        MiniParam(label: "SmHi", value: $gameController.ropeSeamHighlight, range: 0.0...1.0, format: "%.2f")
                    }
                    HStack(spacing: 4) {
                        MiniParam(label: "SmCr", value: $gameController.ropeSeamCrackAmount, range: 0.0...1.0, format: "%.2f")
                        MiniParam(label: "SmCs", value: $gameController.ropeSeamCrackScale, range: 2.0...80.0, format: "%.1f")
                    }
                    HStack(spacing: 6) {
                        Text("Rnd Pos")
                            .font(.system(size: 12))
                            .foregroundColor(.white.opacity(0.8))
                            .frame(width: 70, alignment: .leading)
                        Toggle("", isOn: $gameController.ropeSeamRandomize)
                            .labelsHidden()
                    }
                    if !gameController.ropeSeamRandomize {
                        ParamRow(label: "Seam Pos", value: $gameController.ropeSeamPosition, range: 0.0...1.0, format: "%.2f", defaultValue: 0.5)
                    }
                }
                if gameController.chainMode {
                    HStack(spacing: 4) {
                        MiniParam(label: "Link L", value: $gameController.chainLinkLength, range: 1.5...5.0, format: "%.1f")
                        MiniParam(label: "Thick", value: $gameController.chainLinkThickness, range: 0.1...0.8, format: "%.2f")
                        MiniParam(label: "Width", value: $gameController.chainLinkWidth, range: 0.3...1.5, format: "%.2f")
                    }
                }
                HStack(spacing: 4) {
                    MiniParamInt(label: "Profile", value: $gameController.profileSegments, range: 3...32)
                    MiniParam(label: "Scale", value: $gameController.ropeRadiusScale, range: 0.5...2.0, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Matte", value: $gameController.ropeMatte, range: 0...1, format: "%.2f")
                    MiniParam(label: "Gloss", value: $gameController.ropeGloss, range: 0...2, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Wrap", value: $gameController.ropeDiffuseWrap, range: 0...1, format: "%.2f")
                    MiniParam(label: "SSS", value: $gameController.ropeSubsurface, range: 0...1, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Edge", value: $gameController.ropeEdgeLight, range: 0...0.5, format: "%.2f")
                    MiniParam(label: "Core", value: $gameController.ropeCoreDarken, range: 0...10, format: "%.1f")
                    MiniParam(label: "Satur", value: $gameController.ropeSaturation, range: 0...2, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Bump", value: $gameController.ropeMicroBump, range: 0...3.0, format: "%.3f")
                    MiniParam(label: "B.Scl", value: $gameController.ropeBumpScale, range: 0.5...20.0, format: "%.1f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "B.Ctr", value: $gameController.ropeBumpContrast, range: 0.05...4.0, format: "%.2f")
                    MiniParam(label: "B.Ans", value: $gameController.ropeBumpAniso, range: 0.0...3.0, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "StrTh", value: $gameController.stretchThinning, range: 0.0...1.0, format: "%.2f")
                    MiniParam(label: "AO", value: $gameController.ropeContactAO, range: 0...1, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Glow", value: $gameController.ropeLiftGlow, range: 0...1, format: "%.2f")
                    MiniParam(label: "Opac", value: $gameController.ropeOpacity, range: 0.05...1, format: "%.2f")
                }
                HStack(spacing: 6) {
                    Text("Cracks")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                        .frame(width: 70, alignment: .leading)
                    Toggle("", isOn: $gameController.ropeCracksEnabled)
                        .labelsHidden()
                }
                if gameController.ropeCracksEnabled {
                    HStack(spacing: 4) {
                        MiniParam(label: "CrAmt", value: $gameController.ropeCrackAmount, range: 0.0...1.0, format: "%.2f")
                        MiniParam(label: "CrWdt", value: $gameController.ropeCrackWidth, range: 0.01...0.45, format: "%.3f")
                        MiniParam(label: "CrDpt", value: $gameController.ropeCrackDepth, range: 0.0...1.0, format: "%.2f")
                    }
                }
                HStack(spacing: 4) {
                    MiniParam(label: "StrGl", value: $gameController.ropeStretchGloss, range: 0...1, format: "%.2f")
                    MiniParam(label: "StrSp", value: $gameController.ropeStretchSpec, range: 0...2, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Refl", value: $gameController.ropeEnvReflect, range: 0...10, format: "%.2f")
                    MiniParam(label: "Sprd", value: $gameController.ropeEnvSpread, range: 0.01...0.5, format: "%.3f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Cap S", value: $gameController.capRadiusScale, range: 0.3...2.5, format: "%.2f")
                    MiniParam(label: "Cap D", value: $gameController.capDarken, range: 0...1, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParamInt(label: "CapSg", value: $gameController.capSegments, range: 4...48)
                    MiniParamInt(label: "CapRn", value: $gameController.capRings, range: 2...16)
                }
            }
        }
        .frame(maxHeight: 280)
    }

    private var holesTab: some View {
        VStack(spacing: 3) {
            HStack(spacing: 4) {
                MiniParam(label: "Size", value: $gameController.holeRadiusScale, range: 0.5...2.0, format: "%.2f")
                MiniParamInt(label: "Seg", value: $gameController.holeSegments, range: 12...96)
            }
            HStack(spacing: 4) {
                MiniParam(label: "R", value: $gameController.holeTintR, range: 0...1, format: "%.2f")
                MiniParam(label: "G", value: $gameController.holeTintG, range: 0...1, format: "%.2f")
            }
            HStack(spacing: 4) {
                MiniParam(label: "B", value: $gameController.holeTintB, range: 0...1, format: "%.2f")
                MiniParam(label: "Tint", value: $gameController.holeTintAmount, range: 0...1, format: "%.2f")
            }
        }
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
                HStack(spacing: 4) {
                    MiniParam(label: "Expos", value: $gameController.cartoonExposure, range: 0.5...1.5, format: "%.2f")
                    MiniParam(label: "Bloom", value: $gameController.cartoonBloom, range: 0...0.3, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Edge", value: $gameController.cartoonEdgeStrength, range: 0...1, format: "%.2f")
                    MiniParam(label: "Smth", value: $gameController.cartoonEdgeSmooth, range: 0...1, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Shadw", value: $gameController.cartoonShadowBright, range: 0.1...0.8, format: "%.2f")
                    MiniParam(label: "Wrap", value: $gameController.cartoonWrap, range: 0...0.5, format: "%.2f")
                }
                ParamRowInt(label: "Levels", value: $gameController.cartoonLevels, range: 2...6, defaultValue: 4)
            }
        }
        .frame(maxHeight: 220)
    }

    private var lightTab: some View {
        ScrollView(.vertical, showsIndicators: false) {
            VStack(spacing: 3) {
                HStack(spacing: 4) {
                    MiniParam(label: "Expos", value: $gameController.exposure, range: 0.5...2.5, format: "%.2f")
                    MiniParam(label: "Intens", value: $gameController.lightIntensity, range: 0.1...3.0, format: "%.2f")
                }
                HStack(spacing: 6) {
                    Text("Bloom")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                        .frame(width: 70, alignment: .leading)
                    Toggle("", isOn: $gameController.bloomEnabled)
                        .labelsHidden()
                    ResetButton { gameController.bloomEnabled = true }
                }
                if gameController.bloomEnabled {
                    HStack(spacing: 4) {
                        MiniParam(label: "Bloom", value: $gameController.bloomStrength, range: 0...2.0, format: "%.2f")
                        MiniParam(label: "Render", value: $gameController.renderScale, range: 0.25...2.0, format: "%.2f")
                    }
                } else {
                    ParamRow(label: "Render Scale", value: $gameController.renderScale, range: 0.25...2.0, format: "%.2f", defaultValue: 1.0)
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Dir X", value: $gameController.lightDirX, range: -1...1, format: "%.2f")
                    MiniParam(label: "Dir Y", value: $gameController.lightDirY, range: -1...1, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Dir Z", value: $gameController.lightDirZ, range: -1...1, format: "%.2f")
                    MiniParam(label: "Ambi", value: $gameController.ambient, range: 0...0.6, format: "%.2f")
                }
                HStack(spacing: 6) {
                    Text("Shadows")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                        .frame(width: 70, alignment: .leading)
                    Toggle("", isOn: $gameController.shadowsEnabled)
                        .labelsHidden()
                    Picker("", selection: $gameController.shadowType) {
                        ForEach(GameController.ShadowType.allCases, id: \.rawValue) { t in
                            Text(t.label).foregroundColor(.primary).tag(t)
                        }
                    }
                    .pickerStyle(.menu)
                    .tint(.white)
                }
                HStack(spacing: 6) {
                    Text("Shadow Map")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.8))
                        .frame(width: 70, alignment: .leading)
                    Picker("", selection: $gameController.shadowMapSize) {
                        Text("256").tag(256)
                        Text("512").tag(512)
                        Text("1K").tag(1024)
                        Text("2K").tag(2048)
                        Text("4K").tag(4096)
                    }
                    .pickerStyle(.segmented)
                    .font(.caption2)
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Bias", value: $gameController.shadowBias, range: 0.0001...0.005, format: "%.4f")
                    MiniParam(label: "Dark", value: $gameController.shadowDarkness, range: 0...0.5, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "LtSz", value: $gameController.lightSize, range: 0.002...0.08, format: "%.3f")
                    MiniParam(label: "PCSS", value: $gameController.pcssPenumbraScale, range: 1...500, format: "%.0f")
                }
                Picker("Shadow Dbg", selection: $gameController.shadowDebugMode) {
                    Text("Off").tag(0)
                    Text("Shadow").tag(1)
                    Text("StoredZ").tag(2)
                    Text("FragZ").tag(3)
                    Text("Diff").tag(4)
                }.pickerStyle(.menu).font(.caption2)
            }
        }
        .frame(maxHeight: 280)
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
                    HStack(spacing: 4) {
                        MiniParam(label: "C1 R", value: $gameController.tableColor1R, range: 0...1, format: "%.2f")
                        MiniParam(label: "C1 G", value: $gameController.tableColor1G, range: 0...1, format: "%.2f")
                    }
                    ParamRow(label: "Color1 B", value: $gameController.tableColor1B, range: 0...1, format: "%.2f", defaultValue: 0.13)
                    if gameController.tableStyle == .gradient {
                        HStack(spacing: 4) {
                            MiniParam(label: "C2 R", value: $gameController.tableColor2R, range: 0...1, format: "%.2f")
                            MiniParam(label: "C2 G", value: $gameController.tableColor2G, range: 0...1, format: "%.2f")
                        }
                        ParamRow(label: "Color2 B", value: $gameController.tableColor2B, range: 0...1, format: "%.2f", defaultValue: 0.20)
                    }
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Seed", value: $gameController.woodSeed, range: 0...1, format: "%.2f")
                    MiniParam(label: "Bright", value: $gameController.woodBrightness, range: 0.5...1.5, format: "%.2f")
                }
                ParamRow(label: "Pattern Scale", value: $gameController.woodPatternScale, range: 0.5...7.0, format: "%.1f", defaultValue: 3.0)
            }
        }
        .frame(maxHeight: 220)
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
                HStack(spacing: 4) {
                    MiniParam(label: "Freq", value: $gameController.wormSegFreq, range: 4...80, format: "%.0f")
                    MiniParam(label: "Bulge", value: $gameController.wormSegBulge, range: 0...0.5, format: "%.3f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Thick", value: $gameController.wormThickness, range: 0.5...3.0, format: "%.2f")
                    MiniParam(label: "Taper", value: $gameController.wormTaperLen, range: 0.02...0.4, format: "%.3f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Groove", value: $gameController.wormGrooveDepth, range: 0...1, format: "%.2f")
                    MiniParam(label: "Noise", value: $gameController.wormSkinNoise, range: 0...0.3, format: "%.3f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Belly", value: $gameController.wormBellyBright, range: 0.5...2.0, format: "%.2f")
                    MiniParam(label: "Back", value: $gameController.wormBackDark, range: 0.2...1.5, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "SSS", value: $gameController.wormSSS, range: 0...1, format: "%.2f")
                    MiniParam(label: "Rough", value: $gameController.wormRoughness, range: 0.05...1, format: "%.2f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "Spec", value: $gameController.wormSpecular, range: 0...2, format: "%.2f")
                    MiniParam(label: "Rim", value: $gameController.wormRimStrength, range: 0...0.5, format: "%.3f")
                }
                ParamRow(label: "Eye Size", value: $gameController.wormEyeSize, range: 0...0.06, format: "%.4f", defaultValue: 0.015)
                HStack(spacing: 4) {
                    MiniParam(label: "PulSp", value: $gameController.wormPulseSpeed, range: 0...10, format: "%.1f")
                    MiniParam(label: "PulAm", value: $gameController.wormPulseAmp, range: 0...0.1, format: "%.3f")
                }
                HStack(spacing: 4) {
                    MiniParam(label: "CrwSp", value: $gameController.wormCrawlSpeed, range: 0...15, format: "%.1f")
                    MiniParam(label: "CrwAm", value: $gameController.wormCrawlAmp, range: 0...0.05, format: "%.4f")
                }
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

    private var presetsTab: some View {
        ScrollView(.vertical, showsIndicators: false) {
            VStack(spacing: 8) {
                ForEach(gameController.presets) { preset in
                    Button(action: {
                        gameController.applyPreset(preset)
                    }) {
                        HStack {
                            VStack(alignment: .leading, spacing: 2) {
                                Text(preset.name)
                                    .font(.system(size: 14, weight: .bold))
                                    .foregroundColor(.white)
                                Text(preset.name == "Jelly Beams" ? "Soft, glossy, elastic beams" :
                                     preset.name == "Steel Cable" ? "Stiff, heavy, metallic" : "Standard rubber band")
                                    .font(.system(size: 11))
                                    .foregroundColor(.white.opacity(0.6))
                            }
                            Spacer()
                            Image(systemName: "chevron.right")
                                .font(.system(size: 12, weight: .bold))
                                .foregroundColor(.white.opacity(0.3))
                        }
                        .padding(.horizontal, 12)
                        .padding(.vertical, 10)
                        .background(Color.white.opacity(0.1))
                        .cornerRadius(8)
                    }
                }
            }
        }
        .frame(maxHeight: 220)
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

private struct MiniParam: View {
    let label: String
    @Binding var value: Float
    let range: ClosedRange<Float>
    let format: String

    @State private var textValue: String = ""
    @FocusState private var isEditing: Bool

    var body: some View {
        HStack(spacing: 3) {
            Text(label)
                .font(.system(size: 11))
                .foregroundColor(.white.opacity(0.7))
                .frame(width: 42, alignment: .leading)
                .lineLimit(1)

            Slider(value: $value, in: range)
                .onChange(of: value) { _, newVal in
                    if !isEditing { textValue = String(format: format, newVal) }
                }

            TextField("", text: $textValue)
                .focused($isEditing)
                .frame(width: 40)
                .textFieldStyle(.roundedBorder)
                .font(.system(size: 9, design: .monospaced))
                #if os(iOS)
                .keyboardType(.decimalPad)
                #endif
                .onAppear { textValue = String(format: format, value) }
                .onSubmit { applyText() }
                .onChange(of: isEditing) { _, editing in
                    if !editing { applyText() }
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

private struct MiniParamInt: View {
    let label: String
    @Binding var value: Float
    let range: ClosedRange<Int>

    @State private var textValue: String = ""
    @FocusState private var isEditing: Bool

    var body: some View {
        HStack(spacing: 3) {
            Text(label)
                .font(.system(size: 11))
                .foregroundColor(.white.opacity(0.7))
                .frame(width: 42, alignment: .leading)
                .lineLimit(1)

            Slider(value: $value, in: Float(range.lowerBound)...Float(range.upperBound), step: 1)
                .onChange(of: value) { _, newVal in
                    if !isEditing { textValue = "\(Int(newVal))" }
                }

            TextField("", text: $textValue)
                .focused($isEditing)
                .frame(width: 32)
                .textFieldStyle(.roundedBorder)
                .font(.system(size: 9, design: .monospaced))
                #if os(iOS)
                .keyboardType(.numberPad)
                #endif
                .onAppear { textValue = "\(Int(value))" }
                .onSubmit { applyText() }
                .onChange(of: isEditing) { _, editing in
                    if !editing { applyText() }
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

#if os(iOS)
private func activeWindowScene() -> UIWindowScene? {
    let scenes = UIApplication.shared.connectedScenes.compactMap { $0 as? UIWindowScene }
    return scenes.first { $0.activationState == .foregroundActive } ?? scenes.first
}

private struct OrientationLockedGameView: View {
    @ObservedObject var controller: GameController
    @State private var interfaceOrientation: UIInterfaceOrientation = .portrait

    var body: some View {
        GeometryReader { geo in
            let sz = geo.size
            let (angle, fw, fh) = orientationFrame(interfaceOrientation, container: sz)
            GameView(controller: controller)
                .frame(width: fw, height: fh)
                .rotationEffect(angle)
                .position(x: sz.width * 0.5, y: sz.height * 0.5)
        }
        .onAppear { refreshOrientation() }
        .onReceive(NotificationCenter.default.publisher(for: UIDevice.orientationDidChangeNotification)) { _ in
            refreshOrientation()
        }
    }

    private func refreshOrientation() {
        guard let scene = activeWindowScene() else { return }
        let next = scene.interfaceOrientation
        if next != interfaceOrientation {
            var t = Transaction()
            t.disablesAnimations = true
            withTransaction(t) {
                interfaceOrientation = next
            }
        }
    }

    private func orientationFrame(_ o: UIInterfaceOrientation, container: CGSize) -> (Angle, CGFloat, CGFloat) {
        let w = container.width
        let h = container.height
        switch o {
        case .landscapeLeft:
            return (.degrees(90), h, w)
        case .landscapeRight:
            return (.degrees(-90), h, w)
        case .portraitUpsideDown:
            return (.degrees(180), w, h)
        default:
            return (.zero, w, h)
        }
    }
}
#endif

private struct VictoryOverlay: View {
    let level: Int
    var starCount: Int = 3
    var percentile: Int? = nil
    let onNext: () -> Void

    @State private var titleScale: CGFloat = 0.3
    @State private var titleOpacity: Double = 0
    @State private var buttonOffset: CGFloat = 30
    @State private var buttonOpacity: Double = 0

    var body: some View {
        ZStack {
            VStack(spacing: 20) {
                Text("Level \(level)")
                    .font(.system(size: 42, weight: .heavy, design: .rounded))
                    .minimumScaleFactor(0.5)
                    .lineLimit(1)
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
                            .minimumScaleFactor(0.6)
                            .lineLimit(1)
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
