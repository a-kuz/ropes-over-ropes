import SwiftUI

struct ContentView: View {
    @StateObject private var gameController = GameController()
    @State private var showControls = false
    @State private var selectedTab = 0
    @State private var showLevelPicker = false
    @State private var levelInput = ""
    @State private var dumpMessage: String?
    @State private var shareURL: URL?

    var body: some View {
        ZStack {
            GameView(controller: gameController)
                .ignoresSafeArea()

            VStack {
                HStack {
                    Button(action: {
                        levelInput = "\(gameController.currentLevel)"
                        showLevelPicker = true
                    }) {
                        VStack(alignment: .leading, spacing: 2) {
                            Text("Level \(gameController.currentLevel)")
                                .font(.system(size: 18, weight: .bold, design: .rounded))
                                .foregroundColor(.white)
                            Text("\(Int(gameController.fps)) fps")
                                .font(.system(size: 11, design: .monospaced))
                                .foregroundColor(.white.opacity(0.4))
                        }
                    }
                    .padding(.top, 52)
                    .padding(.leading, 16)

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
                    .padding(.top, 50)
                    .padding(.trailing, 8)

                    Button(action: {
                        if let url = gameController.dumpTopology() {
                            dumpMessage = url.lastPathComponent
                        }
                    }) {
                        Image(systemName: "square.and.arrow.down")
                            .font(.system(size: 20, weight: .semibold))
                            .foregroundColor(.white)
                            .frame(width: 44, height: 44)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    .padding(.top, 50)
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
                    .padding(.top, 50)
                    .padding(.trailing, 8)

                    Button(action: {
                        gameController.undo()
                    }) {
                        Image(systemName: "arrow.uturn.backward")
                            .font(.system(size: 20, weight: .semibold))
                            .foregroundColor(gameController.canUndo ? .white : .white.opacity(0.3))
                            .frame(width: 44, height: 44)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    .disabled(!gameController.canUndo)
                    #if os(macOS)
                    .keyboardShortcut("z", modifiers: .command)
                    #endif
                    .padding(.top, 50)
                    .padding(.trailing, 8)

                    Button(action: {
                        gameController.frictionSoundEnabled.toggle()
                    }) {
                        Image(systemName: gameController.frictionSoundEnabled ? "speaker.wave.2.fill" : "speaker.slash.fill")
                            .font(.system(size: 20, weight: .semibold))
                            .foregroundColor(gameController.frictionSoundEnabled ? .white : .white.opacity(0.5))
                            .frame(width: 44, height: 44)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    .padding(.top, 50)
                    .padding(.trailing, 8)

                    Button(action: {
                        gameController.restartLevel()
                    }) {
                        Image(systemName: "arrow.counterclockwise")
                            .font(.system(size: 20, weight: .semibold))
                            .foregroundColor(.white)
                            .frame(width: 44, height: 44)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    .padding(.top, 50)
                    .padding(.trailing, 20)
                }

                if showControls {
                    VStack(spacing: 0) {
                        HStack {
                            Text("Physics")
                                .font(.system(size: 14, weight: .semibold))
                                .foregroundColor(.white)
                            Spacer()
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

                        // Tab bar
                        HStack(spacing: 0) {
                            TabButton(title: "Rope", index: 0, selected: $selectedTab)
                            TabButton(title: "Solver", index: 1, selected: $selectedTab)
                            TabButton(title: "Drag", index: 2, selected: $selectedTab)
                        }
                        .padding(.horizontal, 10)
                        .padding(.bottom, 6)

                        // Tab content
                        Group {
                            switch selectedTab {
                            case 0: ropeTab
                            case 1: solverTab
                            case 2: dragTab
                            default: ropeTab
                            }
                        }
                        .padding(.horizontal, 10)
                        .padding(.bottom, 10)
                    }
                    .frame(maxHeight: 220)
                    .background(Color.black.opacity(0.5))
                    .cornerRadius(14)
                    .padding(.horizontal, 12)
                    .padding(.top, 6)
                }

                Spacer()
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

            if gameController.showLevelComplete {
                Color.black.opacity(0.4)
                    .ignoresSafeArea()
                    .transition(.opacity)

                VictoryOverlay(level: gameController.currentLevel) {
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

    private var ropeTab: some View {
        VStack(spacing: 3) {
            ParamRowInt(label: "Particles", value: $gameController.particleCount, range: 6...200)
            ParamRow(label: "Gravity", value: $gameController.gravity, range: -20.0...0.0, format: "%.1f")
            ParamRow(label: "Damping", value: $gameController.damping, range: 0.8...1.0, format: "%.3f")
            ParamRow(label: "Tension", value: $gameController.ropeTension, range: 0.50...1.0, format: "%.3f")
        }
    }

    private var solverTab: some View {
        VStack(spacing: 3) {
            ParamRowInt(label: "Constr Iter", value: $gameController.constraintIterations, range: 1...60)
            ParamRowInt(label: "Settle Steps", value: $gameController.settleSteps, range: 1...100)
        }
    }

    private var dragTab: some View {
        VStack(spacing: 3) {
            ParamRow(label: "Drag H", value: $gameController.dragHeight, range: 0.05...1.5, format: "%.3f")
            ParamRow(label: "Lift H", value: $gameController.liftHeight, range: 0.05...1.5, format: "%.3f")
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
        }
    }

    private func applyText() {
        if let parsed = Int(textValue) {
            value = Float(min(max(parsed, range.lowerBound), range.upperBound))
        }
        textValue = "\(Int(value))"
    }
}

private struct ConfettiPiece: Identifiable {
    let id: Int
    let x: CGFloat
    let color: Color
    let width: CGFloat
    let height: CGFloat
    let rotation: Double
    let delay: Double
    let drift: CGFloat
    let duration: Double
}

private struct VictoryOverlay: View {
    let level: Int
    let onNext: () -> Void

    @State private var titleScale: CGFloat = 0.3
    @State private var titleOpacity: Double = 0
    @State private var buttonOffset: CGFloat = 30
    @State private var buttonOpacity: Double = 0
    @State private var confettiLaunched = false

    private let confetti: [ConfettiPiece] = {
        let colors: [Color] = [
            .red, .orange, .yellow, .green, .blue, .purple, .pink,
            Color(red: 1, green: 0.4, blue: 0.7),
            Color(red: 0.3, green: 0.9, blue: 1),
            Color(red: 1, green: 0.85, blue: 0.2),
        ]
        return (0..<40).map { i in
            let sz = CGFloat.random(in: 4...10)
            return ConfettiPiece(
                id: i,
                x: CGFloat.random(in: 0.05...0.95),
                color: colors[i % colors.count],
                width: sz,
                height: sz * CGFloat.random(in: 0.5...1.5),
                rotation: Double.random(in: 0...360),
                delay: Double.random(in: 0...0.4),
                drift: CGFloat.random(in: -30...30),
                duration: Double.random(in: 1.8...3.0)
            )
        }
    }()

    var body: some View {
        ZStack {
            GeometryReader { geo in
                ForEach(confetti) { piece in
                    RoundedRectangle(cornerRadius: piece.width * 0.2)
                        .fill(piece.color)
                        .frame(width: piece.width, height: piece.height)
                        .rotationEffect(.degrees(confettiLaunched ? piece.rotation + 720 : piece.rotation))
                        .position(
                            x: piece.x * geo.size.width + (confettiLaunched ? piece.drift : 0),
                            y: confettiLaunched ? geo.size.height + 40 : -20
                        )
                        .opacity(confettiLaunched ? 0 : 1)
                        .animation(
                            .easeIn(duration: piece.duration).delay(piece.delay),
                            value: confettiLaunched
                        )
                }
            }
            .allowsHitTesting(false)

            VStack(spacing: 24) {
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
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
                confettiLaunched = true
            }
        }
    }
}
