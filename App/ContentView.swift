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

            // Victory overlay
            if gameController.showLevelComplete {
                Text("✓")
                    .font(.system(size: 80, weight: .bold, design: .rounded))
                    .foregroundColor(.white)
                    .shadow(color: .white.opacity(0.6), radius: 20)
                    .transition(.scale.combined(with: .opacity))
                    .animation(.spring(response: 0.4, dampingFraction: 0.6), value: gameController.showLevelComplete)
            }
        }
        .animation(.easeInOut(duration: 0.3), value: gameController.showLevelComplete)
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
                .keyboardType(.numberPad)
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
                .keyboardType(.decimalPad)
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
                .keyboardType(.numberPad)
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
