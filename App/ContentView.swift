import SwiftUI

struct ContentView: View {
    @StateObject private var gameController = GameController()
    @State private var showControls = false
    @State private var selectedTab = 0

    var body: some View {
        ZStack {
            GameView(controller: gameController)
                .ignoresSafeArea()

            VStack {
                HStack {
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
                    VStack(spacing: 4) {
                        HStack(spacing: 8) {
                            Picker("", selection: $selectedTab) {
                                Text("Hook").tag(0)
                                Text("Smooth").tag(1)
                                Text("Render").tag(2)
                            }
                            .pickerStyle(.segmented)

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

                        ScrollView {
                            VStack(spacing: 3) {
                                switch selectedTab {
                                case 0: hookTab
                                case 1: smoothTab
                                case 2: renderTab
                                default: EmptyView()
                                }
                            }
                            .padding(.horizontal, 10)
                            .padding(.vertical, 6)
                        }
                    }
                    .frame(maxHeight: 260)
                    .background(Color.black.opacity(0.5))
                    .cornerRadius(14)
                    .padding(.horizontal, 12)
                    .padding(.top, 6)
                }

                Spacer()
            }
        }
    }

    @ViewBuilder
    private var hookTab: some View {
        ParamRow(label: "Step", value: $gameController.stepMultiplier, range: 0.001...1.0, format: "%.4f")
        ParamRow(label: "Radius", value: $gameController.hookRadiusMultiplier, range: 0.01...5.0, format: "%.3f")
        ParamRow(label: "Limit", value: $gameController.stepLimitMultiplier, range: 0.01...10.0, format: "%.3f")
        ParamRow(label: "Maturity", value: $gameController.maturityDistance, range: 0.001...1.0, format: "%.3f")
    }

    @ViewBuilder
    private var smoothTab: some View {
        ParamRowInt(label: "Subdiv", value: $gameController.smoothSubdivisions, range: 1...32)
        ParamRowInt(label: "Iters", value: $gameController.smoothIterations, range: 0...50)
        ParamRow(label: "Strength", value: $gameController.smoothStrength, range: 0...10.0, format: "%.2f")
    }

    @ViewBuilder
    private var renderTab: some View {
        ParamRow(label: "Drag H", value: $gameController.dragHeight, range: 0.05...1.5, format: "%.3f")
        CompactToggle(label: "Debug Colors", isOn: $gameController.debugSegmentColors)
        CompactToggle(label: "Simple Mode", isOn: $gameController.simpleMode)
        CompactToggle(label: "Hole Deform", isOn: Binding(
            get: { !gameController.disableHoleDeform },
            set: { gameController.disableHoleDeform = !$0 }
        ))
    }
}

private struct CompactToggle: View {
    let label: String
    @Binding var isOn: Bool

    var body: some View {
        Toggle(label, isOn: $isOn)
            .font(.system(size: 13))
            .foregroundColor(.white)
            .tint(.blue)
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
                .frame(width: 55, alignment: .leading)

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
                .frame(width: 55, alignment: .leading)

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
