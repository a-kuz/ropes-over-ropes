import SwiftUI

struct ContentView: View {
    @StateObject private var gameController = GameController()
    @State private var showControls = false
    
    var body: some View {
        ZStack {
            GameView(controller: gameController)
                .ignoresSafeArea()
            
            VStack {
                HStack {
                    Spacer()
                    
                    Button(action: {
                        withAnimation { showControls.toggle() }
                    }) {
                        Image(systemName: "slider.horizontal.3")
                            .font(.system(size: 20, weight: .semibold))
                            .foregroundColor(.white)
                            .frame(width: 44, height: 44)
                            .background(Color.black.opacity(0.6))
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
                            .background(Color.black.opacity(0.6))
                            .clipShape(Circle())
                    }
                    .padding(.top, 50)
                    .padding(.trailing, 20)
                }
                
                if showControls {
                    ScrollView {
                        VStack(spacing: 8) {
                            ParamRow(label: "Step", value: $gameController.stepMultiplier, range: 0.0001...1000.0, format: "%.4f")
                            ParamRow(label: "R", value: $gameController.hookRadiusMultiplier, range: 0.001...1000.0, format: "%.3f")
                            ParamRow(label: "Limit", value: $gameController.stepLimitMultiplier, range: 0.0001...1000.0, format: "%.4f")
                            
                            Toggle("Segments", isOn: $gameController.debugSegmentColors)
                                .foregroundColor(.white)
                            
                            Divider().background(Color.white.opacity(0.3))
                            
                            ParamRowInt(label: "Subdiv", value: $gameController.smoothSubdivisions, range: 1...500)
                            ParamRowInt(label: "Iters", value: $gameController.smoothIterations, range: 0...500)
                            ParamRow(label: "Smooth", value: $gameController.smoothStrength, range: 0...100.0, format: "%.2f")
                            ParamRow(label: "Zone", value: $gameController.smoothZone, range: 0.001...100.0, format: "%.3f")
                        }
                        .padding()
                    }
                    .frame(maxHeight: 400)
                    .background(Color.black.opacity(0.7))
                    .cornerRadius(12)
                    .padding(.horizontal, 20)
                    .padding(.top, 10)
                }
                
                Spacer()
            }
        }
    }
}

struct ParamRow: View {
    let label: String
    @Binding var value: Float
    let range: ClosedRange<Float>
    let format: String
    
    @State private var textValue: String = ""
    @FocusState private var isEditing: Bool
    
    var body: some View {
        HStack {
            Text(label)
                .foregroundColor(.white)
                .frame(width: 55, alignment: .leading)
            
            Slider(value: $value, in: range)
                .onChange(of: value) { _, newVal in
                    if !isEditing {
                        textValue = String(format: format, newVal)
                    }
                }
            
            TextField("", text: $textValue)
                .focused($isEditing)
                .frame(width: 70)
                .textFieldStyle(.roundedBorder)
                .font(.system(size: 11, design: .monospaced))
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

struct ParamRowInt: View {
    let label: String
    @Binding var value: Float
    let range: ClosedRange<Int>
    
    @State private var textValue: String = ""
    @FocusState private var isEditing: Bool
    
    var body: some View {
        HStack {
            Text(label)
                .foregroundColor(.white)
                .frame(width: 55, alignment: .leading)
            
            Slider(value: $value, in: Float(range.lowerBound)...Float(range.upperBound), step: 1)
                .onChange(of: value) { _, newVal in
                    if !isEditing {
                        textValue = "\(Int(newVal))"
                    }
                }
            
            TextField("", text: $textValue)
                .focused($isEditing)
                .frame(width: 50)
                .textFieldStyle(.roundedBorder)
                .font(.system(size: 11, design: .monospaced))
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
