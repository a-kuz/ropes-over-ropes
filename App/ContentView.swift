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
                            HStack {
                                Text("Step")
                                    .foregroundColor(.white)
                                    .frame(width: 55, alignment: .leading)
                                Slider(value: $gameController.stepMultiplier, in: 0.0001...100.0)
                                Text(String(format: "%.4f", gameController.stepMultiplier))
                                    .foregroundColor(.white)
                                    .frame(width: 60)
                                    .font(.system(size: 12))
                            }
                            
                            HStack {
                                Text("R")
                                    .foregroundColor(.white)
                                    .frame(width: 55, alignment: .leading)
                                Slider(value: $gameController.hookRadiusMultiplier, in: 0.001...100.0)
                                Text(String(format: "%.3f", gameController.hookRadiusMultiplier))
                                    .foregroundColor(.white)
                                    .frame(width: 60)
                                    .font(.system(size: 12))
                            }
                            
                            HStack {
                                Text("Limit")
                                    .foregroundColor(.white)
                                    .frame(width: 55, alignment: .leading)
                                Slider(value: $gameController.stepLimitMultiplier, in: 0.0001...100.0)
                                Text(String(format: "%.4f", gameController.stepLimitMultiplier))
                                    .foregroundColor(.white)
                                    .frame(width: 60)
                                    .font(.system(size: 12))
                            }
                            
                            Toggle("Segments", isOn: $gameController.debugSegmentColors)
                                .foregroundColor(.white)
                            
                            Divider().background(Color.white.opacity(0.3))
                            
                            HStack {
                                Text("Subdiv")
                                    .foregroundColor(.white)
                                    .frame(width: 55, alignment: .leading)
                                Slider(value: $gameController.smoothSubdivisions, in: 1...200, step: 1)
                                Text("\(Int(gameController.smoothSubdivisions))")
                                    .foregroundColor(.white)
                                    .frame(width: 40)
                            }
                            
                            HStack {
                                Text("Iters")
                                    .foregroundColor(.white)
                                    .frame(width: 55, alignment: .leading)
                                Slider(value: $gameController.smoothIterations, in: 0...200, step: 1)
                                Text("\(Int(gameController.smoothIterations))")
                                    .foregroundColor(.white)
                                    .frame(width: 40)
                            }
                            
                            HStack {
                                Text("Smooth")
                                    .foregroundColor(.white)
                                    .frame(width: 55, alignment: .leading)
                                Slider(value: $gameController.smoothStrength, in: 0...10.0)
                                Text(String(format: "%.2f", gameController.smoothStrength))
                                    .foregroundColor(.white)
                                    .frame(width: 40)
                            }
                            
                            HStack {
                                Text("Zone")
                                    .foregroundColor(.white)
                                    .frame(width: 55, alignment: .leading)
                                Slider(value: $gameController.smoothZone, in: 0.01...10.0)
                                Text(String(format: "%.2f", gameController.smoothZone))
                                    .foregroundColor(.white)
                                    .frame(width: 40)
                            }
                        }
                        .padding()
                    }
                    .frame(maxHeight: 350)
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
