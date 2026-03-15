import AVFoundation
import simd

final class RubberFrictionSound: @unchecked Sendable {
    private let engine = AVAudioEngine()
    private let sourceNode: AVAudioSourceNode
    private let mixerNode = AVAudioMixerNode()

    private var smoothedVolume: Float = 0
    private var smoothedPitch: Float = 1800
    private var smoothedModRate: Float = 40
    private var targetVolume: Float = 0
    private var targetPitch: Float = 1800
    private var targetModRate: Float = 40
    private var isRunning = false
    var enabled: Bool = true

    private let sampleRate: Double = 44100

    nonisolated(unsafe) private static var _phase1: Double = 0
    nonisolated(unsafe) private static var _phase2: Double = 0
    nonisolated(unsafe) private static var _phase3: Double = 0
    nonisolated(unsafe) private static var _modPhase: Double = 0
    nonisolated(unsafe) private static var _currentPitch: Float = 1800
    nonisolated(unsafe) private static var _currentModRate: Float = 40
    nonisolated(unsafe) private static var _currentVolume: Float = 0
    nonisolated(unsafe) private static var _noiseState: UInt32 = 12345

    init() {
        let sr = sampleRate

        sourceNode = AVAudioSourceNode { _, _, frameCount, bufferList -> OSStatus in
            let abl = UnsafeMutableAudioBufferListPointer(bufferList)
            let pitch = Double(Self._currentPitch)
            let modRate = Double(Self._currentModRate)
            let vol = Self._currentVolume

            for buf in abl {
                let ptr = buf.mData!.assumingMemoryBound(to: Float.self)
                for i in 0..<Int(frameCount) {
                    let modEnv = Float(0.5 + 0.5 * sin(Self._modPhase * 2.0 * .pi))
                    let stickSlip = modEnv * modEnv

                    let sin1 = Float(sin(Self._phase1 * 2.0 * .pi))
                    let sin2 = Float(sin(Self._phase2 * 2.0 * .pi)) * 0.5
                    let sin3 = Float(sin(Self._phase3 * 2.0 * .pi)) * 0.25

                    Self._noiseState = Self._noiseState &* 1664525 &+ 1013904223
                    let noise = (Float(Self._noiseState) / Float(UInt32.max)) * 2.0 - 1.0

                    let tonal = (sin1 + sin2 + sin3) / 1.75
                    let sample = (tonal * 0.7 + noise * 0.3) * stickSlip * vol

                    ptr[i] = sample

                    Self._phase1 += pitch / sr
                    Self._phase2 += (pitch * 2.73) / sr
                    Self._phase3 += (pitch * 4.17) / sr
                    Self._modPhase += modRate / sr

                    if Self._phase1 > 1 { Self._phase1 -= 1 }
                    if Self._phase2 > 1 { Self._phase2 -= 1 }
                    if Self._phase3 > 1 { Self._phase3 -= 1 }
                    if Self._modPhase > 1 { Self._modPhase -= 1 }
                }
            }
            return noErr
        }

        let format = AVAudioFormat(standardFormatWithSampleRate: sampleRate, channels: 1)!

        engine.attach(sourceNode)
        engine.attach(mixerNode)

        engine.connect(sourceNode, to: mixerNode, format: format)
        engine.connect(mixerNode, to: engine.mainMixerNode, format: format)

        mixerNode.outputVolume = 0
    }

    func start() {
        guard !isRunning else { return }
        do {
            #if os(iOS)
            try AVAudioSession.sharedInstance().setCategory(.ambient, mode: .default)
            try AVAudioSession.sharedInstance().setActive(true)
            #endif
            try engine.start()
            isRunning = true
        } catch {
            print("[RubberFrictionSound] Failed to start: \(error)")
        }
    }

    func update(intensity: Float, speed: Float) {
        guard enabled else {
            if smoothedVolume > 0.001 {
                smoothedVolume *= 0.85
                Self._currentVolume = smoothedVolume
                mixerNode.outputVolume = min(smoothedVolume * 3.0, 1.0)
            }
            return
        }

        if !isRunning { start() }

        let speedNorm = min(speed / 0.008, 1.0)
        let overlapNorm = min(intensity / 0.02, 1.0)
        let combined = speedNorm * overlapNorm

        targetVolume = combined * 0.4
        targetPitch = 1200 + speedNorm * 2000
        targetModRate = 25 + speedNorm * 80

        let attackSmooth: Float = 0.12
        let releaseSmooth: Float = 0.04
        let volumeSmooth: Float = targetVolume > smoothedVolume ? attackSmooth : releaseSmooth
        smoothedVolume += (targetVolume - smoothedVolume) * volumeSmooth
        smoothedPitch += (targetPitch - smoothedPitch) * 0.08
        smoothedModRate += (targetModRate - smoothedModRate) * 0.08

        Self._currentPitch = smoothedPitch
        Self._currentModRate = smoothedModRate
        Self._currentVolume = smoothedVolume
        mixerNode.outputVolume = min(smoothedVolume * 3.0, 1.0)
    }

    func fadeOut() {
        targetVolume = 0
        smoothedVolume *= 0.88
        if smoothedVolume < 0.0005 { smoothedVolume = 0 }
        Self._currentVolume = smoothedVolume
        mixerNode.outputVolume = min(smoothedVolume * 3.0, 1.0)
    }
}
