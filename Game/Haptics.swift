import AVFoundation

#if os(iOS)
import UIKit

@MainActor
enum Haptics {
    static func light() {
        UIImpactFeedbackGenerator(style: .light).impactOccurred()
    }
    static func medium() {
        UIImpactFeedbackGenerator(style: .medium).impactOccurred()
    }
    static func success() {
        UINotificationFeedbackGenerator().notificationOccurred(.success)
        SoundPlayer.playVictory()
    }
}
#elseif os(macOS)
import AppKit

@MainActor
enum Haptics {
    static func light() {
        NSHapticFeedbackManager.defaultPerformer.perform(.alignment, performanceTime: .now)
    }
    static func medium() {
        NSHapticFeedbackManager.defaultPerformer.perform(.levelChange, performanceTime: .now)
    }
    static func success() {
        NSHapticFeedbackManager.defaultPerformer.perform(.generic, performanceTime: .now)
        SoundPlayer.playVictory()
    }
}
#endif

enum SoundPlayer {
    nonisolated(unsafe) private static var player: AVAudioPlayer?
    nonisolated(unsafe) private static var popPlayer: AVAudioPlayer?

    static func playVictory() {
        guard let url = Bundle.main.url(forResource: "victory", withExtension: "wav")
                ?? Bundle.main.url(forResource: "victory", withExtension: "mp3")
                ?? Bundle.main.url(forResource: "victory", withExtension: "aif") else {
            playSystemSound()
            return
        }
        do {
            player = try AVAudioPlayer(contentsOf: url)
            player?.volume = 0.6
            player?.play()
        } catch {
            playSystemSound()
        }
    }

    static func playRopeVanish() {
        let sampleRate: Double = 44100
        let duration: Double = 0.55
        let sampleCount = Int(sampleRate * duration)

        var samples = [Float](repeating: 0, count: sampleCount)
        var rng: UInt32 = 0xDEAD_BEEF

        for i in 0..<sampleCount {
            let t = Double(i) / sampleRate
            let progress = t / duration

            let env: Float
            if progress < 0.08 {
                env = Float(progress / 0.08)
            } else {
                let decay = (progress - 0.08) / 0.92
                env = Float(exp(-decay * 4.0))
            }

            rng = rng &* 1664525 &+ 1013904223
            let white = Float(Int32(bitPattern: rng)) / Float(Int32.max)

            let sweepFreq = 200.0 + progress * 600.0
            let sweepPhase = Float(sin(t * sweepFreq * 2.0 * .pi))
            let filtered = white * 0.6 + sweepPhase * 0.15

            let pitchUp = Float(sin(t * (400.0 + progress * 800.0) * 2.0 * .pi))

            samples[i] = (filtered * 0.7 + pitchUp * 0.12) * env * 0.18
        }

        let format = AVAudioFormat(standardFormatWithSampleRate: sampleRate, channels: 1)!
        guard let buffer = AVAudioPCMBuffer(pcmFormat: format, frameCapacity: AVAudioFrameCount(sampleCount)) else { return }
        buffer.frameLength = AVAudioFrameCount(sampleCount)
        if let channelData = buffer.floatChannelData {
            memcpy(channelData[0], samples, sampleCount * MemoryLayout<Float>.size)
        }

        do {
            let tempURL = FileManager.default.temporaryDirectory.appendingPathComponent("rope_vanish.wav")
            let file = try AVAudioFile(forWriting: tempURL, settings: format.settings)
            try file.write(from: buffer)

            popPlayer = try AVAudioPlayer(contentsOf: tempURL)
            popPlayer?.volume = 0.35
            popPlayer?.play()
        } catch {
            #if os(macOS)
            if let sound = NSSound(named: "Pop") ?? NSSound(named: "Tink") {
                sound.volume = 0.3
                sound.play()
            }
            #else
            AudioServicesPlaySystemSound(1054)
            #endif
        }
    }

    private static func playSystemSound() {
        #if os(macOS)
        if let sound = NSSound(named: "Glass") {
            sound.volume = 0.5
            sound.play()
        }
        #else
        AudioServicesPlaySystemSound(1025)
        #endif
    }
}
