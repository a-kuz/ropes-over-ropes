import AVFoundation

#if os(iOS)
import UIKit

enum Haptics {
    static func light()   { UIImpactFeedbackGenerator(style: .light).impactOccurred() }
    static func medium()  { UIImpactFeedbackGenerator(style: .medium).impactOccurred() }
    static func success() {
        UINotificationFeedbackGenerator().notificationOccurred(.success)
        SoundPlayer.playVictory()
    }
}
#elseif os(macOS)
import AppKit

enum Haptics {
    static func light()   { NSHapticFeedbackManager.defaultPerformer.perform(.alignment, performanceTime: .now) }
    static func medium()  { NSHapticFeedbackManager.defaultPerformer.perform(.levelChange, performanceTime: .now) }
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
        let duration: Double = 0.35
        let sampleCount = Int(sampleRate * duration)

        var samples = [Float](repeating: 0, count: sampleCount)
        var phase1: Double = 0
        var phase2: Double = 0

        for i in 0..<sampleCount {
            let t = Double(i) / sampleRate
            let progress = t / duration

            let freqBase = 800.0 + progress * 1400.0
            let freq2 = freqBase * 1.5

            let env: Float
            if progress < 0.05 {
                env = Float(progress / 0.05)
            } else {
                let decay = (progress - 0.05) / 0.95
                env = Float(1.0 - decay * decay)
            }

            let s1 = Float(sin(phase1 * 2.0 * .pi))
            let s2 = Float(sin(phase2 * 2.0 * .pi)) * 0.4

            samples[i] = (s1 + s2) * env * 0.3

            phase1 += freqBase / sampleRate
            phase2 += freq2 / sampleRate
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
            popPlayer?.volume = 0.5
            popPlayer?.play()
        } catch {
            #if os(macOS)
            if let sound = NSSound(named: "Pop") ?? NSSound(named: "Tink") {
                sound.volume = 0.4
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
