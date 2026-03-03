import QuartzCore
import os.log

final class PhysicsProfiler: @unchecked Sendable {
    static let shared = PhysicsProfiler()

    private static let logger = Logger(subsystem: "com.uzls.four", category: "Profiler")

    enum Phase: String, CaseIterable {
        case verletIntegration = "verlet"
        case broadphase        = "broad"
        case frames            = "frames"
        case constraints       = "constr"
        case narrowphase       = "narrow"
        case postCollision     = "postCol"
        case meshBuild         = "mesh"
        case winCheck          = "winChk"
    }

    struct Sample {
        var sum: Double = 0
        var max: Double = 0
        var count: Int = 0

        var avg: Double { count > 0 ? sum / Double(count) : 0 }

        mutating func record(_ us: Double) {
            sum += us
            if us > max { max = us }
            count += 1
        }

        mutating func reset() {
            sum = 0; max = 0; count = 0
        }
    }

    struct Snapshot {
        let phase: Phase
        let avgUs: Double
        let maxUs: Double
        let count: Int
    }

    private var samples: [Phase: Sample] = {
        var d = [Phase: Sample]()
        for p in Phase.allCases { d[p] = Sample() }
        return d
    }()

    private var extraCounters: [String: Int] = [:]

    private var logInterval: Double = 2.0
    private var lastLogTime: Double = 0
    private var phaseStart: Double = 0

    var enabled: Bool = false {
        didSet {
            if enabled && !oldValue {
                reset()
                startTime = CACurrentMediaTime()
                Self.logger.warning("[PROF] ▶ started")
            } else if !enabled && oldValue {
                logFinal()
                Self.logger.warning("[PROF] ■ stopped")
            }
        }
    }

    private var startTime: Double = 0

    func begin() {
        guard enabled else { return }
        phaseStart = CACurrentMediaTime()
    }

    func end(_ phase: Phase) {
        guard enabled else { return }
        let elapsed = (CACurrentMediaTime() - phaseStart) * 1e6
        samples[phase]?.record(elapsed)
    }

    func measure<T>(_ phase: Phase, _ body: () -> T) -> T {
        guard enabled else { return body() }
        let t0 = CACurrentMediaTime()
        let result = body()
        let elapsed = (CACurrentMediaTime() - t0) * 1e6
        samples[phase]?.record(elapsed)
        return result
    }

    func record(_ phase: Phase, microseconds us: Double) {
        guard enabled else { return }
        samples[phase]?.record(us)
    }

    func setCounter(_ name: String, _ value: Int) {
        guard enabled else { return }
        extraCounters[name] = value
    }

    func snapshot() -> [Snapshot] {
        Phase.allCases.compactMap { phase in
            guard let s = samples[phase], s.count > 0 else { return nil }
            return Snapshot(phase: phase, avgUs: s.avg, maxUs: s.max, count: s.count)
        }
    }

    func summaryString() -> String {
        let snap = snapshot()
        guard !snap.isEmpty else { return "no data" }
        let totalAvg = snap.reduce(0.0) { $0 + $1.avgUs }
        let phases = snap.map { "\($0.phase.rawValue)=\(Int($0.avgUs))µs(max \(Int($0.maxUs)))" }
        var counters = "total=\(Int(totalAvg))µs"
        for (k, v) in extraCounters.sorted(by: { $0.key < $1.key }) {
            counters += " \(k)=\(v)"
        }
        return "\(counters) | \(phases.joined(separator: " "))"
    }

    func reset() {
        for phase in Phase.allCases { samples[phase]?.reset() }
        extraCounters.removeAll()
        lastLogTime = 0
    }

    func logIfNeeded() {
        guard enabled else { return }
        let now = CACurrentMediaTime()
        guard now - lastLogTime >= logInterval else { return }
        lastLogTime = now

        let line = summaryString()
        guard line != "no data" else { return }

        Self.logger.warning("[PROF] \(line, privacy: .public)")

        for phase in Phase.allCases {
            samples[phase]?.reset()
        }
        extraCounters.removeAll()
    }

    private func logFinal() {
        let elapsed = CACurrentMediaTime() - startTime
        let line = summaryString()
        Self.logger.warning("[PROF] final (\(String(format: "%.1f", elapsed), privacy: .public)s): \(line, privacy: .public)")
    }
}
