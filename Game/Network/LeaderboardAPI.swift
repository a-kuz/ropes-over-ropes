import Foundation

@MainActor
final class LeaderboardAPI {
    static let shared = LeaderboardAPI()

    #if DEBUG
    private let baseURL = "https://uzls-leaderboard.a-kuz.workers.dev"
    #else
    private let baseURL = "https://uzls-leaderboard.a-kuz.workers.dev"
    #endif

    private let ud = UserDefaults.standard
    private let session: URLSession = {
        let cfg = URLSessionConfiguration.default
        cfg.timeoutIntervalForRequest = 8
        cfg.timeoutIntervalForResource = 15
        cfg.waitsForConnectivity = false
        return URLSession(configuration: cfg)
    }()

    var playerId: String {
        get {
            if let id = ud.string(forKey: "lb.playerId") { return id }
            let id = UUID().uuidString.lowercased()
            ud.set(id, forKey: "lb.playerId")
            return id
        }
        set { ud.set(newValue, forKey: "lb.playerId") }
    }

    var username: String {
        get { ud.string(forKey: "lb.username") ?? "" }
        set { ud.set(newValue, forKey: "lb.username") }
    }

    private var flushing = false

    private init() {}

    // MARK: - Pending queue (persisted)

    struct PendingResult: Codable {
        let levelId: Int
        let moves: Int
        let timeMs: Int
        let date: Date
    }

    private var pendingQueue: [PendingResult] {
        get {
            guard let data = ud.data(forKey: "lb.pending") else { return [] }
            return (try? JSONDecoder().decode([PendingResult].self, from: data)) ?? []
        }
        set {
            ud.set(try? JSONEncoder().encode(newValue), forKey: "lb.pending")
        }
    }

    private func enqueue(levelId: Int, moves: Int, timeMs: Int) {
        var q = pendingQueue
        q.append(PendingResult(levelId: levelId, moves: moves, timeMs: timeMs, date: Date()))
        pendingQueue = q
    }

    // MARK: - Player (stateless — just generates username if needed)

    func ensureRegistered() async {
        if !username.isEmpty {
            await flushQueue()
            return
        }
        do {
            let resp: PlayerResponse = try await post("/api/player", body: ["id": playerId])
            username = resp.username
            await flushQueue()
        } catch {
            print("[Leaderboard] registration failed: \(error)")
        }
    }

    struct PlayerResponse: Decodable {
        let id: String
        let username: String
    }

    func loginAs(_ name: String) {
        let newId = UUID().uuidString.lowercased()
        playerId = newId
        username = name
    }

    func setUsername(_ newName: String) {
        username = newName
    }

    // MARK: - Submit

    struct SubmitResponse: Decodable {
        let total_completions: Int
        let avg_moves: Double
        let best_moves: Int
        let player_best: Int?
        let percentile: Int?
        let is_personal_best: Bool
    }

    @discardableResult
    func submitResult(levelId: Int, moves: Int, timeMs: Int) async -> SubmitResponse? {
        let body: [String: Any] = [
            "player_id": playerId,
            "username": username.isEmpty ? "anon" : username,
            "level_id": levelId,
            "moves": moves,
            "time_ms": timeMs,
        ]

        do {
            let resp: SubmitResponse = try await post("/api/submit", body: body)
            return resp
        } catch {
            print("[Leaderboard] submit failed, queued: \(error)")
            enqueue(levelId: levelId, moves: moves, timeMs: timeMs)
            return nil
        }
    }

    // MARK: - Flush queue

    func flushQueue() async {
        guard !flushing else { return }
        let queue = pendingQueue
        guard !queue.isEmpty else { return }
        flushing = true
        defer { flushing = false }

        var remaining: [PendingResult] = []
        for item in queue {
            let body: [String: Any] = [
                "player_id": playerId,
                "username": username.isEmpty ? "anon" : username,
                "level_id": item.levelId,
                "moves": item.moves,
                "time_ms": item.timeMs,
            ]
            do {
                let _: SubmitResponse = try await post("/api/submit", body: body)
            } catch {
                remaining.append(item)
            }
        }
        pendingQueue = remaining
    }

    // MARK: - Stats

    struct StatsResponse: Decodable {
        let total_completions: Int
        let avg_moves: Double
        let best_moves: Int
        let player_best: Int?
        let percentile: Int?
    }

    func fetchStats(levelId: Int, moves: Int? = nil) async -> StatsResponse? {
        var url = "\(baseURL)/api/stats/\(levelId)?player_id=\(playerId)"
        if let m = moves { url += "&moves=\(m)" }
        return try? await get(url)
    }

    // MARK: - Networking

    enum LeaderboardError: Error {
        case httpError(Int)
    }

    private func post<T: Decodable>(_ path: String, body: Any) async throws -> T {
        let url = URL(string: "\(baseURL)\(path)")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        request.httpBody = try JSONSerialization.data(withJSONObject: body)

        let (data, response) = try await session.data(for: request)
        guard let http = response as? HTTPURLResponse, (200...299).contains(http.statusCode) else {
            let code = (response as? HTTPURLResponse)?.statusCode ?? 0
            throw LeaderboardError.httpError(code)
        }
        return try JSONDecoder().decode(T.self, from: data)
    }

    private func get<T: Decodable>(_ urlString: String) async throws -> T {
        let url = URL(string: urlString)!
        let request = URLRequest(url: url)

        let (data, response) = try await session.data(for: request)
        guard let http = response as? HTTPURLResponse, (200...299).contains(http.statusCode) else {
            let code = (response as? HTTPURLResponse)?.statusCode ?? 0
            throw LeaderboardError.httpError(code)
        }
        return try JSONDecoder().decode(T.self, from: data)
    }
}
