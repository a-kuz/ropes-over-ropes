package com.uzls.four.ui

import android.content.Context
import android.content.SharedPreferences
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json
import java.net.HttpURLConnection
import java.net.URL
import java.util.UUID

class LeaderboardAPI(context: Context) {
    companion object {
        private const val BASE_URL = "https://uzls-leaderboard.a-kuz.workers.dev"
    }

    private val prefs: SharedPreferences =
        context.getSharedPreferences("uzls_leaderboard", Context.MODE_PRIVATE)
    private val json = Json { ignoreUnknownKeys = true; isLenient = true }

    var playerId: String
        get() {
            val id = prefs.getString("playerId", null)
            if (id != null) return id
            val newId = UUID.randomUUID().toString()
            prefs.edit().putString("playerId", newId).apply()
            return newId
        }
        set(value) { prefs.edit().putString("playerId", value).apply() }

    var username: String
        get() = prefs.getString("username", "") ?: ""
        set(value) { prefs.edit().putString("username", value).apply() }

    @Serializable
    data class PlayerResponse(val id: String, val username: String)

    @Serializable
    data class SubmitResponse(
        val total_completions: Int = 0,
        val avg_moves: Double = 0.0,
        val best_moves: Int = 0,
        val player_best: Int? = null,
        val percentile: Int? = null,
        val is_personal_best: Boolean = false,
    )

    @Serializable
    data class StatsResponse(
        val total_completions: Int = 0,
        val avg_moves: Double = 0.0,
        val best_moves: Int = 0,
        val player_best: Int? = null,
        val percentile: Int? = null,
    )

    suspend fun ensureRegistered() {
        if (username.isNotEmpty()) return
        try {
            val body = """{"id":"$playerId"}"""
            val resp = post("/api/player", body)
            val player = json.decodeFromString<PlayerResponse>(resp)
            username = player.username
        } catch (_: Exception) { }
    }

    suspend fun submitResult(levelId: Int, moves: Int, timeMs: Int): SubmitResponse? {
        val name = username.ifEmpty { "anon" }
        val body = """{"player_id":"$playerId","username":"$name","level_id":$levelId,"moves":$moves,"time_ms":$timeMs}"""
        return try {
            val resp = post("/api/submit", body)
            json.decodeFromString<SubmitResponse>(resp)
        } catch (_: Exception) {
            enqueue(levelId, moves, timeMs)
            null
        }
    }

    suspend fun fetchStats(levelId: Int, moves: Int): StatsResponse? {
        return try {
            val resp = get("/api/stats/$levelId?player_id=$playerId&moves=$moves")
            json.decodeFromString<StatsResponse>(resp)
        } catch (_: Exception) { null }
    }

    fun loginAs(name: String) {
        playerId = UUID.randomUUID().toString()
        username = name
    }

    private fun enqueue(levelId: Int, moves: Int, timeMs: Int) {
        val pending = prefs.getString("pending", "") ?: ""
        val entry = "$levelId,$moves,$timeMs"
        prefs.edit().putString("pending", if (pending.isEmpty()) entry else "$pending|$entry").apply()
    }

    suspend fun flushQueue() {
        val pending = prefs.getString("pending", "") ?: ""
        if (pending.isEmpty()) return
        val entries = pending.split("|")
        val remaining = mutableListOf<String>()
        for (entry in entries) {
            val parts = entry.split(",")
            if (parts.size != 3) continue
            try {
                val name = username.ifEmpty { "anon" }
                val body = """{"player_id":"$playerId","username":"$name","level_id":${parts[0]},"moves":${parts[1]},"time_ms":${parts[2]}}"""
                post("/api/submit", body)
            } catch (_: Exception) {
                remaining.add(entry)
            }
        }
        prefs.edit().putString("pending", remaining.joinToString("|")).apply()
    }

    private suspend fun post(path: String, body: String): String = withContext(Dispatchers.IO) {
        val conn = URL("$BASE_URL$path").openConnection() as HttpURLConnection
        conn.requestMethod = "POST"
        conn.setRequestProperty("Content-Type", "application/json")
        conn.connectTimeout = 8000
        conn.readTimeout = 8000
        conn.doOutput = true
        conn.outputStream.use { it.write(body.toByteArray()) }
        conn.inputStream.bufferedReader().readText()
    }

    private suspend fun get(path: String): String = withContext(Dispatchers.IO) {
        val conn = URL("$BASE_URL$path").openConnection() as HttpURLConnection
        conn.connectTimeout = 8000
        conn.readTimeout = 8000
        conn.inputStream.bufferedReader().readText()
    }
}
