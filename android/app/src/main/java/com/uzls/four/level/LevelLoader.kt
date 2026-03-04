package com.uzls.four.level

import android.content.Context
import android.util.Log
import kotlinx.serialization.json.Json

object LevelLoader {

    private const val TAG = "LevelLoader"

    private val json = Json {
        ignoreUnknownKeys = true
        isLenient = true
    }

    fun load(context: Context, levelId: Int): LevelDefinition? {
        Log.i(TAG, "Attempting to load level $levelId")
        val name = String.format("level_%03d.json", levelId)

        // Try "levels/" subdirectory first
        var jsonString = tryLoadAsset(context, "levels/$name")

        // Fallback to root assets
        if (jsonString == null) {
            Log.w(TAG, "Level file not found in 'levels' subdirectory, trying root assets")
            jsonString = tryLoadAsset(context, name)
        }

        if (jsonString == null) {
            Log.e(TAG, "Level file not found: $name")
            return null
        }

        Log.i(TAG, "Loaded ${jsonString.length} chars from level file")
        Log.i(TAG, "JSON content preview (first 200 chars): ${jsonString.take(200)}")

        return try {
            val level = json.decodeFromString<LevelDefinition>(jsonString)
            Log.i(TAG, "Level $levelId loaded successfully: ${level.ropes.size} ropes, ${level.holes.size} holes")
            Log.i(TAG, "Level ID: ${level.id}, holeRadius: ${level.holeRadius}, particlesPerRope: ${level.particlesPerRope}")
            level
        } catch (e: Exception) {
            Log.e(TAG, "Failed to decode level $levelId: ${e.localizedMessage}")
            Log.e(TAG, "Error type: ${e::class.simpleName}")
            null
        }
    }

    private fun tryLoadAsset(context: Context, path: String): String? {
        return try {
            context.assets.open(path).bufferedReader().use { it.readText() }
        } catch (e: Exception) {
            null
        }
    }
}
