package com.uzls.four.level

/**
 * XorShift64 PRNG — bit-identical to the Swift implementation.
 * Uses ULong for unsigned 64-bit arithmetic with the same shifts: shl 13, shr 7, shl 17.
 */
class SeededRNG(seed: ULong) {
    private var state: ULong = if (seed == 0UL) 1UL else seed

    fun nextUInt64(): ULong {
        state = state xor (state shl 13)
        state = state xor (state shr 7)
        state = state xor (state shl 17)
        return state
    }

    fun next(bound: Int): Int {
        if (bound <= 0) return 0
        return (nextUInt64() % bound.toULong()).toInt()
    }
}
