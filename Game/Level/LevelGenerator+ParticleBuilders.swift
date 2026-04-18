import Foundation
import simd

// MARK: - Particle position builders for structured levels

extension LevelGenerator {

    // MARK: - Helix builder

    static func buildHelixParticles(
        cfg: HelixConfig, P: Int, zBase: Float, convergeFrac: Float,
        ropeRadius: Float, colorIdx: inout Int,
        holes: inout [LevelDefinition.Vec2],
        ropes: inout [LevelDefinition.Rope],
        allParticles: inout [[LevelDefinition.Vec2]]
    ) {
        let n = cfg.strandCount
        let dir = simd_normalize(cfg.axis)
        let perp = SIMD2<Float>(-dir.y, dir.x)
        let halfLen = cfg.length / 2

        let minZRadius = n > 1 ? (ropeRadius * 0.6) / sin(Float.pi / Float(n)) : cfg.zRadius
        let zR = max(cfg.zRadius, minZRadius)

        let startPt = cfg.center - dir * halfLen
        let endPt = cfg.center + dir * halfLen

        for k in 0..<n {
            let phase = 2 * Float.pi * Float(k) / Float(n)

            let startOffset = perp * cfg.helixRadius * cos(phase)
            let startHolePos = startPt + startOffset
            let endAngle = 2 * Float.pi * cfg.turns + phase
            let endOffset = perp * cfg.helixRadius * cos(endAngle)
            let endHolePos = endPt + endOffset

            let axisOff = dir * Float(k) * 0.015

            let startHoleIdx = holes.count
            holes.append(.init(xPosition: startHolePos.x + axisOff.x,
                               yPosition: startHolePos.y + axisOff.y))
            let endHoleIdx = holes.count
            holes.append(.init(xPosition: endHolePos.x - axisOff.x,
                               yPosition: endHolePos.y - axisOff.y))

            ropes.append(.init(startHole: startHoleIdx, endHole: endHoleIdx,
                               color: colors[colorIdx % colors.count], radius: ropeRadius))
            colorIdx += 1

            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let t = Float(i) / Float(P - 1)
                let baseXY = startPt + axisOff + (endPt - axisOff - startPt - axisOff) * t
                let angle = 2 * Float.pi * cfg.turns * t + phase
                let env = smoothEnvelope(t: t, convergeFrac: convergeFrac)
                let helixOffset = perp * cfg.helixRadius * cos(angle)
                let x = baseXY.x + helixOffset.x
                let y = baseXY.y + helixOffset.y
                let z = max(0, zBase + zR * sin(angle) * env)
                particles.append(.init(xPosition: x, yPosition: y, zPosition: z))
            }

            particles[0] = .init(xPosition: startHolePos.x + axisOff.x,
                                 yPosition: startHolePos.y + axisOff.y, zPosition: 0)
            particles[P - 1] = .init(xPosition: endHolePos.x - axisOff.x,
                                     yPosition: endHolePos.y - axisOff.y, zPosition: 0)
            allParticles.append(particles)
        }
    }

    // MARK: - Classic braid builder

    static func buildClassicBraidParticles(
        cfg: BraidConfig, P: Int, zBase: Float,
        ropeRadius: Float, colorIdx: inout Int,
        holes: inout [LevelDefinition.Vec2],
        ropes: inout [LevelDefinition.Rope],
        allParticles: inout [[LevelDefinition.Vec2]]
    ) {
        let n = cfg.strandCount
        let dir = simd_normalize(cfg.axis)
        let perp = SIMD2<Float>(-dir.y, dir.x)
        let halfLen = cfg.length / 2
        let halfW = cfg.width / 2

        let slotX: [Float] = (0..<n).map { i in
            n == 1 ? 0 : -halfW + cfg.width * Float(i) / Float(n - 1)
        }

        struct SwapEvent {
            let t: Float
            let slotA: Int
            let slotB: Int
            let strandA: Int
            let strandB: Int
            let aGoesOver: Bool
        }

        var strandInSlot = Array(0..<n)
        var swaps: [SwapEvent] = []
        var overToggle = true

        for c in 0..<cfg.crossings {
            let t = (Float(c) + 0.5) / Float(cfg.crossings)
            let startSlot = c % 2 == 0 ? 0 : 1
            for s in stride(from: startSlot, to: n - 1, by: 2) {
                let sA = strandInSlot[s]
                let sB = strandInSlot[s + 1]
                swaps.append(SwapEvent(t: t, slotA: s, slotB: s + 1,
                                       strandA: sA, strandB: sB,
                                       aGoesOver: overToggle))
                strandInSlot[s] = sB
                strandInSlot[s + 1] = sA
                overToggle.toggle()
            }
        }

        for k in 0..<n {
            var keyframes: [(t: Float, slot: Int)] = [(0, k)]
            var curSlot = k
            for sw in swaps {
                if sw.strandA == k {
                    curSlot = sw.slotB
                    keyframes.append((sw.t, curSlot))
                } else if sw.strandB == k {
                    curSlot = sw.slotA
                    keyframes.append((sw.t, curSlot))
                }
            }
            keyframes.append((1.0, curSlot))

            let startSlotIdx = keyframes.first!.slot
            let endSlotIdx = keyframes.last!.slot
            let startPos = cfg.center - dir * halfLen + perp * slotX[startSlotIdx]
            let endPos = cfg.center + dir * halfLen + perp * slotX[endSlotIdx]

            let startHoleIdx = holes.count
            holes.append(.init(xPosition: startPos.x, yPosition: startPos.y))
            let endHoleIdx = holes.count
            holes.append(.init(xPosition: endPos.x, yPosition: endPos.y))

            ropes.append(.init(startHole: startHoleIdx, endHole: endHoleIdx,
                               color: colors[colorIdx % colors.count], radius: ropeRadius))
            colorIdx += 1

            let zOver = zBase + ropeRadius * 2.0
            let zUnder = max(0, zBase - ropeRadius * 0.5)

            struct ZKey { let t: Float; let z: Float }
            var zKeys: [ZKey] = [ZKey(t: 0, z: 0)]
            let halfCross: Float = 0.3 / Float(max(1, cfg.crossings))

            for sw in swaps {
                let isA = sw.strandA == k
                let isB = sw.strandB == k
                guard isA || isB else { continue }
                let over = isA ? sw.aGoesOver : !sw.aGoesOver
                let zAtCross = over ? zOver : zUnder
                let preT = max(0.005, sw.t - halfCross * 2)
                let postT = min(0.995, sw.t + halfCross * 2)
                zKeys.append(ZKey(t: preT, z: zAtCross))
                zKeys.append(ZKey(t: sw.t, z: zAtCross))
                zKeys.append(ZKey(t: postT, z: zBase))
            }
            zKeys.append(ZKey(t: 1, z: 0))
            zKeys.sort { $0.t < $1.t }

            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let t = Float(i) / Float(P - 1)

                var segIdx = 0
                for j in 1..<keyframes.count - 1 {
                    if keyframes[j].t <= t { segIdx = j }
                }
                let kf0 = keyframes[segIdx]
                let kf1 = keyframes[min(segIdx + 1, keyframes.count - 1)]
                let localT: Float
                if kf1.t > kf0.t {
                    localT = min(1, max(0, (t - kf0.t) / (kf1.t - kf0.t)))
                } else {
                    localT = 0
                }
                let smooth = localT * localT * (3 - 2 * localT)
                let slotOffset = slotX[kf0.slot] * (1 - smooth) + slotX[kf1.slot] * smooth

                let baseXY = cfg.center - dir * halfLen + dir * cfg.length * t
                let pos = baseXY + perp * slotOffset

                var zSeg = 0
                for j in 1..<zKeys.count {
                    if zKeys[j].t > t { break }
                    zSeg = j
                }
                let z0 = zKeys[zSeg]
                let z1 = zKeys[min(zSeg + 1, zKeys.count - 1)]
                let zt: Float = z1.t > z0.t ? min(1, max(0, (t - z0.t) / (z1.t - z0.t))) : 0
                let zSmooth = zt * zt * (3 - 2 * zt)
                let z = z0.z + (z1.z - z0.z) * zSmooth

                particles.append(.init(xPosition: pos.x, yPosition: pos.y, zPosition: max(0, z)))
            }

            particles[0] = .init(xPosition: startPos.x, yPosition: startPos.y, zPosition: 0)
            particles[P - 1] = .init(xPosition: endPos.x, yPosition: endPos.y, zPosition: 0)
            allParticles.append(particles)
        }
    }

    // MARK: - Central knot builder

    static func buildCentralKnotParticles(
        cfg: CentralKnotConfig, P: Int, zBase: Float,
        ropeRadius: Float, colorIdx: inout Int,
        holes: inout [LevelDefinition.Vec2],
        ropes: inout [LevelDefinition.Rope],
        allParticles: inout [[LevelDefinition.Vec2]]
    ) {
        let n = cfg.ropeCount
        let outerCount = n * 4
        let innerCount = max(4, n * 2)

        holes.append(.init(xPosition: cfg.center.x, yPosition: cfg.center.y))

        for i in 0..<innerCount {
            let angle = 2 * Float.pi * Float(i) / Float(innerCount)
            holes.append(.init(xPosition: cfg.center.x + cfg.innerRadius * cos(angle),
                               yPosition: cfg.center.y + cfg.innerRadius * sin(angle)))
        }

        var outerHoleIndices: [Int] = []
        for i in 0..<outerCount {
            let angle = 2 * Float.pi * Float(i) / Float(outerCount)
            outerHoleIndices.append(holes.count)
            holes.append(.init(xPosition: cfg.center.x + cfg.outerRadius * cos(angle),
                               yPosition: cfg.center.y + cfg.outerRadius * sin(angle)))
        }

        for k in 0..<n {
            let startOuter = k * (outerCount / n)
            let endOuter = (startOuter + outerCount / 2 + 1) % outerCount

            let startIdx = outerHoleIndices[startOuter]
            let endIdx = outerHoleIndices[endOuter]

            let startPos = SIMD2<Float>(holes[startIdx].xPosition, holes[startIdx].yPosition)
            let endPos = SIMD2<Float>(holes[endIdx].xPosition, holes[endIdx].yPosition)

            let mid = (startPos + endPos) / 2
            let dir = simd_normalize(endPos - startPos)
            let perp = SIMD2<Float>(-dir.y, dir.x)
            let controlPt = mid + perp * cfg.loopDepth * (k % 2 == 0 ? 1 : -1)

            ropes.append(.init(startHole: startIdx, endHole: endIdx,
                               color: colors[colorIdx % colors.count], radius: ropeRadius))
            colorIdx += 1

            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let t = Float(i) / Float(P - 1)
                let u = 1 - t
                let xy = u * u * startPos + 2 * u * t * controlPt + t * t * endPos

                let zStep = ropeRadius * 1.5
                var z = zBase + zStep * Float(k) + cfg.zRadius * 0.3
                let env = smoothEnvelope(t: t, convergeFrac: 0.03)
                z = z * env

                particles.append(.init(xPosition: xy.x, yPosition: xy.y, zPosition: max(0, z)))
            }

            particles[0] = .init(xPosition: startPos.x, yPosition: startPos.y, zPosition: 0)
            particles[P - 1] = .init(xPosition: endPos.x, yPosition: endPos.y, zPosition: 0)
            allParticles.append(particles)
        }
    }

    // MARK: - Vortex builder

    static func buildVortexParticles(
        cfg: VortexConfig, P: Int, zBase: Float,
        ropeRadius: Float, colorIdx: inout Int,
        holes: inout [LevelDefinition.Vec2],
        ropes: inout [LevelDefinition.Rope],
        allParticles: inout [[LevelDefinition.Vec2]]
    ) {
        let n = cfg.ropeCount

        for k in 0..<n {
            let baseAngle = 2 * Float.pi * Float(k) / Float(n)
            let dir = SIMD2<Float>(cos(baseAngle), sin(baseAngle))

            let startPos = cfg.center - dir * cfg.armLength
            let endPos = cfg.center + dir * cfg.armLength

            let startHoleIdx = holes.count
            holes.append(.init(xPosition: startPos.x, yPosition: startPos.y))
            let endHoleIdx = holes.count
            holes.append(.init(xPosition: endPos.x, yPosition: endPos.y))

            ropes.append(.init(startHole: startHoleIdx, endHole: endHoleIdx,
                               color: colors[colorIdx % colors.count], radius: ropeRadius))
            colorIdx += 1

            let phase = 2 * Float.pi * Float(k) / Float(n)

            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let t = Float(i) / Float(P - 1)
                let baseXY = startPos + (endPos - startPos) * t

                let halfZone = cfg.knotRadius
                let distFromCenter = abs(t - 0.5) * 2
                let normalizedDist = distFromCenter * cfg.armLength
                let env: Float
                if normalizedDist >= halfZone {
                    env = 0
                } else {
                    let s = normalizedDist / halfZone
                    env = 1 - s * s * (3 - 2 * s)
                }

                let angle = phase + 2 * Float.pi * cfg.turns * t
                let perp = SIMD2<Float>(-dir.y, dir.x)
                let orbitXY = perp * cfg.orbitRadius * cos(angle) * env
                let z = max(0, zBase + cfg.zRadius * sin(angle) * env)

                particles.append(.init(xPosition: baseXY.x + orbitXY.x,
                                       yPosition: baseXY.y + orbitXY.y,
                                       zPosition: z))
            }

            particles[0] = .init(xPosition: startPos.x, yPosition: startPos.y, zPosition: 0)
            particles[P - 1] = .init(xPosition: endPos.x, yPosition: endPos.y, zPosition: 0)
            allParticles.append(particles)
        }
    }

    // MARK: - Weave builder

    static func buildWeaveParticles(
        cfg: WeaveConfig, P: Int, zBase: Float,
        ropeRadius: Float, colorIdx: inout Int,
        holes: inout [LevelDefinition.Vec2],
        ropes: inout [LevelDefinition.Rope],
        allParticles: inout [[LevelDefinition.Vec2]]
    ) {
        let halfW = cfg.width / 2
        let halfH = cfg.height / 2
        let bumpWidth: Float = 0.06

        let colXs = (0..<cfg.vCount).map { c in
            cfg.center.x - halfW + cfg.width * Float(c) / Float(max(1, cfg.vCount - 1))
        }
        let rowYs = (0..<cfg.hCount).map { r in
            cfg.center.y - halfH + cfg.height * Float(r) / Float(max(1, cfg.hCount - 1))
        }

        // Horizontal ropes
        for row in 0..<cfg.hCount {
            let y = rowYs[row]
            let startPos = SIMD2<Float>(cfg.center.x - halfW, y)
            let endPos = SIMD2<Float>(cfg.center.x + halfW, y)

            let startHoleIdx = holes.count
            holes.append(.init(xPosition: startPos.x, yPosition: startPos.y))
            let endHoleIdx = holes.count
            holes.append(.init(xPosition: endPos.x, yPosition: endPos.y))

            ropes.append(.init(startHole: startHoleIdx, endHole: endHoleIdx,
                               color: colors[colorIdx % colors.count], radius: ropeRadius))
            colorIdx += 1

            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let t = Float(i) / Float(P - 1)
                let x = startPos.x + (endPos.x - startPos.x) * t
                let env = smoothEnvelope(t: t, convergeFrac: 0.08)

                var zBump: Float = 0
                for (col, colX) in colXs.enumerated() {
                    let dx = x - colX
                    let bump = exp(-(dx * dx) / (2 * bumpWidth * bumpWidth))
                    let sign: Float = (row + col) % 2 == 0 ? 1 : -1
                    zBump += sign * cfg.zRadius * bump
                }

                let z = max(0, zBase + zBump * env)
                particles.append(.init(xPosition: x, yPosition: y, zPosition: z))
            }
            particles[0] = .init(xPosition: startPos.x, yPosition: startPos.y, zPosition: 0)
            particles[P - 1] = .init(xPosition: endPos.x, yPosition: endPos.y, zPosition: 0)
            allParticles.append(particles)
        }

        // Vertical ropes
        for col in 0..<cfg.vCount {
            let x = colXs[col]
            let startPos = SIMD2<Float>(x, cfg.center.y + halfH)
            let endPos = SIMD2<Float>(x, cfg.center.y - halfH)

            let startHoleIdx = holes.count
            holes.append(.init(xPosition: startPos.x, yPosition: startPos.y))
            let endHoleIdx = holes.count
            holes.append(.init(xPosition: endPos.x, yPosition: endPos.y))

            ropes.append(.init(startHole: startHoleIdx, endHole: endHoleIdx,
                               color: colors[colorIdx % colors.count], radius: ropeRadius))
            colorIdx += 1

            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let t = Float(i) / Float(P - 1)
                let y = startPos.y + (endPos.y - startPos.y) * t
                let env = smoothEnvelope(t: t, convergeFrac: 0.08)

                var zBump: Float = 0
                for (row, rowY) in rowYs.enumerated() {
                    let dy = y - rowY
                    let bump = exp(-(dy * dy) / (2 * bumpWidth * bumpWidth))
                    let sign: Float = (row + col) % 2 == 0 ? -1 : 1
                    zBump += sign * cfg.zRadius * bump
                }

                let z = max(0, zBase + zBump * env)
                particles.append(.init(xPosition: x, yPosition: y, zPosition: z))
            }
            particles[0] = .init(xPosition: startPos.x, yPosition: startPos.y, zPosition: 0)
            particles[P - 1] = .init(xPosition: endPos.x, yPosition: endPos.y, zPosition: 0)
            allParticles.append(particles)
        }
    }
}
