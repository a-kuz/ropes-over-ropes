import Metal
import simd

struct RopeSimParams {
    var deltaTime: Float
    var gravity: SIMD3<Float>
    var stretchStiffness: Float
    var iterations: UInt32
    var particleRadius: Float
    var damping: Float
}

struct GridParams {
    var origin: SIMD2<Float>
    var cellSize: Float
    var gridWidth: UInt32
    var gridHeight: UInt32
    var particleCount: UInt32
    var particlesPerRope: UInt32
}

struct RopeMetaGPU {
    var offset: UInt32
    var count: UInt32
    var restLengthBits: UInt32
    var pad1: UInt32
    var pinA: SIMD4<Float>
    var pinB: SIMD4<Float>
}

final class RopeSimulation {
    private let device: MTLDevice
    private let queue: MTLCommandQueue
    private let ropePipeline: MTLComputePipelineState
    private let projectPipeline: MTLComputePipelineState
    private let gridClearPipeline: MTLComputePipelineState
    private let gridBuildPipeline: MTLComputePipelineState
    private let gridCollidePipeline: MTLComputePipelineState

    private(set) var particlePos: MTLBuffer
    private var particlePrev: MTLBuffer
    private var targetPos: MTLBuffer
    private var ropeMeta: MTLBuffer
    private var paramsBuf: MTLBuffer
    private var gridParamsBuf: MTLBuffer
    private var gridHeads: MTLBuffer
    private var gridNext: MTLBuffer

    let ropeCount: Int
    let particlesPerRope: Int
    var projectAlpha: Float = 1.0
    var collisionsEnabled: Bool = false
    var simulationEnabled: Bool = true

    init(device: MTLDevice, ropeCount: Int = 1, particlesPerRope: Int = 6400) {
        self.device = device
        guard let commandQueue = device.makeCommandQueue() else {
            fatalError("Failed to create MTLCommandQueue")
        }
        self.queue = commandQueue
        self.ropeCount = max(1, ropeCount)
        self.particlesPerRope = max(8, particlesPerRope)

        guard let library = device.makeDefaultLibrary(),
              let ropeFunction = library.makeFunction(name: "ropeSimStep"),
              let projectFunction = library.makeFunction(name: "ropeProjectToTargets"),
              let clearFunction = library.makeFunction(name: "gridClear"),
              let buildFunction = library.makeFunction(name: "gridBuild"),
              let collideFunction = library.makeFunction(name: "gridCollide") else {
            fatalError("Failed to load simulation functions")
        }

        do {
            self.ropePipeline = try device.makeComputePipelineState(function: ropeFunction)
            self.projectPipeline = try device.makeComputePipelineState(function: projectFunction)
            self.gridClearPipeline = try device.makeComputePipelineState(function: clearFunction)
            self.gridBuildPipeline = try device.makeComputePipelineState(function: buildFunction)
            self.gridCollidePipeline = try device.makeComputePipelineState(function: collideFunction)
        } catch {
            fatalError(String(describing: error))
        }

        let total = self.ropeCount * self.particlesPerRope
        self.particlePos = device.makeBuffer(length: total * MemoryLayout<SIMD4<Float>>.stride, options: [.storageModeShared])!
        self.particlePrev = device.makeBuffer(length: total * MemoryLayout<SIMD4<Float>>.stride, options: [.storageModeShared])!
        self.targetPos = device.makeBuffer(length: total * MemoryLayout<SIMD4<Float>>.stride, options: [.storageModeShared])!
        self.ropeMeta = device.makeBuffer(length: self.ropeCount * MemoryLayout<RopeMetaGPU>.stride, options: [.storageModeShared])!
        self.paramsBuf = device.makeBuffer(length: MemoryLayout<RopeSimParams>.stride, options: [.storageModeShared])!
        self.gridParamsBuf = device.makeBuffer(length: MemoryLayout<GridParams>.stride, options: [.storageModeShared])!

        let gridWidth: UInt32 = 64
        let gridHeight: UInt32 = 64
        let gridCellCount = Int(gridWidth * gridHeight)
        self.gridHeads = device.makeBuffer(length: gridCellCount * MemoryLayout<Int32>.stride, options: [.storageModeShared])!
        self.gridNext = device.makeBuffer(length: total * MemoryLayout<Int32>.stride, options: [.storageModeShared])!

        let grid = GridParams(
            origin: SIMD2<Float>(-3.0, -3.0),
            cellSize: 0.16,
            gridWidth: gridWidth,
            gridHeight: gridHeight,
            particleCount: UInt32(total),
            particlesPerRope: UInt32(self.particlesPerRope)
        )
        gridParamsBuf.contents().copyMemory(from: [grid], byteCount: MemoryLayout<GridParams>.stride)

        seed()
        setPins(
            ropeIndex: 0,
            pinStart: SIMD3<Float>(-0.82, -0.86, 0),
            pinEnd: SIMD3<Float>(0.83, 0.49, 0)
        )
    }

    func setPins(ropeIndex: Int, pinStart: SIMD3<Float>, pinEnd: SIMD3<Float>) {
        guard ropeIndex >= 0 && ropeIndex < ropeCount else { return }
        let metaPtr = ropeMeta.contents().bindMemory(to: RopeMetaGPU.self, capacity: ropeCount)
        var metaValue = metaPtr[ropeIndex]
        if metaValue.count == 0 { return }
        metaValue.pinA = SIMD4<Float>(pinStart.x, pinStart.y, pinStart.z, 1)
        metaValue.pinB = SIMD4<Float>(pinEnd.x, pinEnd.y, pinEnd.z, 1)
        let segmentRestLength = simd_length(pinEnd - pinStart) / Float(max(1, particlesPerRope - 1))
        metaValue.restLengthBits = segmentRestLength.bitPattern
        metaPtr[ropeIndex] = metaValue
    }

    func setPinsWithSag(ropeIndex: Int, pinStart: SIMD3<Float>, pinEnd: SIMD3<Float>, sagMultiplier: Float) {
        guard ropeIndex >= 0 && ropeIndex < ropeCount else { return }
        let metaPtr = ropeMeta.contents().bindMemory(to: RopeMetaGPU.self, capacity: ropeCount)
        var metaValue = metaPtr[ropeIndex]
        if metaValue.count == 0 { return }
        metaValue.pinA = SIMD4<Float>(pinStart.x, pinStart.y, pinStart.z, 1)
        metaValue.pinB = SIMD4<Float>(pinEnd.x, pinEnd.y, pinEnd.z, 1)
        let actualLength = simd_length(pinEnd - pinStart)
        let effectiveLength = actualLength * sagMultiplier
        let segmentRestLength = effectiveLength / Float(max(1, particlesPerRope - 1))
        metaValue.restLengthBits = segmentRestLength.bitPattern
        metaPtr[ropeIndex] = metaValue
    }

    func deactivateRope(ropeIndex: Int) {
        guard ropeIndex >= 0 && ropeIndex < ropeCount else { return }
        let metaPtr = ropeMeta.contents().bindMemory(to: RopeMetaGPU.self, capacity: ropeCount)
        var metaValue = metaPtr[ropeIndex]
        metaValue.count = 0
        metaPtr[ropeIndex] = metaValue
    }
    
    func restLength(ropeIndex: Int) -> Float {
        guard ropeIndex >= 0 && ropeIndex < ropeCount else { return 0 }
        let metaPtr = ropeMeta.contents().bindMemory(to: RopeMetaGPU.self, capacity: ropeCount)
        let metaValue = metaPtr[ropeIndex]
        let segmentRestLength = Float(bitPattern: metaValue.restLengthBits)
        return segmentRestLength * Float(max(1, particlesPerRope - 1))
    }

    func step(deltaTime: Float) {
        let params = RopeSimParams(
            deltaTime: deltaTime,
            gravity: SIMD3<Float>(0, 0, 0),
            stretchStiffness: 0.95,
            iterations: 8,
            particleRadius: 0.048,
            damping: 0.35
        )
        paramsBuf.contents().copyMemory(from: [params], byteCount: MemoryLayout<RopeSimParams>.stride)

        guard let commandBuffer = queue.makeCommandBuffer() else { return }

        if simulationEnabled {
            if let encoder = commandBuffer.makeComputeCommandEncoder() {
                encoder.setComputePipelineState(ropePipeline)
                encoder.setBuffer(particlePos, offset: 0, index: 0)
                encoder.setBuffer(particlePrev, offset: 0, index: 1)
                encoder.setBuffer(ropeMeta, offset: 0, index: 2)
                encoder.setBuffer(paramsBuf, offset: 0, index: 3)

                let threadsPerThreadgroup = MTLSize(width: 128, height: 1, depth: 1)
                encoder.dispatchThreadgroups(
                    MTLSize(width: ropeCount, height: 1, depth: 1),
                    threadsPerThreadgroup: threadsPerThreadgroup
                )
                encoder.endEncoding()
            }
        }

        if let encoder = commandBuffer.makeComputeCommandEncoder() {
            encoder.setComputePipelineState(projectPipeline)
            encoder.setBuffer(particlePos, offset: 0, index: 0)
            encoder.setBuffer(particlePrev, offset: 0, index: 1)
            encoder.setBuffer(targetPos, offset: 0, index: 2)
            encoder.setBuffer(ropeMeta, offset: 0, index: 3)

            var alpha: Float = max(0, min(1, projectAlpha))
            var ppr: UInt32 = UInt32(particlesPerRope)
            encoder.setBytes(&alpha, length: MemoryLayout<Float>.stride, index: 4)
            encoder.setBytes(&ppr, length: MemoryLayout<UInt32>.stride, index: 5)

            dispatch1D(
                encoder: encoder,
                pipeline: projectPipeline,
                count: ropeCount * particlesPerRope
            )
            encoder.endEncoding()
        }

        if collisionsEnabled {
            for _ in 0..<1 {
            if let encoder = commandBuffer.makeComputeCommandEncoder() {
                encoder.setComputePipelineState(gridClearPipeline)
                encoder.setBuffer(gridHeads, offset: 0, index: 0)
                encoder.setBuffer(gridParamsBuf, offset: 0, index: 1)
                dispatch1D(
                    encoder: encoder,
                    pipeline: gridClearPipeline,
                    count: Int(64 * 64)
                )
                encoder.endEncoding()
            }

            if let encoder = commandBuffer.makeComputeCommandEncoder() {
                encoder.setComputePipelineState(gridBuildPipeline)
                encoder.setBuffer(particlePos, offset: 0, index: 0)
                encoder.setBuffer(gridHeads, offset: 0, index: 1)
                encoder.setBuffer(gridNext, offset: 0, index: 2)
                encoder.setBuffer(gridParamsBuf, offset: 0, index: 3)
                encoder.setBuffer(ropeMeta, offset: 0, index: 4)
                dispatch1D(
                    encoder: encoder,
                    pipeline: gridBuildPipeline,
                    count: ropeCount * particlesPerRope
                )
                encoder.endEncoding()
            }

            if let encoder = commandBuffer.makeComputeCommandEncoder() {
                encoder.setComputePipelineState(gridCollidePipeline)
                encoder.setBuffer(particlePos, offset: 0, index: 0)
                encoder.setBuffer(gridHeads, offset: 0, index: 1)
                encoder.setBuffer(gridNext, offset: 0, index: 2)
                encoder.setBuffer(gridParamsBuf, offset: 0, index: 3)
                encoder.setBuffer(paramsBuf, offset: 0, index: 4)
                encoder.setBuffer(ropeMeta, offset: 0, index: 5)
                encoder.setBuffer(particlePrev, offset: 0, index: 6)
                dispatch1D(
                    encoder: encoder,
                    pipeline: gridCollidePipeline,
                    count: ropeCount * particlesPerRope
                )
                encoder.endEncoding()
            }
            }
        }

        commandBuffer.commit()
    }

    func updateTargets(ropeIndex: Int, positions: [SIMD3<Float>]) {
        if positions.count != particlesPerRope { return }
        let total = ropeCount * particlesPerRope
        let ptr = targetPos.contents().bindMemory(to: SIMD4<Float>.self, capacity: total)
        let base = ropeIndex * particlesPerRope
        for particleIndex in 0..<particlesPerRope {
            let position = positions[particleIndex]
            ptr[base + particleIndex] = SIMD4<Float>(position.x, position.y, position.z, 1)
        }
    }

    private func dispatch1D(encoder: MTLComputeCommandEncoder, pipeline: MTLComputePipelineState, count: Int) {
        let threadWidth = pipeline.threadExecutionWidth
        let threadsPerThreadgroup = MTLSize(width: threadWidth, height: 1, depth: 1)
        let groups = (count + threadWidth - 1) / threadWidth
        encoder.dispatchThreadgroups(MTLSize(width: groups, height: 1, depth: 1), threadsPerThreadgroup: threadsPerThreadgroup)
    }

    func withRopePositions<T>(ropeIndex: Int, _ body: (UnsafeBufferPointer<SIMD3<Float>>) -> T) -> T {
        let total = ropeCount * particlesPerRope
        let raw = particlePos.contents().bindMemory(to: SIMD4<Float>.self, capacity: total)
        let base = raw.advanced(by: ropeIndex * particlesPerRope)
        return body(UnsafeBufferPointer(start: UnsafeRawPointer(base).bindMemory(to: SIMD3<Float>.self, capacity: particlesPerRope), count: particlesPerRope))
    }

    private func seed() {
        let total = ropeCount * particlesPerRope
        let pos = particlePos.contents().bindMemory(to: SIMD4<Float>.self, capacity: total)
        let prev = particlePrev.contents().bindMemory(to: SIMD4<Float>.self, capacity: total)

        for ropeIndex in 0..<ropeCount {
            let startPin = SIMD3<Float>(-0.82, -0.86, 0)
            let endPin = SIMD3<Float>(0.83, 0.49, 0)
            for particleIndex in 0..<particlesPerRope {
                let tValue = Float(particleIndex) / Float(particlesPerRope - 1)
                let point = startPin + (endPin - startPin) * tValue
                let value = SIMD4<Float>(point.x, point.y, 0, 1)
                let index = ropeIndex * particlesPerRope + particleIndex
                pos[index] = value
                prev[index] = value
            }
        }

        let meta = ropeMeta.contents().bindMemory(to: RopeMetaGPU.self, capacity: ropeCount)
        for ropeIndex in 0..<ropeCount {
            let startPin = SIMD3<Float>(-0.82, -0.86, 0)
            let endPin = SIMD3<Float>(0.83, 0.49, 0)
            let segmentRestLength = simd_length(endPin - startPin) / Float(max(1, particlesPerRope - 1)) * 0.84
            meta[ropeIndex] = RopeMetaGPU(
                offset: UInt32(ropeIndex * particlesPerRope),
                count: UInt32(particlesPerRope),
                restLengthBits: segmentRestLength.bitPattern,
                pad1: 0,
                pinA: SIMD4<Float>(0, 0, 0, 0),
                pinB: SIMD4<Float>(0, 0, 0, 0)
            )
        }
    }
}


