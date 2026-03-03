import MetalKit
import os.log

final class Renderer: NSObject, MTKViewDelegate {
    static let logger = Logger(subsystem: "com.uzls.four", category: "Renderer")

    let device: MTLDevice
    let commandQueue: MTLCommandQueue
    let tablePipeline: MTLRenderPipelineState
    let holePipeline: MTLRenderPipelineState
    let ropePipeline: MTLRenderPipelineState
    let postPipeline: MTLRenderPipelineState
    let boardPipeline: MTLRenderPipelineState
    let shadowRopePipeline: MTLRenderPipelineState
    let shadowHolePipeline: MTLRenderPipelineState
    let shadowBoardPipeline: MTLRenderPipelineState
    let bloomThreshold: MTLComputePipelineState
    let bloomBlurH: MTLComputePipelineState
    let bloomBlurV: MTLComputePipelineState
    let bakeWoodPipeline: MTLComputePipelineState
    let bakeBoardWoodVolumePipeline: MTLComputePipelineState

    var depthStateScene: MTLDepthStencilState
    var depthStateBackground: MTLDepthStencilState
    var depthStateShadow: MTLDepthStencilState

    var shadowDepthTex: MTLTexture?
    let shadowMapSize: Int = 2048

    var camera = Camera()
    var cameraZoomScale: Float = 0.95764452219009399
    var cameraBaseOrthoHalfHeight: Float = 2.05
    var cameraDebugMode = false
    var cameraDebugTouchStart: CGPoint?
    var cameraDragActive = false
    var cameraDragStart: CGPoint?
    var time: Float = 0
    var dragVisualEnergy: Float = 0
    var lastDeltaTime: Float = 1.0 / 60.0
    var lastDrawTime: Double = 0
    var currentFPS: Float = 0


    var frameUniforms: MTLBuffer?
    var postParamsBuffer: MTLBuffer?
    var holeInstances: MTLBuffer?
    var holeVB: MTLBuffer?
    var holeIB: MTLBuffer?
    var holeIndexCount: Int = 0

    var holePositions: [SIMD2<Float>] = []
    var holeElevations: [Float] = []
    var holeRadius: Float = 0.105
    var holeOccupied: [Bool] = []
    var profileSegments: Int = 10
    var holeRadiusScale: Float = 0.73367023468017578 { didSet { rebuildHoleInstancesIfNeeded() } }
    var holeTint: SIMD4<Float> = SIMD4<Float>(1, 0.90870898962020874, 1, 1)
    var holeSegments: Int = 19 { didSet { rebuildHoleMeshIfNeeded() } }
    var ropeRadiusScale: Float = 1.0618677139282227
    var squareCrossSection: Bool = false {
        didSet {
            rebuildHoleMeshIfNeeded()
            simulator?.squareCrossSection = squareCrossSection
        }
    }
    var stretchThinning: Float = 0.5 {
        didSet { simulator?.stretchThinning = stretchThinning }
    }
    var exposure: Float = 0.5
    var lightIntensity: Float = 0.40238875150680542
    var lightDir: SIMD3<Float> = SIMD3<Float>(-0.029445827007293701, -0.22127896547317505, 0.87485504150390625)
    var bloomStrength: Float = 0
    var ambient: Float = 0.10394008457660675
    var shadowBias: Float = 0.0013682411517947912
    var shadowDarkness: Float = 0.15453849732875824
    var lightSize: Float = 0.073253527283668518
    var shadowsEnabled: Bool = true
    var shadowType: Int = 2
    var cartoonShaderEnabled: Bool = true
    var cartoonExposure: Float = 1.3291559219360352
    var cartoonBloom: Float = 0
    var cartoonEdgeStrength: Float = 1
    var cartoonLevels: Int = 2
    var cartoonShadowBright: Float = 0.38
    var cartoonWrap: Float = 0.15
    var cartoonEdgeSmooth: Float = 0.5
    var tableStyle: Int = 0
    var tableColor1: SIMD3<Float> = SIMD3<Float>(0.079999998211860657, 0.090000003576278687, 0.12999999523162842)
    var tableColor2: SIMD3<Float> = SIMD3<Float>(0.11999999731779099, 0.12999999523162842, 0.20000000298023224)
    var woodSeed: Float = 0.045950364321470261 { didSet { bakeWoodTexture() } }
    var woodBrightness: Float = 1.0 { didSet { bakeWoodTexture() } }
    var woodPatternScale: Float = 1.7154961824417114 { didSet { bakeWoodTexture() } }
    var boardElevation: Float = 0.12 {
        didSet {
            if levelLoaded { loadLevel(levelId: currentLevelId) }
        }
    }
    private var levelLoaded = false
    var capRadiusScale: Float = 0.90614998340606689
    var capSegments: Int = 12
    var capRings: Int = 6
    var capDarken: Float = 0
    var wormMode: Bool = false
    var wormSegFreq: Float = 28.0
    var wormSegBulge: Float = 0.12
    var wormThickness: Float = 1.35
    var wormTaperLen: Float = 0.12
    var wormGrooveDepth: Float = 0.35
    var wormBellyBright: Float = 1.15
    var wormBackDark: Float = 0.7
    var wormSkinNoise: Float = 0.08
    var wormSSS: Float = 0.25
    var wormRoughness: Float = 0.25
    var wormSpecular: Float = 0.8
    var wormRimStrength: Float = 0.08
    var wormEyeSize: Float = 0.015
    var wormPulseSpeed: Float = 2.5
    var wormPulseAmp: Float = 0.02
    var wormCrawlSpeed: Float = 3.5
    var wormCrawlAmp: Float = 0.012
    var wormSideAmp: Float = 0.008

    var ropeMatte: Float = 0.33392396569252014
    var ropeGloss: Float = 0.69268685579299927
    var ropeDiffuseWrap: Float = 0.0080702323466539383
    var ropeSubsurface: Float = 0.33765289187431335
    var ropeEdgeLight: Float = 0.41452157497406006
    var ropeSaturation: Float = 1.35732102394104
    var ropeMicroBump: Float = 0.142847940325737
    var ropeBumpScale: Float = 3.0
    var ropeContactAO: Float = 1
    var ropeLiftGlow: Float = 0.97152864933013916
    var ropeStretchGloss: Float = 0.7
    var ropeStretchSpec: Float = 1.0
    struct RopeEndpoints {
        var startHole: Int
        var endHole: Int
        var color: SIMD3<Float>
        var radius: Float
        var crossSection: CrossSection
    }

    var ropes: [RopeEndpoints] = []
    var topology: TopologyEngine?
    var simulator: VerletSimulator?
    var lastDragWorld: SIMD2<Float> = .zero
    var dragStartWorld: SIMD2<Float> = .zero

    struct DragState {
        var ropeIndex: Int
        var endIndex: Int
        var originalHoleIndex: Int
    }

    var dragState: DragState?
    var dragWorld: SIMD2<Float> = .zero
    var dragHeight: Float = 0.34999999403953552
    var highlightHoleIndex: Int = -1

    var ropeVB: MTLBuffer?
    var ropeIB: MTLBuffer?
    var ropeIndexCount: Int = 0

    var hdrTex: MTLTexture?
    var sceneDepthTex: MTLTexture?
    var bloomA: MTLTexture?
    var bloomB: MTLTexture?
    var bakedWoodTex: MTLTexture?
    var bakedBoardWoodVolumeTex: MTLTexture?
    var noiseTexture: MTLTexture?
    var woodBoundsMin: SIMD2<Float> = .zero
    var woodBoundsMax: SIMD2<Float> = .zero
    var boardWoodZMin: Float = 0
    var boardWoodZMax: Float = 1
    var lastViewSize: CGSize = CGSize(width: 400, height: 600)
    var renderScale: Float = 1.0 {
        didSet { invalidateOffscreenTextures() }
    }
    
    var boards: [LevelDefinition.Board] = []
    var boardMeshVB: MTLBuffer?
    var boardMeshIB: MTLBuffer?
    var boardMeshIndexCount: Int = 0

    var lastPhysicsLogTime: Double = 0
    let frictionSound = RubberFrictionSound()

    struct MeshStats: Equatable {
        let vertices: Int
        let indices: Int
        let ropeCount: Int
    }
    var lastMeshStats: MeshStats?

    var currentLevelId: Int = 1
    var onLevelComplete: (() -> Void)?
    var onUndoStackChanged: ((Bool) -> Void)?
    var onZoomChanged: ((Float) -> Void)?

    // MARK: - Undo

    struct UndoEntry {
        var simulatorSnapshot: VerletSimulator.Snapshot
        var ropeEndpoints: [RopeEndpoints]
        var holeOccupied: [Bool]
    }

    private var undoStore = RendererUndoStore<UndoEntry>()

    var canUndo: Bool { undoStore.canUndo }

    func pushUndoState() {
        guard let sim = simulator else { return }
        let entry = UndoEntry(
            simulatorSnapshot: sim.takeSnapshot(),
            ropeEndpoints: ropes,
            holeOccupied: holeOccupied
        )
        undoStore.push(entry)
        onUndoStackChanged?(true)
    }

    func performUndo() {
        guard let entry = undoStore.pop(), let sim = simulator else { return }
        dragState = nil
        highlightHoleIndex = -1
        settleCheckTimer = nil
        sim.restoreSnapshot(entry.simulatorSnapshot)
        ropes = entry.ropeEndpoints
        holeOccupied = entry.holeOccupied
        onUndoStackChanged?(undoStore.canUndo)
    }

    /// Timer for delayed win check after drag ends.
    /// Rope needs time to settle before we check crossings.
    var settleCheckTimer: Float? = nil
    let settleCheckDelay: Float = 0.5
    var nextLevelTimer: Float? = nil
    
    // Physics parameters (forwarded to simulator)
    var physicsGravity: Float = -5.0 {
        didSet { simulator?.gravity = physicsGravity }
    }
    var physicsDamping: Float = 0.97 {
        didSet { simulator?.damping = physicsDamping }
    }
    var physicsConstraintIterations: Int = 8 {
        didSet { simulator?.constraintIterations = physicsConstraintIterations }
    }
    var physicsParticleCount: Int = 60 {
        didSet { simulator?.particleCount = physicsParticleCount }
    }
    var physicsSettleSteps: Int = 5 {
        didSet { simulator?.settleSteps = physicsSettleSteps }
    }
    var physicsLiftHeight: Float = 0.30 {
        didSet { simulator?.liftHeight = physicsLiftHeight }
    }
    var physicsRopeTension: Float = 0.98 {
        didSet { simulator?.ropeTension = physicsRopeTension }
    }
    var physicsBendCompliance: Float = 0.0015 {
        didSet { simulator?.bendCompliance = physicsBendCompliance }
    }
    var physicsBendVelocityCoupling: Float = 0.45 {
        didSet { simulator?.bendVelocityCoupling = physicsBendVelocityCoupling }
    }

    init(view: MTKView) {
        guard let device = view.device else { fatalError("Metal device is required") }
        guard let commandQueue = device.makeCommandQueue() else { fatalError("Command queue is required") }

        self.device = device
        self.commandQueue = commandQueue

        guard let library = device.makeDefaultLibrary() else {
            fatalError("Failed to create default Metal library")
        }

        self.tablePipeline = Self.makeTablePipeline(device: device, view: view, library: library)
        self.holePipeline = Self.makeHolePipeline(device: device, view: view, library: library)
        self.ropePipeline = Self.makeRopePipeline(device: device, view: view, library: library)
        self.postPipeline = Self.makePostPipeline(device: device, view: view, library: library)
        self.boardPipeline = Self.makeBoardPipeline(device: device, view: view, library: library)
        self.shadowRopePipeline = Self.makeShadowRopePipeline(device: device, library: library)
        self.shadowHolePipeline = Self.makeShadowHolePipeline(device: device, library: library)
        self.shadowBoardPipeline = Self.makeShadowBoardPipeline(device: device, library: library)
        (self.bloomThreshold, self.bloomBlurH, self.bloomBlurV) = Self.makeBloomPipelines(device: device, library: library)
        self.bakeWoodPipeline = Self.makeComputePipeline(device: device, function: library.makeFunction(name: "bakeWoodKernel")!)
        self.bakeBoardWoodVolumePipeline = Self.makeComputePipeline(device: device, function: library.makeFunction(name: "bakeBoardWoodVolumeKernel")!)
        (self.depthStateScene, self.depthStateBackground, self.depthStateShadow) = Self.makeDepthStates(device: device)

        self.frameUniforms = device.makeBuffer(length: MemoryLayout<FrameUniforms>.stride, options: [.storageModeShared])
        self.postParamsBuffer = device.makeBuffer(length: MemoryLayout<PostParams>.stride, options: [.storageModeShared])
        Self.buildHoleMeshBuffers(device: device, segments: holeSegments, square: squareCrossSection, vertexBuffer: &holeVB, indexBuffer: &holeIB, indexCount: &holeIndexCount)

        self.noiseTexture = Self.generateNoiseTexture(device: device, size: 2048)

        super.init()
    }

    private static func generateNoiseTexture(device: MTLDevice, size: Int) -> MTLTexture? {
        let desc = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .rg8Unorm,
            width: size,
            height: size,
            mipmapped: false
        )
        desc.usage = .shaderRead
        guard let tex = device.makeTexture(descriptor: desc) else { return nil }

        let gridSize = 128
        var rng: UInt32 = 0xDEAD_BEEF
        func lcg() -> Float {
            rng = rng &* 1_664_525 &+ 1_013_904_223
            return Float(rng >> 8) / 16_777_216.0
        }
        var grid = [Float](repeating: 0, count: gridSize * gridSize * 2)
        for i in 0..<grid.count { grid[i] = lcg() }

        func gridVal(_ gx: Int, _ gy: Int, _ ch: Int) -> Float {
            let wx = ((gx % gridSize) + gridSize) % gridSize
            let wy = ((gy % gridSize) + gridSize) % gridSize
            return grid[(wy * gridSize + wx) * 2 + ch]
        }
        func smooth(_ t: Float) -> Float { t * t * (3 - 2 * t) }
        func valueNoise(_ x: Float, _ y: Float, _ freq: Int, _ ch: Int) -> Float {
            let fx = x * Float(freq), fy = y * Float(freq)
            let ix = Int(floor(fx)), iy = Int(floor(fy))
            let tx = smooth(fx - floor(fx)), ty = smooth(fy - floor(fy))
            let c00 = gridVal(ix, iy, ch), c10 = gridVal(ix+1, iy, ch)
            let c01 = gridVal(ix, iy+1, ch), c11 = gridVal(ix+1, iy+1, ch)
            let a = c00 + (c10 - c00) * tx
            let b = c01 + (c11 - c01) * tx
            return a + (b - a) * ty
        }

        var pixels = [UInt8](repeating: 0, count: size * size * 2)
        for y in 0..<size {
            for x in 0..<size {
                let u = Float(x) / Float(size)
                let v = Float(y) / Float(size)
                let idx = (y * size + x) * 2
                for ch in 0..<2 {
                    let o1 = valueNoise(u, v, 16, ch)
                    let o2 = valueNoise(u, v, 37, ch)
                    let o3 = valueNoise(u, v, 79, ch)
                    let o4 = valueNoise(u, v, 128, ch)
                    let fbm = o1 * 0.15 + o2 * 0.30 + o3 * 0.30 + o4 * 0.15
                    let white = lcg()
                    let val = fbm * 0.85 + white * 0.15
                    pixels[idx + ch] = UInt8(min(max(val, 0), 1) * 255)
                }
            }
        }

        tex.replace(
            region: MTLRegionMake2D(0, 0, size, size),
            mipmapLevel: 0,
            withBytes: pixels,
            bytesPerRow: size * 2
        )
        return tex
    }
    
    func loadLevel(levelId: Int) {
        let loadStart = CACurrentMediaTime()
        currentLevelId = levelId
        Self.logger.info("Loading level \(levelId)...")
        
        dragState = nil
        dragWorld = .zero
        simulator = nil
        undoStore.clear()
        onUndoStackChanged?(false)
        
        var t0 = CACurrentMediaTime()
        let level: LevelDefinition
        if let jsonLevel = LevelLoader.load(levelId: levelId) {
            Self.logger.info("Level \(levelId) loaded from JSON: \(jsonLevel.ropes.count) ropes, \(jsonLevel.holes.count) holes")
            level = jsonLevel
        } else {
            Self.logger.info("Level \(levelId) generated procedurally")
            level = LevelGenerator.generate(levelId: levelId, boardElevation: boardElevation)
        }
        let tGenerate = CACurrentMediaTime()

        let levelHoles = level.holes.map { $0.simd }
        let levelHoleElevations = level.holes.map { $0.zPosition }
        let levelHoleRadius = level.holeRadius

        let validatedRopes = level.ropes.filter { rope in
            guard levelHoles.indices.contains(rope.startHole) else { return false }
            guard levelHoles.indices.contains(rope.endHole) else { return false }
            return rope.startHole != rope.endHole
        }

        holePositions = levelHoles
        holeElevations = levelHoleElevations
        holeRadius = levelHoleRadius
        holeOccupied = Array(repeating: false, count: levelHoles.count)
        boards = level.boards ?? []
        rebuildHoleInstances()
        rebuildHoleMeshIfNeeded()
        rebuildBoardMesh()
        let tMesh = CACurrentMediaTime()

        ropes = validatedRopes.map { rope in
            RopeEndpoints(startHole: rope.startHole, endHole: rope.endHole, color: rope.color.simd, radius: rope.radius, crossSection: rope.crossSection)
        }

        let ropeConfigs = ropes.map { rope in
            (startHole: rope.startHole, endHole: rope.endHole, color: rope.color)
        }
        let hookDefs: [TopologyEngine.HookDefinition]? = level.hooks.flatMap { jsonHooks in
            jsonHooks.compactMap { hook -> TopologyEngine.HookDefinition? in
                guard let ropeAIdx = Self.resolveRopeRef(hook.ropeA, hooks: jsonHooks),
                      let ropeBIdx = Self.resolveRopeRef(hook.ropeB, hooks: jsonHooks) else {
                    Self.logger.warning("Failed to resolve hook rope references")
                    return nil
                }
                return TopologyEngine.HookDefinition(
                    ropeA: ropeAIdx,
                    ropeB: ropeBIdx,
                    N: hook.N,
                    ropeAStartIsOver: hook.ropeAStartIsOver
                )
            }
        }
        topology = TopologyEngine(holePositions: levelHoles, ropeConfigs: ropeConfigs, hookDefinitions: hookDefs)
        let tTopology = CACurrentMediaTime()

        let sim = VerletSimulator(holePositions: levelHoles, holeElevations: levelHoleElevations, holeRadius: levelHoleRadius, boards: boards)
        sim.gravity = physicsGravity
        sim.damping = physicsDamping
        sim.constraintIterations = physicsConstraintIterations
        sim.particleCount = physicsParticleCount
        sim.settleSteps = physicsSettleSteps
        sim.liftHeight = physicsLiftHeight
        sim.ropeTension = physicsRopeTension
        sim.bendCompliance = physicsBendCompliance
        sim.bendVelocityCoupling = physicsBendVelocityCoupling
        sim.stretchThinning = stretchThinning
        sim.squareCrossSection = squareCrossSection

        let simRopeConfigs = ropes.map { rope in
            VerletSimulator.RopeConfig(startHole: rope.startHole, endHole: rope.endHole, radius: rope.radius, crossSection: rope.crossSection)
        }

        let simActions: [VerletSimulator.LevelAction] = level.actions?.compactMap { action in
            guard let actionType = VerletSimulator.LevelAction.ActionType(rawValue: action.type) else { return nil }
            return VerletSimulator.LevelAction(type: actionType, ropeIndex: action.ropeIndex, endIndex: action.endIndex, holeIndex: action.holeIndex)
        } ?? []

        t0 = CACurrentMediaTime()
        sim.initializeLevel(ropeConfigs: simRopeConfigs, actions: simActions)
        let tPhysics = CACurrentMediaTime()
        self.simulator = sim

        for ropeIndex in ropes.indices {
            guard sim.bands.indices.contains(ropeIndex) else { continue }
            let band = sim.bands[ropeIndex]
            if let pinStart = band.pinStart {
                ropes[ropeIndex].startHole = pinStart
            }
            if let pinEnd = band.pinEnd {
                ropes[ropeIndex].endHole = pinEnd
            }
        }

        for ropeIndex in ropes.indices {
            let startHoleIndex = ropes[ropeIndex].startHole
            let endHoleIndex = ropes[ropeIndex].endHole
            guard holeOccupied.indices.contains(startHoleIndex), holeOccupied.indices.contains(endHoleIndex) else { continue }
            holeOccupied[startHoleIndex] = true
            holeOccupied[endHoleIndex] = true
        }

        let aspect = Float(lastViewSize.width / max(1, lastViewSize.height))
        let maxElev = holeElevations.max() ?? 0
        camera.fitToHoles(holePositions, holeRadius: holeRadius, aspect: aspect, maxElevation: maxElev)
        cameraBaseOrthoHalfHeight = camera.orthoHalfHeight
        camera.orthoHalfHeight = cameraBaseOrthoHalfHeight / cameraZoomScale

        let tCamera = CACurrentMediaTime()
        bakeWoodTexture()
        bakeBoardWoodVolumeTexture()
        let tWood = CACurrentMediaTime()

        let ms = { (a: Double, b: Double) -> String in String(format: "%.1f", (b - a) * 1000) }
        Self.logger.warning("""
            [LOAD-PROFILE] level=\(levelId) ropes=\(validatedRopes.count) holes=\(levelHoles.count) actions=\(simActions.count) \
            generate=\(ms(loadStart, tGenerate))ms mesh=\(ms(tGenerate, tMesh))ms topology=\(ms(tMesh, tTopology))ms \
            physics=\(ms(t0, tPhysics))ms camera=\(ms(tPhysics, tCamera))ms wood=\(ms(tCamera, tWood))ms \
            TOTAL=\(ms(loadStart, tWood))ms
            """)
        levelLoaded = true
    }

    private static func makePipeline(device: MTLDevice, descriptor: MTLRenderPipelineDescriptor) -> MTLRenderPipelineState {
        do {
            return try device.makeRenderPipelineState(descriptor: descriptor)
        } catch {
            fatalError(String(describing: error))
        }
    }

    private static func makeComputePipeline(device: MTLDevice, function: MTLFunction) -> MTLComputePipelineState {
        do {
            return try device.makeComputePipelineState(function: function)
        } catch {
            fatalError(String(describing: error))
        }
    }

    private static func makeTablePipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "fullscreenVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "tableFragment")
        descriptor.colorAttachments[0].pixelFormat = .rgba16Float
        descriptor.colorAttachments[0].isBlendingEnabled = false
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeHolePipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "holeVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "holeFragment")
        descriptor.colorAttachments[0].pixelFormat = .rgba16Float
        descriptor.colorAttachments[0].isBlendingEnabled = false
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.vertexDescriptor = makeHoleVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeRopePipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "ropeVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "ropeFragment")
        descriptor.colorAttachments[0].pixelFormat = .rgba16Float
        descriptor.colorAttachments[0].isBlendingEnabled = false
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.vertexDescriptor = makeRopeVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeShadowRopePipeline(device: MTLDevice, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "ropeShadowVertex")
        descriptor.depthAttachmentPixelFormat = .depth32Float
        descriptor.vertexDescriptor = makeRopeVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeShadowHolePipeline(device: MTLDevice, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "holeShadowVertex")
        descriptor.depthAttachmentPixelFormat = .depth32Float
        descriptor.vertexDescriptor = makeHoleVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeBoardPipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "boardVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "boardFragment")
        descriptor.colorAttachments[0].pixelFormat = .rgba16Float
        descriptor.colorAttachments[0].isBlendingEnabled = false
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.vertexDescriptor = makeBoardVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeShadowBoardPipeline(device: MTLDevice, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "boardShadowVertex")
        descriptor.depthAttachmentPixelFormat = .depth32Float
        descriptor.vertexDescriptor = makeBoardVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeBoardVertexDescriptor() -> MTLVertexDescriptor {
        let descriptor = MTLVertexDescriptor()
        descriptor.attributes[0].format = .float3
        descriptor.attributes[0].offset = 0
        descriptor.attributes[0].bufferIndex = 0

        descriptor.attributes[1].format = .float3
        descriptor.attributes[1].offset = MemoryLayout<SIMD3<Float>>.stride
        descriptor.attributes[1].bufferIndex = 0

        descriptor.attributes[2].format = .float2
        descriptor.attributes[2].offset = MemoryLayout<SIMD3<Float>>.stride * 2
        descriptor.attributes[2].bufferIndex = 0

        descriptor.layouts[0].stride = MemoryLayout<BoardVertex>.stride
        return descriptor
    }

    private static func makePostPipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "fullscreenVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "postFragment")
        descriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat
        descriptor.colorAttachments[0].isBlendingEnabled = false
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeBloomPipelines(device: MTLDevice, library: MTLLibrary) -> (MTLComputePipelineState, MTLComputePipelineState, MTLComputePipelineState) {
        guard let thresholdFunction = library.makeFunction(name: "bloomThreshold"),
              let blurHFunction = library.makeFunction(name: "bloomBlurH"),
              let blurVFunction = library.makeFunction(name: "bloomBlurV") else {
            fatalError("Failed to load bloom functions")
        }
        return (
            makeComputePipeline(device: device, function: thresholdFunction),
            makeComputePipeline(device: device, function: blurHFunction),
            makeComputePipeline(device: device, function: blurVFunction)
        )
    }

    private static func makeDepthStates(device: MTLDevice) -> (MTLDepthStencilState, MTLDepthStencilState, MTLDepthStencilState) {
        let backgroundDescriptor = MTLDepthStencilDescriptor()
        backgroundDescriptor.depthCompareFunction = .always
        backgroundDescriptor.isDepthWriteEnabled = false

        let sceneDescriptor = MTLDepthStencilDescriptor()
        sceneDescriptor.depthCompareFunction = .lessEqual
        sceneDescriptor.isDepthWriteEnabled = true

        let shadowDescriptor = MTLDepthStencilDescriptor()
        shadowDescriptor.depthCompareFunction = .lessEqual
        shadowDescriptor.isDepthWriteEnabled = true

        guard let backgroundState = device.makeDepthStencilState(descriptor: backgroundDescriptor),
              let sceneState = device.makeDepthStencilState(descriptor: sceneDescriptor),
              let shadowState = device.makeDepthStencilState(descriptor: shadowDescriptor) else {
            fatalError("Failed to create depth states")
        }

        return (sceneState, backgroundState, shadowState)
    }

    private static func makeRopeVertexDescriptor() -> MTLVertexDescriptor {
        let descriptor = MTLVertexDescriptor()
        descriptor.attributes[0].format = .float3
        descriptor.attributes[0].offset = MemoryLayout<RopeVertex>.offset(of: \.position) ?? 0
        descriptor.attributes[0].bufferIndex = 0

        descriptor.attributes[1].format = .float3
        descriptor.attributes[1].offset = MemoryLayout<RopeVertex>.offset(of: \.normal) ?? 0
        descriptor.attributes[1].bufferIndex = 0

        descriptor.attributes[2].format = .float3
        descriptor.attributes[2].offset = MemoryLayout<RopeVertex>.offset(of: \.color) ?? 0
        descriptor.attributes[2].bufferIndex = 0

        descriptor.attributes[3].format = .float2
        descriptor.attributes[3].offset = MemoryLayout<RopeVertex>.offset(of: \.texCoord) ?? 0
        descriptor.attributes[3].bufferIndex = 0

        descriptor.attributes[4].format = .float4
        descriptor.attributes[4].offset = MemoryLayout<RopeVertex>.offset(of: \.params) ?? 0
        descriptor.attributes[4].bufferIndex = 0

        descriptor.layouts[0].stride = MemoryLayout<RopeVertex>.stride
        return descriptor
    }

    private static func makeHoleVertexDescriptor() -> MTLVertexDescriptor {
        let descriptor = MTLVertexDescriptor()
        descriptor.attributes[0].format = .float3
        descriptor.attributes[0].offset = 0
        descriptor.attributes[0].bufferIndex = 0

        descriptor.attributes[1].format = .float3
        descriptor.attributes[1].offset = MemoryLayout<SIMD3<Float>>.stride
        descriptor.attributes[1].bufferIndex = 0

        descriptor.layouts[0].stride = MemoryLayout<HoleVertex>.stride
        return descriptor
    }

    private static func buildHoleMeshBuffers(device: MTLDevice, segments: Int, square: Bool = false, vertexBuffer: inout MTLBuffer?, indexBuffer: inout MTLBuffer?, indexCount: inout Int) {
        let mesh = square ? HoleMeshBuilder.buildSquare() : HoleMeshBuilder.build(segments: segments)
        indexCount = mesh.indices.count
        vertexBuffer = device.makeBuffer(bytes: mesh.vertices, length: mesh.vertices.count * MemoryLayout<HoleVertex>.stride, options: [.storageModeShared])
        indexBuffer = device.makeBuffer(bytes: mesh.indices, length: mesh.indices.count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])
    }

    private func rebuildHoleInstances() {
        guard !holePositions.isEmpty else { return }
        let visualRadius = holeRadius * holeRadiusScale
        holeInstances = Self.makeHoleInstances(device: device, positions: holePositions, elevations: holeElevations, radius: visualRadius)
    }

    private func rebuildHoleInstancesIfNeeded() {
        rebuildHoleInstances()
    }

    private func rebuildHoleMeshIfNeeded() {
        Self.buildHoleMeshBuffers(device: device, segments: holeSegments, square: squareCrossSection, vertexBuffer: &holeVB, indexBuffer: &holeIB, indexCount: &holeIndexCount)
    }

    func rebuildBoardMesh() {
        guard !boards.isEmpty else {
            boardMeshVB = nil
            boardMeshIB = nil
            boardMeshIndexCount = 0
            return
        }

        var vertices: [BoardVertex] = []
        var indices: [UInt32] = []

        for board in boards {
            let hw = board.width * 0.5
            let hh = board.height * 0.5
            let z = board.elevation
            let cx = board.centerX
            let cy = board.centerY
            let base: UInt32 = UInt32(vertices.count)

            let topN = SIMD3<Float>(0, 0, 1)
            vertices.append(BoardVertex(position: SIMD3(cx - hw, cy - hh, z), normal: topN, worldXY: SIMD2(cx - hw, cy - hh)))
            vertices.append(BoardVertex(position: SIMD3(cx + hw, cy - hh, z), normal: topN, worldXY: SIMD2(cx + hw, cy - hh)))
            vertices.append(BoardVertex(position: SIMD3(cx + hw, cy + hh, z), normal: topN, worldXY: SIMD2(cx + hw, cy + hh)))
            vertices.append(BoardVertex(position: SIMD3(cx - hw, cy + hh, z), normal: topN, worldXY: SIMD2(cx - hw, cy + hh)))
            indices.append(contentsOf: [base, base+1, base+2, base, base+2, base+3])

            let sides: [(SIMD3<Float>, SIMD3<Float>, SIMD3<Float>, SIMD3<Float>, SIMD3<Float>)] = [
                (SIMD3(cx - hw, cy - hh, z), SIMD3(cx + hw, cy - hh, z), SIMD3(cx + hw, cy - hh, 0), SIMD3(cx - hw, cy - hh, 0), SIMD3(0, -1, 0)),
                (SIMD3(cx + hw, cy - hh, z), SIMD3(cx + hw, cy + hh, z), SIMD3(cx + hw, cy + hh, 0), SIMD3(cx + hw, cy - hh, 0), SIMD3(1, 0, 0)),
                (SIMD3(cx + hw, cy + hh, z), SIMD3(cx - hw, cy + hh, z), SIMD3(cx - hw, cy + hh, 0), SIMD3(cx + hw, cy + hh, 0), SIMD3(0, 1, 0)),
                (SIMD3(cx - hw, cy + hh, z), SIMD3(cx - hw, cy - hh, z), SIMD3(cx - hw, cy - hh, 0), SIMD3(cx - hw, cy + hh, 0), SIMD3(-1, 0, 0)),
            ]
            for (p0, p1, p2, p3, n) in sides {
                let sb = UInt32(vertices.count)
                vertices.append(BoardVertex(position: p0, normal: n, worldXY: SIMD2(p0.x, p0.y)))
                vertices.append(BoardVertex(position: p1, normal: n, worldXY: SIMD2(p1.x, p1.y)))
                vertices.append(BoardVertex(position: p2, normal: n, worldXY: SIMD2(p2.x, p2.y)))
                vertices.append(BoardVertex(position: p3, normal: n, worldXY: SIMD2(p3.x, p3.y)))
                indices.append(contentsOf: [sb, sb+1, sb+2, sb, sb+2, sb+3])
            }
        }

        boardMeshIndexCount = indices.count
        guard boardMeshIndexCount > 0 else { return }
        boardMeshVB = device.makeBuffer(bytes: vertices, length: vertices.count * MemoryLayout<BoardVertex>.stride, options: [.storageModeShared])
        boardMeshIB = device.makeBuffer(bytes: indices, length: indices.count * MemoryLayout<UInt32>.stride, options: [.storageModeShared])
    }

    func bakeWoodTexture() {
        guard !holePositions.isEmpty else { return }

        var minP = SIMD2<Float>(Float.greatestFiniteMagnitude, Float.greatestFiniteMagnitude)
        var maxP = SIMD2<Float>(-Float.greatestFiniteMagnitude, -Float.greatestFiniteMagnitude)
        for h in holePositions {
            minP = min(minP, h)
            maxP = max(maxP, h)
        }
        let boardSize = max(maxP.x - minP.x, maxP.y - minP.y)
        let padding = max(boardSize * 3.0, 10.0)
        let center = (minP + maxP) * 0.5
        let half = (maxP - minP) * 0.5 + SIMD2<Float>(padding, padding)
        let minBake = center - half
        let maxBake = center + half

        woodBoundsMin = minBake
        woodBoundsMax = maxBake

        let texSize = 8192
        let desc = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .rgba16Float,
            width: texSize,
            height: texSize,
            mipmapped: false
        )
        desc.usage = [.shaderRead, .shaderWrite]
        guard let tex = device.makeTexture(descriptor: desc) else { return }
        bakedWoodTex = tex

        guard let cmdBuf = commandQueue.makeCommandBuffer(),
              let encoder = cmdBuf.makeComputeCommandEncoder() else { return }

        let effectiveSeed = Float(currentLevelId) + woodSeed * 100
        var params = BakeWoodParams(worldMin: minBake, worldMax: maxBake, seed: effectiveSeed, brightness: woodBrightness, patternScale: woodPatternScale)
        encoder.setComputePipelineState(bakeWoodPipeline)
        encoder.setTexture(tex, index: 0)
        encoder.setBytes(&params, length: MemoryLayout<BakeWoodParams>.stride, index: 0)

        let threadWidth = bakeWoodPipeline.threadExecutionWidth
        let threadHeight = max(1, bakeWoodPipeline.maxTotalThreadsPerThreadgroup / threadWidth)
        let threadsPerGroup = MTLSize(width: threadWidth, height: threadHeight, depth: 1)
        let groupsW = (texSize + threadWidth - 1) / threadWidth
        let groupsH = (texSize + threadHeight - 1) / threadHeight
        encoder.dispatchThreadgroups(MTLSize(width: groupsW, height: groupsH, depth: 1), threadsPerThreadgroup: threadsPerGroup)
        encoder.endEncoding()

        cmdBuf.commit()
        cmdBuf.waitUntilCompleted()
    }

    func bakeBoardWoodVolumeTexture() {
        guard !boards.isEmpty, !holePositions.isEmpty else {
            bakedBoardWoodVolumeTex = nil
            return
        }

        let texW = 128
        let texH = 128
        let texD = 64
        let desc = MTLTextureDescriptor()
        desc.textureType = .type3D
        desc.pixelFormat = .rgba16Float
        desc.width = texW
        desc.height = texH
        desc.depth = texD
        desc.mipmapLevelCount = 1
        desc.usage = [.shaderRead, .shaderWrite]
        guard let tex = device.makeTexture(descriptor: desc) else { return }
        bakedBoardWoodVolumeTex = tex

        boardWoodZMin = min(0, boards.map(\.elevation).min() ?? boardElevation)
        boardWoodZMax = max(boards.map(\.elevation).max() ?? boardElevation, 0.001)

        guard let cmdBuf = commandQueue.makeCommandBuffer(),
              let encoder = cmdBuf.makeComputeCommandEncoder() else { return }

        let effectiveSeed = Float(currentLevelId) + woodSeed * 100
        var params = BakeBoardWoodVolumeParams(
            worldMin: SIMD4<Float>(woodBoundsMin.x, woodBoundsMin.y, boardWoodZMin, 0),
            worldMax: SIMD4<Float>(woodBoundsMax.x, woodBoundsMax.y, boardWoodZMax, 0),
            seed: effectiveSeed,
            brightness: woodBrightness
        )
        encoder.setComputePipelineState(bakeBoardWoodVolumePipeline)
        encoder.setTexture(tex, index: 0)
        encoder.setBytes(&params, length: MemoryLayout<BakeBoardWoodVolumeParams>.stride, index: 0)

        let maxThreads = bakeBoardWoodVolumePipeline.maxTotalThreadsPerThreadgroup
        let tgDepth = max(1, min(4, maxThreads / 16))
        let threadsPerGroup = MTLSize(width: 4, height: 4, depth: tgDepth)
        let groupsW = (texW + threadsPerGroup.width - 1) / threadsPerGroup.width
        let groupsH = (texH + threadsPerGroup.height - 1) / threadsPerGroup.height
        let groupsD = (texD + threadsPerGroup.depth - 1) / threadsPerGroup.depth
        encoder.dispatchThreadgroups(MTLSize(width: groupsW, height: groupsH, depth: groupsD), threadsPerThreadgroup: threadsPerGroup)
        encoder.endEncoding()

        cmdBuf.commit()
        cmdBuf.waitUntilCompleted()
    }

    func invalidateOffscreenTextures() {
        hdrTex = nil
    }

    func scaledOffscreenSize(from drawableSize: CGSize) -> CGSize {
        let s = CGFloat(renderScale)
        return CGSize(
            width: max(1, (drawableSize.width * s).rounded()),
            height: max(1, (drawableSize.height * s).rounded())
        )
    }

    /// Resolves a HookRopeRef to a rope index.
    /// - "hole": index is the rope index directly
    /// - "hook": follows the chain through referenced hook's ropeA (hookIndex=0) or ropeB (hookIndex=1)
    private static func resolveRopeRef(_ ref: LevelDefinition.HookRopeRef, hooks: [LevelDefinition.Hook], depth: Int = 0) -> Int? {
        guard depth < 16 else {
            logger.error("resolveRopeRef: recursion depth exceeded")
            return nil
        }
        switch ref.fromType {
        case "hole":
            return ref.index
        case "hook":
            guard hooks.indices.contains(ref.index) else {
                logger.error("resolveRopeRef: hook index \(ref.index) out of range")
                return nil
            }
            let refHook = hooks[ref.index]
            let side = ref.hookIndex ?? 0
            let sideRef = (side == 0) ? refHook.ropeA : refHook.ropeB
            return resolveRopeRef(sideRef, hooks: hooks, depth: depth + 1)
        default:
            logger.error("resolveRopeRef: unknown fromType '\(ref.fromType)'")
            return nil
        }
    }

}
