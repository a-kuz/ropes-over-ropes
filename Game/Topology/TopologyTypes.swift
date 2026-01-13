import simd

enum TopologyNode: Hashable {
    case hole(Int)
    case hook(Int)
    case floating(Int)
}

struct TopologyRope {
    var startHole: Int
    var endHole: Int
    var hooks: [Int]
    var color: SIMD3<Float>
    var active: Bool
    var floatingEnd: Int?
    var floatingPosition: SIMD2<Float>?
}

struct HookSequence {
    var id: Int
    var ropeA: Int
    var ropeB: Int
    var N: Int
    var ropeAStartIsOver: Bool
    var center: SIMD2<Float> = .zero
}

struct HookSequenceGeometry {
    let centers: [SIMD2<Float>]
    let R: Float
    let pathA: [SIMD2<Float>]
    let pathB: [SIMD2<Float>]
}
