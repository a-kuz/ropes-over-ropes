import simd

struct TopologyCrossing: Hashable {
    let id: Int
    let ropeA: Int
    let ropeB: Int
    let position: SIMD2<Float>
    let ropeOver: Int
    let handedness: Int
}

enum TopologyNode: Hashable {
    case hole(Int)
    case crossing(Int)
    case floating(Int)
}

struct TopologyRope {
    var nodes: [TopologyNode]
    var color: SIMD3<Float>
    var active: Bool
}

struct HookPair {
    let crossingIdA: Int
    let crossingIdB: Int
    let ropeA: Int
    let ropeB: Int
    let ropeAUpperEnd: Int
    let ropeBUpperEnd: Int
}

