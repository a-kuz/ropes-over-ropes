#!/usr/bin/env python3
"""Decompose a level JSON with hooks into a physics-ready JSON.

Takes a level with hooks (topological crossings) and runs:
1. decompose_level() — converts hooks into pin + drag action sequence
2. PegboardSim — Verlet physics simulation to execute the actions
3. Outputs a new JSON with updated rope startHole/endHole (no hooks)
   plus an "actions" array for the iOS VerletSimulator to replay.

Usage:
    python3 decompose_level.py input.json output.json [--particles 60]
    python3 decompose_level.py input.json  # overwrites in place
"""

import json
import sys
import os
import numpy as np
from copy import deepcopy


# ================================================================
# Verlet Simulator (minimal port from pegboard_demo.py)
# ================================================================

class PegboardSim:
    def __init__(self, holes, hole_radius=0.1):
        self.holes = np.array(holes, dtype=float)
        self.hole_radius = hole_radius
        self.hole_depth = hole_radius * 1.25
        self.bands = []

        self.gravity_z = -5.0
        self.damping = 0.97
        self.dt = 1.0 / 60.0
        self.constraint_iters = 20
        self.settle_steps = 40
        self.drag_steps = 8
        self.lift_height = 0.30
        self.collision_mult = 1.2

    def add_band(self, radius, n_pts=60):
        self.bands.append({
            'pts': None, 'old': None, 'pins': [None, None],
            'seg_len': 0, 'R': radius, 'n': n_pts,
        })
        return len(self.bands) - 1

    def _h3d(self, hi):
        return np.array([self.holes[hi, 0], self.holes[hi, 1], -self.hole_depth])

    def pin(self, bi, end, hi):
        b = self.bands[bi]
        b['pins'][end] = hi
        if b['pins'][0] is not None and b['pins'][1] is not None and b['pts'] is None:
            p0, p1 = self._h3d(b['pins'][0]), self._h3d(b['pins'][1])
            n = b['n']
            t = np.linspace(0, 1, n)[:, None]
            b['pts'] = np.column_stack([
                p0[0] * (1 - t.ravel()) + p1[0] * t.ravel(),
                p0[1] * (1 - t.ravel()) + p1[1] * t.ravel(),
                np.full(n, self.lift_height)
            ])
            b['pts'][0] = p0
            b['pts'][-1] = p1
            b['old'] = b['pts'].copy()
            b['seg_len'] = np.linalg.norm(p1 - p0) / (n - 1)
            has_others = any(ob['pts'] is not None for ob in self.bands if ob is not b)
            self._do_steps(self.settle_steps, collide=has_others)

    def drag(self, bi, end, to_hi, n_steps=30):
        b = self.bands[bi]
        idx = 0 if end == 0 else -1
        from_pos = b['pts'][idx].copy()
        to_pos = self._h3d(to_hi)
        b['pins'][end] = None

        lift_from = from_pos.copy(); lift_from[2] = self.lift_height
        lift_to = to_pos.copy(); lift_to[2] = self.lift_height

        # Lift
        for t in np.linspace(0, 1, 5)[1:]:
            wp = from_pos + t * (lift_from - from_pos)
            b['pts'][idx] = wp; b['old'][idx] = wp
            self._do_steps(self.drag_steps, collide=True)

        # Traverse
        for t in np.linspace(0, 1, n_steps + 1)[1:]:
            wp = lift_from + t * (lift_to - lift_from)
            b['pts'][idx] = wp; b['old'][idx] = wp
            self._do_steps(self.drag_steps, collide=True)

        # Lower
        for t in np.linspace(0, 1, 5)[1:]:
            wp = lift_to + t * (to_pos - lift_to)
            b['pts'][idx] = wp; b['old'][idx] = wp
            self._do_steps(self.drag_steps, collide=True)

        # Re-pin and settle
        b['pins'][end] = to_hi
        b['pts'][idx] = to_pos; b['old'][idx] = to_pos
        self._do_steps(self.settle_steps, collide=True)

    def _do_steps(self, n, collide=False):
        for _ in range(n):
            self._verlet_step(collide)

    def _verlet_step(self, collide):
        dt2 = self.dt * self.dt
        for b in self.bands:
            if b['pts'] is None: continue
            p, old = b['pts'], b['old']
            temp = p.copy()
            vel = (p - old) * self.damping
            p[1:-1] += vel[1:-1]
            p[1:-1, 2] += self.gravity_z * dt2
            old[:] = temp

        for _ in range(self.constraint_iters):
            for b in self.bands:
                if b['pts'] is None: continue
                self._band_constraints(b)
            if collide:
                active = [b for b in self.bands if b['pts'] is not None]
                for i in range(len(active)):
                    for j in range(i + 1, len(active)):
                        self._collide(active[i], active[j])

    def _band_constraints(self, b):
        p, R, seg_len = b['pts'], b['R'], b['seg_len']
        n = len(p)
        for offset in (0, 1):
            idx = np.arange(offset, n - 1, 2)
            diff = p[idx + 1] - p[idx]
            dist = np.linalg.norm(diff, axis=1, keepdims=True) + 1e-12
            corr = diff * ((dist - seg_len) / dist * 0.5)
            mask0 = idx > 0
            mask1 = (idx + 1) < (n - 1)
            p[idx[mask0]] += corr[mask0]
            p[idx[mask1] + 1] -= corr[mask1]
        if b['pins'][0] is not None: p[0] = self._h3d(b['pins'][0])
        if b['pins'][1] is not None: p[-1] = self._h3d(b['pins'][1])
        p[1:-1, 2] = np.maximum(p[1:-1, 2], R)

    def _collide(self, b1, b2):
        p1, p2 = b1['pts'], b2['pts']
        min_dist = (b1['R'] + b2['R']) * self.collision_mult
        s1, s2 = p1[1:-1], p2[1:-1]
        diff = s1[:, None, :] - s2[None, :, :]
        dist = np.linalg.norm(diff, axis=2) + 1e-12
        mask = dist < min_dist
        if not np.any(mask): return
        overlap = min_dist - dist[mask]
        n_dir = diff[mask] / dist[mask, None]
        corr = n_dir * (overlap[:, None] * 0.5)
        rows, cols = np.where(mask)
        np.add.at(s1, rows, corr)
        np.add.at(s2, cols, -corr)


# ================================================================
# Decompose (from pegboard_demo.py)
# ================================================================

def decompose_level(level):
    holes = np.array([[h['x'], h['y']] for h in level['holes']])
    ropes = level['ropes']
    hooks = level.get('hooks', [])
    nh, nr = len(holes), len(ropes)

    if not hooks:
        actions = []
        for ri, r in enumerate(ropes):
            actions.append(('pin', ri, 0, r['startHole']))
            actions.append(('pin', ri, 1, r['endHole']))
        return actions

    # Parse hooks
    def resolve_rope(ref):
        if ref['fromType'] == 'hole':
            return ref['index']
        parent = hooks[ref['index']]
        side = parent['ropeA'] if ref.get('hookIndex', 0) == 0 else parent['ropeB']
        return resolve_rope(side)

    interactions = []
    for h in hooks:
        rA = resolve_rope(h['ropeA'])
        rB = resolve_rope(h['ropeB'])
        interactions.append((rA, rB, h['N'], h['ropeAStartIsOver']))

    adj = {i: set() for i in range(nr)}
    for rA, rB, _N, _over in interactions:
        adj[rA].add(rB); adj[rB].add(rA)

    backbone = max(range(nr), key=lambda i: len(adj[i]))
    print(f"  backbone: rope {backbone} ({len(adj[backbone])} interactions)")

    def _seg_cross(a, b, c, d):
        r = b - a; s = d - c
        den = r[0] * s[1] - r[1] * s[0]
        if abs(den) < 1e-12: return False, 0, 0
        t = ((c[0] - a[0]) * s[1] - (c[1] - a[1]) * s[0]) / den
        u = ((c[0] - a[0]) * r[1] - (c[1] - a[1]) * r[0]) / den
        return (-0.01 <= t <= 1.01 and -0.01 <= u <= 1.01), t, u

    # Max diagonal
    bb_h0, bb_h1 = max(
        ((i, j) for i in range(nh) for j in range(i + 1, nh)),
        key=lambda p: np.linalg.norm(holes[p[0]] - holes[p[1]]))
    bb_len = np.linalg.norm(holes[bb_h0] - holes[bb_h1])
    print(f"  backbone init: holes {bb_h0}<->{bb_h1}, dist={bb_len:.3f}")

    # Assign hooks to backbone ends
    hook_end = {}
    cur_end = 0
    for rA, rB, _N, _over in interactions:
        other = rB if rA == backbone else rA if rB == backbone else None
        if other is not None and other not in hook_end:
            hook_end[other] = cur_end
            cur_end = 1 - cur_end

    end0_anchors = [ri for ri, e in hook_end.items() if e == 0]
    end1_anchors = [ri for ri, e in hook_end.items() if e == 1]

    # Anchor placement
    used = {bb_h0, bb_h1}
    anchor_info = {}
    min_anchor_len = bb_len * 0.25

    for ri in range(nr):
        if ri == backbone: continue
        r = ropes[ri]
        fs, fe = r['startHole'], r['endHole']
        if ri not in hook_end:
            anchor_info[ri] = (fs, fe)
            used.update({fs, fe})
            continue

        assigned_end = hook_end[ri]
        bb_near = bb_h0 if assigned_end == 0 else bb_h1
        best, best_score = None, np.inf

        def _try_anchor(ha, hb):
            length = np.linalg.norm(holes[ha] - holes[hb])
            if length < min_anchor_len: return np.inf
            ok, t, u = _seg_cross(holes[ha], holes[hb], holes[bb_h0], holes[bb_h1])
            if not ok or t < 0.05 or t > 0.95 or u < 0.05 or u > 0.95: return np.inf
            centrality = abs(t - 0.5) + abs(u - 0.5)
            mid = (holes[ha] + holes[hb]) / 2
            prox = np.linalg.norm(mid - holes[bb_near]) / bb_len
            return centrality * 0.3 + prox

        for h in range(nh):
            if h in used: continue
            if fe not in used and h != fe:
                score = _try_anchor(h, fe)
                if score < best_score: best_score, best = score, (h, fe)
            if fs not in used and h != fs:
                score = _try_anchor(fs, h)
                if score < best_score: best_score, best = score, (fs, h)

        if best is None:
            for h0 in range(nh):
                if h0 in used: continue
                for h1 in range(h0 + 1, nh):
                    if h1 in used: continue
                    score = _try_anchor(h0, h1)
                    if score < best_score: best_score, best = score, (h0, h1)

        if best is None: best = (fs, fe)
        anchor_info[ri] = best
        used.update(best)

    # Drag targets
    def find_target(from_h, anchor_ris, avoid):
        if not anchor_ris: return from_h
        anchor_lines = [(holes[anchor_info[ri][0]], holes[anchor_info[ri][1]]) for ri in anchor_ris]
        best_h, best_score = from_h, np.inf
        for h in range(nh):
            if h == from_h or h in avoid: continue
            d = np.linalg.norm(holes[from_h] - holes[h])
            ratio = d / bb_len
            if ratio > 0.85 or ratio < 0.15: continue
            if not all(_seg_cross(holes[from_h], holes[h], a, b)[0] for a, b in anchor_lines):
                continue
            score = abs(ratio - 0.5)
            if score < best_score: best_score, best_h = score, h
        return best_h

    tgt0 = find_target(bb_h0, end0_anchors, used)
    tgt1 = find_target(bb_h1, end1_anchors, used | {tgt0})
    print(f"  drag targets: end0 {bb_h0}->{tgt0}, end1 {bb_h1}->{tgt1}")

    # Build actions
    actions = []
    actions.append(('pin', backbone, 0, bb_h0))
    actions.append(('pin', backbone, 1, bb_h1))
    for ri in range(nr):
        if ri == backbone: continue
        h0, h1 = anchor_info[ri]
        actions.append(('pin', ri, 0, h0))
        actions.append(('pin', ri, 1, h1))
    if tgt0 != bb_h0:
        actions.append(('drag', backbone, 0, tgt0))
    if tgt1 != bb_h1:
        actions.append(('drag', backbone, 1, tgt1))

    return actions


# ================================================================
# Main: decompose + simulate + write output JSON
# ================================================================

def process_level(input_path, output_path, n_particles=60):
    with open(input_path) as f:
        level = json.load(f)

    print(f"Processing: {input_path}")
    print(f"  {len(level['ropes'])} ropes, {len(level['holes'])} holes, "
          f"{len(level.get('hooks', []))} hooks")

    actions = decompose_level(level)

    print(f"  Actions ({len(actions)}):")
    for i, a in enumerate(actions):
        print(f"    {i}: {a[0]} rope={a[1]} end={a[2]} hole={a[3]}")

    # Run simulation
    holes = np.array([[h['x'], h['y']] for h in level['holes']])
    hr = level['holeRadius']
    ropes = level['ropes']

    sim = PegboardSim(holes, hole_radius=hr)
    band_map = {}

    for a in actions:
        typ, ri, end, hole = a
        if typ == 'pin':
            if ri not in band_map:
                band_map[ri] = sim.add_band(ropes[ri]['radius'], n_pts=n_particles)
            sim.pin(band_map[ri], end, hole)
        else:
            sim.drag(band_map[ri], end, hole, n_steps=30)

    # Read final positions from simulation
    out = deepcopy(level)

    # Update rope endpoints to match physics
    for ri, bi in band_map.items():
        b = sim.bands[bi]
        out['ropes'][ri]['startHole'] = b['pins'][0]
        out['ropes'][ri]['endHole'] = b['pins'][1]

    # Store actions for iOS sequential pin replay
    out_actions = []
    for a in actions:
        out_actions.append({
            'type': a[0],
            'ropeIndex': a[1],
            'endIndex': a[2],
            'holeIndex': a[3]
        })
    out['actions'] = out_actions

    # Remove hooks — no longer needed, topology is baked into actions
    if 'hooks' in out:
        del out['hooks']

    # Write output
    with open(output_path, 'w') as f:
        json.dump(out, f, indent=2)

    print(f"  Output: {output_path}")
    print(f"  Final rope positions:")
    for ri, r in enumerate(out['ropes']):
        print(f"    rope {ri}: {r['startHole']} -> {r['endHole']}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} input.json [output.json] [--particles N]")
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith('--') else input_path

    n_particles = 60
    for i, arg in enumerate(sys.argv):
        if arg == '--particles' and i + 1 < len(sys.argv):
            n_particles = int(sys.argv[i + 1])

    process_level(input_path, output_path, n_particles)
