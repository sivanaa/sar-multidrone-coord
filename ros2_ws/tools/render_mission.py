#!/usr/bin/env python3
"""Render a mission_recorder.py log as an animated GIF of the hybrid
approach: PSO coverage search (explored area, each drone's search goal) and
CBBA task allocation (targets, real bids, who won, hand-offs).

Pure Python + Pillow - no ROS, no matplotlib, no ffmpeg - so it runs
anywhere the log can be copied to:

    python3 tools/render_mission.py run.jsonl run.gif --speed 2

The explored-area shading is recomputed with the nodes' own CoverageMap
(coordination_node/coverage.py) from the recorded positions, so it shows
what the drones' fitness actually saw, not an approximation of it.
"""

import argparse
import json
import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'coordination_node'))
from coordination_node.coverage import CoverageMap, load_risk_map  # noqa: E402

W, H = 1280, 720
MAP = 720                        # left square panel
PANEL_X = MAP + 24
BG = (252, 252, 251)
INK = (11, 11, 11)
MUTED = (100, 99, 95)
GRID = (225, 224, 217)
AREA_EDGE = (150, 149, 143)
UNEXPLORED = (255, 255, 255)
EXPLORED = (196, 222, 246)       # fresh; fades back to UNEXPLORED as it goes stale
RISK = (64, 168, 160)            # flood-risk tint on unsearched ground (teal)
RIVER = (31, 110, 160)
DRONE_COLORS = [(42, 120, 214), (235, 104, 52), (27, 175, 122), (237, 161, 0)]
PENDING = (150, 149, 143)
ALERT = (208, 59, 59)
STATE_NAMES = {0: 'searching (PSO)', 1: 'on targets (CBBA)', 2: 'battery at reserve: home to land'}
UNASSIGNED = 255
FONT_DIR = '/usr/share/fonts/truetype/dejavu'


def font(size, bold=False):
    name = 'DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf'
    try:
        return ImageFont.truetype(os.path.join(FONT_DIR, name), size)
    except OSError:
        return ImageFont.load_default()


def mix(a, b, f):
    return tuple(int(a[i] + (b[i] - a[i]) * f) for i in range(3))


class Mission:
    """Everything the renderer needs, derived once from the log."""

    def __init__(self, path, area_center, area_radius, sensor_radius, revisit_s,
                 risk_map=None):
        with open(path) as f:
            self.records = sorted((json.loads(line) for line in f if line.strip()),
                                  key=lambda r: r['t'])
        if not self.records:
            sys.exit(f'{path} is empty')
        self.t0 = self.records[0]['t']
        for r in self.records:
            r['t'] -= self.t0
        self.t_end = self.records[-1]['t']
        self.coverage = CoverageMap(area_center, area_radius,
                                    sensor_radius=sensor_radius,
                                    revisit_after_s=revisit_s,
                                    risk=load_risk_map(risk_map)[1] if risk_map else None)
        self.river = []
        if risk_map:
            with open(risk_map) as f:
                self.river = json.load(f).get("river", [])
        self.drones = sorted({r['drone'] for r in self.records if r['type'] == 'agent'})
        self.targets = {}            # id -> dict(x, y, t)
        for r in self.records:
            if r['type'] == 'target' and r['id'] not in self.targets:
                self.targets[r['id']] = {'x': r['x'], 'y': r['y'], 't': r['t']}
        self.events = self._events()

    def _owner_timeline(self):
        """Per target, [(t, owner)] from each drone's own bundle - the
        drone actually holding it - with sub-0.5s flickers (both drones
        briefly claiming before consensus) dropped."""
        bundles, raw, done = {}, {}, set()
        for r in self.records:
            if r['type'] != 'bundle':
                continue
            bundles[r['drone']] = set(r['bundle'])
            done.update(r['completed'])
            for tid in self.targets:
                if tid in done:      # leaving the bundle on arrival isn't a change of owner
                    continue
                holders = [d for d, b in bundles.items() if tid in b]
                owner = r['drone'] if r['drone'] in holders else (
                    holders[0] if holders else None)
                if not raw.get(tid) or raw[tid][-1][1] != owner:
                    raw.setdefault(tid, []).append((r['t'], owner))
        timeline = {}
        for tid, changes in raw.items():
            kept = []
            for i, (t, owner) in enumerate(changes):
                nxt = changes[i + 1][0] if i + 1 < len(changes) else math.inf
                last = i + 1 == len(changes)
                if (nxt - t >= 0.5 or last) and (not kept or kept[-1][1] != owner):
                    kept.append((t, owner))
            timeline[tid] = kept
        return timeline

    def _events(self):
        events = []
        claims = {}                  # tid -> {drone: own claimed bid}
        claim_log = []               # (t, tid, drone, bid)
        completed_at = {}
        last_state = {}
        for r in self.records:
            if r['type'] == 'bundle':
                for tid, bid, winner in zip(r['tasks'], r['bids'], r['winners']):
                    if winner == r['drone'] and bid > 0:
                        claim_log.append((r['t'], tid, r['drone'], bid))
                for tid in r['completed']:
                    completed_at.setdefault(tid, (r['t'], r['drone']))
            elif r['type'] == 'agent':
                prev = last_state.get(r['drone'])
                if prev is None:
                    events.append((r['t'], f'Drone {r["drone"]} online, searching', None))
                elif prev != r['state'] and r['state'] == 2:
                    events.append((r['t'], f'Drone {r["drone"]}: battery at reserve - '
                                   'hands off its targets, returning to land', ALERT))
                last_state[r['drone']] = r['state']
        for tid, tgt in self.targets.items():
            events.append((tgt['t'], f'T{tid} detected at ({tgt["x"]:.0f}, {tgt["y"]:.0f})',
                           PENDING))
        for tid, changes in self._owner_timeline().items():
            had_owner = False
            for t, owner in changes:
                if owner is None:
                    continue
                for ct, ctid, d, bid in claim_log:
                    if ctid == tid and ct <= t + 0.5:
                        claims.setdefault(tid, {})[d] = bid
                bids = '  '.join(f'D{d} {b:.2f}' for d, b in sorted(claims.get(tid, {}).items()))
                verb = 'reassigned to' if had_owner else 'won by'
                events.append((t, f'T{tid} {verb} Drone {owner}   [bids {bids}]',
                               DRONE_COLORS[owner % len(DRONE_COLORS)]))
                had_owner = True
        self.owner_timeline = self._owner_timeline()
        for tid, (t, sender) in completed_at.items():
            owners = [o for ct, o in self.owner_timeline.get(tid, []) if ct <= t and o is not None]
            who = f'Drone {owners[-1]}' if owners else f'Drone {sender}'
            events.append((t, f'{who} reached T{tid}', None))
        self.completed_at = {tid: t for tid, (t, _) in completed_at.items()}
        return sorted(events, key=lambda e: e[0])


class Renderer:
    def __init__(self, mission, area_center, area_radius):
        self.m = mission
        cx, cy = area_center
        half = area_radius + 1.5
        # Screen right = shared +y (east), screen up = shared +x (north):
        # the same top-down view as Gazebo's (Gazebo x = shared y).
        self.ymin, self.xmax = cy - half, cx + half
        self.scale = MAP / (2 * half)
        self.area_center, self.area_radius = area_center, area_radius
        self.f_title = font(26, True)
        self.f_sub = font(15)
        self.f_head = font(17, True)
        self.f_body = font(15)
        self.f_small = font(13)
        self.f_label = font(14, True)
        self.cursor = 0
        self.agent = {}              # drone -> latest agent record
        self.trail = {d: [] for d in mission.drones}
        self.battery = {}
        self.bundles = {}

    def px(self, x, y):
        return ((y - self.ymin) * self.scale, (self.xmax - x) * self.scale)

    def advance(self, t):
        recs = self.m.records
        while self.cursor < len(recs) and recs[self.cursor]['t'] <= t:
            r = recs[self.cursor]
            self.cursor += 1
            if r['type'] == 'agent':
                self.agent[r['drone']] = r
                self.trail[r['drone']].append((r['t'], r['x'], r['y']))
                self.m.coverage.mark_seen(r['x'], r['y'], r['t'])
            elif r['type'] == 'battery':
                # PX4 SITL refills a landed drone's simulated battery to
                # 100%; once a drone is returning, keep showing its low.
                returning = self.agent.get(r['drone'], {}).get('state') == 2
                old = self.battery.get(r['drone'])
                self.battery[r['drone']] = (min(old, r['remaining'])
                                            if returning and old is not None
                                            else r['remaining'])
            elif r['type'] == 'bundle':
                self.bundles[r['drone']] = r['bundle']

    def owner(self, tid, t):
        owner = None
        for ct, o in self.m.owner_timeline.get(tid, []):
            if ct <= t:
                owner = o
        return owner

    def frame(self, t):
        self.advance(t)
        img = Image.new('RGB', (W, H), BG)
        d = ImageDraw.Draw(img)
        self._map(d, t)
        self._panel(d, t)
        return img

    def _map(self, d, t):
        cov = self.m.coverage
        s = cov.cell_size
        for cell in cov.area_cells:
            seen = cov.last_seen.get(cell)
            age = None if seen is None else t - seen
            revisit = cov.revisit_after(cell)
            unsearched = mix(UNEXPLORED, RISK, 0.6 * cov.risk.get(cell, 0.0))
            searched = mix(EXPLORED, RISK, 0.35 * cov.risk.get(cell, 0.0))
            fill = unsearched if age is None or age >= revisit else mix(
                searched, unsearched, age / revisit)
            x0, y0 = cell[0] * s, cell[1] * s
            a = self.px(x0 + s, y0)
            b = self.px(x0, y0 + s)
            d.rectangle([a, b], fill=fill, outline=GRID)
        if len(self.m.river) > 1:
            d.line([self.px(*p) for p in self.m.river], fill=RIVER, width=3)
        cx, cy = self.px(*self.area_center)
        r = self.area_radius * self.scale
        d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=AREA_EDGE, width=2)
        d.text((10, MAP - 24), 'search area  (1 m cells; blue = recently searched'
               + ('; teal = flood risk, searched first and more often)'
                  if cov.risk else ')'), font=self.f_small, fill=MUTED)

        for tid, tgt in self.m.targets.items():
            if tgt['t'] > t:
                continue
            owner = self.owner(tid, t)
            done = tid in self.m.completed_at and self.m.completed_at[tid] <= t
            color = PENDING if owner is None else DRONE_COLORS[owner % len(DRONE_COLORS)]
            x, y = self.px(tgt['x'], tgt['y'])
            k = 11
            diamond = [(x, y - k), (x + k, y), (x, y + k), (x - k, y)]
            d.polygon(diamond, fill=mix(color, BG, 0.5) if done else color, outline=color)
            if done:
                d.line([(x - 5, y), (x - 1, y + 5), (x + 6, y - 5)], fill=INK, width=3)
            d.text((x + 13, y - 9), f'T{tid}', font=self.f_label, fill=INK)

        for drone in self.m.drones:
            rec = self.agent.get(drone)
            if rec is None:
                continue
            color = DRONE_COLORS[drone % len(DRONE_COLORS)]
            pts = [self.px(x, y) for tt, x, y in self.trail[drone] if t - tt <= 25]
            if len(pts) > 1:
                d.line(pts, fill=mix(color, BG, 0.45), width=3)
            x, y = self.px(rec['x'], rec['y'])
            if rec['state'] == 0:      # PSO goal while searching
                gx, gy = self.px(*rec['goal'])
                d.line([(x, y), (gx, gy)], fill=mix(color, BG, 0.6), width=1)
                d.ellipse([gx - 5, gy - 5, gx + 5, gy + 5], outline=color, width=2)
            elif rec['state'] == 1:    # lines to the targets it holds
                for tid in self.bundles.get(drone, []):
                    if tid in self.m.targets:
                        tx, ty = self.px(self.m.targets[tid]['x'], self.m.targets[tid]['y'])
                        d.line([(x, y), (tx, ty)], fill=color, width=2)
            ring = ALERT if rec['state'] == 2 else INK
            d.ellipse([x - 11, y - 11, x + 11, y + 11], fill=color, outline=ring, width=2)
            d.text((x - 9, y - 8), f'D{drone}', font=font(11, True), fill=(255, 255, 255))

    def _panel(self, d, t):
        x0 = PANEL_X
        d.text((x0, 22), 'Hybrid PSO search + CBBA tasking', font=self.f_title, fill=INK)
        d.text((x0, 56), '2 PX4 drones in Gazebo, decentralized (no ground station)',
               font=self.f_sub, fill=MUTED)
        d.text((x0, 86), f't = {t:5.1f} s', font=self.f_head, fill=INK)
        cov = self.m.coverage
        d.text((x0 + 150, 86), f'area searched recently: {cov.explored_fraction(t):.0%}',
               font=self.f_head, fill=INK)
        y = 124
        in_area = {c: r for c, r in cov.risk.items() if c in cov.area_cells}
        if in_area:
            fresh = sum(r for c, r in in_area.items()
                        if c in cov.last_seen and t - cov.last_seen[c] < cov.revisit_after(c))
            d.text((x0 + 150, 112), 'flood-risk ground searched: '
                   f'{fresh / sum(in_area.values()):.0%}', font=self.f_head, fill=RISK)
            y = 150
        for drone in self.m.drones:
            color = DRONE_COLORS[drone % len(DRONE_COLORS)]
            rec = self.agent.get(drone)
            d.rounded_rectangle([x0, y, W - 24, y + 58], radius=8, outline=GRID,
                                fill=(255, 255, 255))
            d.ellipse([x0 + 12, y + 18, x0 + 34, y + 40], fill=color)
            state = STATE_NAMES.get(rec['state'], '?') if rec else 'starting'
            if rec and rec['state'] == 1 and self.bundles.get(drone):
                state += ': ' + ', '.join(f'T{tid}' for tid in self.bundles[drone])
            d.text((x0 + 46, y + 8), f'Drone {drone}', font=self.f_head, fill=INK)
            d.text((x0 + 46, y + 32), state, font=self.f_body,
                   fill=ALERT if rec and rec['state'] == 2 else MUTED)
            battery = self.battery.get(drone)
            if battery is not None:
                bx = W - 24 - 140
                d.text((bx, y + 8), f'battery {battery:.0%}', font=self.f_small, fill=MUTED)
                d.rectangle([bx, y + 30, bx + 110, y + 42], outline=AREA_EDGE)
                level = ALERT if battery <= 0.25 else (27, 175, 122)
                d.rectangle([bx + 1, y + 31, bx + 1 + int(108 * min(1, battery)), y + 41],
                            fill=level)
            y += 68

        d.text((x0, y + 6), 'Events', font=self.f_head, fill=INK)
        shown = [e for e in self.m.events if e[0] <= t][-11:]
        y += 34
        for i, (et, text, color) in enumerate(shown):
            age = t - et
            ink = INK if age < 4 else MUTED
            d.text((x0, y), f'{et:5.1f}s', font=self.f_small, fill=MUTED)
            if color:
                d.ellipse([x0 + 52, y + 4, x0 + 62, y + 14], fill=color)
            d.text((x0 + 70, y), text, font=self.f_small, fill=ink)
            y += 24

        ly = H - 30
        d.text((x0, ly), '◆ target (colour = drone that won it, ✓ = reached)   '
               '○ PSO search goal', font=self.f_small, fill=MUTED)


def fixed_palette():
    """One GIF palette for every frame, built from the design colours and
    the blends actually drawn. Per-frame adaptive palettes (the first
    version) drifted: drone 0 went teal and drone 1 brown in later frames
    of a live run, as the map's blues and teals crowded them out."""
    colors = [BG, INK, MUTED, GRID, AREA_EDGE, UNEXPLORED, EXPLORED, RISK, RIVER,
              PENDING, ALERT, (255, 255, 255), (27, 175, 122)] + DRONE_COLORS
    for r in range(6):                  # map cells: risk x freshness
        unsearched = mix(UNEXPLORED, RISK, 0.6 * r / 5)
        searched = mix(EXPLORED, RISK, 0.35 * r / 5)
        colors += [mix(searched, unsearched, a / 9) for a in range(10)]
    for c in DRONE_COLORS + [INK, PENDING, ALERT, RISK, RIVER, (27, 175, 122)]:
        colors += [mix(c, BG, f / 11) for f in range(1, 12)]   # trails, text edges
        colors += [mix(c, (255, 255, 255), f / 5) for f in range(1, 5)]
    colors = list(dict.fromkeys(colors))[:256]
    flat = [v for c in colors for v in c] + [0] * (3 * (256 - len(colors)))
    pal = Image.new('P', (1, 1))
    pal.putpalette(flat)
    return pal


def main():
    p = argparse.ArgumentParser()
    p.add_argument('log')
    p.add_argument('out', help='.gif to write')
    p.add_argument('--speed', type=float, default=2.0, help='playback speed (x real time)')
    p.add_argument('--fps', type=float, default=8.0)
    p.add_argument('--start', type=float, default=0.0, help='skip the first N seconds')
    p.add_argument('--end', type=float, default=None, help='stop at N seconds')
    p.add_argument('--area', type=float, nargs=3, default=[4.0, 4.0, 6.0],
                   metavar=('CX', 'CY', 'R'), help='search area, as the nodes were launched')
    p.add_argument('--sensor-radius', type=float, default=1.5)
    p.add_argument('--revisit', type=float, default=60.0)
    p.add_argument('--frames-dir', help='also write numbered PNG frames here')
    p.add_argument('--risk-map', help='flood-risk map the nodes were given (risk layer)')
    a = p.parse_args()

    mission = Mission(a.log, tuple(a.area[:2]), a.area[2], a.sensor_radius, a.revisit,
                      a.risk_map)
    renderer = Renderer(mission, tuple(a.area[:2]), a.area[2])
    end = mission.t_end if a.end is None else min(a.end, mission.t_end)
    step = a.speed / a.fps
    times = [a.start + i * step for i in range(int((end - a.start) / step) + 1)]
    if a.frames_dir:
        os.makedirs(a.frames_dir, exist_ok=True)

    palette = fixed_palette()
    frames = []
    for i, t in enumerate(times):
        img = renderer.frame(t)
        if a.frames_dir:
            img.save(os.path.join(a.frames_dir, f'{i:05d}.png'))
        frames.append(img.quantize(palette=palette, dither=0))
        if i % 50 == 0:
            print(f'frame {i}/{len(times)}  t={t:.1f}s', flush=True)
    hold = [frames[-1]] * int(a.fps * 2)   # linger on the final state
    frames[0].save(a.out, save_all=True, append_images=frames[1:] + hold,
                   duration=int(1000 / a.fps), loop=0)
    print(f'wrote {a.out}: {len(frames)} frames, {os.path.getsize(a.out) / 1e6:.1f} MB, '
          f'{mission.t_end:.0f}s of mission at {a.speed}x')


if __name__ == '__main__':
    main()
