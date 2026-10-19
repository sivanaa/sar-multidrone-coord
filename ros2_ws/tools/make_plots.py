#!/usr/bin/env python3
"""Report figures from tools/report_sweeps.sh's results.

    python3 tools/make_plots.py            # results/data -> results/figures

Writes one SVG per figure plus a Markdown table of the same numbers
(results/figures/*.md - the table view: every value plotted, for readers
who can't tell the colors apart, and for the report). Plain SVG with no
plotting library, so it runs on the lab machine as is (its matplotlib is
built against NumPy 1 and AirSim needs NumPy 2).

Colors are the reference data-viz palette, light mode; each method keeps
its color in every figure: ours (blue), the comparison it is measured
against - lawnmower / plain CBBA / nearest drone (orange), random (aqua).
"""

import json
import math
import os
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, '..', '..', 'results', 'data')
FIGS = os.path.join(HERE, '..', '..', 'results', 'figures')

SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_2 = '#52514e'
MUTED = '#898781'
GRID = '#e1e0d9'
AXIS = '#c3c2b7'
BLUE, ORANGE, AQUA, YELLOW = '#2a78d6', '#eb6834', '#1baf7a', '#eda100'
FONT = 'system-ui, -apple-system, "Segoe UI", sans-serif'

SEARCH_COLORS = {'pso': BLUE, 'lawnmower': ORANGE, 'random': AQUA,
                 'greedy': YELLOW}  # greedy: tables only (see fig_search)
SEARCH_NAMES = {'pso': 'PSO + coverage map (ours)', 'lawnmower': 'Lawnmower',
                'greedy': 'Coverage map, no swarm motion',
                'random': 'Random waypoints'}


def load(name):
    with open(os.path.join(DATA, name + '.json')) as f:
        return json.load(f)


def reached_times(name):
    """Seconds until every person was reached, per mission (None = never)."""
    return [r['all_reached_s'] for r in load(name)['results']]


def median_done(times):
    done = [t for t in times if t is not None]
    return statistics.median(done) if done else None


def nice_ticks(lo, hi, count=5):
    span = hi - lo
    raw = span / count
    mag = 10 ** math.floor(math.log10(raw))
    step = min((s * mag for s in (1, 2, 2.5, 5, 10) if s * mag >= raw))
    first = math.ceil(lo / step) * step
    ticks, t = [], first
    while t <= hi + 1e-9:
        ticks.append(round(t, 10))
        t += step
    if ticks[-1] < hi:  # the axis must reach past the tallest mark
        ticks.append(round(ticks[-1] + step, 10))
    return ticks


def fmt(v):
    return f'{v:g}' if abs(v) < 1000 else f'{v / 1000:g}K'


class Svg:

    def __init__(self, width, height):
        self.w, self.h = width, height
        self.parts = []

    def add(self, s):
        self.parts.append(s)

    def text(self, x, y, s, size=12, color=INK_2, anchor='start', weight=400):
        s = s.replace('&', '&amp;').replace('<', '&lt;')
        self.add(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" '
                 f'fill="{color}" text-anchor="{anchor}" '
                 f'font-weight="{weight}">{s}</text>')

    def line(self, x1, y1, x2, y2, color=GRID, width=1):
        self.add(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" '
                 f'y2="{y2:.1f}" stroke="{color}" stroke-width="{width}"/>')

    def polyline(self, points, color, width=2):
        pts = ' '.join(f'{x:.1f},{y:.1f}' for x, y in points)
        self.add(f'<polyline points="{pts}" fill="none" stroke="{color}" '
                 f'stroke-width="{width}" stroke-linejoin="round" '
                 f'stroke-linecap="round"/>')

    def dot(self, x, y, color, r=4.5):
        # 2px surface ring keeps it legible where it crosses a line
        self.add(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r + 2}" fill="{SURFACE}"/>'
                 f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r}" fill="{color}"/>')

    def bar(self, x, y_base, y_top, width, color):
        """Column from the baseline: 4px rounded data end, square base."""
        h = y_base - y_top
        if h <= 0:
            return
        r = min(4.0, h, width / 2)
        self.add(f'<path d="M{x:.1f},{y_base:.1f} V{y_top + r:.1f} '
                 f'Q{x:.1f},{y_top:.1f} {x + r:.1f},{y_top:.1f} '
                 f'H{x + width - r:.1f} Q{x + width:.1f},{y_top:.1f} '
                 f'{x + width:.1f},{y_top + r:.1f} V{y_base:.1f} Z" '
                 f'fill="{color}"/>')

    def save(self, path, title, desc):
        body = '\n'.join(self.parts)
        # ids unique per figure, so several can be inlined in one page
        key = os.path.splitext(os.path.basename(path))[0]
        with open(path, 'w') as f:
            f.write(
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" '
                f'height="{self.h}" viewBox="0 0 {self.w} {self.h}" '
                f'font-family=\'{FONT}\' role="img" '
                f'aria-labelledby="{key}-t {key}-d">\n'
                f'<title id="{key}-t">{title}</title>'
                f'<desc id="{key}-d">{desc}</desc>\n'
                f'<rect width="{self.w}" height="{self.h}" fill="{SURFACE}"/>\n'
                f'{body}\n</svg>\n')


class Panel:
    """One plot area: linear scales, recessive y gridlines, baseline."""

    def __init__(self, svg, x, y, w, h, xlim, ylim, yticks, ylabel=None,
                 xticks=None, xlabel=None, title=None, yfmt=fmt):
        self.svg, self.x, self.y, self.w, self.h = svg, x, y, w, h
        self.xlim, self.ylim = xlim, ylim
        if title:
            svg.text(x, y - 12, title, size=13, color=INK, weight=600)
        for t in yticks:
            ty = self.sy(t)
            if t != ylim[0]:
                svg.line(x, ty, x + w, ty, GRID)
            svg.text(x - 8, ty + 4, yfmt(t), size=11, color=MUTED, anchor='end')
        svg.line(x, y + h, x + w, y + h, AXIS)
        if xticks:
            for t in xticks:
                svg.text(self.sx(t), y + h + 16, fmt(t), size=11, color=MUTED,
                         anchor='middle')
        if xlabel:
            svg.text(x + w / 2, y + h + 34, xlabel, size=11, color=INK_2,
                     anchor='middle')
        if ylabel:
            svg.add(f'<text transform="translate({x - 40:.1f},{y + h / 2:.1f}) '
                    f'rotate(-90)" font-size="11" fill="{INK_2}" '
                    f'text-anchor="middle">{ylabel}</text>')

    def sx(self, v):
        lo, hi = self.xlim
        return self.x + (v - lo) / (hi - lo) * self.w

    def sy(self, v):
        lo, hi = self.ylim
        return self.y + self.h - (v - lo) / (hi - lo) * self.h


def legend(svg, x, y, items, kind='square'):
    """One row of (label, color) keys; swatch beside ink-colored text."""
    for label, color in items:
        if kind == 'line':
            svg.line(x, y - 4, x + 16, y - 4, color, 2)
            svg.dot(x + 8, y - 4, color, r=3)
            x += 22
        else:
            svg.add(f'<rect x="{x}" y="{y - 10}" width="10" height="10" '
                    f'rx="2" fill="{color}"/>')
            x += 15
        svg.text(x, y, label, size=12, color=INK_2)
        x += 7.2 * len(label) + 20


def grouped_bars(panel, groups, series, values, colors, bar_w=20, gap=2):
    """values[g][s] (None = skipped); bars in a group touch with a 2px gap."""
    slot = panel.w / len(groups)
    n = len(series)
    for gi, g in enumerate(groups):
        group_w = n * bar_w + (n - 1) * gap
        x0 = panel.x + gi * slot + (slot - group_w) / 2
        for si, s in enumerate(series):
            v = values[gi][si]
            if v is not None:
                panel.svg.bar(x0 + si * (bar_w + gap), panel.sy(panel.ylim[0]),
                              panel.sy(v), bar_w, colors[s])
        panel.svg.text(panel.x + gi * slot + slot / 2, panel.y + panel.h + 16, g,
                       size=11, color=INK_2, anchor='middle')


def md_table(path, header, rows, note=''):
    with open(path, 'w') as f:
        f.write('| ' + ' | '.join(header) + ' |\n')
        f.write('|' + '---|' * len(header) + '\n')
        for r in rows:
            f.write('| ' + ' | '.join(str(c) for c in r) + ' |\n')
        if note:
            f.write('\n' + note + '\n')


# ---------------------------------------------------------------- figures

def fig_cbba_optimality():
    sizes = [(2, 4), (3, 5), (4, 6)]
    data = [load(f'opt_n{n}') for n, _ in sizes]
    svg = Svg(720, 330)
    svg.text(24, 30, 'CBBA assignments score within ~1% of the best possible',
             size=15, color=INK, weight=600)
    svg.text(24, 50, 'Mean score as % of the brute-force optimum, 300 random '
             'problems each; whisker down to the worst 10%', size=12)
    legend(svg, 24, 78, [('CBBA (ours)', BLUE), ('Nearest drone', ORANGE)],
           kind='line')
    p = Panel(svg, 80, 104, 600, 180, (0, len(sizes)), (60, 100),
              [60, 70, 80, 90, 100], ylabel='% of optimal',
              yfmt=lambda v: f'{v:g}%')
    rows = []
    for k, ((n, m), d) in enumerate(zip(sizes, data)):
        cx = p.sx(k + 0.5)
        for key, color, off in (('value_ratio', BLUE, -14),
                                ('nearest_value_ratio', ORANGE, 14)):
            vals = sorted(100 * v for v in d[key])
            mean = statistics.mean(vals)
            p10 = vals[int(0.1 * (len(vals) - 1))]
            svg.line(cx + off, p.sy(p10), cx + off, p.sy(mean), color, 2)
            svg.dot(cx + off, p.sy(mean), color)
            svg.text(cx + off + (10 if off > 0 else -10), p.sy(mean) + 4,
                     f'{mean:.1f}', size=11, color=INK_2,
                     anchor='start' if off > 0 else 'end')
        svg.text(cx, p.y + p.h + 18, f'{n} drones, {m} people', size=11,
                 color=INK_2, anchor='middle')
        v = [100 * x for x in d['value_ratio']]
        nv = [100 * x for x in d['nearest_value_ratio']]
        rows.append([f'{n} / {m}', f'{statistics.mean(v):.1f}', f'{min(v):.1f}',
                     sum(x > 99.99 for x in v), f'{statistics.mean(nv):.1f}',
                     f'{min(nv):.1f}',
                     f'{statistics.median(d["makespan_ratio"]):.2f}',
                     f'{statistics.median(d["wait_ratio"]):.2f}'])
    svg.save(os.path.join(FIGS, 'cbba_optimality.svg'), 'CBBA vs optimum',
             'CBBA mean score 98-99% of optimal vs 95-96% for nearest drone.')
    md_table(os.path.join(FIGS, 'cbba_optimality.md'),
             ['drones / people', 'CBBA mean %', 'CBBA worst %', 'CBBA optimal (of 300)',
              'nearest mean %', 'nearest worst %', 'makespan / best (median)',
              'total wait / best (median)'], rows,
             'Area radius 15m, perfect messages, every person known to every drone.')


def fig_loss():
    losses = [0, 0.2, 0.4, 0.6]
    svg = Svg(720, 330)
    svg.text(24, 30, 'Our CBBA keeps finishing missions as messages are lost',
             size=15, color=INK, weight=600)
    svg.text(24, 50, 'Missions where every person was reached, of 80 '
             '(4 people, 6m area, 200s limit)', size=12)
    legend(svg, 24, 78, [('CBBA with our fixes', BLUE),
                         ('Plain CBBA', ORANGE)], kind='line')
    rows = []
    for k, n in enumerate((3, 4)):
        p = Panel(svg, 80 + k * 330, 120, 270, 160, (-0.03, 0.63), (0, 80),
                  [0, 20, 40, 60, 80], xticks=None,
                  xlabel='messages lost', title=f'{n} drones',
                  ylabel='missions finished' if k == 0 else None)
        for t in losses:
            svg.text(p.sx(t), p.y + p.h + 16, f'{t:.0%}', size=11, color=MUTED,
                     anchor='middle')
        for variant, color in (('plain', ORANGE), ('ours', BLUE)):
            pts = []
            for loss in losses:
                times = reached_times(f'loss_{variant}_n{n}_l{loss:g}')
                pts.append((loss, sum(t is not None for t in times)))
            svg.polyline([(p.sx(x), p.sy(y)) for x, y in pts], color)
            for x, y in pts:
                svg.dot(p.sx(x), p.sy(y), color)
            x, y = pts[-1]
            svg.text(p.sx(x) + 10, p.sy(y) + 4, str(y), size=11, color=INK_2)
        for loss in losses:
            o = reached_times(f'loss_ours_n{n}_l{loss:g}')
            pl = reached_times(f'loss_plain_n{n}_l{loss:g}')
            rows.append([n, f'{loss:.0%}', sum(t is not None for t in o),
                         sum(t is not None for t in pl),
                         f'{median_done(o):.1f}', f'{median_done(pl):.1f}'])
    svg.save(os.path.join(FIGS, 'cbba_message_loss.svg'), 'CBBA under message loss',
             'Missions finished vs message loss, our CBBA vs plain CBBA.')
    md_table(os.path.join(FIGS, 'cbba_message_loss.md'),
             ['drones', 'loss', 'ours finished (of 80)', 'plain finished (of 80)',
              'ours median s', 'plain median s'], rows,
             'Plain CBBA = no re-broadcast, no duplicate merging, no outbid margin, '
             'no bid refresh, 0.3m task arrival radius.')


def ecdf_points(times, panel, t_max):
    n = len(times)
    done = sorted(t for t in times if t is not None)
    pts, frac = [(panel.sx(0), panel.sy(0))], 0.0
    for t in done:
        pts.append((panel.sx(t), panel.sy(frac)))
        frac += 100.0 / n
        pts.append((panel.sx(t), panel.sy(frac)))
    pts.append((panel.sx(t_max), panel.sy(frac)))
    return pts, frac


def fig_search():
    # Lines can cross anywhere, so every pair of colors must stay apart: the
    # palette's first three slots are the most that validate all-pairs.
    # Greedy (the coverage map without the swarm motion) is in the table.
    plotted = ['pso', 'lawnmower', 'random']
    order = ['pso', 'lawnmower', 'greedy', 'random']
    svg = Svg(960, 360)
    svg.text(24, 30, 'Search: PSO matches a lawnmower and is 2-3x faster than '
             'random', size=15, color=INK, weight=600)
    svg.text(24, 50, '% of missions with every person reached by time t '
             '(12m-radius area, 4 people, 60 missions)', size=12)
    legend(svg, 24, 78, [(SEARCH_NAMES[s], SEARCH_COLORS[s]) for s in plotted],
           kind='line')
    rows = []
    t_max = 300
    for k, n in enumerate((2, 3, 4)):
        p = Panel(svg, 70 + k * 300, 124, 250, 170, (0, t_max), (0, 100),
                  [0, 25, 50, 75, 100], xticks=[0, 100, 200, 300],
                  xlabel='seconds', title=f'{n} drones',
                  ylabel='% of missions' if k == 0 else None,
                  yfmt=lambda v: f'{v:g}%')
        for s in reversed(plotted):
            times = reached_times(f'search_base_{s}_n{n}')
            pts, _ = ecdf_points([t if t is None or t <= t_max else None
                                  for t in times], p, t_max)
            svg.polyline(pts, SEARCH_COLORS[s])
        for s in order:
            times = reached_times(f'search_base_{s}_n{n}')
            done = sorted(t for t in times if t is not None)
            rows.append([n, SEARCH_NAMES[s], f'{len(done)}/60',
                         f'{statistics.median(done):.1f}',
                         f'{done[int(0.9 * (len(done) - 1))]:.1f}'])
    svg.save(os.path.join(FIGS, 'search_methods.svg'), 'Search methods',
             'Cumulative share of missions finished over time for four search '
             'methods at 2, 3 and 4 drones.')
    md_table(os.path.join(FIGS, 'search_methods.md'),
             ['drones', 'search', 'finished', 'median s', 'p90 s'], rows,
             'Same CBBA, collision floor and speed cap for every method; '
             'only the search differs.')


def fig_scenarios():
    scen = [('base', 'normal'), ('dropout', 'drone leaves'), ('river', 'river'),
            ('both', 'both')]
    svg = Svg(960, 360)
    svg.text(24, 30, 'When a drone leaves or people cluster by a river, PSO '
             'finds everyone sooner', size=15, color=INK, weight=600)
    svg.text(24, 50, 'Median seconds until every person was reached '
             '(12m-radius area, 4 people, 60 missions; lower is better)', size=12)
    legend(svg, 24, 78, [(SEARCH_NAMES['pso'], BLUE),
                         ('Lawnmower (re-plans when a drone leaves)', ORANGE)])
    rows = []
    for k, n in enumerate((2, 3, 4)):
        vals = []
        for key, _ in scen:
            row = []
            for s in ('pso', 'lawnmower'):
                times = reached_times(f'search_{key}_{s}_n{n}')
                row.append(median_done(times))
            vals.append(row)
            pso = reached_times(f'search_{key}_pso_n{n}')
            lm = reached_times(f'search_{key}_lawnmower_n{n}')
            rows.append([n, dict(scen)[key],
                         f'{median_done(pso):.1f}',
                         f'{sum(t is not None for t in pso)}/60',
                         f'{median_done(lm):.1f}',
                         f'{sum(t is not None for t in lm)}/60',
                         f'{100 * (1 - median_done(pso) / median_done(lm)):+.0f}%'])
        top = max(v for r in vals for v in r if v is not None)
        ticks = nice_ticks(0, top, 4)
        p = Panel(svg, 70 + k * 300, 124, 250, 170, (0, ticks[-1]),
                  (0, ticks[-1]), ticks, title=f'{n} drones',
                  ylabel='seconds' if k == 0 else None)
        grouped_bars(p, [label for _, label in scen], ['pso', 'lawnmower'],
                     vals, SEARCH_COLORS)
    svg.save(os.path.join(FIGS, 'search_scenarios.svg'), 'Search scenarios',
             'Median time to reach everyone, PSO vs lawnmower, in four scenarios.')
    md_table(os.path.join(FIGS, 'search_scenarios.md'),
             ['drones', 'scenario', 'PSO median s', 'PSO finished',
              'lawnmower median s', 'lawnmower finished', 'PSO faster by'], rows,
             'Drone leaves: the last drone hits its battery reserve at 15s. River: '
             'a random river per mission, 80% of people placed by its flood risk, '
             'every drone given the risk map. Both: the two together.')


def fig_revisit():
    cells = [('dropout', 2, 'drone leaves'), ('river', 2, 'river'),
             ('both', 2, 'both'), ('both', 3, 'both')]
    svg = Svg(720, 340)
    svg.text(24, 30, 'A longer, adaptive revisit time fixes PSO\'s unfinished '
             'missions', size=15, color=INK, weight=600)
    svg.text(24, 50, 'Missions where every person was reached within 500s, '
             'of 60 (12m-radius area)', size=12)
    old, new = ORANGE, BLUE
    legend(svg, 24, 78, [('Revisit after 60s (before)', old),
                         ('240s, oldest 10% always due (now)', new)])
    p = Panel(svg, 80, 110, 600, 180, (0, 1), (0, 60), [0, 15, 30, 45, 60],
              ylabel='missions finished')
    vals, rows = [], []
    for key, n, label in cells:
        a = reached_times(f'revisit60_{key}_n{n}')
        b = reached_times(f'search_{key}_pso_n{n}')
        fa, fb = sum(t is not None for t in a), sum(t is not None for t in b)
        vals.append([fa, fb])
        rows.append([n, label, fa, fb, f'{median_done(a):.1f}',
                     f'{median_done(b):.1f}'])
    grouped_bars(p, [f'{n} drones, {label}' for _, n, label in cells],
                 ['old', 'new'], vals, {'old': old, 'new': new}, bar_w=24)
    for gi, (fa, fb) in enumerate(vals):
        slot = p.w / len(cells)
        x0 = p.x + gi * slot + (slot - 50) / 2
        svg.text(x0 + 12, p.sy(fa) - 6, str(fa), size=11, anchor='middle')
        svg.text(x0 + 38, p.sy(fb) - 6, str(fb), size=11, anchor='middle')
    svg.save(os.path.join(FIGS, 'search_revisit.svg'), 'Revisit time fix',
             'Missions finished with 60s vs 240s revisit time in the hardest cases.')
    md_table(os.path.join(FIGS, 'search_revisit.md'),
             ['drones', 'scenario', 'finished, 60s', 'finished, 240s',
              'median s, 60s', 'median s, 240s'], rows,
             'Medians are over the missions that finished.')


def main():
    os.makedirs(FIGS, exist_ok=True)
    for fig in (fig_cbba_optimality, fig_loss, fig_search, fig_scenarios,
                fig_revisit):
        try:
            fig()
            print(f'{fig.__name__}: ok')
        except FileNotFoundError as e:
            print(f'{fig.__name__}: skipped, missing {os.path.basename(e.filename)}')


if __name__ == '__main__':
    main()
