#!/usr/bin/env python3
"""Standalone visualization: logs drone positions/states/detected targets over
time and periodically saves a two-panel PNG (spatial trajectory + distance-
to-target over time). No GUI/X11 needed on the server — open the PNG in an
editor connected to this machine (e.g. VS Code over Remote-SSH, or scp'd to a
local copy) and refresh it to watch a run progress.

Pass --gif-out to also get a playable animation of the whole run, rendered
once the process is stopped (SIGINT/SIGTERM) — this is the better way to
actually see how the drones behaved, since the periodic PNG only ever shows
a cumulative "so far" snapshot, not motion.

Run after sourcing ros2_ws/install/setup.bash, alongside the
coordination_node instances you want to visualize:

    python3 tools/visualize_run.py --num-drones 2 --out /tmp/coord_demo.png \
        --gif-out /tmp/coord_demo.gif

Requires matplotlib in the active environment (`pip install matplotlib` if
it's missing) — and Pillow for --gif-out specifically (`pip install pillow`
if `anim.save(...)` errors out; Pillow isn't always pulled in automatically
alongside matplotlib).
"""

import argparse
import signal

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

import rclpy
from rclpy.node import Node

from coordination_msgs.msg import AgentState, TargetDetected

# Validated categorical palette (colorblind-safe pairing) — drone identity
# always keeps its color; TASK_ALLOCATION vs SEARCH is shown via marker
# style, not a second color, so identity and state never compete for hue.
DRONE_COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4']
TARGET_COLOR = '#d03b3b'  # reserved status color, distinct from any drone hue
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
SECONDARY_INK = '#52514e'
GRID = '#e1e0d9'


def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i + 2], 16) / 255.0 for i in (0, 2, 4))


class RunVisualizer(Node):
    def __init__(self, num_drones, out_path, save_every_sec, gif_out=None):
        super().__init__('run_visualizer')
        self.num_drones = num_drones
        self.out_path = out_path
        self.gif_out = gif_out
        # x-axis for the distance panel is sample index, not wall-clock
        # time - an index is guaranteed strictly increasing by construction
        # (it's just each list's own position), so it can't produce an
        # out-of-order/zigzag plot regardless of any timing irregularity.
        self.tracks = {
            i: {'x': [], 'y': [], 'state': []}
            for i in range(num_drones)
        }
        self.targets = []  # (x, y, target_id)

        # Minimum-separation tracking: a static trajectory plot can't show
        # whether two drones were ever close *at the same time* - two dots
        # sitting on top of each other could be many seconds apart. This
        # instead checks, every time any drone reports a new position,
        # its distance to every other drone's most-recently-known position
        # (the same "last known" basis coordination_node.py's own hard-floor
        # safety check uses), and keeps the smallest ever seen. That's a
        # real answer to "did they get too close", not a guess from a plot.
        self.last_position = {}  # drone_id -> (x, y)
        self.min_separation_seen = None
        self.min_separation_info = None  # (drone_a, drone_b, distance)

        for i in range(num_drones):
            self.create_subscription(
                AgentState, f'/drone_{i}/coordination/agent_state',
                lambda msg, i=i: self._on_agent_state(i, msg), 10)
            self.create_subscription(
                TargetDetected, f'/drone_{i}/coordination/target_detected',
                self._on_target_detected, 10)

        self.create_timer(save_every_sec, self._save_plot)
        self.get_logger().info(
            f'run_visualizer up: watching {num_drones} drones, '
            f'saving to {out_path} every {save_every_sec}s')

    def _on_agent_state(self, drone_id, msg):
        t = self.tracks[drone_id]
        t['x'].append(msg.position.x)
        t['y'].append(msg.position.y)
        t['state'].append(msg.state)

        self.last_position[drone_id] = (msg.position.x, msg.position.y)
        for other_id, (ox, oy) in self.last_position.items():
            if other_id == drone_id:
                continue
            distance = ((msg.position.x - ox) ** 2
                        + (msg.position.y - oy) ** 2) ** 0.5
            if self.min_separation_seen is None or distance < self.min_separation_seen:
                self.min_separation_seen = distance
                self.min_separation_info = (drone_id, other_id, distance)

    def _on_target_detected(self, msg):
        self.targets.append((msg.position.x, msg.position.y, msg.target_id))

    def _task_start_index(self, states):
        """Index of the first TASK_ALLOCATION sample, or None if never."""
        for idx, s in enumerate(states):
            if s == 1:
                return idx
        return None

    def _draw(self, ax_map, ax_dist, limit=None):
        """Render one frame of the trajectory + distance panels, using only
        the first `limit` samples of each drone's history (None = all of
        it). Shared by the periodic PNG snapshot (limit=None, i.e. "so far")
        and each GIF frame (limit=that frame's sample count), so the two
        views can never drift apart in how they draw the same data."""
        ax_map.set_facecolor(SURFACE)
        ax_dist.set_facecolor(SURFACE)
        any_data = False
        # Simplification: the target marker/label is shown from frame 1
        # onward even though it isn't actually detected until partway
        # through the run, since it's a fixed point (not something whose
        # movement over time matters) and there is normally only one per
        # demo run. Good enough for this tool; would need a real detection
        # timestamp per target to animate accurately if that changes.
        first_target = self.targets[0] if self.targets else None

        for i in range(self.num_drones):
            t = self.tracks[i]
            n = len(t['x']) if limit is None else min(limit, len(t['x']))
            if n == 0:
                continue
            any_data = True
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            r, g, b = hex_to_rgb(color)
            tx_hist, ty_hist, tstate_hist = t['x'][:n], t['y'][:n], t['state'][:n]

            # Faint connecting line for continuity, plus a time-graded
            # scatter (dim -> full opacity) so direction of travel is
            # visible at a glance instead of relying on two small markers.
            ax_map.plot(tx_hist, ty_hist, '-', color=color, linewidth=1.3,
                        alpha=0.25, zorder=1)

            # Small direction arrows along the path, at a handful of evenly
            # spaced points, so which way the drone was moving is legible at
            # a glance rather than only inferable from the alpha gradient.
            arrow_stride = max(1, n // 6)
            for idx in range(0, n - 1, arrow_stride):
                ax_map.annotate(
                    '', xy=(tx_hist[idx + 1], ty_hist[idx + 1]),
                    xytext=(tx_hist[idx], ty_hist[idx]),
                    arrowprops=dict(arrowstyle='-|>', color=color, alpha=0.6,
                                     shrinkA=0, shrinkB=0, mutation_scale=11,
                                     linewidth=0),
                    zorder=2)

            alphas = np.linspace(0.25, 1.0, n)
            rgba = np.column_stack([
                np.full(n, r), np.full(n, g), np.full(n, b), alphas])
            ax_map.scatter(tx_hist, ty_hist, c=rgba, s=16, zorder=2,
                           label=f'Drone {i}')

            # Mark the exact moment this drone entered TASK_ALLOCATION.
            task_idx = self._task_start_index(tstate_hist)
            if task_idx is not None:
                ax_map.scatter(
                    [tx_hist[task_idx]], [ty_hist[task_idx]], marker='D',
                    s=90, facecolor=color, edgecolor=INK, linewidth=1.4,
                    zorder=5)
                ax_map.annotate(
                    'task won', (tx_hist[task_idx], ty_hist[task_idx]),
                    textcoords='offset points', xytext=(8, -12),
                    fontsize=8, color=SECONDARY_INK)

            # Start (X) and end/current (triangle) markers — direction is
            # also carried by the alpha gradient, but these give an
            # unambiguous, named anchor at each end of the path.
            ax_map.scatter([tx_hist[0]], [ty_hist[0]], marker='x', s=90,
                           color=color, linewidth=2.2, zorder=4)
            ax_map.scatter([tx_hist[-1]], [ty_hist[-1]], marker='^', s=110,
                           facecolor=color, edgecolor=INK, linewidth=1.2,
                           zorder=4)

            # Distance-to-target-over-sample panel (x-axis is sample index,
            # not wall-clock time - see note above on why).
            if first_target is not None:
                tx, ty, tid = first_target
                dist = [((x - tx) ** 2 + (y - ty) ** 2) ** 0.5
                        for x, y in zip(tx_hist, ty_hist)]
                sample_idx = list(range(len(dist)))
                ax_dist.plot(sample_idx, dist, '-', color=color, linewidth=2,
                             alpha=0.9, label=f'Drone {i}')
                if task_idx is not None:
                    ax_dist.scatter([sample_idx[task_idx]], [dist[task_idx]],
                                    marker='D', s=70, facecolor=color,
                                    edgecolor=INK, linewidth=1.2, zorder=5)

        if first_target is not None:
            tx, ty, tid = first_target
            any_data = True
            ax_map.scatter([tx], [ty], marker='*', s=260,
                           facecolor=TARGET_COLOR, edgecolor=INK,
                           linewidth=1, zorder=6)
            ax_map.annotate(f'target {tid}', (tx, ty),
                            textcoords='offset points', xytext=(8, 8),
                            fontsize=9, color=SECONDARY_INK)

        ax_map.set_title('Trajectory (faint = earlier, solid = later)',
                         color=INK, fontsize=12, fontweight='bold')
        ax_map.set_xlabel('x (m)', color=SECONDARY_INK)
        ax_map.set_ylabel('y (m)', color=SECONDARY_INK)
        ax_map.grid(True, color=GRID, linewidth=0.8)
        ax_map.tick_params(colors=SECONDARY_INK)
        for spine in ax_map.spines.values():
            spine.set_color(GRID)
        if any_data:
            ax_map.legend(frameon=False, labelcolor=INK, loc='best')
        ax_map.set_aspect('equal', adjustable='datalim')

        ax_dist.set_title('Distance to detected target over time',
                          color=INK, fontsize=12, fontweight='bold')
        ax_dist.set_xlabel('sample # (each ~0.5s apart)', color=SECONDARY_INK)
        ax_dist.set_ylabel('distance (m)', color=SECONDARY_INK)
        ax_dist.grid(True, color=GRID, linewidth=0.8)
        ax_dist.tick_params(colors=SECONDARY_INK)
        for spine in ax_dist.spines.values():
            spine.set_color(GRID)
        if first_target is None:
            ax_dist.text(0.5, 0.5, 'no target detected yet',
                        transform=ax_dist.transAxes, ha='center',
                        va='center', color=SECONDARY_INK, fontsize=10)
        else:
            ax_dist.axhline(0, color=GRID, linewidth=1)

    def _finish_figure(self, fig):
        """Figure-level chrome (title, marker legend, min-separation
        readout) shared by both the periodic PNG and every GIF frame."""
        fig.suptitle('Multi-drone coordination — live test run', color=INK,
                     fontsize=14, fontweight='bold', y=0.99)
        fig.text(0.5, 0.935,
                 '✕ start   ▲ end/current   ◆ task won   ★ target',
                 ha='center', fontsize=9.5, color=SECONDARY_INK)
        if self.min_separation_info is not None:
            a, b, d = self.min_separation_info
            sep_color = TARGET_COLOR if d < 1.5 else SECONDARY_INK
            fig.text(0.5, 0.905,
                     f'closest drones ever got: {d:.2f} m '
                     f'(drone {a} / drone {b}, min-separation floor: 1.5 m)',
                     ha='center', fontsize=9, color=sep_color)
        # Reserve the top ~15% of the figure for the text elements above
        # (title, marker legend, min-separation readout), so they never
        # collide with the per-subplot titles.
        fig.tight_layout(rect=[0, 0, 1, 0.85])

    def _save_plot(self):
        fig, (ax_map, ax_dist) = plt.subplots(
            1, 2, figsize=(13, 6), facecolor=SURFACE,
            gridspec_kw={'width_ratios': [1.4, 1]})
        self._draw(ax_map, ax_dist, limit=None)
        self._finish_figure(fig)
        fig.savefig(self.out_path, dpi=150)
        plt.close(fig)
        self.get_logger().info(f'saved {self.out_path}')

    def save_gif(self):
        """Render the whole recorded run as a playable animation, so you can
        watch how the drones actually moved instead of reading a single
        cumulative snapshot. Called once, when the process is stopping
        (see main()'s signal handling) - not on the periodic timer, since
        re-rendering ~40 frames every couple seconds would be wasteful and
        isn't needed until the run is over anyway."""
        if not self.gif_out:
            return
        max_len = max((len(t['x']) for t in self.tracks.values()), default=0)
        if max_len == 0:
            self.get_logger().warning('save_gif: no data recorded, skipping')
            return

        frame_count = min(max_len, 40)
        frame_limits = sorted(set(
            int(round(x)) for x in np.linspace(1, max_len, frame_count)))
        # Hold on the final frame for ~1.5s instead of cutting straight to
        # the loop point, so the end state is actually readable.
        frame_limits += [max_len] * 6

        fig, (ax_map, ax_dist) = plt.subplots(
            1, 2, figsize=(13, 6), facecolor=SURFACE,
            gridspec_kw={'width_ratios': [1.4, 1]})

        def update(limit):
            ax_map.clear()
            ax_dist.clear()
            for txt in list(fig.texts):
                txt.remove()
            self._draw(ax_map, ax_dist, limit=limit)
            self._finish_figure(fig)

        anim = FuncAnimation(fig, update, frames=frame_limits)
        anim.save(self.gif_out, writer=PillowWriter(fps=5))
        plt.close(fig)
        self.get_logger().info(
            f'saved {self.gif_out} ({len(frame_limits)} frames)')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--num-drones', type=int, default=2)
    parser.add_argument('--out', type=str, default='/tmp/coord_demo.png')
    parser.add_argument('--save-every-sec', type=float, default=2.0)
    parser.add_argument(
        '--gif-out', type=str, default=None,
        help='If set, save an animated GIF of the whole run to this path '
             'once the process is stopped (SIGINT/SIGTERM), so you can '
             'watch the run play back instead of reading one static plot.')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = RunVisualizer(
        args.num_drones, args.out, args.save_every_sec, args.gif_out)

    # demo_run.sh stops this process with `kill`/`pkill` (SIGTERM), not
    # Ctrl-C - without catching it, save_gif() below would never run and
    # the whole point of watching the run afterward would be lost. Calling
    # rclpy.shutdown() here makes rclpy.ok() go false, which is what
    # actually ends spin()'s loop (cleanly, no exception), same mechanism
    # ROS2's own examples use for graceful shutdown on a signal.
    def _handle_stop_signal(signum, frame):
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _handle_stop_signal)
    signal.signal(signal.SIGINT, _handle_stop_signal)

    try:
        rclpy.spin(node)
    finally:
        node._save_plot()
        node.save_gif()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
