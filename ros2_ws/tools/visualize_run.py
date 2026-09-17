#!/usr/bin/env python3
"""Standalone visualization: logs drone positions/states/detected targets over
time and periodically saves a two-panel PNG (spatial trajectory + distance-
to-target over time). No GUI/X11 needed on the server — open the PNG in an
editor connected to this machine (e.g. VS Code over Remote-SSH, or scp'd to a
local copy) and refresh it to watch a run progress.

Run after sourcing ros2_ws/install/setup.bash, alongside the
coordination_node instances you want to visualize:

    python3 tools/visualize_run.py --num-drones 2 --out /tmp/coord_demo.png

Requires matplotlib in the active environment (`pip install matplotlib` if
it's missing).
"""

import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
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
    def __init__(self, num_drones, out_path, save_every_sec):
        super().__init__('run_visualizer')
        self.num_drones = num_drones
        self.out_path = out_path
        # x-axis for the distance panel is sample index, not wall-clock
        # time - an index is guaranteed strictly increasing by construction
        # (it's just each list's own position), so it can't produce an
        # out-of-order/zigzag plot regardless of any timing irregularity.
        self.tracks = {
            i: {'x': [], 'y': [], 'state': []}
            for i in range(num_drones)
        }
        self.targets = []  # (x, y, target_id)

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

    def _on_target_detected(self, msg):
        self.targets.append((msg.position.x, msg.position.y, msg.target_id))

    def _task_start_index(self, states):
        """Index of the first TASK_ALLOCATION sample, or None if never."""
        for idx, s in enumerate(states):
            if s == 1:
                return idx
        return None

    def _save_plot(self):
        fig, (ax_map, ax_dist) = plt.subplots(
            1, 2, figsize=(13, 6), facecolor=SURFACE,
            gridspec_kw={'width_ratios': [1.4, 1]})
        ax_map.set_facecolor(SURFACE)
        ax_dist.set_facecolor(SURFACE)
        any_data = False
        first_target = self.targets[0] if self.targets else None

        for i in range(self.num_drones):
            t = self.tracks[i]
            if not t['x']:
                continue
            any_data = True
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            r, g, b = hex_to_rgb(color)
            n = len(t['x'])

            # Faint connecting line for continuity, plus a time-graded
            # scatter (dim -> full opacity) so direction of travel is
            # visible at a glance instead of relying on two small markers.
            ax_map.plot(t['x'], t['y'], '-', color=color, linewidth=1.3,
                        alpha=0.25, zorder=1)
            alphas = np.linspace(0.25, 1.0, n)
            rgba = np.column_stack([
                np.full(n, r), np.full(n, g), np.full(n, b), alphas])
            ax_map.scatter(t['x'], t['y'], c=rgba, s=16, zorder=2,
                           label=f'Drone {i}')

            # Mark the exact moment this drone entered TASK_ALLOCATION.
            task_idx = self._task_start_index(t['state'])
            if task_idx is not None:
                ax_map.scatter(
                    [t['x'][task_idx]], [t['y'][task_idx]], marker='D',
                    s=90, facecolor=color, edgecolor=INK, linewidth=1.4,
                    zorder=5)
                ax_map.annotate(
                    'task won', (t['x'][task_idx], t['y'][task_idx]),
                    textcoords='offset points', xytext=(8, -12),
                    fontsize=8, color=SECONDARY_INK)

            # Start (X) and end/current (triangle) markers — direction is
            # also carried by the alpha gradient, but these give an
            # unambiguous, named anchor at each end of the path.
            ax_map.scatter([t['x'][0]], [t['y'][0]], marker='x', s=90,
                           color=color, linewidth=2.2, zorder=4)
            ax_map.scatter([t['x'][-1]], [t['y'][-1]], marker='^', s=110,
                           facecolor=color, edgecolor=INK, linewidth=1.2,
                           zorder=4)

            # Distance-to-target-over-sample panel (x-axis is sample index,
            # not wall-clock time - see note above on why).
            if first_target is not None:
                tx, ty, tid = first_target
                dist = [((x - tx) ** 2 + (y - ty) ** 2) ** 0.5
                        for x, y in zip(t['x'], t['y'])]
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

        fig.suptitle('Multi-drone coordination — live test run', color=INK,
                     fontsize=14, fontweight='bold')
        fig.text(0.5, 0.965,
                 '✕ start   ▲ end/current   ◆ task won   ★ target',
                 ha='center', fontsize=9.5, color=SECONDARY_INK)
        fig.tight_layout()
        fig.savefig(self.out_path, dpi=150)
        plt.close(fig)
        self.get_logger().info(f'saved {self.out_path}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--num-drones', type=int, default=2)
    parser.add_argument('--out', type=str, default='/tmp/coord_demo.png')
    parser.add_argument('--save-every-sec', type=float, default=2.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = RunVisualizer(args.num_drones, args.out, args.save_every_sec)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
