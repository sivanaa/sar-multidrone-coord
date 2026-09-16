#!/usr/bin/env python3
"""Standalone visualization: logs drone positions/states/detected targets over
time and periodically saves a trajectory plot as a PNG. No GUI/X11 needed on
the server — open the PNG in an editor connected to this machine (e.g. VS
Code over Remote-SSH) and refresh it to watch a run progress.

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


class RunVisualizer(Node):
    def __init__(self, num_drones, out_path, save_every_sec):
        super().__init__('run_visualizer')
        self.num_drones = num_drones
        self.out_path = out_path
        self.tracks = {i: {'x': [], 'y': [], 'state': []} for i in range(num_drones)}
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

    def _save_plot(self):
        fig, ax = plt.subplots(figsize=(8, 6), facecolor=SURFACE)
        ax.set_facecolor(SURFACE)
        any_data = False

        for i in range(self.num_drones):
            t = self.tracks[i]
            if not t['x']:
                continue
            any_data = True
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            ax.plot(t['x'], t['y'], '-', color=color, linewidth=2,
                     alpha=0.85, label=f'Drone {i}', zorder=2)

            # TASK_ALLOCATION points get a filled marker on the same line —
            # state is shown by marker presence/size, not a new color.
            task_x = [x for x, s in zip(t['x'], t['state']) if s == 1]
            task_y = [y for y, s in zip(t['y'], t['state']) if s == 1]
            if task_x:
                ax.scatter(task_x, task_y, s=40, facecolor=color,
                           edgecolor=SURFACE, linewidth=1, zorder=3)

            # Start (hollow) and current (filled, dark ring) position.
            ax.scatter([t['x'][0]], [t['y'][0]], s=70, facecolor=SURFACE,
                       edgecolor=color, linewidth=2, zorder=4)
            ax.scatter([t['x'][-1]], [t['y'][-1]], s=90, facecolor=color,
                       edgecolor=INK, linewidth=1.2, zorder=5)

        for x, y, tid in self.targets:
            any_data = True
            ax.scatter([x], [y], marker='*', s=260, facecolor=TARGET_COLOR,
                       edgecolor=INK, linewidth=1, zorder=6)
            ax.annotate(f'target {tid}', (x, y), textcoords='offset points',
                        xytext=(8, 8), fontsize=9, color=SECONDARY_INK)

        ax.set_title('Multi-drone coordination — live test run',
                      color=INK, fontsize=13, fontweight='bold')
        ax.set_xlabel('x (m)', color=SECONDARY_INK)
        ax.set_ylabel('y (m)', color=SECONDARY_INK)
        ax.grid(True, color=GRID, linewidth=0.8)
        ax.tick_params(colors=SECONDARY_INK)
        for spine in ax.spines.values():
            spine.set_color(GRID)
        if any_data:
            ax.legend(frameon=False, labelcolor=INK, loc='best')
        ax.set_aspect('equal', adjustable='datalim')

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
