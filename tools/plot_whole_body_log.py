#!/usr/bin/env python3
"""Plot ramp/track diagnostics from rosbag2 CSV exports."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def load_csv(path, key_fields):
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser(description='Plot whole-body diagnostics from rosbag CSVs')
    parser.add_argument('--state', type=Path, required=True, help='CSV exported from /whole_body/state')
    parser.add_argument('--cmd', type=Path, required=True, help='CSV exported from /whole_body/cmd_vel')
    parser.add_argument('--arm-plan', type=Path, help='CSV exported from /whole_body/arm_plan')
    parser.add_argument('--base-plan', type=Path, help='CSV exported from /whole_body/base_plan')
    parser.add_argument('--obstacle-grid', type=Path, help='CSV exported from /whole_body/obstacle_grid')
    parser.add_argument('--output', type=Path, help='Save figure to this path')
    parser.add_argument('--save', action='store_true', help='Save plot to analysis/results by default')
    args = parser.parse_args()

    state_rows = load_csv(args.state, ['header.stamp.sec', 'mode', 'mode_label'])
    cmd_rows = load_csv(args.cmd, ['header.stamp.sec', 'twist.linear.x', 'twist.angular.z'])

    if not state_rows or not cmd_rows:
        print('Empty CSV inputs; aborting')
        return

    state_times = [float(r['header.stamp.sec']) for r in state_rows]
    state_modes = [r['mode_label'] for r in state_rows]

    cmd_times = [float(r['header.stamp.sec']) for r in cmd_rows]
    cmd_linear = [float(r['twist.linear.x']) for r in cmd_rows]
    cmd_yaw = [float(r['twist.angular.z']) for r in cmd_rows]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

    axes[0].plot(state_times, state_modes, drawstyle='steps-post')
    axes[0].set_ylabel('Stage')
    axes[0].set_title('Whole-body Stage State')

    axes[1].plot(cmd_times, cmd_linear, label='v [m/s]')
    axes[1].plot(cmd_times, cmd_yaw, label='omega [rad/s]')
    axes[1].set_ylabel('Base command')
    axes[1].legend()

    if args.arm_plan:
        arm_rows = load_csv(args.arm_plan, ['header.stamp.sec'])
        arm_times = [float(r['header.stamp.sec']) for r in arm_rows]
        axes[2].plot(arm_times, range(len(arm_times)), label='Arm plan samples')

    if args.base_plan:
        base_rows = load_csv(args.base_plan, ['header.stamp.sec'])
        base_times = [float(r['header.stamp.sec']) for r in base_rows]
        axes[2].plot(base_times, range(len(base_times)), label='Base plan samples')

    axes[2].set_ylabel('Plan samples')
    axes[2].set_xlabel('Time [s]')
    axes[2].legend()

    fig2, ax2 = plt.subplots(figsize=(6, 6))
    ax2.set_title('Base Path with Obstacles')
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_aspect('equal', adjustable='box')

    if args.base_plan and args.base_plan.exists():
        base_rows_plot = load_csv(args.base_plan, ['pose.position.x', 'pose.position.y'])
        xs_plot = [float(r['pose.position.x']) for r in base_rows_plot]
        ys_plot = [float(r['pose.position.y']) for r in base_rows_plot]
        ax2.plot(xs_plot, ys_plot, label='Base path')

    if args.obstacle_grid and args.obstacle_grid.exists():
        grid_rows = load_csv(args.obstacle_grid, ['info.width', 'info.height', 'info.resolution', 'info.origin.position.x', 'info.origin.position.y', 'data'])
        if grid_rows:
            width = int(grid_rows[0]['info.width'])
            height = int(grid_rows[0]['info.height'])
            res = float(grid_rows[0]['info.resolution'])
            origin_x = float(grid_rows[0]['info.origin.position.x'])
            origin_y = float(grid_rows[0]['info.origin.position.y'])
            data = np.array([int(r['data']) for r in grid_rows]).reshape(height, width)
            xs = np.repeat(np.arange(width) * res + origin_x, height)
            ys = np.tile(np.arange(height) * res + origin_y, width)
            occupied = data.flatten() >= 50
            ax2.scatter(xs[occupied], ys[occupied], c='red', s=5, alpha=0.5, label='Obstacles')

    ax2.legend()

    plt.tight_layout()

    output_path = args.output
    if not output_path and args.save:
        output_dir = Path('analysis/results')
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / 'whole_body_summary.png'

    if output_path:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path)
        fig2.savefig(output_path.with_name(output_path.stem + '_xy' + output_path.suffix))
        print(f'Saved plots to {output_path} and companion figure')
    else:
        plt.show()


if __name__ == '__main__':
    main()
