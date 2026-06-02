import subprocess
import argparse
import signal
import time
import json
import math
import sys
from pathlib import Path

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

MONITOR_SCRIPT = Path(__file__).parent / 'monitor_resources.py'


def parse_args():
    p = argparse.ArgumentParser(description='SLAM Benchmark Batch Runner')
    p.add_argument('--algorithm', default='slam_toolbox', choices=['slam_toolbox', 'amcl'])
    p.add_argument('--noise', type=int, default=0, choices=[0, 5, 10, 20])
    p.add_argument('--world', default='turtlebot3_world')
    p.add_argument('--runs', type=int, default=10)
    p.add_argument('--laps', type=int, default=3)
    p.add_argument('--duration', type=int, default=300)
    p.add_argument('--seed-offset', type=int, default=0)
    p.add_argument('--output-dir', default='/tmp/slam_benchmark_results')
    p.add_argument('--map-file', default='')
    p.add_argument('--cooldown', type=int, default=8)
    p.add_argument('--start-from', type=int, default=1)
    return p.parse_args()


def make_run_id(noise_pct, run_num):
    return f'{noise_pct}pct_{run_num:03d}'


def parse_resource_summary(text):
    """Extract mean/max/min CPU and RAM from a resource_usage.txt [SUMMARY] block."""
    metrics = {}
    in_summary = False
    for line in text.split('\n'):
        line = line.strip()
        if line == '[SUMMARY]':
            in_summary = True
            continue
        if in_summary and ':' in line and not line.startswith('#'):
            key, _, val = line.partition(':')
            try:
                metrics[key.strip()] = float(val.strip())
            except ValueError:
                pass
    return metrics


def parse_resource_timeseries(text):
    """Parse time-series rows from resource_usage.txt → (times, cpus, rams)."""
    times, cpus, rams = [], [], []
    for line in text.split('\n'):
        line = line.strip()
        if not line or line.startswith('#') or line.startswith('['):
            continue
        parts = line.split()
        if len(parts) == 3:
            try:
                times.append(float(parts[0]))
                cpus.append(float(parts[1]))
                rams.append(float(parts[2]))
            except ValueError:
                pass
    return times, cpus, rams


def plot_resource_single(resource_file, output_pdf):
    """Save a 2-subplot CPU%/RAM-over-time PDF for one run."""
    if not HAS_MATPLOTLIB or not resource_file.exists():
        return
    times, cpus, rams = parse_resource_timeseries(resource_file.read_text())
    if not times:
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    ax1.plot(times, cpus, color='tab:blue', linewidth=1.0)
    ax1.set_ylabel('CPU (%)')
    ax1.set_ylim(bottom=0)
    ax1.grid(True, alpha=0.3)

    ax2.plot(times, rams, color='tab:orange', linewidth=1.0)
    ax2.set_ylabel('RAM (MB)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylim(bottom=0)
    ax2.grid(True, alpha=0.3)

    fig.suptitle(f'Resource Usage — {resource_file.parent.name}')
    fig.tight_layout()
    fig.savefig(str(output_pdf))
    plt.close(fig)
    print(f'  → Resource plot: {output_pdf.name}')


def plot_resource_combined(resource_files_and_ids, output_pdf):
    """Save a combined CPU%/RAM plot overlaying all runs."""
    if not HAS_MATPLOTLIB:
        return

    colors = plt.cm.tab10.colors
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    plotted = 0

    for i, (res_file, run_id) in enumerate(resource_files_and_ids):
        if not res_file.exists():
            continue
        times, cpus, rams = parse_resource_timeseries(res_file.read_text())
        if not times:
            continue
        color = colors[i % len(colors)]
        ax1.plot(times, cpus, color=color, alpha=0.7, linewidth=0.9, label=run_id)
        ax2.plot(times, rams, color=color, alpha=0.7, linewidth=0.9, label=run_id)
        plotted += 1

    if not plotted:
        plt.close(fig)
        return

    ax1.set_ylabel('CPU (%)')
    ax1.set_ylim(bottom=0)
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=7, ncol=5)

    ax2.set_ylabel('RAM (MB)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylim(bottom=0)
    ax2.grid(True, alpha=0.3)

    fig.suptitle(f'Resource Usage — All Runs Combined ({plotted} runs)')
    fig.tight_layout()
    fig.savefig(str(output_pdf))
    plt.close(fig)
    print(f'  → Combined resource plot: {output_pdf}')


def parse_evo_output(text):
    metrics = {}
    for line in text.split('\n'):
        line = line.strip()
        for key in ['rmse', 'mean', 'median', 'std', 'min', 'max', 'sse']:
            if line.startswith(key):
                try:
                    metrics[key] = float(line.split()[-1])
                except ValueError:
                    pass
    return metrics


def kill_simulation():
    print('  → Killing simulation processes...')
    targets = [
        'gz sim', 'gzserver', 'gzclient',
        'async_slam_toolbox_node',
        'nav2_amcl', 'nav2_map_server',
        'trajectory_recorder', 'waypoint_navigator',
        'gt_publisher', 'tf_noise_injector',
        'robot_state_publisher', 'parameter_bridge', 'image_bridge',
        'gz-transport-topic',
    ]
    for target in targets:
        subprocess.run(['pkill', '-9', '-f', target], capture_output=True)
    subprocess.run(['pkill', '-9', 'gz'], capture_output=True)
    time.sleep(3)
    print('  → Done cleaning up')


def run_single(args, run_num):
    run_id = make_run_id(args.noise, run_num)
    seed = run_num + args.seed_offset

    print(f'\n{"="*60}')
    print(f'  RUN {run_num}/{args.runs}  →  {run_id}')
    print(f'  Algorithm: {args.algorithm}  |  Noise: {args.noise}%')
    print(f'  World: {args.world}  |  Laps: {args.laps}  |  Seed: {seed}')
    print(f'{"="*60}')

    cmd = [
        'ros2', 'launch', 'slam_benchmark', 'benchmark.launch.py',
        f'algorithm:={args.algorithm}',
        f'noise_pct:={args.noise}',
        f'world:={args.world}',
        f'run_id:={run_id}',
        f'laps:={args.laps}',
        f'duration:={args.duration}',
        f'seed:={seed}',
        f'output_dir:={args.output_dir}',
        'use_rviz:=false',
    ]
    if args.map_file and args.algorithm == 'amcl':
        cmd.append(f'map_file:={args.map_file}')

    result_dir = Path(args.output_dir) / args.world / args.algorithm / \
                 f'noise_{args.noise}pct' / run_id
    result_dir.mkdir(parents=True, exist_ok=True)

    (result_dir / 'README.txt').write_text(
        f'run_id:       {run_id}\n'
        f'seed:         {seed}\n'
        f'algorithm:    {args.algorithm}\n'
        f'noise_pct:    {args.noise}\n'
        f'world:        {args.world}\n'
        f'laps:         {args.laps}\n'
        f'duration_max: {args.duration}s\n'
        f'seed_offset:  {args.seed_offset}\n'
    )

    resource_file = result_dir / 'resource_usage.txt'
    monitor_proc = subprocess.Popen(
        [sys.executable, str(MONITOR_SCRIPT), str(resource_file),
         '--algorithm', args.algorithm],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    print(f'  → Resource monitor started (pid {monitor_proc.pid}, target={args.algorithm})')

    def stop_monitor():
        if monitor_proc.poll() is not None:
            return
        try:
            monitor_proc.send_signal(signal.SIGTERM)
            monitor_proc.wait(timeout=5)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            monitor_proc.kill()

    try:
        timeout = args.duration + 90
        subprocess.run(cmd, timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f'  ⚠ Timeout — killing run')
        kill_simulation()
    except KeyboardInterrupt:
        print('\n  ✗ Interrupted')
        stop_monitor()
        kill_simulation()
        sys.exit(0)
    finally:
        stop_monitor()

    time.sleep(2)

    plot_resource_single(resource_file, result_dir / 'resource_plot.pdf')

    ate_file = result_dir / 'ate_results.txt'
    if ate_file.exists() and ate_file.stat().st_size > 0:
        print(f'  ✓ Results saved: {result_dir}')
        _print_resource_summary(resource_file)
        return str(result_dir), True
    else:
        print(f'  ✗ No results in {result_dir}')
        return str(result_dir), False


def _print_resource_summary(resource_file):
    if not resource_file.exists():
        return
    m = parse_resource_summary(resource_file.read_text())
    if not m:
        return
    print(f'  CPU: mean={m.get("cpu_mean_pct",0):.1f}%  '
          f'max={m.get("cpu_max_pct",0):.1f}%  '
          f'RAM: mean={m.get("ram_mean_mb",0):.0f} MB  '
          f'max={m.get("ram_max_mb",0):.0f} MB')


def compute_and_print_average(args, result_dirs):
    print(f'\n{"#"*60}')
    print(f'  COMPUTING AVERAGE OVER {len(result_dirs)} RUNS')
    print(f'{"#"*60}\n')

    all_ate = []
    all_rpe = []
    valid_runs = []
    all_resources = []
    resource_series = []

    for rd in result_dirs:
        d = Path(rd)
        ate_file = d / 'ate_results.txt'
        rpe_file = d / 'rpe_results.txt'
        res_file = d / 'resource_usage.txt'

        if ate_file.exists():
            m = parse_evo_output(ate_file.read_text())
            if 'rmse' in m:
                all_ate.append(m)
                valid_runs.append(d.name)
                print(f'  {d.name}: ATE RMSE={m["rmse"]:.6f}m')

        if rpe_file.exists():
            m = parse_evo_output(rpe_file.read_text())
            if 'rmse' in m:
                all_rpe.append(m)

        if res_file.exists():
            rm = parse_resource_summary(res_file.read_text())
            if rm and 'n_samples' in rm and rm['n_samples'] > 0:
                all_resources.append(rm)
            resource_series.append((res_file, d.name))

    if not all_ate:
        print('  ✗ No valid runs to average!')
        return None

    def avg(runs_data):
        keys = set(k for r in runs_data for k in r.keys())
        result = {}
        for key in keys:
            vals = [r[key] for r in runs_data if key in r]
            if vals:
                mean = sum(vals) / len(vals)
                std = math.sqrt(sum((v-mean)**2 for v in vals) / (len(vals) - 1)) if len(vals) > 1 else 0.0
                result[key] = {
                    'mean': mean, 'std': std,
                    'min': min(vals), 'max': max(vals),
                    'n': len(vals), 'values': vals
                }
        return result

    ate_avg = avg(all_ate)
    rpe_avg = avg(all_rpe)

    res_avg = {}
    if all_resources:
        for metric in ['cpu_mean_pct', 'cpu_max_pct', 'cpu_min_pct',
                       'ram_mean_mb', 'ram_max_mb', 'ram_min_mb']:
            vals = [r[metric] for r in all_resources if metric in r]
            if vals:
                res_avg[metric] = {
                    'mean': sum(vals) / len(vals),
                    'max': max(vals),
                    'min': min(vals),
                    'n': len(vals),
                    'values': vals,
                }

    noise_dir = Path(args.output_dir) / args.world / args.algorithm / \
                f'noise_{args.noise}pct'
    batch_summary_dir = noise_dir / 'batch_summary'
    batch_summary_dir.mkdir(exist_ok=True)

    lines = [
        'SLAM BENCHMARK - AVERAGE SUMMARY',
        f'Algorithm: {args.algorithm}',
        f'Noise:     {args.noise}%',
        f'World:     {args.world}',
        f'Runs:      {len(valid_runs)} / {args.runs}',
        f'Run IDs:   {", ".join(valid_runs)}',
        '',
        'ATE RMSE per run:',
    ]
    for rd in result_dirs:
        d = Path(rd)
        ate_file = d / 'ate_results.txt'
        if ate_file.exists():
            m = parse_evo_output(ate_file.read_text())
            if 'rmse' in m:
                lines.append(f'  {d.name}: {m["rmse"]:.6f} m')

    lines += ['', 'ATE (mean ± std over N runs):', '-'*55]
    for key in ['rmse', 'mean', 'median', 'std', 'min', 'max']:
        if key in ate_avg:
            m = ate_avg[key]
            lines.append(f'  {key:<8} {m["mean"]:.6f} ± {m["std"]:.6f}  '
                        f'[{m["min"]:.6f}, {m["max"]:.6f}]')

    lines += ['', 'RPE (mean ± std over N runs):', '-'*55]
    for key in ['rmse', 'mean', 'median', 'std', 'min', 'max']:
        if key in rpe_avg:
            m = rpe_avg[key]
            lines.append(f'  {key:<8} {m["mean"]:.6f} ± {m["std"]:.6f}  '
                        f'[{m["min"]:.6f}, {m["max"]:.6f}]')

    if res_avg:
        lines += ['', f'SLAM Resource Usage ({len(all_resources)} runs):', '-'*55]
        for label, key in [('CPU mean %', 'cpu_mean_pct'),
                            ('CPU max %',  'cpu_max_pct'),
                            ('RAM mean MB', 'ram_mean_mb'),
                            ('RAM max MB',  'ram_max_mb')]:
            if key in res_avg:
                m = res_avg[key]
                lines.append(f'  {label:<12}  mean={m["mean"]:.1f}  '
                             f'max={m["max"]:.1f}  min={m["min"]:.1f}')

    summary_text = '\n'.join(lines)
    (batch_summary_dir / 'average_summary.txt').write_text(summary_text)
    (batch_summary_dir / 'average_summary.json').write_text(
        json.dumps({
            'algorithm': args.algorithm, 'noise_pct': args.noise,
            'world': args.world, 'n_runs': len(valid_runs),
            'run_ids': valid_runs, 'ate': ate_avg, 'rpe': rpe_avg,
            'resources': res_avg,
        }, indent=2)
    )

    for rd in result_dirs:
        d = Path(rd)
        gt = d / 'ground_truth.txt'
        est = d / 'estimated.txt'
        if gt.exists() and est.exists():
            plot = batch_summary_dir / f'{d.name}_ate.pdf'
            subprocess.run(
                ['evo_ape', 'tum', str(gt), str(est),
                 '--align', '--no_warnings',
                 '--save_plot', str(plot), '--plot_mode', 'xy'],
                capture_output=True, timeout=30
            )

    plot_resource_combined(resource_series, batch_summary_dir / 'resource_combined.pdf')

    r = ate_avg.get('rmse', {})
    rr = rpe_avg.get('rmse', {})
    cpu_m = res_avg.get('cpu_mean_pct', {})
    cpu_x = res_avg.get('cpu_max_pct', {})
    ram_m = res_avg.get('ram_mean_mb', {})
    ram_x = res_avg.get('ram_max_mb', {})
    print(f"""
╔══════════════════════════════════════════════════════════════╗
║              AVERAGE OVER {len(valid_runs)} RUNS{' '*(31-len(str(len(valid_runs))))}║
╠══════════════════════════════════════════════════════════════╣
║  Algorithm:  {args.algorithm:<46}║
║  Noise:      {args.noise}%{' '*(45-len(str(args.noise)))}║
║  World:      {args.world:<46}║
╠══════════════════════════════════════════════════════════════╣
║  ATE RMSE per run:                                          ║""")
    for i, m in enumerate(all_ate):
        rval = m.get('rmse', 0)
        label = f'Run {i+1}:'
        val_str = f'{rval:.6f} m'
        print(f'║    {label:<8} {val_str:<47}║')
    ate_rmse_str  = f'{r.get("mean", 0):.6f} ± {r.get("std", 0):.6f} m'
    ate_range_str = f'[{r.get("min", 0):.6f}, {r.get("max", 0):.6f}] m'
    rpe_rmse_str  = f'{rr.get("mean", 0):.6f} ± {rr.get("std", 0):.6f} m'
    print('╠══════════════════════════════════════════════════════════════╣')
    print(f'║  ATE RMSE:  {ate_rmse_str:<47}║')
    print(f'║  ATE Range: {ate_range_str:<47}║')
    print('╠══════════════════════════════════════════════════════════════╣')
    print(f'║  RPE RMSE:  {rpe_rmse_str:<47}║')
    if res_avg:
        cpu_mean_str = f'{cpu_m.get("mean",0):.1f}%'
        cpu_max_str  = f'{cpu_x.get("max",0):.1f}%'
        ram_mean_str = f'{ram_m.get("mean",0):.0f} MB'
        ram_max_str  = f'{ram_x.get("max",0):.0f} MB'
        cpu_line = f'mean={cpu_mean_str}  max={cpu_max_str}'
        ram_line = f'mean={ram_mean_str}  max={ram_max_str}'
        print(f'╠══════════════════════════════════════════════════════════════╣')
        print(f'║  CPU usage: {cpu_line:<47}║')
        print(f'║  RAM usage: {ram_line:<47}║')
    print(f'╠══════════════════════════════════════════════════════════════╣')
    print(f'║  Summary: {str(batch_summary_dir / "average_summary.txt"):<49}║')
    print(f'╚══════════════════════════════════════════════════════════════╝')

    return ate_avg


def main():
    args = parse_args()

    print(f"""
╔══════════════════════════════════════════════════════════════╗
║              SLAM BENCHMARK BATCH RUNNER                    ║
╠══════════════════════════════════════════════════════════════╣
║  Algorithm:  {args.algorithm:<46}║
║  Noise:      {args.noise}%{' '*(45-len(str(args.noise)))}║
║  World:      {args.world:<46}║
║  Runs:       {args.runs:<46}║
║  Laps/Run:   {args.laps:<46}║
║  Duration:   max {args.duration}s per run{' '*(35-len(f'max {args.duration}s per run'))}║
║  Output:     {args.output_dir:<46}║
╚══════════════════════════════════════════════════════════════╝
""")

    result_dirs = []
    n_ok = 0

    try:
        for run_num in range(args.start_from, args.runs + 1):
            kill_simulation()

            result_dir, success = run_single(args, run_num)

            if success:
                n_ok += 1
            result_dirs.append(result_dir)

            if run_num < args.runs:
                print(f'\n  Cooldown {args.cooldown}s...')
                try:
                    time.sleep(args.cooldown)
                except KeyboardInterrupt:
                    raise
    except KeyboardInterrupt:
        print('\n  ✗ Batch interrupted — cleaning up...')
        kill_simulation()
        if result_dirs:
            print(f'\n  {n_ok}/{len(result_dirs)} runs completed before interrupt.')
            compute_and_print_average(args, result_dirs)
        sys.exit(1)

    print(f'\n  {n_ok}/{args.runs} runs successful — computing average...')
    time.sleep(2)
    compute_and_print_average(args, result_dirs)


if __name__ == '__main__':
    main()
