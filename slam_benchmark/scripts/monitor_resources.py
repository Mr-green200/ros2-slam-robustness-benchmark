import argparse
import signal
import sys
import time
import psutil


SAMPLE_INTERVAL = 1.0

PROCESS_TARGETS = {
    'slam_toolbox': 'async_slam_toolbox_node',
    'amcl':         'nav2_amcl',
}


def find_process(target):
    """Return the first psutil.Process whose name or cmdline contains target."""
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            cmdline = ' '.join(proc.info['cmdline'] or [])
            if target in cmdline or target in (proc.info['name'] or ''):
                return proc
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return None


def write_summary(samples, out_file):
    """Append a [SUMMARY] block with aggregate CPU/RAM stats."""
    if not samples:
        out_file.write('\n[SUMMARY]\nno_samples: true\n')
        return

    cpu_vals = [s[1] for s in samples]
    ram_vals = [s[2] for s in samples]

    out_file.write('\n[SUMMARY]\n')
    out_file.write(f'n_samples:    {len(samples)}\n')
    out_file.write(f'cpu_mean_pct: {sum(cpu_vals)/len(cpu_vals):.2f}\n')
    out_file.write(f'cpu_max_pct:  {max(cpu_vals):.2f}\n')
    out_file.write(f'cpu_min_pct:  {min(cpu_vals):.2f}\n')
    out_file.write(f'ram_mean_mb:  {sum(ram_vals)/len(ram_vals):.1f}\n')
    out_file.write(f'ram_max_mb:   {max(ram_vals):.1f}\n')
    out_file.write(f'ram_min_mb:   {min(ram_vals):.1f}\n')
    out_file.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output_file', help='Path to write resource usage data')
    parser.add_argument('--algorithm', default='slam_toolbox',
                        choices=list(PROCESS_TARGETS.keys()))
    parser.add_argument('--interval', type=float, default=SAMPLE_INTERVAL)
    args = parser.parse_args()

    target = PROCESS_TARGETS[args.algorithm]
    samples = []
    proc = None
    running = True

    def on_signal(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    with open(args.output_file, 'w') as f:
        f.write(f'# Resource monitor: {target} (algorithm={args.algorithm})\n')
        f.write('# timestamp_s  cpu_pct  ram_mb\n')
        f.flush()

        start_time = time.time()

        while running and proc is None:
            proc = find_process(target)
            if proc is None:
                if time.time() - start_time > 30:
                    f.write(f'# ERROR: {target} not found within 30 s\n')
                    f.flush()
                    break
                time.sleep(1)

        if proc is not None:
            try:
                proc.cpu_percent(interval=None)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                proc = None

        t0 = time.time()
        while running:
            time.sleep(args.interval)
            if proc is None:
                proc = find_process(target)
                if proc is None:
                    continue
                try:
                    proc.cpu_percent(interval=None)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    proc = None
                continue

            try:
                cpu = proc.cpu_percent(interval=None)
                ram = proc.memory_info().rss / 1024 / 1024
                ts = time.time() - t0
                samples.append((ts, cpu, ram))
                f.write(f'{ts:.1f}  {cpu:.2f}  {ram:.1f}\n')
                f.flush()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                proc = None

        write_summary(samples, f)


if __name__ == '__main__':
    main()
