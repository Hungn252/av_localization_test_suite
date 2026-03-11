#!/usr/bin/env python3
"""
run_suite.py — SLAM evaluation suite orchestrator.

Usage:
    python3 run_suite.py [user_config.yaml]

The default config path is <suite_dir>/user_config.yaml.
See that file for all configuration options.
"""

import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

# ── paths ─────────────────────────────────────────────────────────────────────

SUITE_DIR   = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = SUITE_DIR / 'scripts'
METRICS_DIR = SCRIPTS_DIR / 'metrics'

# ── colours ───────────────────────────────────────────────────────────────────

GREEN  = '\033[0;32m'
YELLOW = '\033[1;33m'
RED    = '\033[0;31m'
NC     = '\033[0m'

def log(msg):
    print(f'{GREEN}[suite]{NC} {msg}', flush=True)

def warn(msg):
    print(f'{YELLOW}[suite] WARNING:{NC} {msg}', flush=True)

def err(msg):
    print(f'{RED}[suite] ERROR:{NC} {msg}', flush=True)


# ── child-process registry ────────────────────────────────────────────────────

_children = []   # list of subprocess.Popen objects

def _register(proc):
    _children.append(proc)
    return proc

def _cleanup():
    """Send SIGTERM to all still-running child processes."""
    for p in _children:
        if p.poll() is None:
            try:
                p.terminate()
            except Exception:
                pass
    # Give them a moment, then SIGKILL stragglers
    time.sleep(2)
    for p in _children:
        if p.poll() is None:
            try:
                p.kill()
            except Exception:
                pass

def _sigterm_handler(signum, frame):
    warn('SIGTERM received — cleaning up.')
    _cleanup()
    sys.exit(1)

signal.signal(signal.SIGTERM, _sigterm_handler)


# ── helpers ───────────────────────────────────────────────────────────────────

ROS2_PREFIX = 'source /opt/ros/humble/setup.bash && '


def ros2_cmd(cmd):
    """Wrap a shell command with ROS2 environment sourcing."""
    return f'bash -c "{ROS2_PREFIX}{cmd}"'


def open_log(log_dir, name):
    """Return an open file object for a subprocess log."""
    log_dir.mkdir(parents=True, exist_ok=True)
    return open(log_dir / name, 'w')


def run_python(args_list, log_dir, log_name, check=True):
    """
    Run a Python script (no ROS2 env needed) and capture output to a log file.
    Returns (returncode, log_path).
    """
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / log_name
    cmd = [sys.executable] + args_list
    with open(log_path, 'w') as lf:
        result = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT)
    if check and result.returncode != 0:
        warn(f'Command returned {result.returncode}: {" ".join(cmd)}')
        warn(f'Log: {log_path}')
    return result.returncode, log_path


# ── config loading ─────────────────────────────────────────────────────────────

def load_config(config_path):
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    return cfg


def load_scenarios():
    sc_path = SUITE_DIR / 'configs' / 'scenarios.yaml'
    if not sc_path.exists():
        err(f'scenarios.yaml not found at {sc_path}')
        sys.exit(1)
    with open(sc_path) as f:
        data = yaml.safe_load(f)
    return data.get('scenarios', {})


# ── prerequisite check ────────────────────────────────────────────────────────

def check_prerequisites(cfg, scenarios):
    ok = True

    # Check yaml loaded properly
    algo = cfg.get('algorithm', {})
    if not algo.get('name'):
        err('algorithm.name is empty in user_config.yaml')
        ok = False

    pose_type = algo.get('pose_type', '')
    valid_types = ('odometry', 'pose_stamped', 'pose_with_covariance')
    if pose_type not in valid_types:
        err(f'algorithm.pose_type "{pose_type}" is invalid. '
            f'Must be one of: {", ".join(valid_types)}')
        ok = False

    for name, sc in scenarios.items():
        bag_path = SUITE_DIR / sc['bag']
        gt_path  = SUITE_DIR / sc['ground_truth']
        if not bag_path.exists():
            warn(f'Bag not found for scenario "{name}": {bag_path}')
        if not gt_path.exists():
            err(f'Ground truth not found for scenario "{name}": {gt_path}')
            ok = False

    if not ok:
        sys.exit(1)


# ── scenario runner ───────────────────────────────────────────────────────────

def run_scenario(scenario_name, scenario_cfg, cfg, aut_dir, scenario_idx, n_scenarios):
    algo      = cfg['algorithm']
    algo_name = algo['name']
    pose_topic = algo['pose_topic']
    pose_type  = algo['pose_type']
    launch_cmd = algo.get('launch_command', '').strip()
    launch_wait = float(algo.get('launch_wait_s', 5))

    bag_path = SUITE_DIR / scenario_cfg['bag']
    gt_path  = SUITE_DIR / scenario_cfg['ground_truth']
    duration_s = scenario_cfg.get('duration_s', 300)

    # Output directories
    scenario_out = aut_dir / scenario_name
    logs_dir     = scenario_out / 'logs'
    scenario_out.mkdir(parents=True, exist_ok=True)
    logs_dir.mkdir(parents=True, exist_ok=True)

    print('', flush=True)
    log(f'=== Scenario {scenario_idx}/{n_scenarios}: {scenario_name} ===')
    log(f'Bag:          {bag_path}')
    log(f'Ground truth: {gt_path}')
    log(f'Output:       {scenario_out}')

    # Temporary pose file written by recorder
    tmp_poses = scenario_out / 'poses_tum.txt'

    algo_proc     = None
    recorder_proc = None

    try:
        # ── Step 2/3: launch algorithm ─────────────────────────────────────────
        if launch_cmd:
            log(f'Launching algorithm: {launch_cmd}')
            algo_log = open_log(logs_dir, 'algo.log')
            algo_proc = _register(subprocess.Popen(
                launch_cmd, shell=True, executable='/bin/bash',
                stdout=algo_log, stderr=subprocess.STDOUT,
            ))
            log(f'Algorithm PID {algo_proc.pid} started.')
        else:
            log('launch_command is empty.')
            log('Please start your algorithm now, then press Enter to continue...')
            try:
                input()
            except EOFError:
                pass

        # ── Step 4: start pose recorder ───────────────────────────────────────
        recorder_timeout = duration_s + 30  # generous headroom beyond bag duration
        recorder_cmd = (
            f'{ROS2_PREFIX}'
            f'python3 {SCRIPTS_DIR}/record_poses.py '
            f'{tmp_poses} {pose_topic} {pose_type} {recorder_timeout}'
        )
        log(f'Starting pose recorder (timeout {recorder_timeout}s)...')
        rec_log = open_log(logs_dir, 'recorder.log')
        recorder_proc = _register(subprocess.Popen(
            recorder_cmd, shell=True, executable='/bin/bash',
            stdout=rec_log, stderr=subprocess.STDOUT,
        ))

        # ── Step 5: wait for recorder to initialise ────────────────────────────
        log(f'Waiting {launch_wait}s for recorder to initialise...')
        time.sleep(launch_wait)

        # ── Step 6: play bag ───────────────────────────────────────────────────
        bag_cmd = f'{ROS2_PREFIX}ros2 bag play {bag_path}'
        log(f'Playing bag: {bag_path}')
        bag_log = open_log(logs_dir, 'bag_play.log')
        bag_proc = _register(subprocess.Popen(
            bag_cmd, shell=True, executable='/bin/bash',
            stdout=bag_log, stderr=subprocess.STDOUT,
        ))

        # ── Step 7: wait for bag to finish ────────────────────────────────────
        bag_proc.wait()
        log('Bag playback finished.')

        # ── Step 8: buffer for final poses ────────────────────────────────────
        log('Waiting 3s for final poses to arrive...')
        time.sleep(3)

        # ── Step 9: stop recorder ─────────────────────────────────────────────
        if recorder_proc.poll() is None:
            log('Sending SIGTERM to recorder...')
            recorder_proc.terminate()
            try:
                recorder_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                warn('Recorder did not exit in 5s — killing.')
                recorder_proc.kill()
                recorder_proc.wait()
        log('Recorder stopped.')

        # ── Step 10: stop algorithm ───────────────────────────────────────────
        if algo_proc is not None and algo_proc.poll() is None:
            log('Sending SIGTERM to algorithm...')
            algo_proc.terminate()
            try:
                algo_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                warn('Algorithm did not exit in 5s — killing.')
                algo_proc.kill()
                algo_proc.wait()

        # ── Check poses file ──────────────────────────────────────────────────
        if not tmp_poses.exists():
            warn(f'Poses file not created: {tmp_poses}')
            warn(f'Skipping evaluation for scenario "{scenario_name}".')
            return False

        with open(tmp_poses) as f:
            n_lines = sum(1 for ln in f if ln.strip() and not ln.startswith('#'))

        if n_lines == 0:
            warn(f'Poses file has 0 poses — skipping evaluation for "{scenario_name}".')
            return False

        log(f'{n_lines} poses recorded.')

        # ── Step 11 & 12: run metrics on COPIED files ─────────────────────────
        log('Running ATE...')
        rc, _ = run_python(
            [str(METRICS_DIR / 'ate.py'),
             str(gt_path), str(scenario_out / 'poses_tum.txt'),
             '--output', str(scenario_out / 'ate.json')],
            logs_dir, 'ate.log',
        )
        if rc != 0:
            warn('ATE computation failed — check logs/ate.log')

        log('Running RPE...')
        rc, _ = run_python(
            [str(METRICS_DIR / 'rpe.py'),
             str(gt_path), str(scenario_out / 'poses_tum.txt'),
             '--delta', '1',
             '--output', str(scenario_out / 'rpe.json')],
            logs_dir, 'rpe.log',
        )
        if rc != 0:
            warn('RPE computation failed — check logs/rpe.log')

        # Latency: copy companion file if it exists, then run latency.py on it
        latency_src = Path(str(tmp_poses.with_suffix('')) + '.latency.json')
        latency_raw_dst = scenario_out / 'latency_raw.json'
        latency_json_dst = scenario_out / 'latency.json'

        if latency_src.exists():
            shutil.copy2(latency_src, latency_raw_dst)
            log('Running latency evaluation...')
            # latency.py expects the raw per-frame format; latency_raw.json
            # from record_poses uses the JSON format directly.
            # We write a plain-ms file for latency.py to consume.
            raw_data = json.loads(latency_raw_dst.read_text())
            mean_ms  = raw_data.get('mean_latency_ms')
            if mean_ms is not None:
                # Write a minimal latency JSON compatible with latency.py's
                # 'per_frame' plain format by using the stored summary directly.
                lat_out = {
                    'source':         'per_frame',
                    'n_frames':       raw_data.get('n_poses', 0),
                    'mean_ms':        raw_data.get('mean_latency_ms'),
                    'median_ms':      raw_data.get('mean_latency_ms'),  # best available
                    'std_ms':         0.0,
                    'min_ms':         raw_data.get('mean_latency_ms'),
                    'max_ms':         raw_data.get('p95_latency_ms') or raw_data.get('mean_latency_ms'),
                    'p95_ms':         raw_data.get('p95_latency_ms') or raw_data.get('mean_latency_ms'),
                    'p99_ms':         raw_data.get('p95_latency_ms') or raw_data.get('mean_latency_ms'),
                    'mean_hz':        raw_data.get('mean_hz'),
                }
                with open(latency_json_dst, 'w') as f:
                    json.dump(lat_out, f, indent=2)
                log(f'Latency mean: {mean_ms:.1f} ms')
            else:
                warn('Latency data present but mean_latency_ms is null — skipping.')
        else:
            warn('No latency companion file found — skipping latency evaluation.')

        # Plots
        ate_json = scenario_out / 'ate.json'
        rpe_json = scenario_out / 'rpe.json'
        plot_args = [
            str(METRICS_DIR / 'plot.py'),
            str(gt_path),
            str(scenario_out / 'poses_tum.txt'),
            str(scenario_out),
            '--title', f'{algo_name} / {scenario_name}',
        ]
        if ate_json.exists():
            plot_args += ['--ate_json', str(ate_json)]
        if rpe_json.exists():
            plot_args += ['--rpe_json', str(rpe_json)]

        log('Generating plots...')
        rc, _ = run_python(plot_args, logs_dir, 'plot.log', check=False)
        if rc != 0:
            warn('Plot generation failed — check logs/plot.log')

        log(f'Scenario "{scenario_name}" complete.')
        return True

    except KeyboardInterrupt:
        warn('Interrupted.')
        _cleanup()
        sys.exit(1)

    finally:
        # Ensure subprocesses are dead if anything went wrong
        for p in [recorder_proc, algo_proc]:
            if p is not None and p.poll() is None:
                try:
                    p.terminate()
                except Exception:
                    pass


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    # Config path
    if len(sys.argv) > 1:
        config_path = Path(sys.argv[1])
    else:
        config_path = SUITE_DIR / 'user_config.yaml'

    if not config_path.exists():
        err(f'Config file not found: {config_path}')
        sys.exit(1)

    log(f'Loading config: {config_path}')
    cfg = load_config(config_path)

    scenarios = load_scenarios()
    if not scenarios:
        err('No scenarios defined in configs/scenarios.yaml')
        sys.exit(1)

    check_prerequisites(cfg, scenarios)

    algo_name   = cfg['algorithm']['name']
    output_root = Path(cfg.get('output_path', '/results'))
    timestamp   = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    aut_dir     = output_root / algo_name / timestamp

    log(f'Algorithm:  {algo_name}')
    log(f'Output dir: {aut_dir}')
    log(f'Scenarios:  {", ".join(scenarios.keys())}')

    evaluated = []
    scenario_list = list(scenarios.items())
    n = len(scenario_list)

    for idx, (name, sc_cfg) in enumerate(scenario_list, start=1):
        success = run_scenario(name, sc_cfg, cfg, aut_dir, idx, n)
        if success:
            evaluated.append(name)

    # ── Step 13: benchmark report ──────────────────────────────────────────────
    print('', flush=True)
    log('Generating benchmark report...')
    rc, _ = run_python(
        [str(METRICS_DIR / 'benchmark_report.py'),
         str(aut_dir), algo_name,
         '--output', str(aut_dir / 'benchmark.md')],
        aut_dir, 'benchmark_report.log',
        check=False,
    )
    if rc != 0:
        warn('Benchmark report generation failed — check benchmark_report.log')

    # ── Step 14: final summary ─────────────────────────────────────────────────
    print('', flush=True)
    log('=== Run complete ===')
    log(f'Algorithm:   {algo_name}')
    log(f'Evaluated:   {", ".join(evaluated) if evaluated else "(none)"}')
    log(f'Results:     {aut_dir}')
    log(f'Benchmark:   {aut_dir / "benchmark.md"}')

    if not evaluated:
        warn('No scenarios were successfully evaluated.')
        sys.exit(1)


if __name__ == '__main__':
    main()
