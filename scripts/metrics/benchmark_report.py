#!/usr/bin/env python3
"""
benchmark_report.py — generate a markdown benchmark comparing a user's
algorithm against all available baselines, across all evaluated scenarios.

Usage:
    python3 benchmark_report.py <aut_dir> <algo_name> [--output report.md]

Arguments:
    aut_dir     Directory written by run_suite.py for the user's algorithm.
                Expected layout: <aut_dir>/<scenario>/ate.json
                                 <aut_dir>/<scenario>/rpe.json
                                 <aut_dir>/<scenario>/latency.json  (optional)
    algo_name   Label to use for the user's algorithm in the report.

Options:
    --output    Path for the output markdown file.
                Default: <aut_dir>/benchmark.md

Baseline discovery:
    Baselines are loaded from <suite_dir>/baselines/<algo>/<scenario>/metrics.json
    where <suite_dir> is three directories above this script
    (scripts/metrics/ → scripts/ → suite/).
"""

import argparse
import json
import sys
from pathlib import Path

# SUITE_DIR: go up from scripts/metrics/ → scripts/ → suite root
SUITE_DIR = Path(__file__).resolve().parent.parent.parent


# ── loaders ──────────────────────────────────────────────────────────────────

def _load_json(path):
    try:
        return json.loads(Path(path).read_text())
    except Exception:
        return None


def load_baselines():
    """
    Return dict: { scenario -> [ {algorithm, ate_rmse, ...}, ... ] }
    Discovers all baselines/<algo>/<scenario>/metrics.json files.
    """
    baselines_root = SUITE_DIR / 'baselines'
    result = {}
    if not baselines_root.is_dir():
        return result

    for metrics_path in sorted(baselines_root.glob('*/*/metrics.json')):
        data = _load_json(metrics_path)
        if data is None:
            continue
        scenario = metrics_path.parent.name
        result.setdefault(scenario, []).append(data)

    return result


def load_aut_metrics(aut_dir, scenario):
    """
    Load user algorithm metrics for one scenario.
    Returns a dict with unified keys, or None if no data found.
    """
    base = Path(aut_dir) / scenario
    ate  = _load_json(base / 'ate.json')
    rpe  = _load_json(base / 'rpe.json')
    lat  = _load_json(base / 'latency.json')

    if ate is None and rpe is None:
        return None

    m = {}
    if ate:
        m['ate_rmse']      = ate.get('ate_rmse')
        m['ate_mean']      = ate.get('ate_mean')
        m['ate_max']       = ate.get('ate_max')
    if rpe:
        m['rpe_trans_rmse']  = rpe.get('trans_rmse')
        m['rpe_trans_mean']  = rpe.get('trans_mean')
        m['rpe_rot_rmse_deg'] = rpe.get('rot_rmse_deg')
    if lat:
        m['latency_mean_ms'] = lat.get('mean_ms')
    return m


# ── formatting ────────────────────────────────────────────────────────────────

def _fmt(value, decimals=4, suffix=''):
    if value is None:
        return 'N/A'
    try:
        return f'{float(value):.{decimals}f}{suffix}'
    except (TypeError, ValueError):
        return 'N/A'


def _row(algo_name, m, is_aut=False):
    """Return a markdown table row string."""
    name_cell = f'**{algo_name}**' if is_aut else algo_name
    ate_rmse  = _fmt(m.get('ate_rmse'),       4, ' m')
    ate_mean  = _fmt(m.get('ate_mean'),        4, ' m')
    rpe_trans = _fmt(m.get('rpe_trans_rmse'),  4, ' m')
    rpe_rot   = _fmt(m.get('rpe_rot_rmse_deg'), 4, ' deg')
    latency   = _fmt(m.get('latency_mean_ms'), 1, ' ms')
    return (
        f'| {name_cell} | {ate_rmse} | {ate_mean} | '
        f'{rpe_trans} | {rpe_rot} | {latency} |'
    )


# ── report generation ─────────────────────────────────────────────────────────

def generate(aut_dir, algo_name, output_path=None):
    aut_dir = Path(aut_dir)
    if output_path is None:
        output_path = aut_dir / 'benchmark.md'
    else:
        output_path = Path(output_path)

    baselines = load_baselines()

    # Collect all scenarios: union of baselines and what the AUT has evaluated
    aut_scenarios = set()
    for child in sorted(aut_dir.iterdir()):
        if child.is_dir() and (child / 'ate.json').exists():
            aut_scenarios.add(child.name)

    all_scenarios = sorted(set(baselines.keys()) | aut_scenarios)

    if not all_scenarios:
        print('[benchmark_report] No scenarios found — nothing to report.', file=sys.stderr)
        return

    lines = [
        '# SLAM Benchmark Report',
        '',
        f'User algorithm: **{algo_name}**',
        '',
    ]

    summary_lines = []

    header = (
        '| Algorithm | ATE RMSE | ATE Mean | RPE Trans RMSE | RPE Rot RMSE | Latency |'
    )
    separator = (
        '|-----------|:--------:|:--------:|:--------------:|:------------:|:-------:|'
    )

    for scenario in all_scenarios:
        lines.append(f'## Scenario: {scenario}')
        lines.append('')
        lines.append(header)
        lines.append(separator)

        aut_m = load_aut_metrics(aut_dir, scenario)
        scenario_baselines = baselines.get(scenario, [])

        # Baselines first
        for bm in scenario_baselines:
            bname = bm.get('algorithm', 'unknown')
            lines.append(_row(bname, bm, is_aut=False))

        # User algorithm
        if aut_m is not None:
            lines.append(_row(algo_name, aut_m, is_aut=True))
            summary_lines.append(
                f'  {scenario}: ATE RMSE={_fmt(aut_m.get("ate_rmse"), 4)} m  '
                f'RPE trans RMSE={_fmt(aut_m.get("rpe_trans_rmse"), 4)} m  '
                f'latency={_fmt(aut_m.get("latency_mean_ms"), 1)} ms'
            )
        else:
            lines.append(f'| _{algo_name} (not evaluated)_ | — | — | — | — | — |')

        lines.append('')

    # Write file
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text('\n'.join(lines))
    print(f'[benchmark_report] Report saved → {output_path}')

    # Print summary to stdout
    print()
    print(f'=== Benchmark Summary: {algo_name} ===')
    if summary_lines:
        for s in summary_lines:
            print(s)
    else:
        print('  No evaluated scenarios found for this algorithm.')
    print()

    return output_path


# ── CLI ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument('aut_dir',   help='AUT output directory (written by run_suite.py)')
    ap.add_argument('algo_name', help='Label for the user algorithm')
    ap.add_argument('--output',  default=None,
                    help='Output markdown path (default: <aut_dir>/benchmark.md)')
    args = ap.parse_args()

    generate(args.aut_dir, args.algo_name, output_path=args.output)


if __name__ == '__main__':
    main()
