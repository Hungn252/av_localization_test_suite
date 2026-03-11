#!/usr/bin/env python3
"""
Latency evaluation.

Accepts a latency log file in one of two formats:

  1. Plain list — one runtime value per line (in ms):
         12.3
         11.8
         13.1
         ...

  2. KISS-ICP table format (algorithm_metrics.txt / result_metrics.log):
         | Average Runtime |  12   | ms    |
         | Average Frequency |  85   | Hz    |

For format 1, full statistics are computed.
For format 2, only the reported averages are extracted.

Usage:
    python3 latency.py <metrics_file> [--output metrics.json]
"""

import argparse
import json
import re
import sys

import numpy as np


# ── Parsers ───────────────────────────────────────────────────────────────────

def _try_plain(lines):
    """Try to parse as one float per line. Returns array or None."""
    values = []
    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        try:
            values.append(float(line))
        except ValueError:
            return None
    return np.array(values) if values else None


def _try_table(lines):
    """
    Parse KISS-ICP table format. Returns dict with extracted values or None.
    Looks for lines like:  | Average Runtime |  12   | ms    |
    """
    result = {}
    runtime_pat   = re.compile(r'\|\s*Average Runtime\s*\|\s*([\d.]+)\s*\|\s*ms', re.IGNORECASE)
    frequency_pat = re.compile(r'\|\s*Average Frequency\s*\|\s*([\d.]+)\s*\|\s*Hz', re.IGNORECASE)
    for line in lines:
        m = runtime_pat.search(line)
        if m:
            result['avg_runtime_ms'] = float(m.group(1))
        m = frequency_pat.search(line)
        if m:
            result['avg_frequency_hz'] = float(m.group(1))
    return result if result else None


def load(path):
    with open(path) as f:
        lines = f.readlines()

    # Try plain per-frame format first
    values = _try_plain(lines)
    if values is not None and len(values) > 0:
        return 'per_frame', values

    # Fall back to table format
    table = _try_table(lines)
    if table:
        return 'table', table

    sys.exit(f'Unrecognised format in: {path}')


# ── Main ─────────────────────────────────────────────────────────────────────

def compute(path):
    kind, data = load(path)

    if kind == 'per_frame':
        m = {
            'source':         'per_frame',
            'n_frames':       int(len(data)),
            'mean_ms':        float(np.mean(data)),
            'median_ms':      float(np.median(data)),
            'std_ms':         float(np.std(data)),
            'min_ms':         float(np.min(data)),
            'max_ms':         float(np.max(data)),
            'p95_ms':         float(np.percentile(data, 95)),
            'p99_ms':         float(np.percentile(data, 99)),
        }
    else:  # table
        m = {'source': 'table'}
        if 'avg_runtime_ms' in data:
            m['mean_ms'] = data['avg_runtime_ms']
        if 'avg_frequency_hz' in data:
            m['avg_frequency_hz'] = data['avg_frequency_hz']

    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('metrics_file', help='Latency log or algorithm_metrics.txt')
    ap.add_argument('--output',     help='Save metrics as JSON to this path')
    args = ap.parse_args()

    m = compute(args.metrics_file)

    w = 18
    print(f"\n{'Metric':<{w}}  {'Value':>12}")
    print('─' * (w + 15))
    print(f"{'source':<{w}}  {m['source']:>12}")
    if m['source'] == 'per_frame':
        print(f"{'n_frames':<{w}}  {m['n_frames']:>12d}")
        print(f"{'mean':<{w}}  {m['mean_ms']:>11.2f} ms")
        print(f"{'median':<{w}}  {m['median_ms']:>11.2f} ms")
        print(f"{'std':<{w}}  {m['std_ms']:>11.2f} ms")
        print(f"{'min':<{w}}  {m['min_ms']:>11.2f} ms")
        print(f"{'max':<{w}}  {m['max_ms']:>11.2f} ms")
        print(f"{'p95':<{w}}  {m['p95_ms']:>11.2f} ms")
        print(f"{'p99':<{w}}  {m['p99_ms']:>11.2f} ms")
    else:
        if 'mean_ms' in m:
            print(f"{'avg runtime':<{w}}  {m['mean_ms']:>11.1f} ms")
        if 'avg_frequency_hz' in m:
            print(f"{'avg frequency':<{w}}  {m['avg_frequency_hz']:>10.1f} Hz")

    if args.output:
        with open(args.output, 'w') as f:
            json.dump(m, f, indent=2)
        print(f'\nSaved → {args.output}')

    return m


if __name__ == '__main__':
    main()
