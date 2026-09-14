#!/usr/bin/env python3
"""Reproduce outdoor diagnostics and frozen-state tuning comparisons (not simulation).

Usage: python3 review_outdoor_lqr.py /path/to/experiments --output /tmp/outdoor-review
Requires numpy and matplotlib. Databases are opened read-only; no ROS required.
"""
import argparse
import json
import sqlite3
from pathlib import Path

import numpy as np


def events(run, category):
    result = []
    for db in sorted((run / 'hosts').glob('*/telemetry.sqlite3')):
        with sqlite3.connect(db.resolve().as_uri() + '?mode=ro', uri=True) as connection:
            for stamp, stage, payload in connection.execute(
                    'SELECT monotonic_ns, stage, payload_json FROM events '
                    'WHERE category=? ORDER BY monotonic_ns', (category,)):
                result.append((db.parent.name, stamp, stage, json.loads(payload)))
    return result


def percentiles(values):
    return dict(zip(('p50', 'p95', 'p99', 'max'),
                    np.percentile(values, [50, 95, 99, 100]).tolist())) if values else None


def gain(r, q=1.):
    # Scalar DARE for A=1, B=-0.001; return positive feedback magnitude.
    b = .001
    p = (q + np.sqrt(q*q + 4 * q*r / b**2)) / 2
    return b * p / (r + b**2 * p)


def review(run):
    allocation = events(run, 'allocation')
    assert len({(h, p['source_id'], p['map_epoch'], p['configuration_revision'])
                for h, _, _, p in allocation}) == 1, 'review requires one sender/model per run'
    rows = [p for _, _, _, p in allocation]
    active = [r for r in rows if r['debt_0'] + r['debt_1'] > 0]
    latency = events(run, 'latency')
    delivered = events(run, 'delivery')
    delivered_ids = {(p['map_epoch'], p['version']) for _, _, _, p in delivered}
    timeouts = [p for _, _, _, p in latency if p.get('ack_lost')]
    pipeline = [p for _, _, _, p in events(run, 'pipeline')]
    successful = [p['update_completion_rtt_ms'] for _, _, _, p in latency
                  if p.get('ack_received')]
    total_dt = sum(r['actual_dt'] for r in active)
    both = [r for r in active if min(r['debt_0'], r['debt_1']) > 0]
    comparisons = {}
    for delta_q, backlog_r in ((1., .0001), (10., .0001), (100., .0001),
                               (1000., .0001), (1., 1.)):
        ratio = gain(.0001, delta_q) / gain(backlog_r)
        shares = [ratio*r['debt_0']/(ratio*r['debt_0']+r['debt_1']) for r in both]
        comparisons[f'delta_Q={delta_q},backlog_R={backlog_r}'] = {
            'delta_to_backlog_gain_ratio': ratio,
            'delta_share_when_both_pending': percentiles(shares),
        }
    # Same eligibility as exploratory identification, excluding idle state pairs.
    pairs = [(a, b) for a, b in zip(rows, rows[1:])
             if b['step'] == a['step'] + 1
             and abs(b['actual_dt']-a['nominal_dt']) <= .2*a['nominal_dt']
             and a['debt_0'] + a['debt_1'] > 0]
    fit = None
    if len(pairs) >= 4:
        f = np.array([[a['debt_0'], a['debt_1'], a['allocated_rate_0'],
                       a['allocated_rate_1']] for a, b in pairs])
        y = np.array([[b[f'debt_{i}'] - b[f'disturbance_{i}'] + a[f'disturbance_{i}']
                       for i in range(2)] for a, b in pairs])
        scale = np.maximum(np.linalg.norm(f, axis=0), 1e-12)
        coef, _, rank, singular = np.linalg.lstsq(f/scale, y, rcond=None)
        coef /= scale[:, None]
        fit = {'samples': len(pairs), 'rank': int(rank),
               'scaled_condition': float(singular[0]/singular[-1]),
               'A': coef[:2].T.tolist(), 'B': coef[2:].T.tolist(),
               'rmse': np.sqrt(np.mean((y-f@coef)**2, axis=0)).tolist(),
               'accepted_for_tuning': False,
               'reason': 'Closed-loop saturated data, delayed ACKs and timing-filter bias; no independent validation.'}
    summary = {
        'run': run.name, 'steps': len(rows), 'active_steps': len(active),
        'both_pending_steps': len(both),
        'initial_sample_debt': [rows[0][f'debt_{i}'] for i in range(2)],
        'initial_sample_step': rows[0]['step'],
        'final_pending': [rows[-1][f'pending_count_{i}'] for i in range(2)],
        'saturated_active_fraction': sum(r['projection_scale'] < 1 for r in active)/len(active),
        'active_dt_s': percentiles([r['actual_dt'] for r in active]),
        'control_ms': percentiles([r['computation_ms'] for r in active]),
        'debt_cycle_ms': percentiles([r['debt_cycle_ms'] for r in active]),
        'clearing_ms': percentiles([r['clearing_ms'] for r in pipeline]),
        'processing_ms': percentiles([r['processing_ms'] for r in pipeline]),
        'prior_overlap_fraction': percentiles([r['static_prior_voxels']/r['unique_voxels']
                                              for r in pipeline if r['unique_voxels']]),
        'defer_0_2_exceeded_fraction': sum(r['actual_dt'] >= .2 for r in active)/len(active),
        'offered_bytes_per_active_second': sum(r['wire_bytes_0']+r['wire_bytes_1'] for r in active)/total_dt,
        'credit_time_retained_fraction': sum(min(r['actual_dt'], 2*r['nominal_dt']) for r in active)/total_dt,
        'oldest_pending_s': max(r['oldest_pending_age'] for r in rows),
        'ack_rtt_ms': percentiles(successful), 'ack_successes': len(successful),
        'ack_timeouts': len(timeouts),
        'timeouts_with_receiver_delivery': sum((p['map_epoch'], p['version']) in delivered_ids for p in timeouts),
        'receiver_delivery_events': len(delivered),
        'receiver_temporal_regressions': sum(p['applied_temporal_regressions'] for _, _, _, p in delivered),
        'receiver_stale_voxels': sum(p['stale_voxels_rejected'] for _, _, _, p in delivered),
        'capacity_methods': sorted({r['capacity_method'] for r in rows}),
        'frozen_state_comparisons': comparisons, 'exploratory_fit': fit,
    }
    return summary, allocation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('experiments', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 3, figsize=(15, 10))
    summaries = []
    for col, name in enumerate(('lqr_outdoor', 'lqr_outdoor_2', 'lqr_outdoor_3')):
        summary, records = review(args.experiments / name)
        summaries.append(summary)
        t = [(stamp-records[0][1])/1e9 for _, stamp, _, p in records]
        rows = [p for _, _, _, p in records]
        for i, label in enumerate(('DELTA', 'BACKLOG')):
            axes[0, col].plot(t, [r[f'pending_count_{i}'] for r in rows], label=label)
        axes[0, col].set(title=name, ylabel='Pending voxels')
        axes[1, col].plot(t, [r['actual_dt'] for r in rows], label='Actual interval')
        axes[1, col].axhline(.1, color='black', linestyle='--', label='Nominal 0.1 s')
        axes[1, col].axhline(.2, color='red', linestyle=':', label='Old defer 0.2 s')
        axes[1, col].set(ylabel='Seconds')
        for cost in (1., 100.):
            ratio = gain(.0001, cost)/gain(.0001)
            shares = [ratio*r['debt_0']/(ratio*r['debt_0']+r['debt_1'])
                      if min(r['debt_0'], r['debt_1']) > 0 else np.nan for r in rows]
            axes[2, col].plot(t, shares, label=f'DELTA Q={cost}')
        axes[2, col].set(ylabel='DELTA share, frozen states', xlabel='Sender elapsed seconds', ylim=(0, 1))
    for ax in axes.flat:
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(args.output/'outdoor_lqr.png', dpi=150)
    plt.close(fig)
    (args.output/'summary.json').write_text(json.dumps(summaries, indent=2, allow_nan=False)+'\n')
    print(args.output/'summary.json')


if __name__ == '__main__':
    main()
