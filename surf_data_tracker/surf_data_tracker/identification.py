"""Offline, exploratory least squares for the discrete DELTA/BACKLOG model."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from .analysis import load_events


def fit_model(x, u, next_x, disturbance):
    features = np.column_stack((x, u))
    scale = np.maximum(np.linalg.norm(features, axis=0), 1e-12)
    coefficients, _, rank, singular = np.linalg.lstsq(features / scale, next_x - disturbance, rcond=None)
    coefficients = coefficients / scale[:, None]
    residual = next_x - disturbance - features @ coefficients
    return {'A': coefficients[:2].T.tolist(), 'B': coefficients[2:].T.tolist(),
            'sample_count': len(x), 'rank': int(rank),
            'scaled_condition_number': float(singular[0] / singular[-1]) if singular[-1] > 0 else None,
            'rmse': np.sqrt(np.mean(residual ** 2, axis=0)).tolist(),
            'warning': 'Exploratory fit only. ACK delays, saturation, insufficient excitation and time-varying priorities can invalidate this model.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    args = parser.parse_args()
    events, _ = load_events(args.run_directory)
    rows = []
    for e in events:
        if e['category'] == 'allocation':
            rows.append({'time_ns': e['wall_time_ns'], **e['payload']})
    if not rows:
        parser.error('no high-resolution allocation events found')
    output = args.run_directory
    fields = sorted({k for r in rows for k in r})
    with (output / 'allocation_steps.csv').open('w') as f:
        writer = csv.DictWriter(f, fields); writer.writeheader(); writer.writerows(rows)
    x, u, y, disturbances = [], [], [], []
    for before, after in zip(rows, rows[1:]):
        if any(before[k] != after[k] for k in ('source_id', 'map_epoch', 'configuration_revision')):
            continue
        if after['step'] != before['step'] + 1:
            continue
        if abs(after['actual_dt'] - before['nominal_dt']) > before['nominal_dt'] * .2:
            continue
        x.append([before[f'debt_{i}'] for i in range(2)])
        u.append([before[f'allocated_rate_{i}'] for i in range(2)])
        y.append([after[f'debt_{i}'] for i in range(2)])
        disturbances.append([after.get(f'disturbance_{i}', 0) - before.get(f'disturbance_{i}', 0) for i in range(2)])
    if len(x) < 4:
        parser.error('need at least four consecutive samples with matching epoch/revision and dt')
    result = fit_model(np.array(x), np.array(u), np.array(y), np.array(disturbances))
    (output / 'allocation_identification.json').write_text(json.dumps(result, indent=2) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = [(r['time_ns'] - rows[0]['time_ns']) / 1e9 for r in rows]
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for i, name in enumerate(('DELTA', 'BACKLOG')):
        axes[0].plot(t, [r[f'debt_{i}'] for r in rows], label=name)
        axes[1].plot(t, [r[f'allocated_rate_{i}'] for r in rows], label=name)
    axes[1].plot(t, [r['usable_capacity'] for r in rows], label='Usable capacity', linestyle='--')
    axes[2].plot(t, [sum(r[f'wire_bytes_{i}'] for i in range(2))/r['actual_dt'] for r in rows], label='Offered CDR bytes/s')
    for ax, label in zip(axes, ('Weighted debt', 'Allocation bytes/s', 'Offered load bytes/s')):
        ax.set_ylabel(label); ax.legend(); ax.grid(alpha=.3)
    axes[-1].set_xlabel('Seconds'); fig.tight_layout(); fig.savefig(output / 'allocation.png'); plt.close(fig)
    # Keep offered map load, successful receiver delivery, and raw radio state separate.
    fig, axes = plt.subplots(3, 2, figsize=(13, 10))
    axes[0, 0].scatter([r['debt_0'] for r in rows], [r['debt_1'] for r in rows], s=8)
    axes[0, 0].set(xlabel='DELTA debt', ylabel='BACKLOG debt')
    axes[0, 1].plot(t, [r.get('projection_scale', float('nan')) for r in rows])
    axes[0, 1].set(xlabel='Seconds', ylabel='Projection scale')
    for i, name in enumerate(('DELTA', 'BACKLOG')):
        axes[1, 0].plot(t, [r.get(f'pending_count_{i}', float('nan')) for r in rows], label=name)
        axes[1, 1].plot(t, [r.get(f'requested_rate_{i}', float('nan')) for r in rows], label=name+' requested')
        axes[1, 1].plot(t, [r[f'wire_bytes_{i}']/r['actual_dt'] for r in rows], label=name+' offered', alpha=.6)
    axes[1, 0].set(ylabel='Outstanding voxel count', xlabel='Seconds'); axes[1, 0].legend()
    axes[1, 1].set(ylabel='Bytes/s', xlabel='Seconds'); axes[1, 1].legend()
    network = [e for e in events if e['category'] == 'network' and e['stage'].startswith('halow')]
    nt = [(e['wall_time_ns']-rows[0]['time_ns'])/1e9 for e in network]
    for key in ('s1g_average_throughput_mbps', 'tx_interface_mbps', 'rx_interface_mbps'):
        axes[2, 0].plot(nt, [e['payload'].get(key, float('nan')) for e in network], label=key)
    axes[2, 0].set(ylabel='Mbps (distinct measurements)', xlabel='Seconds'); axes[2, 0].legend()
    axes[2, 1].scatter([e['payload'].get('s1g_mcs', float('nan')) for e in network],
                      [e['payload'].get('s1g_last_retry_count', float('nan')) for e in network], s=10)
    axes[2, 1].set(xlabel='Driver MCS', ylabel='Last retry count')
    for ax in axes.flat: ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(output / 'allocation_diagnostics.png'); plt.close(fig)
    print(json.dumps(result, indent=2))
