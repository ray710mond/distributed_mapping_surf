"""Offline, exploratory least squares for the discrete DELTA/BACKLOG model."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from .analysis import load_events


def _fit_features(features, targets):
    scale = np.maximum(np.linalg.norm(features, axis=0), 1e-12)
    scaled = features / scale
    coefficients, _, rank, singular = np.linalg.lstsq(scaled, targets, rcond=None)
    coefficients = coefficients / scale[:, None]
    residual = targets - features @ coefficients
    return coefficients, residual, rank, singular


def fit_model(x, u, next_x, disturbance):
    features = np.column_stack((x, u))
    coefficients, residual, rank, singular = _fit_features(
        features, next_x - disturbance)
    return {'A': coefficients[:2].T.tolist(), 'B': coefficients[2:].T.tolist(),
            'sample_count': len(x), 'rank': int(rank),
            'scaled_condition_number': float(singular[0] / singular[-1]) if singular[-1] > 0 else None,
            'rmse': np.sqrt(np.mean(residual ** 2, axis=0)).tolist(),
            'warning': 'Exploratory fit only. ACK delays, saturation, insufficient excitation and time-varying priorities can invalidate this model.'}


def fit_delayed_model(x, input_history, next_x, disturbance, validation_fraction=.2):
    """Fit x[k+1] from x[k] and u[k], ..., u[k-lag].

    The returned effective B is the sum of the finite-impulse-response input
    coefficients. It is a diagnostic and is not a delay-free LQR plant matrix.
    """
    features = np.column_stack((x, input_history.reshape(len(x), -1)))
    targets = next_x - disturbance
    validation_count = max(1, int(len(x) * validation_fraction))
    training_count = len(x) - validation_count
    if training_count < features.shape[1]:
        raise ValueError('not enough training samples for delayed fit')
    coefficients, training_residual, rank, singular = _fit_features(
        features[:training_count], targets[:training_count])
    validation_residual = targets[training_count:] - features[training_count:] @ coefficients
    lag_count = input_history.shape[1]
    b_lags = coefficients[2:].reshape(lag_count, 2, 2).transpose(0, 2, 1)
    return {
        'A': coefficients[:2].T.tolist(),
        'B_lags': b_lags.tolist(),
        'B_effective': b_lags.sum(axis=0).tolist(),
        'lag_steps': lag_count - 1,
        'sample_count': len(x),
        'training_sample_count': training_count,
        'validation_sample_count': validation_count,
        'rank': int(rank),
        'feature_count': int(features.shape[1]),
        'scaled_condition_number':
            float(singular[0] / singular[-1]) if singular[-1] > 0 else None,
        'training_rmse': np.sqrt(np.mean(training_residual ** 2, axis=0)).tolist(),
        'validation_rmse': np.sqrt(np.mean(validation_residual ** 2, axis=0)).tolist(),
    }


def _continuous_segments(rows):
    segments, current = [], []
    for row in rows:
        if current:
            before = current[-1]
            continuous = (
                all(before[k] == row[k] for k in
                    ('source_id', 'map_epoch', 'configuration_revision')) and
                row['step'] == before['step'] + 1 and
                abs(row['actual_dt'] - before['nominal_dt']) <= before['nominal_dt'] * .2)
            if not continuous:
                if len(current) > 1:
                    segments.append(current)
                current = []
        current.append(row)
    if len(current) > 1:
        segments.append(current)
    return segments


def _input(row, kind):
    if kind == 'offered_rate':
        dt = row['actual_dt']
        return [row[f'wire_bytes_{i}'] / dt if dt > 0 else 0 for i in range(2)]
    prefix = {'requested_rate': 'requested_rate',
              'allocated_rate': 'allocated_rate'}[kind]
    return [row[f'{prefix}_{i}'] for i in range(2)]


def _delayed_samples(segments, input_kind, lag_steps):
    x, inputs, y, disturbances = [], [], [], []
    for segment in segments:
        for index in range(lag_steps, len(segment) - 1):
            before, after = segment[index:index + 2]
            x.append([before[f'debt_{i}'] for i in range(2)])
            inputs.append([
                _input(segment[index - lag], input_kind)
                for lag in range(lag_steps + 1)])
            y.append([after[f'debt_{i}'] for i in range(2)])
            disturbances.append([
                after.get(f'disturbance_{i}', 0) - before.get(f'disturbance_{i}', 0)
                for i in range(2)])
    return tuple(np.asarray(values, dtype=float) for values in (x, inputs, y, disturbances))


def delayed_input_diagnostics(segments, max_lag_steps):
    candidates = sorted(set(
        lag for lag in (0, 1, 2, 5, 10, 20, max_lag_steps)
        if lag <= max_lag_steps))
    diagnostics = {}
    for input_kind in ('requested_rate', 'allocated_rate', 'offered_rate'):
        fits = []
        for lag in candidates:
            samples = _delayed_samples(segments, input_kind, lag)
            if len(samples[0]) < 10:
                continue
            try:
                fit = fit_delayed_model(*samples)
            except ValueError:
                continue
            fits.append(fit)
        if not fits:
            continue
        best = min(fits, key=lambda fit: sum(value * value for value in fit['validation_rmse']))
        diagnostics[input_kind] = {
            'selected_lag_steps': best['lag_steps'],
            'selected_by': 'minimum chronological holdout RMSE',
            'selected_fit': best,
            'candidate_validation_rmse': {
                str(fit['lag_steps']): fit['validation_rmse'] for fit in fits},
        }
    return diagnostics


def acknowledged_service_diagnostics(segments):
    acknowledged = np.zeros(2)
    offered_bytes = np.zeros(2)
    accounting_residuals = []
    for segment in segments:
        for before, after in zip(segment, segment[1:]):
            ack = np.array([
                after.get(f'acknowledged_debt_{i}', 0) -
                before.get(f'acknowledged_debt_{i}', 0) for i in range(2)])
            disturbance = np.array([
                after.get(f'disturbance_{i}', 0) - before.get(f'disturbance_{i}', 0)
                for i in range(2)])
            current = np.array([before[f'debt_{i}'] for i in range(2)])
            following = np.array([after[f'debt_{i}'] for i in range(2)])
            acknowledged += ack
            offered_bytes += np.array([before[f'wire_bytes_{i}'] for i in range(2)])
            accounting_residuals.append(following - (current + disturbance - ack))
    residuals = np.asarray(accounting_residuals)
    return {
        'acknowledged_debt': acknowledged.tolist(),
        'offered_cdr_bytes': offered_bytes.tolist(),
        'acknowledged_debt_per_offered_byte': np.divide(
            acknowledged, offered_bytes, out=np.zeros(2), where=offered_bytes > 0).tolist(),
        'state_accounting_rmse':
            np.sqrt(np.mean(residuals ** 2, axis=0)).tolist() if len(residuals) else [],
        'note': 'Acknowledged debt is receiver-confirmed service. Offered bytes are not time-aligned to ACKs.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--max-lag-steps', type=int, default=30,
                        help='largest input-history window considered (default: 30)')
    args = parser.parse_args()
    if args.max_lag_steps < 0:
        parser.error('--max-lag-steps must be nonnegative')
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
    segments = _continuous_segments(rows)
    x, input_history, y, disturbances = _delayed_samples(
        segments, 'allocated_rate', 0)
    u = input_history[:, 0, :]
    if len(x) < 4:
        parser.error('need at least four consecutive samples with matching epoch/revision and dt')
    result = fit_model(x, u, y, disturbances)
    result['delayed_input_diagnostics'] = delayed_input_diagnostics(
        segments, args.max_lag_steps)
    result['acknowledged_service'] = acknowledged_service_diagnostics(segments)
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
