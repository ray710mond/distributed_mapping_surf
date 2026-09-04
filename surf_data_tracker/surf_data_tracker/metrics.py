"""Measurement math shared by the recorder, analyzer, and tests."""

import math
from bisect import bisect_left
from statistics import fmean, median, pstdev


def safe_reduction(input_value, output_value):
    """Return saved, percent saved, and ratio without inventing infinities."""
    if input_value is None or output_value is None or input_value < 0 or output_value < 0:
        return None, None, None
    saved = input_value - output_value
    if input_value == 0:
        return saved, None, None
    percent = 100.0 * saved / input_value
    ratio = input_value / output_value if output_value else None
    return saved, percent, ratio


def percentile(values, probability):
    values = sorted(float(value) for value in values if value is not None and math.isfinite(value))
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return values[lower]
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def describe(values):
    clean = [float(value) for value in values if value is not None and math.isfinite(value)]
    if not clean:
        return {key: None for key in (
            'sample_count', 'mean', 'median', 'min', 'max', 'stddev',
            'p5', 'p50', 'p95', 'p99')}
    return {
        'sample_count': len(clean), 'mean': fmean(clean), 'median': median(clean),
        'min': min(clean), 'max': max(clean), 'stddev': pstdev(clean),
        'p5': percentile(clean, .05), 'p50': percentile(clean, .50),
        'p95': percentile(clean, .95), 'p99': percentile(clean, .99),
    }


def distance(position_a, position_b):
    if position_a is None or position_b is None:
        return None, None
    dx, dy, dz = (position_a[index] - position_b[index] for index in range(3))
    return math.sqrt(dx * dx + dy * dy + dz * dz), math.sqrt(dx * dx + dy * dy)


def nearest_pose(samples, timestamp_ns, max_age_ns):
    """Return the closest time-ordered pose if it is fresh enough."""
    if not samples:
        return None
    times = [sample['time_ns'] for sample in samples]
    index = bisect_left(times, timestamp_ns)
    candidates = samples[max(0, index - 1):min(len(samples), index + 1)]
    nearest = min(candidates, key=lambda sample: abs(sample['time_ns'] - timestamp_ns))
    age_ns = abs(nearest['time_ns'] - timestamp_ns)
    return (nearest, age_ns) if age_ns <= max_age_ns else None
