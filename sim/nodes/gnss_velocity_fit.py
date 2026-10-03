"""Local generalized least-squares velocity observations from GNSS positions."""

import math

import numpy as np


def fit_position_velocity(samples, window_s=8.0, min_samples=6, min_span_s=6.0,
                          outlier_mahalanobis_sq=16.27):
    """Fit a 3D constant-velocity model to ``(stamp, position, covariance)`` fixes.

    Position covariances are treated as known measurement covariances. At most
    one fix is removed, using its normalized residual, then the GLS fit is run
    again. Returns ``None`` with a reason when the history is not usable.
    """
    if window_s <= 0 or min_samples < 3 or min_span_s < 0:
        raise ValueError("invalid GNSS velocity fit configuration")
    valid = []
    previous_stamp = -math.inf
    for stamp, position, covariance in samples:
        stamp = float(stamp)
        p = np.asarray(position, dtype=float).reshape(3)
        R = np.asarray(covariance, dtype=float).reshape(3, 3)
        if not math.isfinite(stamp) or stamp <= previous_stamp:
            return {"reason": "timestamps_not_strictly_increasing"}
        previous_stamp = stamp
        if not np.all(np.isfinite(p)) or not np.all(np.isfinite(R)):
            return {"reason": "non_finite_measurement"}
        R = 0.5 * (R + R.T)
        eig = np.linalg.eigvalsh(R)
        if eig[0] <= 1e-9 or eig[-1] > 1e4:
            return {"reason": "invalid_position_covariance"}
        valid.append((stamp, p, R))

    if len(valid) < min_samples:
        return {"reason": "insufficient_samples", "sample_count": len(valid)}
    newest = valid[-1][0]
    selected = [s for s in valid if newest - s[0] <= window_s + 1e-9]
    if len(selected) < min_samples:
        return {"reason": "insufficient_samples_in_window", "sample_count": len(selected)}
    span = selected[-1][0] - selected[0][0]
    if span < min_span_s:
        return {"reason": "insufficient_time_span", "sample_count": len(selected), "span_s": span}

    def solve(fixes):
        times = np.asarray([item[0] for item in fixes])
        tref = float(np.mean(times))
        A = np.zeros((3 * len(fixes), 6))
        y = np.concatenate([item[1] for item in fixes])
        W = np.zeros((3 * len(fixes), 3 * len(fixes)))
        for k, (stamp, _, covariance) in enumerate(fixes):
            block = slice(3 * k, 3 * k + 3)
            A[block, :3] = np.eye(3)
            A[block, 3:] = np.eye(3) * (stamp - tref)
            W[block, block] = np.linalg.inv(covariance)
        normal = A.T @ W @ A
        try:
            parameter_covariance = np.linalg.inv(normal)
            parameters = parameter_covariance @ A.T @ W @ y
        except np.linalg.LinAlgError:
            return None
        residuals = [item[1] - A[3*k:3*k+3] @ parameters for k, item in enumerate(fixes)]
        distances = [float(r @ np.linalg.solve(item[2], r)) for r, item in zip(residuals, fixes)]
        return parameters, parameter_covariance, residuals, distances

    result = solve(selected)
    if result is None:
        return {"reason": "singular_fit", "sample_count": len(selected), "span_s": span}
    rejected = []
    distances = result[3]
    worst = int(np.argmax(distances))
    if len(selected) > min_samples and distances[worst] > outlier_mahalanobis_sq:
        rejected.append(selected[worst][0])
        selected = selected[:worst] + selected[worst + 1:]
        result = solve(selected)
        if result is None:
            return {"reason": "singular_fit_after_outlier_rejection", "sample_count": len(selected)}

    parameters, parameter_covariance, residuals, _ = result
    cov_v = 0.5 * (parameter_covariance[3:, 3:] + parameter_covariance[3:, 3:].T)
    residual_rms = float(np.sqrt(np.mean(np.concatenate(residuals) ** 2)))
    return {
        "velocity": parameters[3:].copy(),
        "covariance": cov_v,
        "sample_count": len(selected),
        "span_s": selected[-1][0] - selected[0][0],
        "residual_rms_m": residual_rms,
        "rejected_stamps": rejected,
        "reason": "ok",
    }
