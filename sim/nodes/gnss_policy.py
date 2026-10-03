"""Pure policy functions shared by the ROS GNSS gate and tests."""


def gnss_available(cutoff_s: float, elapsed_s: float) -> bool:
    """Negative cutoff disables denial; otherwise fixes are valid strictly before elapsed cutoff."""
    return cutoff_s < 0 or elapsed_s < cutoff_s
