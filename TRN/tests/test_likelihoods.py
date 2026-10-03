"""Measurement likelihoods."""
import math

import numpy as np

from trn.filters.likelihoods import loglik_baseline, loglik_proposed

P = dict(pvis=0.8, lam_s=0.5, w_can=0.5, l_can=15.0, lam_c=0.005, rmax=5000.0, rmin=1.0, r50=6500.0, wd=700.0, pmax=1.0)


def prop(ranges, rho, sig=5.0):
    rho = np.atleast_1d(np.asarray(rho, float))[:, None]
    out = np.empty(rho.shape[0]); pg = np.empty(rho.shape)
    loglik_proposed(np.asarray(ranges, float)[None, :], rho, np.full(rho.shape, sig), *P.values(), out, pg)
    return out, pg[:, 0]


def test_baseline_gaussian_last_echo():
    out = np.empty(2)
    ranges = np.array([[1500.0, 2000.0, np.nan]])
    loglik_baseline(ranges, np.array([[2000.0], [2010.0]]), np.full((2, 1), 5.0), np.array([True]), out)
    assert out[0] - out[1] == np.float64(0.5 * 4.0)


def test_baseline_skips_missing():
    out = np.empty(1)
    loglik_baseline(np.full((1, 3), np.nan), np.array([[2000.0]]), np.full((1, 1), 5.0), np.array([True]), out)
    assert out[0] == 0.0


def test_proposed_peaks_at_ground_echo():
    rho = np.linspace(1900, 2100, 201)
    ll, pg = prop([2000.0, np.nan, np.nan], rho)
    assert abs(rho[np.argmax(ll)] - 2000.0) <= 2.0
    assert pg[100] > 0.9


def test_proposed_cloud_echo_is_harmless():
    """A single short (cloud) echo far above the ground carries almost no position preference."""
    ll, pg = prop([800.0, np.nan, np.nan], np.array([1990.0, 2000.0, 2010.0]))
    assert np.ptp(ll) < 0.05
    assert pg.max() < 0.05


def test_proposed_echo_beyond_ground_penalised():
    """An echo longer than the predicted ground cannot be cloud/canopy -> particle predicting short range loses."""
    ll, _ = prop([2000.0, np.nan, np.nan], np.array([1950.0, 2050.0]))
    assert ll[0] < ll[1] - 1.0


def test_proposed_no_return_depends_only_on_pd():
    ll, _ = prop([np.nan] * 3, np.array([1000.0, 2000.0]))
    assert math.isclose(math.exp(ll[0]), 1 - 0.8 / (1 + math.exp((1000 - 6500) / 700)), rel_tol=1e-9)


def test_proposed_cloud_plus_ground_uses_ground():
    rho = np.linspace(1950, 2050, 101)
    ll, pg = prop([700.0, 2000.0, np.nan], rho)
    assert abs(rho[np.argmax(ll)] - 2000.0) <= 2.0
