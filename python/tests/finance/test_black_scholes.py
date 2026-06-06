"""Tests for the Black-Scholes Python binding.

Canonical parameters used throughout:
  S=100, K=100 (ATM), r=5%, σ=20%, T=1 year
  → call ≈ 10.45,  put ≈ 5.57
"""

import math

from numerixpy.finance.black_scholes import OptionType, bs_price

# ---------------------------------------------------------------------------
# Canonical parameters
# ---------------------------------------------------------------------------
_S = 100.0
_K = 100.0
_r = 0.05
_v = 0.20
_T = 1.0


# ---------------------------------------------------------------------------
# Spot-price tests
# ---------------------------------------------------------------------------


def test_call_price_atm():
    """ATM European call should be approximately 10.45."""
    price = bs_price(_S, _K, _r, _v, _T, OptionType.EUROPEAN_CALL)
    assert abs(price - 10.45) < 0.1


def test_put_price_atm():
    """ATM European put should be approximately 5.57."""
    price = bs_price(_S, _K, _r, _v, _T, OptionType.EUROPEAN_PUT)
    assert abs(price - 5.57) < 0.1


# ---------------------------------------------------------------------------
# Arbitrage identity
# ---------------------------------------------------------------------------


def test_put_call_parity():
    """C - P = S - K·e^(-rT) must hold up to floating-point precision."""
    call = bs_price(_S, _K, _r, _v, _T, OptionType.EUROPEAN_CALL)
    put = bs_price(_S, _K, _r, _v, _T, OptionType.EUROPEAN_PUT)
    expected = _S - _K * math.exp(-_r * _T)
    assert abs((call - put) - expected) < 1e-6


# ---------------------------------------------------------------------------
# Monotonicity in the underlying
# ---------------------------------------------------------------------------


def test_call_increases_with_spot():
    """Call is monotone increasing in the underlying price."""
    low = bs_price(90.0, _K, _r, _v, _T, OptionType.EUROPEAN_CALL)
    high = bs_price(110.0, _K, _r, _v, _T, OptionType.EUROPEAN_CALL)
    assert low < high


def test_put_decreases_with_spot():
    """Put is monotone decreasing in the underlying price."""
    low = bs_price(90.0, _K, _r, _v, _T, OptionType.EUROPEAN_PUT)
    high = bs_price(110.0, _K, _r, _v, _T, OptionType.EUROPEAN_PUT)
    assert low > high


# ---------------------------------------------------------------------------
# Monotonicity in volatility (positive vega)
# ---------------------------------------------------------------------------


def test_call_increases_with_volatility():
    """Call price is monotone increasing in volatility (positive vega)."""
    low = bs_price(_S, _K, _r, 0.10, _T, OptionType.EUROPEAN_CALL)
    high = bs_price(_S, _K, _r, 0.30, _T, OptionType.EUROPEAN_CALL)
    assert low < high


def test_put_increases_with_volatility():
    """Put price is monotone increasing in volatility (positive vega)."""
    low = bs_price(_S, _K, _r, 0.10, _T, OptionType.EUROPEAN_PUT)
    high = bs_price(_S, _K, _r, 0.30, _T, OptionType.EUROPEAN_PUT)
    assert low < high


# ---------------------------------------------------------------------------
# Boundary behaviour
# ---------------------------------------------------------------------------


def test_deep_itm_call_approaches_intrinsic():
    """Deep ITM call price converges to S - K·e^(-rT)."""
    price = bs_price(200.0, _K, _r, _v, _T, OptionType.EUROPEAN_CALL)
    intrinsic = 200.0 - _K * math.exp(-_r * _T)
    assert price > 0.99 * intrinsic


def test_deep_otm_call_near_zero():
    """Deep OTM call (S ≪ K) has negligible price."""
    price = bs_price(50.0, _K, _r, _v, _T, OptionType.EUROPEAN_CALL)
    assert price < 0.1


def test_deep_itm_put_approaches_intrinsic():
    """Deep ITM put (S ≪ K) converges to K·e^(-rT) - S."""
    price = bs_price(20.0, _K, _r, _v, _T, OptionType.EUROPEAN_PUT)
    intrinsic = _K * math.exp(-_r * _T) - 20.0
    assert price > 0.99 * intrinsic


def test_deep_otm_put_near_zero():
    """Deep OTM put (S ≫ K) has negligible price."""
    price = bs_price(200.0, _K, _r, _v, _T, OptionType.EUROPEAN_PUT)
    assert price < 0.1
