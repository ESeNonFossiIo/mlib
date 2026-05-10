from ctypes import POINTER, byref, c_double, c_uint64
from enum import IntEnum

from mlibpy.bind.load_symbols import evaluateFunction


class OptionType(IntEnum):
    """Option contract type for Black-Scholes pricing.

    .. addedversion:: mlib_version_placeholder
    """

    #: The option is a European call option, which gives the holder the right to buy the underlying asset at the strike price on the expiration date.
    EUROPEAN_CALL = 0
    #: The option is a European put option, which gives the holder the right to sell the underlying asset at the strike price on the expiration date.
    EUROPEAN_PUT = 1


def bs_price(
    S: float,
    K: float,
    r: float,
    v: float,
    T: float,
    opt_type: OptionType,
) -> float:
    """Price a European option using the Black-Scholes model via the C++ binding.

    .. addedversion:: mlib_version_placeholder

    Args:
        S (float): Current underlying price.
        K (float): Strike price.
        r (float): Continuously compounded risk-free rate.
        v (float): Annualised volatility (sigma).
        T (float): Time to maturity in years.
        opt_type (OptionType): EUROPEAN_CALL or EUROPEAN_PUT.

    Returns:
        float: Option price in the same currency unit as S and K.
    """
    price = c_double(0.0)
    evaluateFunction(
        "BSPricer",
        [c_double, c_double, c_double, c_double, c_double, c_uint64, POINTER(c_double)],
        [S, K, r, v, T, int(opt_type), byref(price)],
        c_uint64,
    )
    return price.value
