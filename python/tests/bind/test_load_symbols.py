import pytest

import mlibpy.bind.load_symbols as ls


def _force_real_load():
    """Ensure the real library handle is cached on loadMLIB."""
    ls.loadMLIB()


def test_loadmlib_returns_cached_handle():
    """The second call returns the same cached library handle."""
    first = ls.loadMLIB()
    second = ls.loadMLIB()
    assert first is second


@pytest.mark.parametrize("fake_platform", ["darwin", "win32", "freebsd"])
def test_loadmlib_non_linux_platform(fake_platform):
    """Non-Linux platforms pick a different (or empty) extension and fall back
    to the bare library name; loading that file then fails on this Linux host.
    'freebsd' also exercises the fall-through past every platform branch."""
    _force_real_load()
    saved_platform = ls.platform
    saved_cache = ls.loadMLIB.mlib

    del ls.loadMLIB.mlib
    ls.platform = fake_platform
    try:
        with pytest.raises(OSError):
            ls.loadMLIB()
    finally:
        ls.platform = saved_platform
        ls.loadMLIB.mlib = saved_cache


def test_evaluate_function_without_ret_type():
    """evaluateFunction with ret_type omitted skips the restype assignment."""
    result = ls.evaluateFunction("version_major", [], [])
    assert isinstance(result, int)
