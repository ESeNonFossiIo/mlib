from unittest import mock

import pytest

import numerixpy.bind.load_symbols as ls


def test_loadnumerix_returns_cached_handle():
    """The first call loads the library; the second returns the cached handle."""
    first = ls.loadNUMERIX()
    second = ls.loadNUMERIX()
    assert first is second


@pytest.mark.parametrize(
    "fake_platform,expected_ext",
    [
        ("darwin", "dylib"),
        ("win32", "dll"),
        ("freebsd", ""),
    ],
)
def test_loadnumerix_platform_selects_extension(fake_platform, expected_ext):
    """loadNUMERIX derives the library extension from sys.platform and, when no
    candidate file is found, falls back to the bare 'lib_numerix_bindings.<ext>'.

    'freebsd' is an unrecognised platform: it exercises the fall-through past
    every platform branch (lib_ext stays empty).

    The test is hermetic: sys.platform, isfile and cdll are all mocked, so it
    neither touches the filesystem nor depends on which libraries are present.
    """
    fake_cdll = mock.Mock()

    # loadNUMERIX caches the handle on the function object; drop it so the body
    # re-runs, and drop it again afterwards so later real calls reload cleanly.
    if hasattr(ls.loadNUMERIX, "numerix"):
        del ls.loadNUMERIX.numerix

    with mock.patch.object(ls, "platform", fake_platform), mock.patch.object(
        ls, "isfile", return_value=False
    ), mock.patch.object(ls, "cdll", fake_cdll):
        try:
            handle = ls.loadNUMERIX()
        finally:
            if hasattr(ls.loadNUMERIX, "numerix"):
                del ls.loadNUMERIX.numerix

    assert handle is fake_cdll.LoadLibrary.return_value
    fake_cdll.LoadLibrary.assert_called_once()
    loaded_path = fake_cdll.LoadLibrary.call_args[0][0]
    assert loaded_path.endswith("lib_numerix_bindings." + expected_ext)


def test_evaluate_function_without_ret_type():
    """evaluateFunction with ret_type omitted skips the restype assignment."""
    result = ls.evaluateFunction("version_major", [], [])
    assert isinstance(result, int)
