from ctypes import addressof, create_string_buffer

from numerixpy.bind.types import from_ccharp_to_str


def test_from_ccharp_to_str_ascii():
    """Decode a raw C string pointer holding ASCII text."""
    buf = create_string_buffer(b"numerix")
    assert from_ccharp_to_str(addressof(buf)) == "numerix"


def test_from_ccharp_to_str_utf8():
    """Decode a raw C string pointer holding multi-byte UTF-8 text."""
    buf = create_string_buffer("café".encode("utf-8"))
    assert from_ccharp_to_str(addressof(buf)) == "café"


def test_from_ccharp_to_str_empty():
    """An empty C string decodes to the empty Python string."""
    buf = create_string_buffer(b"")
    assert from_ccharp_to_str(addressof(buf)) == ""
