import os
import tempfile
import pytest

from numerixpy.utility.file_utils import file_exists, get_number_of_lines


def test_file_exists_true(tmp_path):
    f = tmp_path / "sample.txt"
    f.write_text("line1\nline2\n")
    assert file_exists(str(f)) is True


def test_file_exists_false():
    assert file_exists("/tmp/numerix_nonexistent_file_xyz.txt") is False


def test_get_number_of_lines(tmp_path):
    f = tmp_path / "lines.txt"
    f.write_text("a\nb\nc\n")
    assert get_number_of_lines(str(f)) == 3


def test_get_number_of_lines_single(tmp_path):
    f = tmp_path / "one.txt"
    f.write_text("only one line\n")
    assert get_number_of_lines(str(f)) == 1
