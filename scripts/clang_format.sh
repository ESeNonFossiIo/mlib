#!/usr/bin/env bash
# Apply or check clang-format across the repository.
#
# Usage:
#   scripts/clang_format.sh           # rewrite files in place
#   scripts/clang_format.sh --check   # exit non-zero if any file would change
#                                     # (prints unified diff, does not modify)

set -euo pipefail

CLANG_FORMAT="${CLANG_FORMAT:-clang-format}"

if ! command -v "${CLANG_FORMAT}" >/dev/null 2>&1; then
    echo "error: ${CLANG_FORMAT} not found in PATH" >&2
    exit 127
fi

REPO_ROOT="$(git -C "$(dirname "$0")/.." rev-parse --show-toplevel)"
cd "${REPO_ROOT}"

# Directories that contain hand-written C/C++ sources tracked in git.
TARGET_DIRS=(source include tests main gui python/src)

mapfile -t FILES < <(
    git ls-files -- \
        "${TARGET_DIRS[@]/%/\/*.cc}" \
        "${TARGET_DIRS[@]/%/\/*.cpp}" \
        "${TARGET_DIRS[@]/%/\/*.h}" \
        "${TARGET_DIRS[@]/%/\/*.hpp}" \
        2>/dev/null | sort -u
)

if [[ ${#FILES[@]} -eq 0 ]]; then
    echo "No C/C++ files found under: ${TARGET_DIRS[*]}" >&2
    exit 0
fi

mode="apply"
if [[ "${1:-}" == "--check" ]]; then
    mode="check"
fi

if [[ "${mode}" == "apply" ]]; then
    printf 'Formatting %d files...\n' "${#FILES[@]}"
    printf '%s\0' "${FILES[@]}" | xargs -0 -P "$(nproc 2>/dev/null || echo 4)" "${CLANG_FORMAT}" -i
    echo "Done."
    exit 0
fi

# --check: produce a unified diff without modifying files.
fail=0
for f in "${FILES[@]}"; do
    if ! diff -u "${f}" <("${CLANG_FORMAT}" "${f}") > /tmp/clang_format_diff.$$ 2>/dev/null; then
        if [[ -s /tmp/clang_format_diff.$$ ]]; then
            echo "::error file=${f}::clang-format would reformat this file"
            sed "s|^--- ${f}|--- a/${f}|; s|^+++ -|+++ b/${f}|" /tmp/clang_format_diff.$$
            fail=1
        fi
    fi
done
rm -f /tmp/clang_format_diff.$$

if [[ ${fail} -ne 0 ]]; then
    echo
    echo "Some files do not match clang-format. Run: scripts/clang_format.sh"
    exit 1
fi
echo "All files conform to clang-format."
