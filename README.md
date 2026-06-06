# numerix

[![CodeQL](https://github.com/ESeNonFossiIo/numerix/actions/workflows/codeql.yml/badge.svg?branch=master)](https://github.com/ESeNonFossiIo/numerix/actions/workflows/codeql.yml)
[![Ubuntu](https://github.com/ESeNonFossiIo/numerix/actions/workflows/linux-ubuntu.yml/badge.svg?branch=master)](https://github.com/ESeNonFossiIo/numerix/actions/workflows/linux-ubuntu.yml)
[![Fedora](https://github.com/ESeNonFossiIo/numerix/actions/workflows/linux-fedora.yml/badge.svg?branch=master)](https://github.com/ESeNonFossiIo/numerix/actions/workflows/linux-fedora.yml)
[![OSX](https://github.com/ESeNonFossiIo/numerix/actions/workflows/osx.yml/badge.svg?branch=master)](https://github.com/ESeNonFossiIo/numerix/actions/workflows/osx.yml)
[![Windows](https://github.com/ESeNonFossiIo/numerix/actions/workflows/windows.yml/badge.svg?branch=master)](https://github.com/ESeNonFossiIo/numerix/actions/workflows/windows.yml)
[![Style](https://github.com/ESeNonFossiIo/numerix/actions/workflows/style-check.yml/badge.svg?branch=master)](https://github.com/ESeNonFossiIo/numerix/actions/workflows/style-check.yml)

[![C++ Coverage](https://raw.githubusercontent.com/ESeNonFossiIo/numerix/badges/coverage-cpp.svg)](https://github.com/ESeNonFossiIo/numerix/actions/workflows/coverage.yml)
[![Python Coverage](https://raw.githubusercontent.com/ESeNonFossiIo/numerix/badges/coverage-python.svg)](https://github.com/ESeNonFossiIo/numerix/actions/workflows/coverage.yml)

## Build

### Quick Start

```bash
mkdir -p build && cd build
cmake -D CMAKE_BUILD_TYPE=Release -D ENABLE_ALL_TESTS=ON ..
ninja -j4
ctest --output-on-failure
```

### Build Options

- `-D CMAKE_BUILD_TYPE=Release` - Optimized build (or Debug for debugging symbols)
- `-D ENABLE_ALL_TESTS=ON` - Compile all test suite

### VS Code Integration

This project includes pre-configured VS Code tasks and debug configurations:

- **Build (Ctrl+Shift+B)** - Configures and builds the project
- **Debug (F5)** - Builds with debug symbols and launches debugger
- Build type selection (Release/Debug) happens via VS Code prompt

See `.vscode/tasks.json` and `.vscode/launch.json` for details.

## Configuration

```bash
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D ENABLE_ALL_TESTS=ON ..
ninja -j4
```

Add to `.bash_profile`:

```bash
export LIBNUMERIX_DIR="path_to_installation_dir"
```

## Notes

- v0.10.0 Remove the template for points. Some compatibility issues may occur.
- All 121 unit tests passing with GCC C++15 and above
- Cross-platform support: Linux, macOS, Windows

## Developer Guide

### Tests

- To make test pass use `make_test_pass("pcl/pcl_00");`
- Run specific test: `ctest -R test_name --output-on-failure`
- Run all tests: `ctest --output-on-failure`
- Rerun failed tests: `ctest --rerun-failed --output-on-failure`

### Code formatting

The project uses **clang-format** for C/C++ (`.clang-format`) and **Black** for
Python. The `Style` GitHub Action enforces these in CI.

#### Manual formatting

Apply formatting to all tracked C/C++ files:
```bash
./scripts/clang_format.sh
```

Check only (no files modified; exits non-zero if any file is out of sync):
```bash
./scripts/clang_format.sh --check
```

Use CMake targets for convenience:
```bash
make clang_format   # reformat in-place
make check_format   # check only (CI uses this)
```

#### Pre-commit hooks (optional but recommended)

Install local git hooks to auto-format on every commit:

```bash
pip install pre-commit
pre-commit install
```

Then on the next `git commit`:

- **clang-format** will check/reformat staged C/C++ files
- **Black** will check/reformat staged Python files
- If changes are made, the commit is blocked; review and re-stage with `git add`, then commit again

Run hooks manually (without committing):
```bash
pre-commit run --all-files           # check all files
pre-commit run clang-format --all-files
pre-commit run black --all-files
```

### Compilation

#### Precompiler FLAGS

- `NUMERIX_USE_PCL_WITH_VTK` - PCL with VTK support
- `NUMERIX_USE_PCL` - Point Cloud Library support
- `NUMERIX_USE_EIGEN3` - Eigen3 linear algebra
- `BUILD_PY_ENV` - Python bindings

#### Recent Improvements (May 2026)

##### Code Quality

- Added comprehensive Doxygen documentation to Complex, Logger, Huffman, and INI classes
- Implemented C++11 default/delete semantics for constructors
- Added `inline` keywords to template methods
- Implemented move semantics where appropriate

##### Fixes

- Fixed Windows/Linux portability issue in logger.cc (sprintf_s → snprintf)
- Fixed unused return value warning in ini.cc (std::remove_if)
- Added missing headers to huffman.h
- Fixed deprecation warning in complex.h

## ChangeLog

### Version 0

| TAG    | COMMIT     | DATE       | NOTES                  |
|--------|------------|------------|------------------------|
| v0.1.0 | Add Travis | 2026-03-XX | Continuous Integration |
| v0.0.0 | First Tag  | 2026-01-XX | Initial Release        |

## TODO

- [ ] compilare i test prima di runnarli in maniera automatica
- [ ] Cambiare la versione in doxygen
- [ ] Add guard for python
- [ ] python tests
- [ ] python doc
- [ ] python syntax
- [ ] environment
- [ ] macro run python
