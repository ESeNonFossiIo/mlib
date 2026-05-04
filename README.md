
[![CodeQL](https://github.com/ESeNonFossiIo/mlib/actions/workflows/codeql.yml/badge.svg)](https://github.com/ESeNonFossiIo/mlib/actions/workflows/codeql.yml)

[![Linux Ubuntu (Debug)](https://github.com/ESeNonFossiIo/mlib/actions/workflows/linux-ubuntu_debug.yml)](https://github.com/ESeNonFossiIo/mlib/actions/workflows/linux-ubuntu_debug.yml)
[![Linux Ubuntu (Release)](https://github.com/ESeNonFossiIo/mlib/actions/workflows/linux-ubuntu_release.yml)](https://github.com/ESeNonFossiIo/mlib/actions/workflows/linux-ubuntu_release.yml)
[![OSX](https://github.com/ESeNonFossiIo/mlib/actions/workflows/osx.yml/badge.svg?branch=master)](https://github.com/ESeNonFossiIo/mlib/actions/workflows/osx.yml)
[![Windows (Release)](https://github.com/ESeNonFossiIo/mlib/actions/workflows/windows_release.yml)](https://github.com/ESeNonFossiIo/mlib/actions/workflows/windows_release.yml)


# Build

## Quick Start
```bash
mkdir -p build && cd build
cmake -D CMAKE_BUILD_TYPE=Release -D ENABLE_ALL_TESTS=ON ..
make -j4
ctest --output-on-failure
```

## Build Options
- `-D CMAKE_BUILD_TYPE=Release` - Optimized build (or Debug for debugging symbols)
- `-D ENABLE_ALL_TESTS=ON` - Compile all test suite

## VS Code Integration
This project includes pre-configured VS Code tasks and debug configurations:

- **Build (Ctrl+Shift+B)** - Configures and builds the project
- **Debug (F5)** - Builds with debug symbols and launches debugger
- Build type selection (Release/Debug) happens via VS Code prompt

See `.vscode/tasks.json` and `.vscode/launch.json` for details.

# Configuration

```bash
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D ENABLE_ALL_TESTS=ON ..
make -j4
```

Add to `.bash_profile`:
```bash
export LIBMLIB_DIR="path_to_installation_dir"
```

# Notes

- v0.10.0 Remove the template for points. Some compatibility issues may occur.
- All 121 unit tests passing with GCC C++15 and above
- Cross-platform support: Linux, macOS, Windows

# Developer Guide

## Tests

- To make test pass use `make_test_pass("pcl/pcl_00");`
- Run specific test: `ctest -R test_name --output-on-failure`
- Run all tests: `ctest --output-on-failure`
- Rerun failed tests: `ctest --rerun-failed --output-on-failure`

## Compilation

### Precompiler FLAGS

- `MLIB_USE_PCL_WITH_VTK` - PCL with VTK support
- `MLIB_USE_PCL` - Point Cloud Library support
- `MLIB_USE_EIGEN3` - Eigen3 linear algebra
- `BUILD_PY_ENV` - Python bindings

### Recent Improvements (May 2026)

#### Code Quality
- Added comprehensive Doxygen documentation to Complex, Logger, Huffman, and INI classes
- Implemented C++11 default/delete semantics for constructors
- Added `inline` keywords to template methods
- Implemented move semantics where appropriate

#### Fixes
- Fixed Windows/Linux portability issue in logger.cc (sprintf_s → snprintf)
- Fixed unused return value warning in ini.cc (std::remove_if)
- Added missing headers to huffman.h
- Fixed deprecation warning in complex.h

# ChangeLog

## Version 0
|TAG|COMMIT|DATE|NOTES|
|---|------|----|-----|
|v0.1.0|Add Travis|2026-03-XX|Continuous Integration|
|v0.0.0|First Tag|2026-01-XX|Initial Release|

# TODO
- [ ] compilare i test prima di runnarli in maniera automatica
- [ ] Cambiare la versione in doxygen
- [ ] Add guard for python
- [ ] python tests
- [ ] python doc
- [ ] python syntax
- [ ] environment
- [ ] macro run python
