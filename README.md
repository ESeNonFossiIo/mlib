
[![OSX](https://github.com/ESeNonFossiIo/mlib/actions/workflows/osx.yml/badge.svg?branch=master)](https://github.com/ESeNonFossiIo/mlib/actions/workflows/osx.yml)
[![linux](https://github.com/ESeNonFossiIo/mlib/actions/workflows/linux.yml/badge.svg?branch=master)](https://github.com/ESeNonFossiIo/mlib/actions/workflows/linux.yml)
[![windows](https://github.com/ESeNonFossiIo/mlib/actions/workflows/windows.yml/badge.svg?branch=master)](https://github.com/ESeNonFossiIo/mlib/actions/workflows/windows.yml)

# Build:
```bash
  mkdir build && cd build
  cmake -D ENABLE_ALL_TESTS=ON ..
  make -j4
  ctest --output-on-failure
```

# Configuration:
```bash
  mkdir build
  cd build
  make -j4
```

add to `.bash_profile`
```
  export LIBMLIB_DIR="path_to_installation_dir"
```
#Note

 - v0.10.0 Remove the template for points. Some compatibility issues may occur.

#Developer

## Tests:

 - To make test pass use   `make_test_pass("pcl/pcl_00");`

## Compilation:

### Precompiler FLAGS:

- MLIB_USE_PCL_WITH_VTK
- MLIB_USE_PCL
- MLIB_USE_EIGEN3
- BUILD_PY_ENV

# ChangeLog

## Version 0
|TAG|COMMIT|
|---|------|
|v0.1.0|Add Travis|
|v0.0.0|First Tag|

# TODO
- [ ] compilare i test prima di runnarli in maniera automatica
- [ ] Cambiare la versione in doxygen
- [ ] Add guard for python
- [ ] python tests
- [ ] python doc
- [ ] python syntax
- [ ] enviroment
- [ ] macro run python
