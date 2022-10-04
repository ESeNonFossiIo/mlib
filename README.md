
[![Build Status](https://travis-ci.org/ESeNonFossiIo/mlib.svg?branch=master)](https://travis-ci.org/freeCodeCamp/how-to-contribute-to-open-source)

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
