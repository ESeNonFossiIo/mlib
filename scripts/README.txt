
[![Build Status](https://travis-ci.org/ESeNonFossiIo/numerix.svg?branch=master)](https://travis-ci.org/freeCodeCamp/how-to-contribute-to-open-source)

# Configuration:

```bash
  mkdir build
  cd build
  make -j4
```

add to `.bash_profile`
```
  export LIBNUMERIX_DIR="path_to_installation_dir"
```
#Note

 - v0.10.0 Remove the template for points. Some compatibility issues may occur.

#Developer

## Tests:

 - To make test pass use   `make_test_pass("pcl/pcl_00");`

## Compilation:

### Precompiler FLAGS:

- NUMERIX_USE_PCL_WITH_VTK
- NUMERIX_USE_PCL
- NUMERIX_USE_EIGEN3
