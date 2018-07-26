# Configuration:

```bash
  mkdir build
  cd build
  make -j4
```

add to `.bash_profile`
```
  export LIB_MYLIB_DIR="path_to_installation_dir"
```
#Note

 - v0.10.0 Remove the template for points. Some compatibility issues may occur.

#Developer

## Tests:

 - To make test pass use   `make_test_pass("pcl/pcl_00");`


## Compilation:

### Precompiler FLAGS:

- _MYLIB_USE_PCL_WITH_VTK
- _MYLIB_USE_PCL
- _MYLIB_USE_EIGEN3
