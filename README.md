
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


# TODO

- [ ] compilare i test prima di runnarli in maniera automatica
- [x] script per generare i test ed i confronti
- [x] sistemare i test dopo l'aggiunta del namespace
- [x] Fix compile bug in point_04  [fix initializer_list list for 1 element]
- [x] namespace
- [x] implementare la gestione del README.md ogni volta che viene generata una nuova tag
- [x] make plot usare file ini
- [x] aggiungere todo al README.md
- [x] sistemare `make fix_tests***` nel caso in cui i test siano contenuti in una sottocartella
- [x] correggere VAR_m_ZERO_TOLLERANCE in VAR_MLIB_ZERO_TOLERANCE
- [x] sistemare i nomi dei tag
- [x] se non definito i test devono passare [provare stampando il file.output]
- [x] classe per gestire i log
- [ ] Cambiare la versione in doxygen
