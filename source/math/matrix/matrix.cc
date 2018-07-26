#include "mlib/math/matrix/matrix.h"

#include <iomanip>  // std::setprecision
#include <stdexcept>
#include <cmath>
#include <cassert>

namespace _mlib
{

  template<typename T>
  void Matrix<T>::
  range_check(size_t i, size_t j) const
  {
    if(rows <= i)
      {
#ifdef WITH_CPP11
        throw std::range_error(" n rows = " + std::to_string(rows) +
                               " : n row out of range");
#else
        throw std::range_error(" n row out of range");
#endif //WITH_CPP11
      }
    if(cols <= j)
      {
#ifdef WITH_CPP11
        throw std::range_error(" n cols = " + std::to_string(cols) +
                               " : n col out of range");
#else
        throw std::range_error(" n col out of range");
#endif //WITH_CPP11


      }
  }

  template<typename T>
  Matrix<T>::
  Matrix(const Matrix<T>& M)
    :
    rows(M.rows),
    cols(M.cols),
    elements(M.elements)
  {
  }

  template<typename T>
  Matrix<T>::
  Matrix(std::vector<std::vector<double>>& M)
  {
    rows = M.size();
    cols = M[0].size();
    elements.resize(M.size() * M[0].size(), T(0.0));
    for(unsigned int i = 0; i<rows; ++i)
      for(unsigned int j = 0; j<cols; ++j)
        elements[i*cols+j] = M[i][j];
  }

  template<typename T>
  Matrix<T>::
  Matrix(size_t rows, size_t cols, const T *elements)
    : rows(rows), cols(cols), elements(rows*cols,T(0.0))
  {
    if(rows == 0 || cols == 0)
      throw std::range_error("attempt to create a degenerate Matrix");
    // initialze from array
    if(elements)
      for(size_t i=0; i<rows*cols; i++)
        this->elements[i] = elements[i];
  }

  template<typename T>
  void
  Matrix<T>::
  resize(size_t rows_, size_t cols_, const T *elements_)
  {
    rows = rows_;
    cols = cols_;
    elements.resize(rows*cols,T(0.0));

    if(rows == 0 || cols == 0)
      throw std::range_error("attempt to create a degenerate Matrix");
    // initialze from array
    if(elements_)
      for(size_t i=0; i<rows*cols; i++)
        this->elements[i] = elements_[i];
  }

  template<typename T>
  Matrix<T>::
  Matrix(std::initializer_list<std::initializer_list<T>> list)
    :
    rows(list.size()),
    cols(list.begin()->size()),
    elements(list.size() *list.begin()->size(),T(0.0))
  {
    size_t i = 0;
    for(auto l : list)
      for(auto v : l)
        {
          elements[i] = v;
          i++;
        }
  }

  template<typename T>
  Matrix<T>::
  ~Matrix()
  {
  }

  template<typename T>
  T&
  Matrix<T>::
  operator[](size_t i)
  {
    return elements[i];
  }

  template<typename T>
  const T&
  Matrix<T>::
  operator[](size_t i) const
  {
    return elements[i];
  }

  template<typename T>
  const T&
  Matrix<T>::
  operator()(size_t i) const
  {
    return elements[i];
  }

  template<typename T>
  T&
  Matrix<T>::
  operator()(size_t i, size_t j)
  {
    range_check(i,j);
    return elements[i*cols+j];
  }


  template<typename T>
  const T&
  Matrix<T>::
  operator()(size_t i, size_t j) const
  {
    range_check(i,j);
    return elements[i*cols+j];
  }


  template<typename T>
  const T&
  Matrix<T>::
  element(size_t i, size_t j) const
  {
    range_check(i,j);
    return elements[i*cols+j];
  }

  template<typename T>
  T&
  Matrix<T>::
  element(size_t i, size_t j)
  {
    range_check(i,j);
    return elements[i*cols+j];
  }

  template<typename T>
  size_t
  Matrix<T>::
  r() const
  {
    return rows;
  }

  template<typename T>
  size_t
  Matrix<T>::
  c() const
  {
    return cols;
  }

  template<typename T>
  Matrix<T>
  Matrix<T>::
  c(const size_t& i) const
  {
    assert(i<cols);
    Matrix<T> new_matrix(rows,1);
    for(unsigned int j = 0; j < rows; ++j)
      new_matrix[j] = elements[i + cols*j];

    return new_matrix;
  }

  template<typename T>
  Matrix<T>
  Matrix<T>::
  r(const size_t& i) const
  {
    assert(i<rows);
    Matrix<T> new_matrix(1,cols);
    for(unsigned int j = 0; j < cols; ++j)
      new_matrix[j] = elements[j + cols*i];

    return new_matrix;
  }

  template<typename T>
  Matrix<T>&
  Matrix<T>::
  operator*= (const T& a)
  {
    for(unsigned int i = 0; i<elements.size(); ++i)
      this->elements[i] *= a;
    return *this;
  }

  template<typename T>
  Matrix<T>
  Matrix<T>::
  operator* (const T& a) const
  {
    return Matrix<T> (*this).operator*= (a);
  }

  template<typename T>
  Matrix<T>&
  Matrix<T>::
  operator/= (const T& a)
  {
    for(unsigned int i = 0; i<elements.size(); ++i)
      this->elements[i] /= a;
    return *this;
  }

  template<typename T>
  Matrix<T>
  Matrix<T>::
  operator/ (const T& a)
  {
    return Matrix<T> (*this).operator/= (a);
  }

  template<typename T>
  Matrix<T>&
  Matrix<T>::
  operator+= (const Matrix<T>& M)
  {
    for(unsigned int i=0; i<rows*cols; ++i)
      this->elements[i] += M.elements[i];
    return *this;
  }

  template<typename T>
  Matrix<T>&
  Matrix<T>::
  operator-= (const Matrix<T>& M)
  {
    for(unsigned int i=0; i<rows*cols; ++i)
      this->elements[i] -= M.elements[i];
    return *this;
  }

  template<typename T>
  Matrix<T>
  Matrix<T>::
  operator+ (const Matrix<T>& M) const
  {
    return Matrix<T> (*this).operator+= (M);
  }

  template<typename T>
  Matrix<T>
  Matrix<T>::
  operator- (const Matrix<T>& M) const
  {
    return Matrix<T> (*this).operator-= (M);
  }

  template<typename T>
  Matrix<T>
  Matrix<T>::
  operator* (const Matrix<T>& M) const
  {
    Matrix<T> output(rows, M.c());
    for(unsigned int i =0; i<rows; ++i)
      for(unsigned int j =0; j<M.c(); ++j)
        for(unsigned int k =0; k<cols; ++k)
          output.elements[i*M.c()+j] += this->elements[i*cols+k] *
                                        M.elements[k*M.c()+j];
    return output;
  }

  template<typename T>
  Matrix<T>&
  Matrix<T>::
  operator*= (const Matrix<T>& M)
  {
    return *this = *this * M;
  }

  template<typename T>
  Matrix<T>
  Matrix<T>::
  transpose() const
  {
    Matrix<T> output(cols, rows);
    for(unsigned int i =0; i<rows; ++i)
      for(unsigned int j =0; j<cols; ++j)
        output(j,i) = this->elements[i*cols+j];
    return output;
  }

  template<typename T>
  Matrix<T>
  Matrix<T>::
  t() const
  {
    return transpose();
  }

  template<typename T>
  T
  Matrix<T>::
  trace() const
  {
    T sum(0.0);
    for(size_t i = 0; i < rows; ++i)
      sum += elements[i*cols + i];
    return sum;
  }

  template<typename T>
  T
  Matrix<T>::
  l_inf_norm() const
  {
    T max_element(0.0);
    for(auto it = this->elements.begin();
        it!=this->elements.end(); ++it)
      if(*it **it > max_element * max_element)
        max_element = *it;
    return std::abs(max_element);
  }

  template<typename T>
  T
  Matrix<T>::
  l_2_norm() const
  {
    T sum(0.0);
    for(auto it = this->elements.begin();
        it!=this->elements.end(); ++it)
      sum += *it **it;
    return std::sqrt(sum);
  }


  template<typename T>
  T
  Matrix<T>::
  l_p_norm(const unsigned int& p) const
  {
    T sum(0.0);
    for(auto it = this->elements.begin();
        it!=this->elements.end(); ++it)
      sum += std::pow(std::abs(*it), p);
    return std::pow(sum, 1.0 / (1.0 * p));
  }

  template<typename T>
  T
  Matrix<T>::
  l_1_norm() const
  {
    T sum(0.0);
    for(auto it = this->elements.begin();
        it!=this->elements.end(); ++it)
      sum += std::abs(*it);
    return sum;
  }

  template<typename T>
  Matrix<T>
  Matrix<T>::
  cofactor(size_t i,size_t j) const
  {
    Matrix<T> cofactor(rows-1, cols-1);
    for(size_t a = 0; a+1 < rows; ++a)
      {
        for(size_t b = 0; b+1 < cols; ++b)
          {
            size_t i_pos = a < i ? a : a+1;
            size_t j_pos = b < j ? b : b+1;
            cofactor(a, b) = this->elements[i_pos * cols + j_pos];
          }
      }
    return cofactor;
  }

  template<typename T>
  T
  Matrix<T>::
  det() const
  {
    assert(cols == rows);
    if(cols < 2)
      {
        return this->elements[0];
      }
    else
      {
        T sum(0.0);
        int sign = -1;
        for(size_t i = 0; i < cols; ++i)
          {
            sign *= -1;
            auto cof = this->cofactor(0,i);
            sum += sign * this->elements[i] * cof.det();
          }
        return sum;
      }
  }

  template<typename T>
  Matrix<T>
  Matrix<T>::
  inv() const
  {
    assert(cols == rows);
    auto det = this->det();
    // assert(std::abs(det) > VAR_MLIB_ZERO_TOLERANCE);

    Matrix<T> inverse(rows, cols);

    T sum(0.0);
    int sign = -1;
    for(size_t i = 0; i < rows; ++i)
      for(size_t j = 0; j < cols; ++j)
        {
          auto cof = this->cofactor(i,j);
          inverse(i,j) = std::pow(-1, i+j) * cof.det() / det;
        }
    return inverse.t();
  }

  template<typename T>
  Matrix<T>
  Matrix<T>::
  operator~() const
  {
    return transpose();
  }

  template<typename T>
  Matrix<T>&
  Matrix<T>::
  operator= (const Matrix<T>& M)
  {
    this->resize(M.rows, M.cols);
    for(size_t i=0; i<rows*cols; i++)
      elements[i] = M.elements[i];
    return *this;
  }

// Friend class
////////////////////////////////////////////////////////////////////////////////
  template<typename S>
  std::ostream&
  operator<< (std::ostream& output, const Matrix<S>& M)
  {
#ifndef WINDOWS
    std::string bar = std::string(13*M.c() + M.c()+1, '-');
    output << std::setprecision(5) << std::endl;
    for(unsigned int i = 0; i< M.r(); ++i)
      {
        output << " " << bar << std::endl;
        output << " | ";
        for(unsigned int j = 0; j< M.c(); ++j)
          output << std::setw(11) << M(i,j) << " | ";

        output << std::endl;
      }
    output <<  " " << bar << std::endl;
#endif
    return output;
  }

  template<typename S>
  Matrix<S>
  operator* (const double& a, const Matrix<S>& M)
  {
    return M*a;
  }

// identityMatrix
////////////////////////////////////////////////////////////////////////////////
  IdentityMatrix::
  IdentityMatrix(size_t size)
    :
    Matrix<double> (size, size)
  {
    for(unsigned int i = 0; i<size; ++i)
      this->elements[i + i*size] = 1.0;
  }


// Explicit instantiation
////////////////////////////////////////////////////////////////////////////////
  template class Matrix<double>;
  template std::ostream& operator<< (std::ostream&,
                                     const Matrix<double>&);
  template Matrix<double> operator* (const double&,
                                     const Matrix<double>&);

}
