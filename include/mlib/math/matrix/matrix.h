#ifndef  __m_MATRIX_H__
#define  __m_MATRIX_H__

#include <vector>
#include <iostream>
#include <initializer_list>  // std::initializer_list

/** \addtogroup math
 *  @{
 */
namespace mlib
{
  template<typename T = double>
  class Matrix
  {
  public:
    /**
     * constructor
     */
    Matrix(size_t rows, size_t cols, const T *elements = 0);

    /**
     * constructor
     */
    Matrix(std::vector<std::vector<double>>&);

    /**
     * constructor
     */
    Matrix(std::initializer_list<std::initializer_list<T>>
           list);

    /**
     * constructor
     */
    Matrix(const Matrix<T>&);

    /**
     * constructor
     */
    Matrix()
      :
      elements(),
      rows(0),
      cols(0)
    {}

    /**
     * destructor
     */
    ~Matrix();

    /**
    *
    */
    void
    resize(size_t rows_, size_t cols_, const T *elements_ = 0);

    /**
     * Return the number of coloumns.
     * @return number of coloumns.
     */
    size_t r() const;

    /**
     * Return the number of rows.
     * @return number of rows.
     */
    size_t c() const;

    /**
     * Return the number of rows.
     * @return number of rows.
     */
    Matrix<T> c(const size_t& i) const;

    /**
     * Return the number of coloumns.
     * @return number of coloumns.
     */
    Matrix<T>  r(const size_t& i) const;

    /**
     * [operator[] description]
     * @param  i [description]
     * @return   [description]
     */
    T& operator[](size_t i);

    /**
     * [operator[] description]
     * @param  i [description]
     * @return   [description]
     */
    const T& operator[](size_t i) const;

    /**
     * [operator() description]
     * @param  i [description]
     * @return   [description]
     */
    const T& operator()(size_t i) const;

    /**
     * [operator description]
     * @return [description]
     */
    T& operator()(size_t i, size_t j);

    /**
     * [operator description]
     * @return [description]
     */
    const T& operator()(size_t i, size_t j) const;

    /**
     * [element description]
     * @param  i [description]
     * @param  j [description]
     * @return   [description]
     */
    const T& element(size_t i, size_t j) const;

    /**
     * [element description]
     * @param  i [description]
     * @param  j [description]
     * @return   [description]
     */
    T& element(size_t i, size_t j);

  protected:
    /**
     * [range_check description]
     * @param i [description]
     * @param j [description]
     */
    void range_check(size_t i, size_t j) const;

    /**
     * Array of elements
     */
    std::vector<T> elements;

    /**
     * Number of rows.
     */
    size_t rows;

    /**
     * Number of coloumns.
     */
    size_t cols;

  public:
    /**
     *
     */
    Matrix<T>& operator*= (const T& a);

    /**
     *
     */
    Matrix<T>& operator/= (const T& a);

    /**
     *
     */
    Matrix<T> operator* (const T& a) const;

    /**
     *
     */
    Matrix<T> operator/ (const T& a);

    /**
     *
     */
    Matrix<T>& operator+= (const Matrix<T>& M);

    /**
     *
     */
    Matrix<T>& operator-= (const Matrix<T>& M);

    /**
     *
     */
    Matrix<T>& operator*= (const Matrix<T>& M);

    /**
     *
     */
    Matrix<T> operator+ (const Matrix<T>& M) const;

    /**
     *
     */
    Matrix<T> operator- (const Matrix<T>& M) const;

    /**
     *
     */
    Matrix<T> operator* (const Matrix<T>& M) const;

    /**
     *
     */
    Matrix<T>& operator= (const Matrix<T>& M);

    /**
     * @brief transpose
     */
    Matrix<T> transpose() const;

    /**
     * @brief transpose
     */
    Matrix<T> t() const;

    /**
     * @brief transpose
     */
    Matrix<T> cofactor(size_t i,size_t j) const;

    /**
     * @brief trace
     */
    T trace() const;

    /**
     * @brief l infinity norm
     */
    T
    l_inf_norm() const;

    /**
     * @brief l2 norm
     */
    T
    l_2_norm() const;

    /**
     * @brief l1 norm
     */
    T
    l_1_norm() const;

    /**
     * @brief lp norm
     */
    T
    l_p_norm(const unsigned int& p) const;

    /**
     * [det description]
     * @param  m [description]
     * @return   [description]
     */
    T det() const;

    /**
     * [inverse description]
     * @param  m [description]
     * @return   [description]
     */
    Matrix<T> inv() const;

    /**
     *
     */
    Matrix<T> operator~() const;

    /**
     *
     */
    template<typename S>
    friend
    std::ostream&
    operator<< (std::ostream& output, const Matrix<S>& M);

    /**
     *
     */
    template<typename S>
    friend
    Matrix<S>
    operator* (const double& a, const Matrix<S>& M);
  };

  typedef Matrix<double> Matrixd;

// identityMatrix
////////////////////////////////////////////////////////////////////////////////
  class IdentityMatrix : public Matrix<double>
  {
  public:
    IdentityMatrix(size_t size);
  };
}
/** @}*/
#endif // __m_MATRIX_H__
