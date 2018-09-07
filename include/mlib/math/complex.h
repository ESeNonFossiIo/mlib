#ifndef  __m_COMPLEX_H__
#define  __m_COMPLEX_H__

#include <vector>
#include <iostream>
#include <initializer_list>  // std::initializer_list

/** \addtogroup math
 *  @{
 */
namespace mlib
{
  template<typename T = int>
  class Complex
  {
  public:
    /**
     * constructor
     */
    Complex(const T& real_ = 0, const T& img_ = 0)
      :
      img(img_),
      real(real_)
    {};

    /**
     * destructor
     */
    ~Complex() {};

    T i() const
    {
      return img;
    };
    T r() const
    {
      return real;
    };

  public:
    /**
     *
     */
    Complex<T>& operator*= (const T& r)
    {
      return this * r;
    };
    Complex<T>& operator*= (const Complex<T>& c)
    {
      return this * c;
    };

    const Complex<T>& operator* (const T& a) const
    {
      T i = img  * a;
      T r = real * a;
      return Complex<T>(i, r);
    };

    const Complex<T> operator* (const Complex<T>& c) const
    {
      std::cout << ">" << real*c.i() << std::endl;
      std::cout << ">" << img*c.r() << std::endl;
      std::cout << ">" << real*c.r() << std::endl;
      std::cout << ">" << img*c.i() << std::endl;

      T i = real*c.i() + img*c.r();
      T r = real*c.r() - img*c.i();
      Complex<T> ret(r, i);
      std::cout << ">" << i << std::endl;
      std::cout << ">" << r << std::endl;
      std::cout << ">" << Complex<T>(r, i) << std::endl;

      return ret;
    };


    template<typename S>
    friend
    std::ostream&
    operator<< (std::ostream& output, const Complex<S>& c)
    {
      if(c.i() >= 0)
        {
          output << c.r() << " + " << c.i() << "i" ;
        }
      else
        {
          output << c.r() << " - " << std::abs(c.i()) << "i" ;
        }
      return output;
    };

  private:
    T img;
    T real;
  };


}
/** @}*/
#endif // __m_COMPLEX_H__
