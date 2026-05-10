#ifndef __m_COMPLEX_H__
#define __m_COMPLEX_H__

#include <cmath>
#include <initializer_list> // std::initializer_list
#include <iostream>
#include <vector>

/** \addtogroup math
 *  @{
 */
namespace mlib {
/**
 * \brief Template class representing complex numbers with real and imaginary parts.
 *
 * \tparam T Numeric type for real and imaginary components (default: int)
 *
 * This class provides basic complex number arithmetic operations including
 * multiplication with scalars and other complex numbers, assignment operations,
 * and output formatting.
 */
template <typename T = int>
class Complex {
public:
    /**
     * \brief Constructor that initializes a complex number with real and imaginary parts.
     *
     * \param real_ Real part of the complex number (default: 0)
     * \param img_ Imaginary part of the complex number (default: 0)
     */
    inline Complex(const T& real_ = 0, const T& img_ = 0) : img(img_), real(real_)
    {
    }

    /**
     * \brief Copy constructor (uses default implementation).
     */
    Complex(const Complex<T>& c) = default;

    /**
     * \brief Move constructor (uses default implementation).
     */
    Complex(Complex<T>&& c) = default;

    /**
     * \brief Destructor (uses default implementation).
     */
    ~Complex() = default;

    /**
     * \brief Get the imaginary part of the complex number.
     * \return The imaginary component.
     */
    inline T i() const
    {
        return img;
    }

    /**
     * \brief Get the real part of the complex number.
     * \return The real component.
     */
    inline T r() const
    {
        return real;
    }

public:
    /**
     * \brief Multiply-assign operator with a scalar value.
     *
     * \param r Scalar value to multiply by
     * \return Reference to the modified complex number
     */
    inline Complex<T> operator*=(const T& r)
    {
        *this = (*this) * r;
        return *this;
    }

    /**
     * \brief Copy assignment operator.
     *
     * \param c Complex number to copy from
     * \return Reference to the modified complex number
     */
    inline Complex<T> operator=(const Complex<T>& c)
    {
        this->img = c.i();
        this->real = c.r();
        return *this;
    }

    /**
     * \brief Move assignment operator.
     *
     * \param c Complex number to move from
     * \return Reference to the modified complex number
     */
    inline Complex<T> operator=(Complex<T>&& c)
    {
        this->img = std::move(c.img);
        this->real = std::move(c.real);
        return *this;
    }

    /**
     * \brief Multiply-assign operator with another complex number.
     *
     * \param c Complex number to multiply by
     * \return Reference to the modified complex number
     */
    inline Complex<T> operator*=(const Complex<T>& c)
    {
        *this = (*this) * c;
        return *this;
    }

    /**
     * \brief Multiply operator with a scalar value.
     *
     * \param a Scalar value to multiply by
     * \return New complex number resulting from the multiplication
     */
    inline Complex<T> operator*(const T& a) const
    {
        T i = img * a;
        T r = real * a;
        return Complex<T>(r, i);
    }

    /**
     * \brief Multiply operator with another complex number.
     *
     * Uses the formula: (a + bi) * (c + di) = (ac - bd) + (ad + bc)i
     *
     * \param c Complex number to multiply by
     * \return New complex number resulting from the multiplication
     */
    inline Complex<T> operator*(const Complex<T>& c) const
    {
        T i = real * c.i() + img * c.r();
        T r = real * c.r() - img * c.i();
        return Complex<T>(r, i);
    }

    /**
     * \brief Output stream operator for formatting complex numbers.
     *
     * Formats as "a + bi" or "a - bi" depending on the sign of the imaginary part.
     *
     * \tparam S Numeric type of the complex number
     * \param output Output stream reference
     * \param c Complex number to output
     * \return Reference to the output stream
     */
    template <typename S>
    friend inline std::ostream& operator<<(std::ostream& output, const Complex<S>& c)
    {
        if (c.i() >= 0) {
            output << c.r() << " + " << c.i() << "i";
        } else {
            output << c.r() << " - " << std::abs(c.i()) << "i";
        }
        return output;
    }

private:
    T img;  ///< Imaginary part
    T real; ///< Real part
};

} // namespace mlib
/** @}*/
#endif // __m_COMPLEX_H__
