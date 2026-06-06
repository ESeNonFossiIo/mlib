#ifndef __m_MATRIX_DECOMPOSITION_H__
#define __m_MATRIX_DECOMPOSITION_H__

#include "numerix/math/matrix/matrix.h"

/** \addtogroup math
 *  @{
 */
namespace numerix {
void SVD(const Matrixd& A, Matrixd& U, Matrixd& W, Matrixd& V);

void QR(const Matrixd& A, Matrixd& Q, Matrixd& R);
} // namespace numerix
/** @}*/
#endif // __m_MATRIX_DECOMPOSITION_H__
