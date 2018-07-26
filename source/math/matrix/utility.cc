#include "mlib/math/matrix/utility.h"
#include "mlib/math/matrix/rotation.h"
#include "mlib/math/matrix/decomposition.h"

namespace _mlib
{
  void swap_lines(Matrix<double>& M,
                  const unsigned int& l1,
                  const unsigned int& l2)
  {
    for(unsigned int i = 0; i < M.c(); ++i)
      {
        double tmp = M(l1, i);
        M(l1, i) = M(l2, i);
        M(l2, i) = tmp;
      }
  };

  void swap_lines(Matrix<double>& M,
                  Point& b,
                  const unsigned int& l1,
                  const unsigned int& l2)
  {
    swap_lines(M, l1, l2);
    double tmp_b = b[l1];
    b[l1] = b[l2];
    b[l2] = tmp_b;
  };

  std::pair<Matrix<double>, Point>
  upper_triangular(const Matrix<double>& M, const Point& b)
  {
    Matrix<double> M_temp(M);
    Point b_temp(b);

    for(unsigned int i = 0; i<M.r(); ++i)
      {
        size_t i_max = i;
        double v_max = M_temp(i,i);
        for(unsigned int j = i; j<M_temp.r(); ++j)
          {
            if(std::abs(M_temp(j,i)) > std::abs(v_max))
              {
                i_max = j;
                v_max = M_temp(j,i);
              }
          }
        swap_lines(M_temp, b_temp, i, i_max);
        for(unsigned int h = i+1; h<M_temp.r(); ++h)
          {
            double k = M_temp(h,i) / M_temp(i, i);
            M_temp(h,i) = 0.0;
            for(unsigned int j = i+1;   j<M_temp.c(); ++j)
              {
                M_temp(h,j) -= k * M_temp(i, j);
              }
            b_temp[h] -= k * b_temp[i];
          }
      }
    return std::make_pair(M_temp, b_temp);
  };

  Point
  solve_upper_triangular(const Matrix<double>& M, const Point& b)
  {
    assert(M.r() == b.dim());
    assert(M.c() == b.dim());

    Point x(b);
    for(int i = b.dim() - 1; i>=0; --i)
      {
        double sum = b[i];
        for(unsigned j = i+1; j < b.dim(); j++)
          {
            sum -= M(i,j) * x[j];
          }
        x[i] = sum/M(i,i);
      }
    return x;
  };

  Point
  gauss_elimination_upper(const Matrix<double>& M, const Point& b)
  {
    assert(M.r() == b.r());
    assert(M.c() == b.r());

    auto S = upper_triangular(M, b);
    return solve_upper_triangular(S.first, S.second);
  }

  double
  rototranslation_matrix(const std::vector<Point>& p,
                         const std::vector<Point>& q,
                         Matrix<double>& M,
                         Point& b)
  {
    // x' = Mx + b
    assert(p.size() == q.size());
    assert(p.size() > 3);

    size_t np = p.size();

    Point pp(p[0]);
    std::vector<double> wp(p.size(), 1.0);
    pp = centroid(p,wp);
    std::vector<Point> p_(p);
    for(unsigned int i = 0; i < p.size(); ++i)
      {
        p_[i] -= pp;
      }

    Point qq(q[0]);
    std::vector<double> wq(q.size(), 1.0);
    qq = centroid(q,wq);
    std::vector<Point> q_(q);
    for(unsigned int i = 0; i < q.size(); ++i)
      {
        q_[i] -= qq;
      }

    Matrixd X(pp.dim(), p.size());
    for(unsigned int i = 0; i < pp.dim(); i++)
      for(unsigned int j = 0; j < p.size(); j++)
        X(i,j) = p_[j][i];

    Matrixd Y(qq.dim(), q.size());
    for(unsigned int i = 0; i < qq.dim(); i++)
      for(unsigned int j = 0; j < q.size(); j++)
        Y(i,j) = q_[j][i];

    Matrixd Z(p.size(), q.size());
    for(unsigned int i = 0; i < q.size(); i++)
      Z(i,i) = wp[i];

    Matrixd S = X * Z * Y.t();

    Matrixd U,V,W;
    SVD(S,U,W,V);

    Matrixd WW(qq.dim(),qq.dim());
    for(unsigned int j = 0; j < qq.dim(); j++)
      {
        WW(j,j) = 1.0;
      }
    WW(qq.dim()-1,qq.dim()-1) = (V*U.t()).det();

    M.resize(qq.dim(),qq.dim());
    M = V * WW * U.t();

    b.resize(qq.dim());
    b = qq - M*pp;

    double error(0.0);
    for(unsigned int i = 0; i < np; ++i)
      {
        error += ((M*p[i] + b) - q[i]).l_2_norm();
      }

    return error/(1.0 * np);
  };

  double
  rototranslation_matrix_2d(const Point& p1, const Point& p2,
                            const Point& q1, const Point& q2,
                            Matrix<double>& M,
                            Point& t)
  {

    double s = ((p2 - p1).t() * (q2 - q1))[0];
    s /= (p2 - p1).l_2_norm();
    s /= (q2 - q1).l_2_norm();
    double alpha = std::acos(s);

    Angle beta(-1*alpha);
    Rotation2DMatrix R(beta);

    M.resize(R.r(),R.c());
    M = R;
    t.resize(2);
    t = p1 - q1;
    return 0.0;
  };

  double
  best_affine_transformation(const std::vector<Point>& p,
                             const std::vector<Point>& q,
                             Matrix<double>& M,
                             Point& b)
  {
    // x' = Ax + b
    assert(p.size() > 3);

    unsigned int np   = p.size();
    unsigned int size = 3 * np;

    Point s;
    s.resize(size);
    Matrixd m(size,12);

    M.resize(3,3);
    b.resize(3);

    for(unsigned int i = 0; i < np; ++i)
      {
        for(unsigned int j = 0; j < 3; ++j)
          {
            for(unsigned int k = 0; k < 3; ++k)
              {
                m(i*3+j, 3*j+k) = p[i][k];
              }
            m(i*3+j, 9 + j) = 1.0;
            s[3*i + j] =  q[i][j];
          }
      }

    auto x = gauss_elimination_upper(m.t()*m, m.t()*s);

    for(unsigned int i = 0; i < 3; ++i)
      {
        for(unsigned int j = 0; j < 3; ++j)
          {
            M(i,j) = x[3*i + j];
          }
        b[i] = x[9+i];
      }


    double error(0.0);
    for(unsigned int i = 0; i < np; ++i)
      {
        error += ((M*p[i] + b) - q[i]).l_2_norm();
      }

    return error/(1.0 * np);
  };

  RotoTranslationMatrixHandler::
  RotoTranslationMatrixHandler(const std::vector<Point>& u_,
                               const std::vector<Point>& v_)
    :
    u(u_),
    v(v_)
  {
    assert(u.size() == v.size());
    np = u.size();
    for(unsigned int i  = 0; i<np; ++i)
      {
        b.push_back(1);
      }
    error = rototranslation_matrix(u,v,M,t);
  };

  void
  RotoTranslationMatrixHandler::
  update_mask(const std::vector<size_t>& b_)
  {
    b = b_;
    np = 0;
    for(auto v: b)
      {
        np += v;
      }
    // The number of points should be greater than 4
    assert(np>3);

    std::vector<Point> u_new;
    std::vector<Point> v_new;
    for(unsigned int i  = 0; i<u.size(); ++i)
      {
        if(b[i] == 1)
          {
            u_new.push_back(u[i]);
            v_new.push_back(v[i]);
          }
      }
    error = rototranslation_matrix(u_new,v_new,M,t);
  };

  double
  RotoTranslationMatrixHandler::
  get_error_on_point(const size_t& i)
  {
    return (M*u[i] + t - v[i]).l_2_norm();
  };

  double
  RotoTranslationMatrixHandler::
  get_volume_diff()
  {
    return M.det();
  };

  double
  RotoTranslationMatrixHandler::
  get_error()
  {
    return error;
  };

  size_t
  RotoTranslationMatrixHandler::
  get_number_of_points()
  {
    return np;
  };

  Matrix<double>&
  RotoTranslationMatrixHandler::
  get_matrix()
  {
    return M;
  };

  Point&
  RotoTranslationMatrixHandler::
  get_translation()
  {
    return t;
  };
}
