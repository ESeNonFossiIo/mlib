#include "mlib/math/utility.h"
#include  <iostream>
// std::abs
#include <cmath>
#include <assert.h>     /* assert */

namespace _mlib
{

  template <typename T>
  int
  sgn(
    T val)
  {
    return (T(0) < val) - (val < T(0));
  }

  template int sgn(int);
  template int sgn(double);
  template int sgn(float);


  double
  truncate_decimals(
    double num,
    double size)
  {
    return int (num/size) *size;
  }

  std::vector<double>
  truncate_decimals_vec(
    std::vector<double>& vec,
    double size)
  {
    std::vector<double> v(vec);
    for(auto
        it = v.begin();
        it != v.end();
        ++it)
      {
        *it = _mlib::truncate_decimals(*it,size);
      }
    return v;
  }

  template <typename T>
  T
  truncate(
    T num,
    T min,
    T max)
  {
    if(num < min)
      return min;
    else if(num > max)
      return max;
    else
      return num;
  }

  template <typename T>
  std::vector<T>
  truncate_vec(
    std::vector<T>& vec,
    T min,
    T max
  )
  {
    std::vector<T> v(vec);
    for(unsigned int i = 0; i < vec.size(); ++i)
      {
        v[i] = _mlib::truncate<T> (vec[i], min, max);
      }
    return v;
  }

  template int truncate(int,int,int);
  template double truncate(double,double,double);
  template float truncate(float,float,float);
  template std::vector<int> truncate_vec(std::vector<int>&,
                                         int,int);
  template std::vector<double> truncate_vec(
    std::vector<double>&,double,double);
  template std::vector<float> truncate_vec(std::vector<float>
                                           &,float,float);


  template <typename T>
  std::function<T(T)>
  normalize_range(const T& val1, const T& val2)
  {
    assert(val2>val1);
    return [&val1, &val2](T t)
    {
      return (t - val1)/(val2 - val1);
    };
  };
  template std::function<double(double)> normalize_range(const double&,
                                                         const double&);

  std::vector<double>
  difference(
    std::vector<double>& vec,
    double step,
    bool left)
  {
    std::vector<double> v(vec);
    if(left)
      for(unsigned int i = 1; i < vec.size(); ++i)
        v[i] = (vec[i] - vec[i-1]) /step;
    else
      for(unsigned int i = 0; i < vec.size()-1; ++i)
        v[i] = (vec[i+1] - vec[i]) /step;
    return v;
  }

  std::vector<double>
  difference(
    std::vector<double>& y,
    std::vector<double>& x,
    bool left)
  {
    std::vector<double> v(y);
    if(left)
      for(unsigned int i = 1; i < y.size(); ++i)
        v[i] = (y[i] - y[i-1]) / (x[i] - x[i-1]);
    else
      for(unsigned int i = 0; i < y.size()-1; ++i)
        v[i] = (y[i+1] - y[i]) / (x[i+1] - x[i]);
    return v;
  }

  std::vector<double>
  accumulate(
    std::vector<double>& vec,
    double step,
    bool left)
  {
    std::vector<double> v(vec);
    if(left)
      for(unsigned int i = 1; i < vec.size(); ++i)
        v[i] =  step * vec[i] + v[i-1];
    else
      for(int i = vec.size()-2; i >= 0 ; --i)
        v[i] =  step * vec[i] +  v[i+1];
    return v;
  }

  std::vector<double> force_mean(
    std::vector<double>& vec,
    unsigned int consecutive,
    double toll)
  {
    std::vector<double> v(vec);

    for(int i = consecutive;
        i < vec.size()-consecutive; ++i)
      {
        // calcola la media tra -consecutive e +consecutive compresi.
        double media = 0.0;
        for(int j = -1 * (int) consecutive; j<= (int)consecutive; ++j)
          {
            media += vec[i+j];
          }
        media /= double(2*consecutive+1);

        // se il valore di v_i vicino a meno di una tolleraza a media allora
        // v_i viene posto uguale a media
        bool check = true;
        for(int j = -1 * (int)consecutive; j<= (int)consecutive; ++j)
          {
            check &= (std::abs(vec[i+j] - media) < toll);
          }

        if(check)
          {
            for(int j = -1 * (int)consecutive; j<= (int)consecutive; ++j)
              v[i+j] = media;
            int j = consecutive + 1;

            if(i+j > vec.size()-consecutive-1)
              break;

            while(std::abs(vec[i+j] - media) < toll && i+j < vec.size() - (int)consecutive)
              {
                v[i+j] = media;
                j++;
              };
            i += j + consecutive - 1;
          }
      }
    return v;
  }

  std::vector<int>
  flat_part(
    std::vector<double>& vec,
    double toll,
    unsigned int min_num_zeroes)
  {
    // set the fist element to zero.
    std::vector<int> v = {0};

    for(unsigned int i = 0; i < vec.size(); ++i)
      {
        unsigned int j=0;
        while(std::abs(vec[i] - vec[i+j]) <toll && j+i<vec.size())
          ++j;
        if(j>=1)
          --j;

        if(j>min_num_zeroes)
          {
            v.push_back(i);
            v.push_back(i+j-2);
          }
        i=i+j;
      }

    // check that the last element is equal to the lenght of vec.
    if(v[v.size()-1]!=vec.size()-1)
      v.push_back(vec.size()-1);

    return v;
  };

  std::vector<int>
  straight_part(
    std::vector<double>& vec,
    double toll,
    unsigned int min_num_zeroes)
  {
    // set the fist element to zero.
    std::vector<int> v = {0};

    for(unsigned int i = 0; i < vec.size()-1; ++i)
      {
        unsigned int j=0;
        while(std::abs(
                std::abs(vec[i] - vec[i+j])
                -
                j*std::abs(vec[i] - vec[i+1])
              )
              <toll &&
              j+i<vec.size())
          ++j;
        if(j>=1)
          --j;

        if(j>min_num_zeroes)
          {
            v.push_back(i);
            v.push_back(i+j-2);
          }
        i=i+j;
      }

    // check that the last element is equal to the lenght of vec.
    if(v[v.size()-1]!=vec.size()-1)
      v.push_back(vec.size()-1);

    return v;
  };

  std::vector<double>
  interpolate(
    std::vector<double>& vec,
    std::vector<int>& interpolation,
    double toll
  )
  {
    std::vector<double> v(vec);
    for(unsigned int i = 0; i < interpolation.size()-1; ++i)
      {
        int j_init = interpolation[i];
        int j_end  = interpolation[i+1];

        if(
          std::abs(
            std::abs(vec[j_init] - vec[j_init+j_end])
            -
            (j_end-j_init) *std::abs(vec[j_init] - vec[j_init+1])
          )
          >toll)
          for(int j = j_init; j < j_end; ++j)
            {
              v[j] = vec[j_init] + (j-j_init) /double(j_end-j_init) *
                     (vec[j_end-1] - vec[j_init]);
            }
        else
          for(int j = j_init; j < j_end; ++j)
            v[j] = vec[j_init];
      }
    return v;
  };

  std::vector<double>
  apply_lambda(
    std::vector<double>& vec,
    std::function<double(double)> func
  )
  {
    std::vector<double> v(vec);
    for(unsigned int i = 0; i < vec.size()-1; ++i)
      v[i] = func(vec[i]);
    return v;
  }

  std::vector<double>
  remove_singularities(
    std::vector<double>& vec,
    double toll_zero,
    double toll_jump,
    unsigned int item_before,
    unsigned int item_after,
    unsigned int singularity_lenght,
    bool left)
  {
    std::vector<double> v(vec);
    for(unsigned int i = item_before;
        i < vec.size() - item_after - singularity_lenght;
        ++i)
      {
        bool status = true;

        // Controlla che i primi valori siano allineati
        for(unsigned int j = 2; j <= item_before; ++j)
          if(std::abs(vec[i-1]-vec[i-j]) > toll_zero)
            status = false;

        // Controlla che i secondi valori siano allineati
        for(unsigned int j = 0; j < item_after; ++j)
          if(std::abs(
               vec[i+singularity_lenght]-
               vec[i+singularity_lenght+j]) > toll_zero)
            status = false;

        // Controlla che la singolarità non sia allineata né prima né dopo
        for(unsigned int j = 0; j < singularity_lenght; ++j)
          if(std::abs(vec[i-1]-vec[i+j]) <= toll_jump
             || std::abs(vec[i+singularity_lenght]-vec[i+j]) <=
             toll_jump)
            status = false;


        if(status)
          for(unsigned int j = 0; j < singularity_lenght; ++j)
            {
              if(left)
                v[i+j] = vec[i-1];
              else
                v[i+j] = vec[i+singularity_lenght];
            }
      }
    return v;
  }
}
