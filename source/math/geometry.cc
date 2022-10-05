#include "mlib/math/geometry.h"

#include <cmath>
#include <iostream>

namespace mlib
{

  std::vector<unsigned int>
  rectify(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const double& mean_tolerance,
    const unsigned int& min_n_zeroes)
  {
    std::vector<unsigned int> v;
    v.push_back(0);

    for(unsigned int i = 0; i < x.size()-1; ++i)
      {
        unsigned int j=0;
        while(i+j+1 < x.size())
          {
            if(abs(
                 abs(x[i] - x[i+1]) *abs(y[i] - y[i+1+j])
                 -
                 abs(x[i] - x[i+1+j]) *abs(y[i] - y[i+1])
               )
               <mean_tolerance)
              {
                ++j;
              }
            else
              {
                break;
              }
          }

        if(j>min_n_zeroes)
          {
            v.push_back(i);
            v.push_back(i+j);
            i=i+j;
          }

      }

    v.push_back(static_cast<unsigned int>(x.size()-1));

    return v;
  }

  std::vector<std::pair<unsigned int, unsigned int>>
                                                  extract_uniform_grid(
                                                    const std::vector<double>& x,
                                                    const unsigned int& step)
  {
    std::vector<std::pair<unsigned int, unsigned int>> v;
    v.push_back(std::make_pair(0, 0));

    for(unsigned int i = 0; i<x.size(); i+=step)
      {
        if(i+step < x.size())
          v.push_back(std::make_pair(i, i+step));
      }
    unsigned int xs = static_cast<unsigned int>(x.size()-1);
    v.push_back(std::make_pair(xs, xs));

    return v;
  }

  std::vector<std::pair<unsigned int, unsigned int>>
                                                  extract_linear_parts(
                                                    const std::vector<double>& x,
                                                    const std::vector<double>& y,
                                                    const double& mean_tolerance,
                                                    const unsigned int& min_n_zeroes)
  {
    std::vector<std::pair<unsigned int, unsigned int>> v;
    v.push_back(std::make_pair(0, 0));

    for(unsigned int i = 0; i < x.size()-1; ++i)
      {
        unsigned int j=0;
        while(i+j<x.size())
          {
            if(abs(
                 abs(x[i] - x[i+1]) *abs(y[i] - y[i+j])
                 -
                 abs(x[i] - x[i+j]) *abs(y[i] - y[i+1])
               )
               <mean_tolerance
              )
              {
                ++j;
              }
            else
              {
                break;
              }
          }

        if(j>min_n_zeroes && i+j < y.size())
          {
            if(v.size() >0
               &&
               i - v[v.size()-1].second < 2
               &&
               (y[i] - y[i+j]) / (x[i] - x[i+j]) -
               (y[v[v.size()-1].first] - y[v[v.size()-1].second])
               /
               (x[v[v.size()-1].first] - x[v[v.size()-1].second])
               <1e-6
              )
              {
                v[v.size()-1].second = i+j;
                i=i+j;
              }
            else
              {
                std::pair<unsigned int, unsigned int> new_element =
                  std::make_pair(i, i+j);
                v.push_back(new_element);
                i=i+j;
              }
          }

      }



    v.push_back(std::make_pair(x.size()-1, x.size()-1));

    return v;
  }

  std::vector<std::pair<unsigned int, unsigned int>>
                                                  merge_increasing_parts(
                                                    const std::vector<double>& x,
                                                    const std::vector<double>& y,
                                                    const bool equal)
  {
    std::vector<std::pair<unsigned int, unsigned int>> v;

    v.push_back(std::make_pair(0, 0));

    for(unsigned int i = 0; i < x.size()-1; ++i)
      {
        unsigned int j=0;
        if(equal)
          while(i+j+1 < x.size() &&
                y[i+j+1] - y[i+j] >= 0) ++j;
        else
          while(i+j+1 < x.size() &&
                ((y[i+j+1] - y[i+j])) > 0) ++j;

        v.push_back(std::make_pair(i, i+j));

        i=i+j+1;
      }

    v.push_back(std::make_pair(x.size()-1, x.size()-1));

    return v;
  }

  std::vector<std::pair<unsigned int, unsigned int>>
                                                  merge_decreasing_parts(
                                                    const std::vector<double>& x,
                                                    const std::vector<double>& y,
                                                    const bool equal)
  {
    std::vector<std::pair<unsigned int, unsigned int>> v;

    v.push_back(std::make_pair(0, 0));

    for(unsigned int i = 0; i < x.size()-1; ++i)
      {
        unsigned int j=0;
        if(equal)
          while(i+j+1 < x.size() &&
                y[i+j+1] - y[i+j] <= 0) ++j;
        else
          while(i+j+1 < x.size() &&
                ((y[i+j+1] - y[i+j])) < 0) ++j;

        v.push_back(std::make_pair(i, i+j));

        i=i+j+1;
      }

    v.push_back(std::make_pair(x.size()-1, x.size()-1));

    return v;
  }

  std::vector<double>
  interpolate_straight_parts(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const std::vector<std::pair<unsigned int, unsigned int>>&
    idx
  )
  {
    std::vector<double> v(y);

    for(unsigned int i = 0; i < idx.size(); ++i)
      {
        unsigned int i_start = idx[i].first;
        unsigned int i_end = idx[i].second;
        if(i_end < y.size())
          {
            double slope = (y[i_end] - y[i_start]) / (x[i_end] -
                                                      x[i_start]);
            for(unsigned int j=i_start+1; j<i_end+1; ++j)
              v[j] = v[j-1] + (x[j] - x[j-1]) * slope;
          }
      }

    return v;
  }
}
