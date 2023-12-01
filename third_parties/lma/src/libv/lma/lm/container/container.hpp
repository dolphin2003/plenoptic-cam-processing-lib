/**

\file
\author Datta Ramadasan
//==============================================================================
//         Copyright 2015 INSTITUT PASCAL UMR 6602 CNRS/Univ. Clermont II
//
//          Distributed under the Boost Software License, Version 1.0.
//                 See accompanying file LICENSE.txt or copy at
//                     http://www.boost.org/LICENSE_1_0.txt
//==============================================================================

*/

#ifndef __OPTIMISATION2_CONTAINER_CONTAINER_HPP__
#define __OPTIMISATION2_CONTAINER_CONTAINER_HPP__

#include <cstddef>
#include <libv/lma/lm/trait/accessor.hpp>
#include <libv/lma/lm/trait/adl.hpp>
#include <boost/type_traits/is_floating_point.hpp>
#include <boost/utility/enable_if.hpp>
#include "residual.hpp"
#include "../ba/nan_error.hpp"

namespace lma
{
  inline bool is_invalid(double d) { return !std::isfinite(d);}
  inline bool is_valid(double d) { return !is_invalid(d); }

  template<class> struct Rows;
  template<class> struct Cols;
  

  template<class Tag, std::size_t I, std::size_t J> struct ContainerOption {};
  
  template<class Tag, std::size_t X, std::size_t Y> struct Blocker;
  
  template<class Float, class Tag> 
  const Float& make_view(const Float& residual, Tag, typename boost::enable_if<boost::is_floating_point<Float>>::type* =0)
  {
    return residual;
  }
  
  template<class Float> const Float& transpose(const Float& f)
  {
    return f;
  }
  
  template<class Float> 
  Float squared_norm(Float d, typename boost::enable_if<boost::is_floating_point<Float>>::type* =0)
  { return d*d;}
  
  template<class Float, size_t N> 
  Eigen::Map<Eigen::Matrix<Float,N,1>> array_to_eigen(std::array<Float,N>& array)
  {
    return Eigen::Map<Eigen::Matrix<Float,N,1>>(array.data());
  }
  
////////////////////////////////////////////////////////
  template<class Float> void apply_small_increment(Float& obj, Float delta, const v::numeric_tag<0> &, const Adl&, typename boost::enable_if<boost::is_floating_point<Float>>::type* =0)
  {
    obj += delta;
  }
  template<class Float> void apply_increment(Float& o, const Float* inc, const Adl&, typename boost::enable_if<boost::is_floating_point<Float>>::type* =0)
  {
    o += inc[0];
  }
////////////////////////////////////////////////////////

  namespace detail
  {
    template<class Obj, class Float> void internal_apply_increment(Obj& o, const Float* inc)
    {
      apply_increment(o,inc,Adl());
    }


    template<class Obj, class Float, int I> void internal_apply_small_increment(Obj& o, Float flt, const v::numeric_tag<I> &tag)
    {
      apply_small_increment(o,flt,tag,Adl());
    }
  }
////////////////////////////////////////////////////////


  template <class T, int n> void invert_symmetric_matrix(T a[n][n], T ai[n][n])
  {
    for(int i = 0; i < n; ++i)
    {
      if(a[i][i] > 0)
      {
      	a[i][i] = std::sqrt(a[i][i]);
      	for(int j = i + 1; j < n; ++j) 
      	  a[j][i] = a[j][i] / a[i][i];
      	for(int j = i + 1; j < n; ++j)
      	  for(int k = j; k < n; ++k)
      	    a[k][j] -= a[k][i] * a[j][i];
      }
    }

    for(int i = 0; i < n; ++i)
    {
      if(a[i][i] != 0)
	      a[i][i] = 1.0f / a[i][i];
    }
    for(int i = 1; i < n; ++i)
    {
      if(a[i][i] != 0)
      {
      	for(int j = 0; j < i; ++j)
      	{
      	  T 