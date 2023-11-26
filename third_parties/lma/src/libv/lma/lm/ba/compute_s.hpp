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

#ifndef __OPTIMISATION2_BA_COMPUTE_S_HPP__
#define __OPTIMISATION2_BA_COMPUTE_S_HPP__

#include "container.hpp"
#include <libv/lma/time/tictoc.hpp>
#include <libv/lma/lm/omp/omp.hpp>

namespace lma
{

  template<class A, class B, class C, class Bundle, class U, class T>
  void S__U_WY_Init1(Table<A,C,T>& tAC, const Table<A,B,T>& , const Table<C,B,T>& , const Bundle& bundle, const U& u)
  {
//     std::cout << " init 1 " << tAC.name() << std::endl;
    const auto& BA = ttt::at<A,B>(bundle.vab_map);
    const auto& BC = ttt::at<C,B>(bundle.vab_map);

    for(auto i = u.indice.first() ; i < u.indice.size() ; ++i)
      for(auto j = u.indice.first(i) ; j < u.indice.size(i) ; ++j)
        tAC.indice.add(i,u.indice(i,j));

    for(auto ib = BA.first() ; ib < BA.size() ; ++ib)
      for(auto iba = BA.first(ib) ; iba < BA.size(ib) ; ++iba)
      {
        auto a1 = BA(ib,iba);
        for(auto ibc = BC.first(ib) ; ibc < BC.size(ib) ; ++ibc)
          tAC.indice.add(a1,BC(ib,ibc));
      }
  }
  
  template<class A, class B, class C, class Bundle, class Vec, class Vab, class T>
  void S__U_WY_Init2(Table<A,C,T>& tAC, const Table<A,B,T>& , const Table<C,B,T>& , const Bundle& bundle, Vec& vec, const Vab& vab)
  {
//     std::cout << " init 2 AC : " << tAC.name() << std::endl;
    const auto& BA = ttt::at<A,B>(bundle.vab_map);
    const auto& BC = ttt::at<C,B>(bundle.vab_map);

    const auto& BA2 = ttt::at<A,B>(vab);
    const auto& BC2 = ttt::at<C,B>(vab);
    
    for(auto ib = BA.first() ; ib < BA.size() ; ++ib)
      for(auto iba = BA.first(ib) ; iba < BA.size(ib) ; ++iba)
        for(auto ibc = BC.first(ib) ; ibc < BC.size(ib) ; ++ibc)
        {
          auto a1 = BA(ib,iba);
          auto a2 = BC(ib,ibc);
          auto is = tAC.indice.get(a1,a2);
          auto r1 = BA2.reverse(ib,iba);
          auto r2 = BC2.reverse(ib,ibc);
          vec.liste_op.emplace_back(a1(),a2(),r1(),is(),r2());
        }
  }
  
  template<class A, class T>
  void copy_indice(Table<A,A,T,Symetric>& tAC, const Table<A,A,T,Symetric>& u)
  {
    for(auto i = u.first() ; i < u.size() ; ++i)
      for(auto j = u.first(i) ; j < u.size(i) ; ++j)
        if (i <= u.indice(i,j))
          tAC.indice.add(i,u.indice(i,j));
  }
  
  template<class A, class T>
  void copy_indice(Table<A,A,T,Symetric>& tAC, const Table<A,A,T,Diagonal>& u)
  {
    for(auto i = u.first() ; i < u.size() ; ++i)
      tAC.indice.add(i,i);
  }
  
  template<class A, class B, class Bundle, class Flt, class T, class _1, class _2, class _3>
  void S__U_WY_Init1(Table<A,A,T,Symetric>& tAC, const Table<A,B,T,_1>& , const Table<A,B,T,_2>& , const Bundle& bundle, const Table<A,A,Flt,_3>& u)
  {
    const auto& BA  = ttt::at<A,B>(bundle.vab