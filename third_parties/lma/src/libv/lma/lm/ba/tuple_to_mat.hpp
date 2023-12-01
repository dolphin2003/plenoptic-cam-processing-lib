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

#ifndef __OPTIMISATION2_BA_TO_MAT_HPP__
#define __OPTIMISATION2_BA_TO_MAT_HPP__

#include <libv/lma/global.hpp>
#include <libv/lma/lm/ba/container.hpp>
#include <libv/lma/lm/ba/mat.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/find.hpp>
#include "utils.hpp"
#include <Eigen/Sparse>
#include "../container/container.hpp"
/**
 * Convertir un tuple en matrice dense
 */

namespace lma {
  template<class Tag, class Mat, class Keys, class Tuple, class Dim> struct ConvertToMat
  {
    Mat& mat;
    const Tuple& tuple;
    const Dim& dim;
    
	template<class T, int I, int J>
    void set_(Mat& m, int x, int y, const Eigen::Matrix<T,I,J>& block)
    {
      for(int i = 0 ; i < I ; ++i)
        for(int j = 0 ; j < J ; ++j)
          m(x+i,y+j) = block(i,j);
//       Blocker<Tag,I,J>::view(m,x,y) = block;
    }
    
#ifdef USE_TOON
    template<int I, int J, class T> 
    void set_(Mat& m, int x, int y, const TooN::Matrix<I,J,T>& block)
    {
      for(int i = 0 ; i < I ; ++i)
        for(int j = 0 ; j < J ; ++j)
          m(x+i,y+j) = block(i,j);
    }
    template<int I, class T> 
    void set_(Mat& m, int x, int y, const TooN::Vector<I,T>& block)
    {
      for(int i = 0 ; i < I ; ++i)
          m(x+i,y) = block[i];
    }
#endif

#ifdef USE_BLAZE
    template<size_t I, size_t J, class T> 
    void set_(Mat& m, int x, int y, const blaze::StaticMatrix<T,I,J>& block)
    {
      for(int i = 0 ; i < I ; ++i)
        for(int j = 0 ; j < J ; ++j)
          m(x+i,y+j) = block(i,j);
    }
    template<size_t I, class T> 
    void set_(Mat& m, int x, int y, const blaze::StaticVector<T,I>& block)
    {
      for(size_t i = 0 ; i < I ; ++i)
          m(x+i,y) = block[i];
    }
#endif

    
    template<class T, int I, int J> 
    void set_(Eigen::SparseMatrix<T>& m, int x, int y, const Eigen::Matrix<T,I,J>& block)
    {
      for(size_t i = 0 ; i < I ; ++i)
        for(size_t j = 0 ; j < J ; ++j)
          m.insert(x+i,y+j) = block(i,j);
    }
    
    ConvertToMat(Mat& mat_, const Tuple& tuple_, const Dim& dim_):mat(mat_),tuple(tuple_),dim(dim_) {}
    
    template<class A, class B> struct Pos : mpl::find<A,B>::type::pos {};
    
    template<class KeyA, class KeyB,class Value> void operator()(ttt::wrap<bf::pair<bf::pair<KeyA,KeyB>,Value>>)
    {
      add_to_mat<
                  Pos<Keys,KeyA>::value,
                  Pos<Keys,KeyB>::value
                >(bf::at_key< bf::pair<KeyA,KeyB> > (tuple));
    }
    
    template<int I, int J, class Key1, class Key2, class T, class _> void add_to_mat(const Table<Key1,Key2,T,_>& table)
    {
      for(auto i = table.first() ; i < table.size() ; ++i)
        for(auto j_ = table.first(i) ; j_ < table.size(i) ; ++j_)
          set_(mat,bf::at_c<I>(dim)+i()*table.I,bf::at_c<J>(dim)+table.indice(i,j_)()*table.J,table(i,j_));
    }

    template<int I, int J, class Key, class T> void add_to_mat(const Table<Key,Key,T,Diagonal>& table)
    {
      for(auto i = table.first() ; i < table.size() ; ++i)
      {
        const int x_ = bf::at_c<I>(dim)+i()*table.I;
        set_(mat,x_,x_,table(i));
      }
    }
    
    template<int I, int J, class Key, class T> void add_to_mat(const Table<Key,Key,T,Symetric>& table)
    {
      for(auto i = table.first() ; i < table.size() ; ++i)
        for(auto j_ = table.first(i) ; j_ < table.size(i) ; ++j_)
        {
          if (i<=table.indice(i,j_))
            set_(mat,bf::at_c<I>(dim)+i()*table.I,bf::at_c<J>(dim)+table.indice(i,j_)()*table.J,table(i,j_));
        }
    }
    
  };
  
  template<class Tag,class Keys, class TupleKeys, class SizeTuple, class Tuple> typename ContainerOption<Tag,0,0>::MatrixDD to_mat(const Tuple& tuple, SizeTuple size_tuple)
  {
    typedef typename ContainerOption<Tag,0,0