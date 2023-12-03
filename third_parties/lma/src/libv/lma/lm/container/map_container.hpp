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

#ifndef __OPTIMISATION2_CONTAINER_MAP_CONTAINER_HPP__
#define __OPTIMISATION2_CONTAINER_MAP_CONTAINER_HPP__

#include <libv/lma/global.hpp>

#include <boost/mpl/vector.hpp>
#include <boost/mpl/transform.hpp>
#include <boost/fusion/include/as_map.hpp>
#include <boost/fusion/include/at_key.hpp>
#include <boost/fusion/include/io.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/mpl.hpp>
#include <boost/type_traits/remove_pointer.hpp>
#include "typed_index_container.hpp"

#include <boost/mpl/apply.hpp>
#include <boost/mpl/transform.hpp>

namespace lma
{

/**
 * \struct ConvertTransform
 * \brief apply the converter UnaryConverter to the result of applying UnaryFunctor to each element of List
 *
 * \param List a type list
 * \param UnaryConverter unary functor who convert mpl::vector to real container, usually fusion::as_map or fusion::as_vector
 * \param UnaryFunctor an unary functor to apply to each element of List
 * \return the result of applying those two functors
 */

  template<class List, class UnaryConverter, class UnaryFunctor> struct ConvertTransform
  {
    typedef typename mpl::apply<
                                    UnaryConverter,
                                    typename  mpl::transform<
                                                                List,
                                                                UnaryFunctor
                                                              >::type
                                  >::type type;
  };


  struct Clear
  {
    template<template<class,class> class Pair, class Key, class Value> void operator()(Pair<Key,Value>& pair) const
    {
      pair.second.clear();
    }
  };
  

  template<class L, class Fonctor> class MultiContainer
  {
    public:
    typedef L Liste;
    typedef typename ConvertTransform<
                                        L,
                                        br::as_map<mpl::_1>,
                                        Fonctor
                                      >::type Map;


      Map map_;//! map public pour g++-4.7 ??

      static const std::size_t NbType = boost::mpl::size<L>::value;
      static const std::size_t NbContainer = boost::fusion::result_of::size<Map>::value;

      //! add a value to the associate container when Value == Key
      template<class Value> ttt::Indice<Valu