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

#ifndef __OPTIMISATION2_ALGO_LM_MANY_CLASSES_SCHUR_COMPLEMENT_HPP__
#define __OPTIMISATION2_ALGO_LM_MANY_CLASSES_SCHUR_COMPLEMENT_HPP__

#include <libv/lma/ttt/traits/wrap.hpp>
#include <libv/lma/global.hpp>
#include <libv/lma/lm/ba/computing.hpp>
#include <libv/lma/lm/ba/make_type.hpp>
#include <libv/lma/lm/ba/create_hessian.hpp>
#include <libv/lma/lm/omp/omp.hpp>
#include <boost/mpl/advance.hpp>
#include <boost/mpl/remove_if.hpp>
#include <boost/mpl/contains.hpp>
#include <boost/mpl/assert.hpp>
#include <boost/type_traits.hpp>
#include <boost/fusion/include/as_map.hpp>
#include <libv/core/tag.hpp>

namespace lma
{

  template<class VTYPE, class Ap, class H, class P, class Y> stru