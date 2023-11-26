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

#ifndef __OPTIMISATION2_ALGO_LM_MANY_CLASSES_SCHUR_COMPLEMENT_SCHUR_COMPLEMENT5_HPP__
#define __OPTIMISATION2_ALGO_LM_MANY_CLASSES_SCHUR_COMPLEMENT_SCHUR_COMPLEMENT5_HPP__

#include "../levmar.hpp"
#include <libv/lma/lm/ba/initialize.hpp>
#include <libv/lma/lm/ba/compute_s.hpp>
#include <libv/lma/lm/bundle/make_sparse_indice.hpp>
#include "schur.hpp"
#include "implicit_schur.hpp"
#include <boost/fusion/include/pair.hpp>
#include <libv/lma/lm/ba/tuple_to_mat.hpp>

namespace lma
{
  template<class BA, class ExplictSCHUR, class ImplicitSCHUR> struct BABYS
  {
    const BA& ba;
    const ExplictSCHUR& eschur;
    const ImplicitSCHUR& ischur;

    typedef typename ExplictSCHUR::TypeS Hessian;
    typedef typename ExplictSCHUR::OptimizeKeys OptimizeKeys;

    const typename ExplictSCHUR::TupleS& A() const { return eschur.s; }
    const typename ImplicitSCHUR::TupleResiduUs& B() const { return ischur.bs; }

    BABYS(const BA& ba_, const ExplictSCHUR& schur_, const ImplicitSCHUR& ischur_):ba(ba_),eschur(schur_),ischur(ischur_){}

    template<class AP, class P> void prodAP(AP& ap, const P& p) const
    {
//       mpl::for_each<Hessian,ttt::wrap_>(prod_ap_p(ap,eschur.s,p));
//       typedef typename MetaProd<typename TrigSup<Hessian>::type,typename ImplicitSCHUR::TupleResiduUs>::type keys;
//       std::cout << std::endl << ttt::name<keys>() << std::endl;
//       for_each<keys>(std::tie(ap,eschur.s,p),AABB());

      for_each<MetaProd<typename AddTranspose<Hessian>::type,typename ImplicitSCHUR::TupleResiduUs>>(std::tie(ap,eschur.s,p),AABB());
    }
  };

  template<class VAB> struct CopyReverseIndice
  {
    const VAB& vab;
    CopyReverseIndice(const VAB& vab_):vab(vab_){}

    template<template<class,class> class Pair, class Key, class Value> void operator()(Pair<Key,Value>& pair) const
    {
      auto& sics = pair.second;
      bf::for_each(sics,CopyReverseIndice<VAB>(vab));
    }

    template<template<class,class> class Pair, class Key, class A, class B> void operator()(Pair<Key,SIC2<A,B>>& pair) const
    {
      auto& sics_ab = pair.second;
      const auto& sic_ba = bf::at_key<B>(bf::at_key<A>(vab));

     // trop dur de savoir ce que ca fait et si c'est vraiment utile...
      for(auto i = sic_ba.first() ; i < sic_ba.size() ; ++i)
        for(auto j = sic_ba.first(i) ; j < sic_ba.size(i) ; ++j)
          sics_ab.add(sic_ba(i,j),i,j);
    }
  };

  template<class BA, class NumericTag, class KeyUs> struct SchurExplicit
  {
    typedef typename BA::Keys Keys;

    // K_ est le nombre de famille à mettre dans KeyUs
    static const size_t K = mpl::size<Keys>::value - Size<NumericTag>::value;
    
    typedef typename For<K,mpl::size<Keys>::value,Keys,PushBack>::type KeyVs;
    
//     typedef typename MakeH<typename BA::Float,KeyUs>::type TypeS;
    typedef typename ListS<typename BA::ListeHessien,KeyUs>::type TypeS;
    
    typedef typename br::as_map< TypeS >::type TupleS;

    struct Vec { std::vector< bf::vector<int,int,int,int,int> > liste_op; };

    typedef typename br::as_map< 
				 typename mpl::transform<
							  typename mpl::transform<TypeS,br::first<mpl::_1>>::type,
							  bf::pair<mpl::_1,Vec>
							>::type
			       >::type PrecalcVector;

    typedef KeyUs OptimizeKeys;
    typedef typename MakeSparseIndiceContainer2<Keys>::type VABMap;
    
    // tuple de vector des indices pré-calculés pour le calcul de S.
    
    typedef typename br::as_map<typename mpl::transform<KeyVs,bf::pair<mpl::_1,PrecalcVector>>::type>::type TUPLEVECTORINDICE;
    TUPLEVECTORINDICE tuple_v;
    VABMap svab;
    TupleS s;

    const TupleS& A() const { return s; }
    
    SchurExplicit()
    {
//       std::cout << " schur : " << ttt::name<TypeS>() << std::endl;
    }

    template<class Indices> void init(const Indices& vab)
    {
      bf::for_each(svab,CopyReverseIndice<Indices>(vab));
    }
  };

  struct UpdateIndice
  {
    template<class Pair> void operator()(Pair& pair) const
    {
      pair.second.indice.update();
      pair.second.resize(pair.second.indice);
    }
  };

  template<class Bdl, class NormEq, class MatrixTag_, class K> class ExplicitSchur : ImplicitSchur<Bdl,NormEq,MatrixTag_,K>
  {
    public:
    typedef MatrixTag_ MatrixTag;
    typedef ImplicitSchur<Bdl,NormEq,MatrixTag_,K> parent;
    typedef Bdl Bundle;
    typedef utils::Tic<false> Tic;


    typedef Bas<Bundle,MatrixTag> Ba;
    typedef typename Ba::Hessian Hessian;
    typedef typename Ba::Keys Keys;
    typedef typename parent::SchurCont ParentSchur;
    typedef SchurExplicit<Ba,K,typename ParentS