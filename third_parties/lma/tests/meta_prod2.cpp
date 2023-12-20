#ifdef USE_BRIGAND

#include <brigand/brigand.hpp>
//#include <brigand/types/integer.hpp>
#include <iostream>

#include <libv/lma/ttt/traits/naming.hpp>

struct A{};
struct B{};
struct C{};
struct D{};
struct E{};
struct F{};
struct G{};

namespace ttt
{
  template<class T> struct Name<brigand::type_<T>>
  {
    static std::string name()
    {
      return std::string("brigand::type_<") + color.bold() + color.red() + ttt::name<T>() + color.reset() + ">";
    }
  };

  template<class A, class B> struct Name<brigand::pair<A,B>>
  {
    static std::string name()
    {
      return std::string("brigand::pair<") + ttt::name<A>() + "," + ttt::name<B>() + ">";
    }
  };

  template<template<class ...> class List, class ... T> struct Name<List<T...>>
  {
    static std::string name(){
      std::string str;
      brigand::for_each<brigand::list<T...>>([&str]
        (auto t) { str += ttt::name<decltype(t)>() + ","; });
     return str ; }
  };

  template<class T, T value> struct Name<std::integral_constant<T,value>>
  {
    static std::string name()
    {
      return std::string("std::IntegralConstant<")+ttt::name<T>()+","+std::to_string(value)+">";
    }
  };

  template<> struct Name<A>{ static std::string name