#include <libv/lma/numeric/divers.hpp>
#include <libv/lma/lma.hpp>

using namespace lma;

typedef double type;
struct A : Eigen::Matrix<type,1,1> {};
struct B : Eigen::Matrix<type,1,1> {};
struct C : Eigen::Matrix<type,1,1> {};


namespace ttt
{
  template<> struct Name<A> { static std::string name(){ return "A"; } };
  template<> struct Name<B> { static std::string name(){ return "B"; } };
  template<> struct Name<C> { static std::string name(){ return "C"; } };
}
  
  struct F1
  {
    bool operator()(const B& b, const C& c, type& r) const
    {
      r = b.x() * c.x();
      return true;
    }
  };
  
  struct F2
  {
    bool operator()(const A& a, const C& c, type& r) const
    {
      r = a.x() * c.x();
      return true;
    }
  };
  
namespace lma
{
  template<> struct Size<A> { static const size_t value = 1; };
  template<> struct Size<B> { static const size_t value = 