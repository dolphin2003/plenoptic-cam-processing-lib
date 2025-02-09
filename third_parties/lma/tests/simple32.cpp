
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
  template<> struct Size<B> { static const size_t value = 1; };
  template<> struct Size<C> { static const size_t value = 1; };
};


template<class Solver, class AlgoTag, class V0, class V1, class V2> void solve(V0 v0, V1 v1, V2 v2)
{
  Solver bundle(1,1);

  for(size_t j = 0 ; j < v1.size() ; ++j)
    for(size_t k = 0 ; k < v2.size() ; ++k)
      bundle.add(F1(),&v1[j],&v2[k]);
      
  for(size_t i = 0 ; i < v0.size() ; ++i)
      for(size_t k = 0 ; k < v2.size() ; ++k)
        bundle.add(F2(),&v0[i],&v2[k]);
  
  bundle.solve(AlgoTag(),lma::minimal_verbose());
}


int main()
{
  size_t size0 = 1;
  size_t size1 = 1;
  size_t size2 = 1;

  std::vector<A,Eigen::aligned_allocator<A>> v0(size0);
  std::vector<B,Eigen::aligned_allocator<B>> v1(size1);
  std::vector<C,Eigen::aligned_allocator<C>> v2(size2);

  for(auto& x : v0)
    x << random(1.0);

  for(auto& x : v1)
    x << random(1.0);
  
  for(auto& x : v2)
    x << random(1.0);

  typedef Solver<mpl::vector<A,B,C>,F1,F2> Solver;

  solve<Solver,DENSE_SCHUR_>(v0,v1,v2);
  return 0;
}