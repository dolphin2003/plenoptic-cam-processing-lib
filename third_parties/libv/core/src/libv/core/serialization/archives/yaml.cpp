#include <libv/core/serialization/archives/yaml.hpp>
#if defined LIBV_CORE_HAS_YAML_ARCHIVES
#include <libv/core/auto_load.hpp>
#include <libv/core/serialization/serializable.hpp>
#include <fstream>

namespace v {
namespace core {
namespace {

template<class T> static void load_recursive(const YAML::Node &, T *&, const size_t *, size_t);
template<class T> static void save_recursive(YAML::Emitter &, const T *&, const size_t *, size_t);

template<class T>
static void load_(const YAML::Node &node, T &value)
{
  assert(node.IsScalar());
  value = node.as<T>();
}

template<class T>
static void load_(const YAML::Node &node, T *&values, const size_t *extents, size_t rank)
{
  if(rank)
  {
    load_recursive(node, values, extents, rank);
  }
  else
  {
    load_(node, *values);
    ++values;
  }
}

static void load_(const YAML::Node &node, char *&values, const size_t *extents, size_t rank)
{
  if(rank > 1)
  {
    load_recursive(node, values, extents, rank);
  }
  else
  {
    assert(node.IsScalar());
    assert(!rank || node.Scalar().size() == *extents);
    copy(node.Scalar().begin(), node.Scalar().end(), values);
    values += node.Scalar().size();
  }
}

template<class T>
static void load_recursive(const YAML::Node &node, T *&values, const size_t *extents, size_t rank)
{
  assert(node.IsSequence());
  for(auto &p: node)
  {
    load_(p, values, extents + 1, rank - 1);
  }
}

template<class T>
static void save_(YAML::Emitter &out, T value)
{
  out << YAML::Value << value;
}

template<class T>
static void save_(YAML::Emitter &out, const T *&values, const size_t *extents, size_t rank)
{
  if(rank)
  {
    save_recursive(out, values, extents, rank);
  }
  else
  {
    save_(out, *values);
    ++values;
  }
}

static void save_(YAML::Emitter &out, const char *&values, const size_t *extents, size_t rank)
{
  if(rank > 1)
  {
    save_recursive(out, values, extents, rank);
  }
  else
  {
    assert(rank);
    save_(out, std::string(values, values + *extents));
    values += *extents;
  }
}

template<class T>
static void save_recursive(YAML::Emitter &out, const T *&values, const size_t *