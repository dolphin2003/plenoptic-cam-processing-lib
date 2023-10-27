
/**

\file
\author Alexis Wilhelm (2013)
\copyright 2013 Institut Pascal

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <libv/core/auto_load.hpp>
#include <libv/core/serialization/archives/simple_binary.hpp>
#include <libv/core/serialization/serializable.hpp>
#include <algorithm>
#include <fstream>
#include <istream>

#include <libv/core/found/boost_iostreams>
#if defined(LIBV_CORE_BOOST_IOSTREAMS_FOUND)
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#endif

namespace v {
namespace core {
namespace {

static const size_t extents[] = {1};

// rank      : number of dimensions
// extents[] : table containing the size of each dimension

template<class T>
void load_(std::istream &stream, T *values, const size_t *extents, size_t rank)
{
  stream.read(reinterpret_cast<char *>(values), accumulate(extents, extents + rank, sizeof(T), std::multiplies<size_t>()));
}
template<>
void load_<size_t>(std::istream &stream, size_t *values, const size_t *extents, size_t rank)
{
  // uint64_t is portable between 32 and 64 bits systems (size_t is not)