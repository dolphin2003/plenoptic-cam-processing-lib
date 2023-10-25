
/**

\file
\author Alexis Wilhelm (2012-2013)
\copyright 2012-2013 Institut Pascal

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

#ifndef LIBV_CORE_IMAGE_IMAGE_HPP
#define LIBV_CORE_IMAGE_IMAGE_HPP

#include <type_traits>
#include <limits>
#include <stdexcept>
#include <libv/core/memory/aligned_allocator.hpp>
#include <libv/core/array_initializer.hpp>
#include <libv/core/assert.hpp>
#include <libv/core/type_traits/enable_if.hpp>
#include <libv/core/tag.hpp>

#include "format.hpp"
#include "iterator.hpp"
#include "layout.hpp"
#include "size.hpp"
#include "storage.hpp"
#include "traversal.hpp"

namespace v {
namespace core {
/// \addtogroup image
/// \{

/// Define a const type, given the corresponding non-const type.
#define _DEFINE_CONST_TYPE(x)\
  /** \copydoc x */\
  typedef typename const_this_type::x const_##x;

/// An image.
/// \implements std::DefaultConstructible
/// \implements std::RandomAccessContainer
template<class T>
class Image
  : public image_::image_layout<Image<T>, typename T::scalar_type, typename std::conditional<T::has_value_semantic, const typename T::scalar_type, typename T::scalar_type>::type &, T::flags & IMAGE_LAYOUT_MASK, T::plane_count, T::row_count == 1 && T::column_count == 1>
  , protected image_::image_size<1, T::row_count>
  , protected image_::image_size<2, T::column_count>
  , protected image_::image_size<3, T::plane_count>
  , protected image_::image_size<-1, T::row_step>
{
  template<class> friend class Image;
  template<class> friend class image_::image_iterator;
  template<class> friend struct image_::image_traversal_pixel;
  template<class> friend class image_::image_traversal_scalar;
  typedef T format;
  typedef Image this_type;
  typedef typename std::conditional<T::has_value_semantic, Image<typename T::template with_scalar_type<const typename T::scalar_type>::type>, this_type>::type const_this_type;
  typedef image_::image_size<1, T::row_count> the_row_count;
  typedef image_::image_size<2, T::column_count> the_column_count;
  typedef image_::image_size<3, T::plane_count> the_plane_count;
  typedef image_::image_size<-1, T::row_step> the_row_step;
  typename image_::image_storage_type<typename T::scalar_type, T::byte_count>::type data_;

  template<class X>
  struct is_convertible
  {
    static const bool value = (!T::row_count || T::row_count == X::row_count) && (!T::column_count || T::column_count == X::column_count) && (!T::plane_count || T::plane_count == X::plane_count);
  };

public:

  /// Same as \e this, but with \e _new_scalar_type as the scalar type.
  template<class _new_scalar_type>
  struct with_scalar_type
  {
    /// Result
    typedef Image<typename T::template with_scalar_type<_new_scalar_type>::type> type;
  };

  /// Type of scalar.
  typedef typename T::scalar_type scalar_type;

  /// Type of reference to a scalar.
  typedef scalar_type &reference_to_scalar;

  /// Type of pointer to a scalar.
  typedef scalar_type *pointer_to_scalar;

  /// Type of scalar iterator.
  typedef pointer_to_scalar scalar_iterator;

  typedef typename std::remove_cv<scalar_type>::type mutable_scalar_type;

  /// Type of image.
  typedef Image<ImageFormat<scalar_type, T::naked_flags | IMAGE_SEMANTIC_VALUE, T::row_count, T::column_count, T::plane_count, T::row_step, T::column_step> > image_type;

  /// Type of proxy.
  typedef Image<ImageFormat<scalar_type, T::naked_flags | IMAGE_SEMANTIC_REFERENCE, T::row_count, T::column_count, T::plane_count, T::row_step, T::column_step> > proxy_type;

  /// Type of view.
  typedef Image<ImageFormat<scalar_type, T::naked_flags | IMAGE_SEMANTIC_POINTER, T::row_count, T::column_count, T::plane_count, T::row_step, T::column_step> > view_type;

  /// Type of row.
  typedef Image<ImageFormat<scalar_type, T::naked_flags | IMAGE_SEMANTIC_VALUE, 1, T::column_count, T::plane_count, -1, -1> > row_type;

  /// Type of reference to a row.
  typedef typename row_type::proxy_type reference_to_row;

  /// Type of pointer to a row.
  typedef const reference_to_row *pointer_to_row;

  /// Type of row iterator.
  typedef image_::image_iterator<reference_to_row> row_iterator;

  /// Type of column.
  typedef Image<ImageFormat<scalar_type, T::naked_flags | IMAGE_SEMANTIC_VALUE, T::row_count, 1, T::plane_count, T::row_step, -1> > column_type;

  /// Type of reference to a column.
  typedef typename column_type::proxy_type reference_to_column;

  /// Type of pointer to a column.
  typedef const reference_to_column *pointer_to_column;

  /// Type of column iterator.
  typedef image_::image_iterator<reference_to_column> column_iterator;

  /// Type of pixel.
  typedef Image<ImageFormat<scalar_type, T::naked_flags | IMAGE_SEMANTIC_VALUE, 1, 1, T::plane_count, 1, 1> > pixel_type;

  /// Type of reference to a pixel.
  typedef typename pixel_type::proxy_type reference_to_pixel;

  /// Type of pointer to a pixel.
  typedef const reference_to_pixel *pointer_to_pixel;

  /// Type of pixel iterator.
  typedef image_::image_iterator<reference_to_pixel> pixel_iterator;

  _DEFINE_CONST_TYPE(pointer_to_scalar)
  _DEFINE_CONST_TYPE(pointer_to_pixel)
  _DEFINE_CONST_TYPE(pointer_to_row)
  _DEFINE_CONST_TYPE(pointer_to_column)
  _DEFINE_CONST_TYPE(reference_to_scalar)
  _DEFINE_CONST_TYPE(reference_to_pixel)
  _DEFINE_CONST_TYPE(reference_to_row)
  _DEFINE_CONST_TYPE(reference_to_column)
  _DEFINE_CONST_TYPE(scalar_iterator)
  _DEFINE_CONST_TYPE(pixel_iterator)
  _DEFINE_CONST_TYPE(row_iterator)
  _DEFINE_CONST_TYPE(column_iterator)
  _DEFINE_CONST_TYPE(proxy_type)
  _DEFINE_CONST_TYPE(view_type)

  /// \name Random Access Container Concept
  /// \{

  /// \copydoc std::RandomAccessContainer::difference_type
  typedef ptrdiff_t difference_type;

  /// \copydoc std::RandomAccessContainer::size_type
  typedef size_t size_type;

  /// \copydoc std::RandomAccessContainer::value_type
  typedef typename std::conditional<T::dimension == 1, scalar_type, typename std::conditional<T::row_count != 1, row_type, pixel_type>::type>::type value_type;

  /// \copydoc std::RandomAccessContainer::reference
  typedef typename std::conditional<T::dimension == 1, reference_to_scalar, typename std::conditional<T::row_count != 1, reference_to_row, reference_to_pixel>::type>::type reference;

  /// \copydoc std::RandomAccessContainer::pointer
  typedef typename std::conditional<T::dimension == 1, pointer_to_scalar, typename std::conditional<T::row_count != 1, pointer_to_row, pointer_to_pixel>::type>::type pointer;

  /// \copydoc std::RandomAccessContainer::iterator
  /// \todo what if rows ≠ 1 && columns = 1 && planes = 1 && row_step ≠ ±1 ?
  typedef typename std::conditional<T::dimension == 1, pointer_to_scalar, image_::image_iterator<reference> >::type iterator;

  /// \copydoc std::RandomAccessContainer::reverse_iterator
  typedef std::reverse_iterator<iterator> reverse_iterator;

  _DEFINE_CONST_TYPE(pointer)
  _DEFINE_CONST_TYPE(reference)
  _DEFINE_CONST_TYPE(iterator)
  _DEFINE_CONST_TYPE(reverse_iterator)

  size_type
  size(void) const
  {
    return T::row_count != 1 ? height() : T::column_count != 1 ? width() : depth();
  }

  /// \copydoc std::RandomAccessContainer::max_size
  /// \todo use row_step and other dimensions
  static size_type
  max_size(void)
  {
    return std::numeric_limits<size_type>::max() / sizeof(scalar_type);
  }

  bool
  empty(void) const
  {
    return !size();
  }

  iterator
  begin(void)
  {
    return begin(type_tag<iterator>());
  }

  const_iterator
  begin(void) const
  {
    return begin(type_tag<iterator>());
  }

  iterator
  end(void)
  {
    return begin() + size();
  }

  const_iterator
  end(void) const
  {
    return begin() + size();
  }

  reverse_iterator
  rbegin(void)
  {
    return reverse_iterator(end());
  }

  const_reverse_iterator
  rbegin(void) const
  {
    return const_reverse_iterator(end());
  }

  reverse_iterator
  rend(void)
  {
    return reverse_iterator(begin());
  }

  const_reverse_iterator
  rend(void) const
  {
    return const_reverse_iterator(begin());
  }

  reference
  operator[](size_type i)
  {
    return begin()[difference_type(i)];
  }

  const_reference
  operator[](size_type i) const
  {
    return begin()[difference_type(i)];
  }

  template<class X> bool
  operator==(const X &other) const
  {
    if(height() != other.height() || width() != other.width() || depth() != other.depth())
    {
      return false;
    }
    V_FOR_EACH_SCALAR(p, *this)
    {
      if(*p != other.scalar(p.row, p.column, p.plane))
      {
        return false;
      }
    }
    return true;
  }

  template<class X> bool
  operator!=(const X &other) const
  {
    return !(*this == other);
  }

  /// \}

  /// \name Constructors and destructor
  /// \{

  /// \copydoc std::DefaultConstructible::constructor
  Image(void)
  {
    new(this) this_type(0, 0, 0, 0, 0, numeric_tag<T::byte_count>());
  }

  /// \copydoc std::Assignable::constructor
  Image(const this_type &other)
  {
    copy_construct(other);
  }

  /// \copydoc Image(const this_type &)
  template<class X>
  Image(const Image<X> &other V_ENABLE_IF_TPL(is_convertible<X>::value)
  {
    copy_construct(other);
  }

  /// \copydoc Image(const this_type &)
  template<class X> explicit
  Image(const Image<X> &other V_ENABLE_IF_TPL(!is_convertible<X>::value)
  {
    copy_construct(other);
  }

  /// Constructor for pointer semantic or reference semantic.
  /// \param data An existing memory buffer.
  /// \param rows The height of the image.
  /// \param columns The width of the image.
  /// \param row_step The distance between two consecutive rows.
  Image V_ENABLE_IF(!T::has_value_semantic && !T::row_count && !T::column_count && T::plane_count, pointer_to_scalar) data, size_type rows, size_type columns, size_type row_step)
  {
    new(this) this_type(data, rows, row_step, columns, 0, numeric_tag<T::byte_count>());
  }

  /// Constructor for value semantic.
  /// \param rows The height of the image.
  /// \param columns The width of the image.
  Image V_ENABLE_IF(T::has_value_semantic && !T::row_count && !T::column_count && T::plane_count && T::row_step, size_type) rows, size_type columns)
  {
    new(this) this_type(0, rows, 0, columns, 0, numeric_tag<T::byte_count>());
  }

  /// Constructor for a static image.
  /// \param initializer An array initializer built with new_array().
  template<class X>
  Image(const ArrayInitializer<X, T::byte_count> &initializer)
  {
    initializer.apply(begin_scalar());
  }
