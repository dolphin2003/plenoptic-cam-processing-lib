/**

\file
\author Clement Deymier (2013)
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

#ifndef LIBV_GEOMETRY_ROTATION_HPP
#define LIBV_GEOMETRY_ROTATION_HPP

#include <Eigen/Core>

#include "global.hpp"

namespace v {
namespace geometry {
/// \addtogroup geometry
/// \{

LIBV_GEOMETRY_EXPORT void apply_rotation(Eigen::Matrix3d &, const Eigen::Vector3d &);
LIBV_GEOMETRY_EXPORT void apply_small_rotation(Eigen::Matrix3d &, const Eigen::Vector3d &);
LIBV_GEOMETRY_EXPORT double geodesic_distance(const Eigen::Matrix3d &, const Eigen::Matrix3d &);
LIBV_GEOMETRY_EXPORT Eigen::Matrix3d rotation_log(const Eigen::Matrix3d &a);
LIBV_GEOMETRY_EXPORT Eigen::Matrix3d rotation_exp(const Eigen::Matrix3d &a);
LIBV_GEOMETRY_EXPORT Eigen::Matrix3d rotation_linear_interpolation(const Eigen::Matrix3d &, const Eigen::Matrix3d &, double);

/** \brief A function to make 3x3 matrix a rotation matrix.
  This function is based on the SVD decomposition.
  \param m: input 3x3 matrix
  \return a rotation matrix (near from the input matrix)
*/
LIBV_GEOMETRY_EXPORT Eigen::Matrix3d rotation_orthogonalize(const Eigen::Matrix3d & m);

/** \brief A function to average a set of rotation matrices.
  This function use an iterative algorithm based on the Lie algera.

  \tparam _iterator: an forward iterator type on rotation matrices (\c Eigen::Ma