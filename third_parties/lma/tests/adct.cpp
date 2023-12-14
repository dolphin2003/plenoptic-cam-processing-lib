#ifdef USE_CERES

#include "ceres/rotation.h"
#include "ceres/jet.h"
#include <Eigen/Dense>
#include <TooN/TooN.h>
#include <cmath>
#include <libv/lma/color/console.hpp>
#include <libv/lma/string/string_utiliy.hpp>
#include <libv/lma/time/tictoc.hpp>
#include <libv/lma/numeric/ad/rt/ad.hpp>
#include <libv/lma/numeric/ad/ct/adct.hpp>
#include <typeinfo>

  typedef Eigen::Matrix<double,9,1> Camera;
  typedef Eigen::Matrix<double,3,1> P