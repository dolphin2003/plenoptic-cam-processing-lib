#include "p3p.h"

#include "geometry/pose.h"

/**

Solve the quartic equation \f$ ax^4+bx^3+cx^2+dx+e=0 \f$.

Use <a href="http://en.wikipedia.org/wiki/Quartic_function#Ferrari.27s_solution">Ferrari’s closed form solution</a>.

\return
A list of 4 real solutions.

*/
Eigen::Vector4d solve_quartic
( double A ///< The coefficient of degree 4
, double B ///< The coefficient of degree 3
, double C ///< The coefficient of degree 2
, double D ///< The coefficient of degree 1
, double E ///< The coefficient of degree 0
)
{
  double alpha = -3 * B*B / (8 * A*A) + C / A;
  double beta = B*B*B / (8 * A*A*A) - B * C / (2 * A*A) + D / A;
  double gamma = -3 * B*B*B*B / (256 * A*A*A*A) + B*B * C / (16 * A*A*A) - B * D / (4 * A*A) + E / A;

  std::complex<double> P = -alpha*alpha / 12 - gamma;
  std::complex<double> Q = -alpha*alpha*alpha / 108 + alpha * gamma / 3 - beta*beta / 8;
  std::complex<double> R = -Q / 2. + std::sqrt(Q*Q / 4. + P*P*P / 27.);
  std::complex<double> U = std::pow(R, 1. / 3.);
  std::complex<double> y = U.real()
    ? -5 * alpha / 6 - P / (3. * U) + U
    : -5 * alpha / 6 - std::pow(Q, 1. / 3.);
  std::co