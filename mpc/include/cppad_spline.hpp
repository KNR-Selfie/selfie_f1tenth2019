#ifndef CPPAD_SPLINE_HPP
#define CPPAD_SPLINE_HPP

#include <spline.h>
#include <cppad/cppad.hpp>

namespace CppAD
{

class spline
{
public:
  spline(std::vector<double> x, std::vector<double> y);
  AD<double> operator() (AD<double> x) const;

private:
  tk::spline impl_;
};

spline::spline(std::vector<double> x, std::vector<double> y)
{
  impl_.set_points(x, y);
}

AD<double> spline::operator()(AD<double> x) const
{
  size_t n=impl_.m_x.size();

  // find the closest point m_x[idx] < x, idx=0 even if x<m_x[0]
  std::vector<double>::const_iterator it;
  it=std::lower_bound(impl_.m_x.begin(),impl_.m_x.end(),x);
  int idx=std::max( int(it-impl_.m_x.begin())-1, 0);

  AD<double> h=x-impl_.m_x[idx];
  AD<double> interpol;
  if(x<impl_.m_x[0]) {
    // extrapolation to the left
    interpol=(impl_.m_b0*h + impl_.m_c0)*h + impl_.m_y[0];
  } else if(x>impl_.m_x[n-1]) {
    // extrapolation to the right
    interpol=(impl_.m_b[n-1]*h + impl_.m_c[n-1])*h + impl_.m_y[n-1];
  } else {
    // interpolation
      interpol=((impl_.m_a[idx]*h + impl_.m_b[idx])*h + impl_.m_c[idx])*h + impl_.m_y[idx];
  }
  return interpol;
}

} // namespace CppAD

#endif // CPPAD_SPLINE_HPP
