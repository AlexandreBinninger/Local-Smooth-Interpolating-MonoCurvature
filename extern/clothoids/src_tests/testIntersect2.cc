//#define _USE_MATH_DEFINES
#include "Clothoids.hh"

using namespace std;
using G2lib::real_type;
using G2lib::int_type;

int
main() {

  G2lib::CircleArc C0, C1;
#if 0
  real_type x0     = 0;
  real_type y0     = 2;
  real_type theta0 = 0;
  real_type k0     = 1.0/3.0;
  real_type L0     = 10;
  C0.build( x0, y0, theta0, k0, L0 );

  real_type x1     = 0;
  real_type y1     = 2;
  real_type theta1 = 3.1415/4;
  real_type k1     = 1.5*k0;
  real_type L1     = 10;
  C1.build( x1, y1, theta1, k1, L1 );
#else
  real_type x0     = 0;
  real_type y0     = 0;
  real_type theta0 = 3.7688;
  real_type k      = -1.51933;
  real_type L      = 1.66304;
  C0.build( x0, y0, theta0, k, L );

  x0     = -1.20978;
  y0     = -4.05915;
  theta0 = 1.18758;
  k      = 0.112684;
  L      = 6.18734;
  C1.build( x0, y0, theta0, k, L );

#endif

  G2lib::IntersectList ilist;
  C0.intersect( C1, ilist, false );

  cout << "L0 = " << C0.length() << '\n';
  cout << "L1 = " << C1.length() << '\n';

  for ( size_t i = 0; i < ilist.size(); ++i )
    cout << "s1[ " << i << "] = " << ilist[i].first << '\n';

  for ( size_t i = 0; i < ilist.size(); ++i )
    cout << "s2[ " << i << "] = " << ilist[i].second << '\n';

  for ( size_t i = 0; i < ilist.size(); ++i )
    cout << "x[" << i << "] = " << setw(10) << C0.X(ilist[i].first)
         << " y[" << i << "] = " << setw(10) << C0.Y(ilist[i].first)
         << "\nx[" << i << "] = " << setw(10) << C1.X(ilist[i].second)
         << " y[" << i << "] = " << setw(10) << C1.Y(ilist[i].second)
         << "\n\n";

  cout << "\n\nALL DONE FOLKS!!!\n";

  return 0;
}
