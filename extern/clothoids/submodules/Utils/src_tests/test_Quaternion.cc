/*--------------------------------------------------------------------------*\
 |                                                                          |
 |  Copyright (C) 2017                                                      |
 |                                                                          |
 |         , __                 , __                                        |
 |        /|/  \               /|/  \                                       |
 |         | __/ _   ,_         | __/ _   ,_                                | 
 |         |   \|/  /  |  |   | |   \|/  /  |  |   |                        |
 |         |(__/|__/   |_/ \_/|/|(__/|__/   |_/ \_/|/                       |
 |                           /|                   /|                        |
 |                           \|                   \|                        |
 |                                                                          |
 |      Enrico Bertolazzi                                                   |
 |      Dipartimento di Ingegneria Industriale                              |
 |      Universita` degli Studi di Trento                                   |
 |      email: enrico.bertolazzi@unitn.it                                   |
 |                                                                          |
\*--------------------------------------------------------------------------*/

#include "Utils.hh"

int
main() {

  Utils::Quaternion<double> Q1, Q2, Q3;

  Q1.setup(3,1,0,0);
  Q2.setup(0,5,1,-2);

  std::cout << "Q1 = " << Q1 << '\n';
  std::cout << "Q2 = " << Q2 << '\n';

  Q3 = Q1*Q2;
  std::cout << "Q3 = Q1*Q2 = " << Q3 << '\n';

  Q3 = Q2*Q1;
  std::cout << "Q3 = Q2*Q1 = " << Q3 << '\n';

  Q3 = Q1*Q1;
  std::cout << "Q3 = Q1*Q1 = " << Q3 << '\n';

  Q3 = Q2*Q2;
  std::cout << "Q3 = Q2*Q2 = " << Q3 << '\n';

  Q3 = Q1;
  Q3.conj();
  std::cout << "Q1.conj() = " << Q3 << '\n';

  Q3 = Q1;
  Q3.invert();
  std::cout << "Q1.invert() = " << Q3 << '\n';

  std::cout << "All done folks\n\n";
  return 0;
}
