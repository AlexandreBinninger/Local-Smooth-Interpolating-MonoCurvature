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

static
void
mem_info( char const msg[] ) {
  fmt::print(
    "{}: mem actual {} max {}\n",
    msg,
    Utils::outBytes( Utils::AllocatedBytes ),
    Utils::outBytes( Utils::MaximumAllocatedBytes )
  );
}

static
void
do_test1() {
  mem_info("do_test1 IN");
  Utils::Malloc<double> mem("test1");
  mem.allocate(100);
  double * ptr = mem(100);
  fmt::print( "ptr = {}\n", static_cast<void*>(ptr) );
  mem.free();
  mem_info("do_test1 OUT");
}

static
void
do_test2() {
  mem_info("do_test2 IN");
  Utils::Malloc<double> mem("test1");
  mem.allocate(100);
  double * ptr = mem(100);
  fmt::print( "ptr = {}\n", static_cast<void*>(ptr) );
  mem_info("do_test2 OUT");
}

int
main() {
  mem_info("main IN");
  do_test1();
  mem_info("A");
  do_test2();
  mem_info("B");
  do_test1();
  mem_info("C");
  do_test2();
  mem_info("main OUT");
  std::cout << "All done folks!\n\n";
  return 0;
}
