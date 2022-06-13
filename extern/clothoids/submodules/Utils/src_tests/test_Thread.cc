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

class Counter {
  Utils::BinarySearch<int> bs;
public:
  Counter() {
    bool ok ;
    int * pdata = bs.search( std::this_thread::get_id(), ok );
    *pdata = 0;
  }

  void
  inc() {
    bool ok;
    int * pdata = bs.search( std::this_thread::get_id(), ok );
    if ( !ok ) std::cerr << "Counter::inc failed thread\n";
    ++(*pdata);
  }

  void
  print() {
    bool ok;
    int * pdata = bs.search( std::this_thread::get_id(), ok );
    if ( !ok ) std::cerr << "Counter::inc failed thread\n";
    fmt::print( "thread {}, counter = {}\n", std::this_thread::get_id(), *pdata );
  }
};

static
void
do_test() {
  Counter c;
  for ( int i = 0; i < 10000000; ++i ) {
    //Utils::sleep_for_milliseconds(1);
    c.inc();
  }
  c.print();
}

int
main() {
  std::vector<std::thread> threads_tab;
  for ( int i = 0; i < 100; ++i ) {
    //Utils::sleep_for_milliseconds(1);
    threads_tab.push_back(std::thread(do_test));
  }
  for ( auto & t : threads_tab ) t.join();
  std::cout << "All done folks!\n\n";
  return 0;
}
