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

static void fun1( int i );
static void fun2( int i );
static void fun3( int i );
static void fun4( int i );

static
void
fun5( char const str[] ) {
  std::cout << "fun3: " << str << '\n';
  Utils::printTrace(__LINE__,__FILE__,"in fun3",std::cerr);
}

static
void
fun1( int i ) {
  std::cout << "in fun1\n";
  if ( i > 0 ) {
    fun2( i-1 );
  } else {
    std::string str = fmt::format("format {}",i);
    fun5( str.c_str() );
  }
}

static 
void
fun2( int i ) {
  std::cout << "in fun2\n";
  if ( i > 0 ) {
    fun3( i-1 );
  } else {
    std::string str = fmt::format("format {}",i);
    fun5( str.c_str() );
  }
}

static 
void
fun3( int i ) {
  std::cout << "in fun3\n";
  if ( i > 0 ) {
    fun4( i-1 );
  } else {
    std::string str = fmt::format("format {}",i);
    fun5( str.c_str() );
  }
}

static 
void
fun4( int i ) {
  std::cout << "in fun4\n";
  if ( i > 0 ) {
    fun1( i-1 );
  } else {
    std::string str = fmt::format("format {}",i);
    fun5( str.c_str() );
  }
}

int
main() {
  try {
    int i = 6;
    std::cout << "call fun1\n";
    fun1( i );
  } catch ( std::exception const & exc ) {
    std::cout << "Error: " << exc.what() << '\n';
  } catch ( ... ) {
    std::cout << "Unknown error\n";
  }
  std::cout << "All done folks\n\n";
  return 0;
}
