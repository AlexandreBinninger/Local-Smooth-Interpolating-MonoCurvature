#include <iostream>
#include "Utils.hh"

using std::string;
using std::vector;
using std::cout;

int
main() {
  string const str = "pippo,pluto paperino;nonna papera,,,zorro";
  string const sep = " ,;";
  vector<string> res;

  Utils::split_string( str, sep, res );

  cout << "STR: " << str << "\n";
  for ( auto & e : res ) cout << "TOKEN:" << e << "\n";
  cout << "\nAll Done Folks!\n";
  return 0;
}
