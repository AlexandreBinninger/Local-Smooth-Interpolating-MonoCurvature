#include <iostream>
#include "Utils.hh"

int
main() {
  Utils::Table::Style style;
  Utils::Table::Table table;
  std::cout << "Starting table test ...\n\n";
  try {

    std::vector<std::vector<std::string>> rows{
      {"Karl Kangaroo", "13. Sep 1988", "jumping"},
      {"Austin Ape", "24. Jul 2000", "climbing, jumping,\nsurfing"},
      {"..."},
      {"Bertha Bear", "3. Feb 1976", "sleeping", "Sherwood Forest"},
      {"Toni Tiger", "31. Jan 1935", "sleeping, hunting"},
      {"Paul Penguin", "6. Oct 1954"},
      {"Gira Giraffe", "10. Sep 1943", "", "London Zoo"},
      {"To be continued ..."}
    };

    style.paddingLeft(3);
    style.paddingRight(2);

    table.setup( style, rows );
    table.title("A simple Test-Table");
    table.headings({"Name", "Birthday", "Tags", "Adress"});
    table.alignColumn(1, Utils::Table::Alignment::RIGHT);
    table.alignColumn(2, Utils::Table::Alignment::CENTER);
    table[2][0].colSpan(4);
    table[7][0].colSpan(3);
    table.alignColumn(5-1, Utils::Table::Alignment::RIGHT);

  } catch ( std::exception const & e ) {
    std::cerr << e.what() << std::endl;
  }
  std::cout << table;
  std::cout << "\nStopping table test ...\n";
  return 0;
}
