#include "BondGraph.hpp"
#include "Component.hpp"
#include <iostream>

int main() {
  Component<ComponentType::SE> se{"se"};     // voltage (effort) source
  Component<ComponentType::R> r{"r"};        // resistor
  Component<ComponentType::C> c{"c"};        // resistor
  Component<ComponentType::J1> j1{"j1"};     // 1 Junction (in series)
  Component<ComponentType::J0> j0{"j0"};     // 1 Junction (in series)
  Component<ComponentType::J0> j00{"j00"};   // 1 Junction (in series)
  Component<ComponentType::J0> j000{"j000"}; // 1 Junction (in series)
  BondGraph bg;
  bg.addComponent(&se);
  bg.addComponent(&r);
  bg.addComponent(&c);
  bg.addComponent(&j1);
  bg.addComponent(&j0);
  bg.addComponent(&j00);
  bg.addComponent(&j000);

  // Now make the connections
  bg.connect(se, j0);
  bg.connect(j0, j1);
  bg.connect(j1, j00);
  bg.connect(j00, r);
  bg.connect(j1, j000);
  bg.connect(j000, c);

  std::cout << bg;
  // Test try to simplify it
  bg.simplify();
  std::cout << "After simplification\n";
  std::cout << bg;
  return 0;
}
