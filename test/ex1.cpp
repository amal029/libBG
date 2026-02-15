#include "BondGraph.hpp"
#include "Component.hpp"
// #include "expression.hpp"
#include <iostream>

int main() {

  // Declare a Capacitor
  Component<ComponentType::C> c{"c"};    // capacitor
  Component<ComponentType::SE> se{"se"}; // voltage (effort) source
  Component<ComponentType::R> r{"r"};    // resistor
  Component<ComponentType::J1> j{"1"};   // 1 Junction (in series)


  // // Add the components to the BondGraph
  BondGraph bg;
  bg.addComponent(&c);
  bg.addComponent(&se);
  bg.addComponent(&r);
  bg.addComponent(&j);

  // Make the connections -- we don't have to do it this way. We can use
  // the moved object, but be careful in that case -- it is
  // implementation defined then.
  bg.connect(se, j);
  bg.connect(j, r);
  bg.connect(j, c);

  // Print the bond graph
  std::cout << bg; // This is done by traversing the graph

  return 0;
}
