#include "BondGraph.hpp"
#include "Component.hpp"

#include <iostream>

int main() {
  // Declare a Capacitor
  Component<ComponentType::C> c{"c", PrefCausality::I};    // capacitor
  Component<ComponentType::SE> se{"se"}; // voltage (effort) source
  Component<ComponentType::R> r{"r"};    // resistor
  Component<ComponentType::J1> j{"1"};   // 1 Junction (in series)

  Component c1{std::move(c)};

  // Add the components to the BondGraph
  BondGraph bg;
  bg.addComponent(std::move(c1));
  bg.addComponent(std::move(se));
  bg.addComponent(std::move(r));
  bg.addComponent(std::move(j));

  auto &j1 = bg.getComponent<ComponentType::J1>("1");

  // Make the connections -- we don't have to do it this way. We can use
  // the moved object, but be careful in that case -- it is
  // implementation defined then.
  bg.connect(bg.getComponent<ComponentType::SE>("se"), j1);
  bg.connect(j1, bg.getComponent<ComponentType::R>("r"));
  bg.connect(j1, bg.getComponent<ComponentType::C>("c"));

  // Print the bond graph
  std::cout << bg; // This is done by traversing the graph

  return 0;
}
