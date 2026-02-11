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
  bg.addComponent(std::move(se));
  bg.addComponent(std::move(r));
  bg.addComponent(std::move(c));
  bg.addComponent(std::move(j1));
  bg.addComponent(std::move(j0));
  bg.addComponent(std::move(j00));
  bg.addComponent(std::move(j000));

  // Now make the connections
  bg.connect(bg.getComponent<ComponentType::SE>("se"),
             bg.getComponent<ComponentType::J0>("j0"));
  bg.connect(bg.getComponent<ComponentType::J0>("j0"),
             bg.getComponent<ComponentType::J1>("j1"));
  bg.connect(bg.getComponent<ComponentType::J1>("j1"),
             bg.getComponent<ComponentType::J0>("j00"));
  bg.connect(bg.getComponent<ComponentType::J0>("j00"),
             bg.getComponent<ComponentType::R>("r"));
  bg.connect(bg.getComponent<ComponentType::J1>("j1"),
             bg.getComponent<ComponentType::J0>("j000"));
  bg.connect(bg.getComponent<ComponentType::J0>("j000"),
             bg.getComponent<ComponentType::C>("c"));

  std::cout << bg;
  // Test try to simplify it
  bg.simplify();
  std::cout << "After simplification\n";
  std::cout << bg;
  return 0;
}
