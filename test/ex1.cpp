#include "BondGraph.hpp"
#include "Component.hpp"
// #include "expression.hpp"
#include <iostream>
#include <variant>

int main() {

  // Add the components to the BondGraph
  BondGraph bg("ex1");
  size_t cid = bg.addComponent(Component<ComponentType::C>{"c"});
  size_t seid = bg.addComponent(Component<ComponentType::SE>{"se"});
  size_t rid = bg.addComponent(Component<ComponentType::R>{"r"});
  size_t id1 = bg.addComponent(Component<ComponentType::J1>{"1"});

  auto *c = std::get_if<Component<ComponentType::C>>(&bg.getComponentAt(cid));
  auto *se =
      std::get_if<Component<ComponentType::SE>>(&bg.getComponentAt(seid));
  auto *r = std::get_if<Component<ComponentType::R>>(&bg.getComponentAt(rid));
  auto *j = std::get_if<Component<ComponentType::J1>>(&bg.getComponentAt(id1));

  // Make the connections -- we don't have to do it this way. We can use
  // the moved object, but be careful in that case -- it is
  // implementation defined then.
  bg.connect(*se, *j);
  bg.connect(*j, *r);
  bg.connect(*j, *c);

  // Print the bond graph
  std::cout << bg; // This is done by traversing the graph

  return 0;
}
