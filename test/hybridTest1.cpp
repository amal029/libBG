#include "BondGraph.hpp"
#include "Component.hpp"
#include "Solver.hpp"
#include "euler.hpp"
#include "expression.hpp"
#include <cfloat>
#include <cstring>
#include <iostream>
#include <variant>

void print_state_eqns(const expression_t &res, const char *name,
                      expressionAst &ast) {
  std::cout << std::format("State eq {}: ", name);
  print_expression_t(std::cout, res, ast);
  std::cout << "\n";
}

int main() {
  BondGraph<2> bg("HybridExample1", false);
  size_t seid = bg.addComponent(Component<ComponentType::SE>{"se"});
  size_t a1id = bg.addComponent(Component<ComponentType::J1>{"a1"});
  size_t b0id = bg.addComponent(Component<ComponentType::J0>{"b0"});
  size_t L1id = bg.addComponent(Component<ComponentType::L>{"L1"});
  size_t c1id = bg.addComponent(Component<ComponentType::J1>{"c1"});
  // size_t r1id = bg.addComponent(Component<ComponentType::R,
  // Modulated::T>{"r1"});
  size_t cap1id = bg.addComponent(Component<ComponentType::C>{"cap1"});
  size_t d0id = bg.addComponent(Component<ComponentType::J0>{"d0"});
  size_t L2id = bg.addComponent(Component<ComponentType::L>{"L2"});
  size_t e1id = bg.addComponent(Component<ComponentType::J1>{"e1"});
  size_t r2id = bg.addComponent(Component<ComponentType::R>{"r2"});
  size_t cap2id = bg.addComponent(Component<ComponentType::C>{"cap2"});

  // Now connect the components
  auto *se =
      std::get_if<Component<ComponentType::SE>>(&bg.getComponentAt(seid));
  auto *a1 =
      std::get_if<Component<ComponentType::J1>>(&bg.getComponentAt(a1id));
  auto *b0 =
      std::get_if<Component<ComponentType::J0>>(&bg.getComponentAt(b0id));
  auto *L1 = std::get_if<Component<ComponentType::L>>(&bg.getComponentAt(L1id));
  auto *c1 =
      std::get_if<Component<ComponentType::J1>>(&bg.getComponentAt(c1id));
  auto *cap1 =
      std::get_if<Component<ComponentType::C>>(&bg.getComponentAt(cap1id));
  auto *d0 =
      std::get_if<Component<ComponentType::J0>>(&bg.getComponentAt(d0id));
  auto *L2 = std::get_if<Component<ComponentType::L>>(&bg.getComponentAt(L2id));
  auto *e1 =
      std::get_if<Component<ComponentType::J1>>(&bg.getComponentAt(e1id));
  auto *r2 = std::get_if<Component<ComponentType::R>>(&bg.getComponentAt(r2id));
  auto *cap2 =
      std::get_if<Component<ComponentType::C>>(&bg.getComponentAt(cap2id));

  // Now make the connections
  bg.connect(*se, *a1);
  bg.connect(*a1, *b0);
  bg.connect(*b0, *L1);
  bg.connect(*b0, *c1);
  bg.connect(*c1, *cap1);
  // bg.connect(*c1, *r1);
  bg.connect(*c1, *d0);
  bg.connect(*d0, *L2);
  bg.connect(*d0, *e1);
  bg.connect(*e1, *r2);
  bg.connect(*e1, *cap2);

  // Add the junctions that are switching junctions
  bg.addSwitch(a1);
  bg.addSwitch(e1);

  // Now make all the bond graphs for the hybrid system
  bg.buildSwitchBondGraphs();

  for (auto &x : bg.getSwitchedGraphs()) {
    x.simplify();
    x.assignCausality();
    expressionAst ast = x.generateStateSpace();
    // Now just get the state equations for the storage elements
    std::cout << "------------------------------\n";
    auto *cap1 =
        std::get_if<Component<ComponentType::C>>(&x.getComponentAt(cap1id));
    auto *L1 =
        std::get_if<Component<ComponentType::L>>(&x.getComponentAt(L1id));
    auto *L2 =
        std::get_if<Component<ComponentType::L>>(&x.getComponentAt(L2id));
    auto *cap2 =
        std::get_if<Component<ComponentType::C>>(&x.getComponentAt(cap2id));
    print_state_eqns(cap1->getStateEq(ast), "Cap1", ast);
    print_state_eqns(L1->getStateEq(ast), "L1", ast);
    print_state_eqns(cap2->getStateEq(ast), "Cap2", ast);
    print_state_eqns(L2->getStateEq(ast), "L2", ast);
  }
  // XXX: How do we set the transitions and how do we set the constant
  // values for all the introduced 0 value se/sf?

  return 0;
}
