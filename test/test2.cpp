#include "BondGraph.hpp"
#include "Component.hpp"
#include "expression.hpp"
#include <iostream>

int main() {
  Component<ComponentType::SE> se{"se"};
  Component<ComponentType::J0> u0{"u0"};
  Component<ComponentType::J1> j1{"j1"};
  Component<ComponentType::J0> u12{"u12"};
  Component<ComponentType::R> r{"r"};
  Component<ComponentType::J0> u2{"u2"};
  Component<ComponentType::J1> j2{"j2"};
  Component<ComponentType::J0> u23{"u23"};
  Component<ComponentType::L> l1{"l1"};
  Component<ComponentType::J0> u3{"u3"};
  Component<ComponentType::GY> gy{"gy"};
  // The new ones
  Component<ComponentType::J1> o{"o"};
  Component<ComponentType::L> l2{"l2"};
  Component<ComponentType::R> r2{"r2"};
  Component<ComponentType::TF> tf{"tf"};
  Component<ComponentType::J1> v1{"v1"};
  Component<ComponentType::SE> se2{"se2"};
  Component<ComponentType::L> l3{"l3"};

  // Now add these to the bondgraph
  BondGraph bg;
  bg.addComponent(&se);
  bg.addComponent(&u0);
  bg.addComponent(&j1);
  bg.addComponent(&u12);
  bg.addComponent(&r);
  bg.addComponent(&u2);
  bg.addComponent(&j2);
  bg.addComponent(&u23);
  bg.addComponent(&l1);
  bg.addComponent(&u3);
  bg.addComponent(&gy);
  bg.addComponent(&o);
  bg.addComponent(&l2);
  bg.addComponent(&r2);
  bg.addComponent(&tf);
  bg.addComponent(&v1);
  bg.addComponent(&se2);
  bg.addComponent(&l3);

  // Now connect components
  bg.connect(se, u0);
  bg.connect(u0, j1);
  bg.connect(j1, u12);
  bg.connect(u12, r);
  bg.connect(j1, u2);
  bg.connect(u2, j2);
  bg.connect(j2, u23);
  bg.connect(u23, l1);
  bg.connect(j2, u3);
  bg.connect(u3, gy);
  bg.connect(gy, o);
  bg.connect(o, l2);
  bg.connect(o, r2);
  bg.connect(o, tf);
  bg.connect(tf, v1);
  bg.connect(v1, l3);
  bg.connect(se2, v1);

  // Try simplifying this graph
  bg.simplify(); // works fine.

  // Now do causal analysis
  bg.assignCausality();
  // Now produce the state space equations
  expressionAst ast = bg.generateStateSpace();
  const expression_t &res = l1.getStateEq(ast);
  std::cout << std::format("State eq l1: ");
  print_expression_t(std::cout, res, ast);
  std::cout << "\n";

  const expression_t &res2 = l2.getStateEq(ast);
  std::cout << std::format("State eq l2: ");
  print_expression_t(std::cout, res2, ast);
  std::cout << "\n";

  const expression_t &res3 = l3.getStateEq(ast);
  std::cout << std::format("State eq l3: ");  
  print_expression_t(std::cout, res3, ast);
  std::cout << "\n";
}
