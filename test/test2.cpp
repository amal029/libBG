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
  bg.addComponent(std::move(se));
  bg.addComponent(std::move(u0));
  bg.addComponent(std::move(j1));
  bg.addComponent(std::move(u12));
  bg.addComponent(std::move(r));
  bg.addComponent(std::move(u2));
  bg.addComponent(std::move(j2));
  bg.addComponent(std::move(u23));
  bg.addComponent(std::move(l1));
  bg.addComponent(std::move(u3));
  bg.addComponent(std::move(gy));
  bg.addComponent(std::move(o));
  bg.addComponent(std::move(l2));
  bg.addComponent(std::move(r2));
  bg.addComponent(std::move(tf));
  bg.addComponent(std::move(v1));
  bg.addComponent(std::move(se2));
  bg.addComponent(std::move(l3));

  // Now connect components
  bg.connect(bg.getComponent<ComponentType::SE>("se"),
             bg.getComponent<ComponentType::J0>("u0"));
  bg.connect(bg.getComponent<ComponentType::J0>("u0"),
             bg.getComponent<ComponentType::J1>("j1"));
  bg.connect(bg.getComponent<ComponentType::J1>("j1"),
             bg.getComponent<ComponentType::J0>("u12"));
  bg.connect(bg.getComponent<ComponentType::J0>("u12"),
             bg.getComponent<ComponentType::R>("r"));
  bg.connect(bg.getComponent<ComponentType::J1>("j1"),
             bg.getComponent<ComponentType::J0>("u2"));
  bg.connect(bg.getComponent<ComponentType::J0>("u2"),
             bg.getComponent<ComponentType::J1>("j2"));
  bg.connect(bg.getComponent<ComponentType::J1>("j2"),
             bg.getComponent<ComponentType::J0>("u23"));
  bg.connect(bg.getComponent<ComponentType::J0>("u23"),
             bg.getComponent<ComponentType::L>("l1"));
  bg.connect(bg.getComponent<ComponentType::J1>("j2"),
             bg.getComponent<ComponentType::J0>("u3"));
  bg.connect(bg.getComponent<ComponentType::J0>("u3"),
             bg.getComponent<ComponentType::GY>("gy"));
  bg.connect(bg.getComponent<ComponentType::GY>("gy"),
             bg.getComponent<ComponentType::J1>("o"));
  bg.connect(bg.getComponent<ComponentType::J1>("o"),
             bg.getComponent<ComponentType::L>("l2"));
  bg.connect(bg.getComponent<ComponentType::J1>("o"),
             bg.getComponent<ComponentType::R>("r2"));
  bg.connect(bg.getComponent<ComponentType::J1>("o"),
             bg.getComponent<ComponentType::TF>("tf"));
  bg.connect(bg.getComponent<ComponentType::TF>("tf"),
             bg.getComponent<ComponentType::J1>("v1"));
  bg.connect(bg.getComponent<ComponentType::J1>("v1"),
             bg.getComponent<ComponentType::L>("l3"));
  bg.connect(bg.getComponent<ComponentType::SE>("se2"),
             bg.getComponent<ComponentType::J1>("v1"));

  // Try simplifying this graph
  bg.simplify(); // works fine.

  // Now do causal analysis
  bg.assignCausality();
  // Now produce the state space equations
  expressionAst ast = bg.generateStateSpace();
  const expression_t &res =
      bg.getComponent<ComponentType::L>("l1").getStateEq(ast);
  std::cout << std::format("State eq l1: ");
  print_expression_t(std::cout, res, ast);
  std::cout << "\n";

  const expression_t &res2 =
      bg.getComponent<ComponentType::L>("l2").getStateEq(ast);
  std::cout << std::format("State eq l2: ");
  print_expression_t(std::cout, res2, ast);
  std::cout << "\n";

  const expression_t &res3 =
      bg.getComponent<ComponentType::L>("l3").getStateEq(ast);
  std::cout << std::format("State eq l3: ");  
  print_expression_t(std::cout, res3, ast);
  std::cout << "\n";
}
