#include "BondGraph.hpp"
#include "Component.hpp"
#include <cstddef>
#include <iostream>
#include <unordered_map>
#include <variant>
#include <vector>
#include "Solver.hpp"
#include "expression.hpp"

void print_state_eqns(const expression_t &res, const char *name,
                      expressionAst &ast) {
  std::cout << std::format("State eq {}: ", name);
  print_expression_t(std::cout, res, ast);
  std::cout << "\n";
}

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

  // This is the capacitance junction
  Component<ComponentType::J0> v2{"v2"};
  Component<ComponentType::C> c{"c"};

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
  // Add the extra components
  bg.addComponent(&v2);
  bg.addComponent(&c);

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

  bg.connect(tf, v2);
  bg.connect(v2, c);
  bg.connect(v2, v1);
  bg.connect(se2, v1);
  bg.connect(v1, l3);

  // Modify these
  // bg.connect(tf, v1);
  // bg.connect(v1, l3);
  // bg.connect(se2, v1);

  // Try simplifying this graph
  bg.simplify(); // works fine.

  // Now do causal analysis
  bg.assignCausality();

  // Now produce the state space equations
  expressionAst ast = bg.generateStateSpace();
  const expression_t &res = l1.getStateEq(ast);
  const expression_t &res2 = l2.getStateEq(ast);
  const expression_t &resc = c.getStateEq(ast);
  const expression_t &res3 = l3.getStateEq(ast);

  print_state_eqns(res, "l1", ast);
  print_state_eqns(res2, "l2", ast);
  print_state_eqns(resc, "C", ast);
  print_state_eqns(res3, "l3", ast);

  // Get the slope of a given state equation
  // Make the consts first
  component_map_t<double> consts;
  consts[&se] = 1.0;
  consts[&r] = 1;
  consts[&l1] = 2;
  consts[&gy] = 2;
  consts[&l2] = 2;
  consts[&r2] = 1;
  consts[&tf] = -2;
  consts[&c] = 2;
  consts[&se2] = -2;
  consts[&l3] = 2;
  std::vector<storageVariant> storageComponents{&l1, &l2, &c, &l3};
  std::vector<double> initValues{0, 1, 2, 10};
  Solver<double> s{ast, std::move(consts), storageComponents};
  // Print the things again
  print_state_eqns(res, "l1", ast);
  print_state_eqns(res2, "l2", ast);
  print_state_eqns(resc, "C", ast);
  print_state_eqns(res3, "l3", ast);

  // IMPORTANT: Note that the initValues and result will be in order
  // declared for storageComponents above.
  std::vector<double> result;
  s.dxdt(initValues, result);
  std::cout << "DxDt\n";
  for (size_t counter = 0; counter < storageComponents.size(); ++counter) {
    const char *name = std::visit([](auto const &x) { return x->getName(); },
                                  storageComponents[counter]);
    std::cout << name << ": " << result[counter] << "\n";
  }
}
