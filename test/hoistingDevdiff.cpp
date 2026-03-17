#include "BondGraph.hpp"
#include "Component.hpp"
#include "Solver.hpp"
#include "expression.hpp"
#include <cfloat>
#include <cstddef>
#include <cstring>
#include <iostream>

void print_state_eqns(const expression_t &res, const char *name,
                      expressionAst &ast) {
  std::cout << std::format("State eq {}: ", name);
  print_expression_t(std::cout, res, ast);
  std::cout << "\n";
}

int main() {
  BondGraph bg("hoistingDevicediff");
  size_t seid = bg.addComponent(Component<ComponentType::SE>{"se"});
  size_t u0id = bg.addComponent(Component<ComponentType::J0>{"u0"});
  size_t j1id = bg.addComponent(Component<ComponentType::J1>{"j1"});
  size_t u12id = bg.addComponent(Component<ComponentType::J0>{"u12"});
  size_t rid = bg.addComponent(Component<ComponentType::R>{"r"});
  size_t u2id = bg.addComponent(Component<ComponentType::J0>{"u2"});
  size_t j2id = bg.addComponent(Component<ComponentType::J1>{"j2"});
  size_t u23id = bg.addComponent(Component<ComponentType::J0>{"u23"});
  size_t l1id = bg.addComponent(Component<ComponentType::L>{"l1"});
  size_t u3id = bg.addComponent(Component<ComponentType::J0>{"u3"});
  size_t gyid = bg.addComponent(Component<ComponentType::GY>{"gy"});
  // The new ones
  size_t oid = bg.addComponent(Component<ComponentType::J1>{"o"});
  size_t l2id = bg.addComponent(Component<ComponentType::L>{"l2"});
  size_t r2id = bg.addComponent(Component<ComponentType::R>{"r2"});
  size_t tfid = bg.addComponent(Component<ComponentType::TF>{"tf"});

  size_t v1id = bg.addComponent(Component<ComponentType::J1>{"v1"});
  size_t se2id = bg.addComponent(Component<ComponentType::SE>{"se2"});
  size_t l3id = bg.addComponent(Component<ComponentType::L>{"l3"});

  auto *se =
      std::get_if<Component<ComponentType::SE>>(&bg.getComponentAt(seid));
  auto *u0 =
      std::get_if<Component<ComponentType::J0>>(&bg.getComponentAt(u0id));
  auto *j1 =
      std::get_if<Component<ComponentType::J1>>(&bg.getComponentAt(j1id));
  auto *u12 =
      std::get_if<Component<ComponentType::J0>>(&bg.getComponentAt(u12id));
  auto *r = std::get_if<Component<ComponentType::R>>(&bg.getComponentAt(rid));
  auto *u2 =
      std::get_if<Component<ComponentType::J0>>(&bg.getComponentAt(u2id));
  auto *j2 =
      std::get_if<Component<ComponentType::J1>>(&bg.getComponentAt(j2id));
  auto *u23 =
      std::get_if<Component<ComponentType::J0>>(&bg.getComponentAt(u23id));
  auto *l1 = std::get_if<Component<ComponentType::L>>(&bg.getComponentAt(l1id));
  auto *u3 =
      std::get_if<Component<ComponentType::J0>>(&bg.getComponentAt(u3id));
  auto *gy =
      std::get_if<Component<ComponentType::GY>>(&bg.getComponentAt(gyid));
  auto *o = std::get_if<Component<ComponentType::J1>>(&bg.getComponentAt(oid));
  auto *l2 = std::get_if<Component<ComponentType::L>>(&bg.getComponentAt(l2id));
  auto *r2 = std::get_if<Component<ComponentType::R>>(&bg.getComponentAt(r2id));
  auto *tf =
      std::get_if<Component<ComponentType::TF>>(&bg.getComponentAt(tfid));
  auto *v1 =
      std::get_if<Component<ComponentType::J1>>(&bg.getComponentAt(v1id));
  auto *se2 =
      std::get_if<Component<ComponentType::SE>>(&bg.getComponentAt(se2id));
  auto *l3 = std::get_if<Component<ComponentType::L>>(&bg.getComponentAt(l3id));

  // Now connect components
  bg.connect(*se, *u0);
  bg.connect(*u0, *j1);
  bg.connect(*j1, *u12);
  bg.connect(*u12, *r);
  bg.connect(*j1, *u2);
  bg.connect(*u2, *j2);
  bg.connect(*j2, *u23);
  bg.connect(*u23, *l1);
  bg.connect(*j2, *u3);
  bg.connect(*u3, *gy);
  bg.connect(*gy, *o);
  bg.connect(*o, *l2);
  bg.connect(*o, *r2);
  bg.connect(*o, *tf);

  bg.connect(*tf, *v1);
  bg.connect(*se2, *v1);
  bg.connect(*v1, *l3);

  // Try simplifying this graph
  bg.simplify(); // works fine.

  // Now do causal analysis
  bg.assignCausality();

  // Now produce the state space equations
  expressionAst ast = bg.generateStateSpace();
  const expression_t &res = l1->getStateEq(ast);
  const expression_t &res2 = l2->getStateEq(ast);
  const expression_t &res3 = l3->getStateEq(ast);

  print_state_eqns(res, "l1", ast);
  print_state_eqns(res2, "l2", ast);
  print_state_eqns(res3, "l3", ast);

  // Generate modellica code
  component_map_t<double> constsm;
  constsm[&bg.getComponentAt(seid)] = 1.0;
  constsm[&bg.getComponentAt(rid)] = 1;
  constsm[&bg.getComponentAt(l1id)] = 2;
  constsm[&bg.getComponentAt(gyid)] = 2;
  constsm[&bg.getComponentAt(l2id)] = 2;
  constsm[&bg.getComponentAt(r2id)] = 1;
  constsm[&bg.getComponentAt(tfid)] = -2;
  constsm[&bg.getComponentAt(se2id)] = -2;
  constsm[&bg.getComponentAt(l3id)] = 2;

  // Set some outputs for plotting?
  l1->component2Signal("l1", Causality::Flow);
  l2->component2Signal("l2", Causality::Flow);
  l3->component2Signal("l3", Causality::Flow);

  // The initial values for this system
  storage_map_t<double> initialValues{{l1, 1}, {l2, 0}, {l3, 0}};
  bg.generateModellica(ast, std::move(constsm), std::move(initialValues));
  return 0;
}
