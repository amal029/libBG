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
  // The capacitors
  Component<ComponentType::C> c1{"c1"};
  Component<ComponentType::C> c2{"c2"};
  Component<ComponentType::C> c3{"c3"};
  Component<ComponentType::C> ctheta1{"ctheta1"};
  Component<ComponentType::C> ctheta2{"ctheta2"};
  

  // The resistors
  Component<ComponentType::R> rtheta{"rtheta"};
  Component<ComponentType::R, Modulated::T> r1{"r1"};
  Component<ComponentType::R, Modulated::T> r2{"r2"};
  Component<ComponentType::R, Modulated::T> r3{"r3"};
  Component<ComponentType::R> rp{"rp"};

  // The zero junctions
  Component<ComponentType::J0> j0{"j0"};
  Component<ComponentType::J0> j01{"j01"};
  Component<ComponentType::J0> j02{"j02"};
  Component<ComponentType::J0> j03{"j03"};
  Component<ComponentType::J0> j04{"j04"};

  // The one junctions
  Component<ComponentType::J1> j1{"j1"};
  Component<ComponentType::J1> j11{"j11"};

  // The output component XXX: This should be later changed when the
  // whole model is put together.
  Component<ComponentType::SF, Modulated::T> output{"output"};

  // The modulated se
  Component<ComponentType::SE, Modulated::T> se{"se"};
  Component<ComponentType::SF, Modulated::T> sf{"sf"};

  // The battery bond graph
  BondGraph battery{"battery"};

  // Then add the components to the battery
  battery.addComponent(&ctheta1); // 0
  battery.addComponent(&ctheta2); // 1
  battery.addComponent(&c1);      // 2
  battery.addComponent(&c2);
  battery.addComponent(&c3);
  battery.addComponent(&rtheta);
  battery.addComponent(&r1);
  battery.addComponent(&r2);
  battery.addComponent(&r3);
  battery.addComponent(&j0);
  battery.addComponent(&j01);
  battery.addComponent(&j02);
  battery.addComponent(&j03);
  battery.addComponent(&j04);
  battery.addComponent(&j1);
  battery.addComponent(&j11);
  battery.addComponent(&output);
  battery.addComponent(&se);
  battery.addComponent(&sf);
  battery.addComponent(&rp);

  // Now make connect the components
  battery.connect(sf, j0);
  battery.connect(j0, ctheta1);
  // The effort of ctheta1 is given as output signal theta
  ctheta1.component2Signal("theta", Causality::Effort);

  battery.connect(j0, j1);
  battery.connect(j1, rtheta);
  battery.connect(se, j1);

  // Now connect the others
  battery.connect(j01, c1);
  battery.connect(j01, r1);
  battery.connect(j02, r2);
  battery.connect(j02, c2);
  battery.connect(j03, c3);
  battery.connect(j03, r3);

  // Now connect the 1 junction to all 0 junctions
  battery.connect(j11, j01);
  battery.connect(j11, j02);
  battery.connect(j11, j03);
  battery.connect(j11, ctheta2);
  ctheta2.component2Signal("q", Causality::Effort);

  // Now connect the j04 to j11
  battery.connect(j04, j11);
  battery.connect(j04, rp);
  battery.connect(output, j04);

  // Give the output from all the r1, r2, r3, and rp resistors
  r1.component2Signal("vr1", Causality::Effort);
  r1.component2Signal("ir1", Causality::Flow);
  r2.component2Signal("vr2", Causality::Effort);
  r2.component2Signal("ir2", Causality::Flow);
  r3.component2Signal("vr3", Causality::Effort);
  r3.component2Signal("ir3", Causality::Flow);
  rp.component2Signal("vrp", Causality::Effort);
  rp.component2Signal("irp", Causality::Flow);

  battery.simplify();

  // Assign causality
  battery.assignCausality();

  // Generate state space
  expressionAst ast = battery.generateStateSpace();
  // Get the equations
  print_state_eqns(ctheta1.getStateEq(ast), "Ctheta1", ast);
  print_state_eqns(ctheta2.getStateEq(ast), "Ctheta2", ast);
  print_state_eqns(c1.getStateEq(ast), "c1", ast);
  print_state_eqns(c2.getStateEq(ast), "c2", ast);
  print_state_eqns(c3.getStateEq(ast), "c3", ast);

  // The consts
  component_map_t<double> consts;
  consts[&se] = 22; // theta_a
  consts[&ctheta1] = 615.3;
  consts[&ctheta2] = 615.3;
  consts[&rtheta] = 0.01;
  consts[&rp] = 500;
  consts[&c1] = 51.079;
  consts[&c2] = 51.216;
  consts[&c3] = 567.56;

  // The outputs that we want
  ctheta1.component2Signal("theta", Causality::Effort);
  ctheta2.component2Signal("q", Causality::Effort);
  r3.component2Signal("i3", Causality::Flow);
  r3.component2Signal("v3", Causality::Effort);
  r2.component2Signal("v2", Causality::Effort);
  r2.component2Signal("i2", Causality::Flow);
  r1.component2Signal("v1", Causality::Effort);
  r1.component2Signal("i1", Causality::Flow);
  rp.component2Signal("vp", Causality::Effort);
  rp.component2Signal("ip", Causality::Flow);
  // output.component2Signal("iB", Causality::Flow);
  

  // Moduldated input values
  sf.signal2ModulatedComponent("PB");
  r3.signal2ModulatedComponent("R3");
  r2.signal2ModulatedComponent("R2");
  r1.signal2ModulatedComponent("R1");
  output.signal2ModulatedComponent("InputB");

  // Generate modellica model
  battery.generateModellica(ast, std::move(consts));

  return 0;
}
