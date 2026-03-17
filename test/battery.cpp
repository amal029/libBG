#include "BondGraph.hpp"
#include "Component.hpp"
#include "Solver.hpp"
#include "expression.hpp"
#include <cfloat>
#include <cstddef>
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

  // The battery bond graph
  BondGraph battery{"battery"};

  // The capacitors
  size_t c1id = battery.addComponent(Component<ComponentType::C>{"c1", battery.getID()});
  size_t c2id = battery.addComponent(Component<ComponentType::C>{"c2", battery.getID()});
  size_t c3id = battery.addComponent(Component<ComponentType::C>{"c3", battery.getID()});
  size_t ctheta1id = battery.addComponent(Component<ComponentType::C>{"ctheta1", battery.getID()});
  size_t ctheta2id =
    battery.addComponent(Component<ComponentType::C>{"ctheta2", battery.getID()});

  // The resistors
  size_t rthetaid = battery.addComponent(Component<ComponentType::R>{"rtheta", battery.getID()});
  size_t r1id = battery.addComponent(Component<ComponentType::R, Modulated::T>{"r1", battery.getID()});
  size_t r2id = battery.addComponent(Component<ComponentType::R, Modulated::T>{"r2", battery.getID()});
  size_t r3id = battery.addComponent(Component<ComponentType::R, Modulated::T>{"r3", battery.getID()});
  size_t rpid = battery.addComponent(Component<ComponentType::R>{"rp", battery.getID()});

  // The zero junctions
  size_t j0id = battery.addComponent(Component<ComponentType::J0>{"j0", battery.getID()});
  size_t j01id = battery.addComponent(Component<ComponentType::J0>{"j01", battery.getID()});
  size_t j02id = battery.addComponent(Component<ComponentType::J0>{"j02", battery.getID()});
  size_t j03id = battery.addComponent(Component<ComponentType::J0>{"j03", battery.getID()});
  size_t j04id = battery.addComponent(Component<ComponentType::J0>{"j04", battery.getID()});

  // The one junctions
  size_t j1id = battery.addComponent(Component<ComponentType::J1>{"j1", battery.getID()});
  size_t j11id = battery.addComponent(Component<ComponentType::J1>{"j11", battery.getID()});

  // The output component XXX: This should be later changed when the
  // whole model is put together.
  size_t outputid = battery.addComponent(
					 Component<ComponentType::SF, Modulated::T>{"output", battery.getID()});

  // The modulated se
  size_t seid = battery.addComponent(Component<ComponentType::SE, Modulated::T>{"se", battery.getID()});
  size_t sfid =
    battery.addComponent(Component<ComponentType::SF, Modulated::T>{"sf", battery.getID()});

  auto *sf = std::get_if<Component<ComponentType::SF, Modulated::T>>(
      &battery.getComponentAt(sfid));
  auto *output = std::get_if<Component<ComponentType::SF, Modulated::T>>(
      &battery.getComponentAt(outputid));

  auto *se = std::get_if<Component<ComponentType::SE, Modulated::T>>(
      &battery.getComponentAt(seid));

  auto *j0 =
      std::get_if<Component<ComponentType::J0>>(&battery.getComponentAt(j0id));
  auto *j01 =
      std::get_if<Component<ComponentType::J0>>(&battery.getComponentAt(j01id));
  auto *j02 =
      std::get_if<Component<ComponentType::J0>>(&battery.getComponentAt(j02id));
  auto *j03 =
      std::get_if<Component<ComponentType::J0>>(&battery.getComponentAt(j03id));
  auto *j04 =
      std::get_if<Component<ComponentType::J0>>(&battery.getComponentAt(j04id));

  auto *j1 =
      std::get_if<Component<ComponentType::J1>>(&battery.getComponentAt(j1id));
  auto *j11 =
      std::get_if<Component<ComponentType::J1>>(&battery.getComponentAt(j11id));
  
  auto *c1 =
      std::get_if<Component<ComponentType::C>>(&battery.getComponentAt(c1id));
  auto *c2 =
      std::get_if<Component<ComponentType::C>>(&battery.getComponentAt(c2id));
  auto *c3 =
      std::get_if<Component<ComponentType::C>>(&battery.getComponentAt(c3id));
  
  auto *r1 = std::get_if<Component<ComponentType::R, Modulated::T>>(
      &battery.getComponentAt(r1id));
  auto *r2 = std::get_if<Component<ComponentType::R, Modulated::T>>(
      &battery.getComponentAt(r2id));
  auto *r3 = std::get_if<Component<ComponentType::R, Modulated::T>>(
      &battery.getComponentAt(r3id));
  auto *rp =
      std::get_if<Component<ComponentType::R>>(&battery.getComponentAt(rpid));

  auto *ctheta1 = std::get_if<Component<ComponentType::C>>(
      &battery.getComponentAt(ctheta1id));
  auto *ctheta2 = std::get_if<Component<ComponentType::C>>(
      &battery.getComponentAt(ctheta2id));

  auto *rtheta = std::get_if<Component<ComponentType::R>>(
      &battery.getComponentAt(rthetaid));

  // Now make connect the components
  battery.connect(*sf, *j0);
  battery.connect(*j0, *ctheta1);

  // The effort of ctheta1 is given as output signal theta
  ctheta1->component2Signal("theta", Causality::Effort);

  battery.connect(*j0, *j1);
  battery.connect(*j1, *rtheta);
  battery.connect(*se, *j1);

  // Now connect the others
  battery.connect(*j01, *c1);
  battery.connect(*j01, *r1);
  battery.connect(*j02, *r2);
  battery.connect(*j02, *c2);
  battery.connect(*j03, *c3);
  battery.connect(*j03, *r3);

  // Now connect the 1 junction to all 0 junctions
  battery.connect(*j11, *j01);
  battery.connect(*j11, *j02);
  battery.connect(*j11, *j03);
  battery.connect(*j11, *ctheta2);
  ctheta2->component2Signal("q", Causality::Effort);

  // Now connect the j04 to j11
  battery.connect(*j04, *j11);
  battery.connect(*j04, *rp);
  battery.connect(*output, *j04);

  battery.simplify();

  // Assign causality
  battery.assignCausality();

  // Generate state space
  expressionAst ast = battery.generateStateSpace();
  // Get the equations
  print_state_eqns(ctheta1->getStateEq(ast), "Ctheta1", ast);
  print_state_eqns(ctheta2->getStateEq(ast), "Ctheta2", ast);
  print_state_eqns(c1->getStateEq(ast), "c1", ast);
  print_state_eqns(c2->getStateEq(ast), "c2", ast);
  print_state_eqns(c3->getStateEq(ast), "c3", ast);

  // The consts
  component_map_t<double> consts;
  consts[&battery.getComponentAt(seid)] = 22; // theta_a
  consts[&battery.getComponentAt(ctheta1id)] = 615.3;
  consts[&battery.getComponentAt(ctheta2id)] = 106360;
  consts[&battery.getComponentAt(rthetaid)] = 0.01;
  consts[&battery.getComponentAt(rpid)] = 500;
  consts[&battery.getComponentAt(c1id)] = 51.079;
  consts[&battery.getComponentAt(c2id)] = 51.216;
  consts[&battery.getComponentAt(c3id)] = 567.56;

  // The outputs that we want
  ctheta1->component2Signal("theta", Causality::Effort);
  ctheta2->component2Signal("q", Causality::Effort);
  r3->component2Signal("i3", Causality::Flow);
  r3->component2Signal("v3", Causality::Effort);
  r2->component2Signal("v2", Causality::Effort);
  r2->component2Signal("i2", Causality::Flow);
  r1->component2Signal("v1", Causality::Effort);
  r1->component2Signal("i1", Causality::Flow);
  rp->component2Signal("vp", Causality::Effort);
  rp->component2Signal("ip", Causality::Flow);
  // output.component2Signal("iB", Causality::Flow);
  

  // Moduldated input values
  sf->signal2ModulatedComponent("PB");
  r3->signal2ModulatedComponent("R3");
  r2->signal2ModulatedComponent("R2");
  r1->signal2ModulatedComponent("R1");
  output->signal2ModulatedComponent("InputB");

  // Generate modellica model
  battery.generateModellica(ast, std::move(consts));

  return 0;
}
