#include "BondGraph.hpp"
#include "Component.hpp"
#include "Solver.hpp"
#include "euler.hpp"
#include "expression.hpp"
#include <cfloat>
#include <cstring>
#include <fstream>
#include <iostream>
#include <variant>
#include <vector>

void print_state_eqns(const expression_t &res, const char *name,
                      expressionAst &ast) {
  std::cout << std::format("State eq {}: ", name);
  print_expression_t(std::cout, res, ast);
  std::cout << "\n";
}

// Use global variables for making things more efficient.
Solver<double> *gs = nullptr;
std::vector<double> xT(2, 0);

// We need this function signature, because the library demands it.
void toIntegrate(double t, double x[], double dxdt[]) {
  std::span<double> ptr(dxdt, gs->getComponentSize());
  std::memcpy(xT.data(), x, 2 * sizeof(double));
  gs->dxdt(xT, ptr); // getting the derivative
}

// Integrator for the system
void integrate(Solver<double> &s) {
  const int N = 10000;
  gs = &s; // Set the global Solver
  const int M = s.getComponentSize();
  
  std::vector<double> xv;
  // Reserve enough space in input
  xv.reserve(M * (N + 1));

  // Stack allocated
  double t[N + 1] = {0};

  // Call the integrator -- multi-step mode
  assert(M == 2);
  double tspan[2] = {0, 2};
  double initial[2] = {0, 0};
  euler(toIntegrate, tspan, initial, N, 2, t, xv.data());
  std::ofstream file("/tmp/test1.csv");
  assert(file.is_open());
  file << "Time(sec),L,C\n";
  for (size_t j = 0; j < N + 1; ++j) {
    file << t[j] << "," << xv[j * M] << "," << xv[1 + j * M] << "\n";
  }
  file.close();
}

int main() {
  BondGraph bg("RLClowpass");
  size_t seid = bg.addComponent(Component<ComponentType::SE>{"se"});
  size_t rid = bg.addComponent(Component<ComponentType::R>{"r"});
  size_t cid = bg.addComponent(Component<ComponentType::C>{"c"});
  size_t lid = bg.addComponent(Component<ComponentType::L>{"l"});
  size_t j1id = bg.addComponent(Component<ComponentType::J1>{"j1"});
  size_t j0id = bg.addComponent(Component<ComponentType::J0>{"j0"});

  auto *se =
      std::get_if<Component<ComponentType::SE>>(&bg.getComponentAt(seid));
  auto *j1 =
      std::get_if<Component<ComponentType::J1>>(&bg.getComponentAt(j1id));
  auto *r = std::get_if<Component<ComponentType::R>>(&bg.getComponentAt(rid));
  auto *c = std::get_if<Component<ComponentType::C>>(&bg.getComponentAt(cid));
  auto *l = std::get_if<Component<ComponentType::L>>(&bg.getComponentAt(lid));
  auto *j0 =
      std::get_if<Component<ComponentType::J0>>(&bg.getComponentAt(j0id));
  
  // Now make the connections
  bg.connect(*se, *j1);
  bg.connect(*j1, *l);
  bg.connect(*j1, *j0);
  bg.connect(*j0, *c);
  bg.connect(*j0, *r);

  // Test try to simplify it
  bg.simplify();
  // Perform causality analysis
  bg.assignCausality();

  // Get the state equations
  expressionAst ast = bg.generateStateSpace();
  // Print the state equations
  print_state_eqns(l->getStateEq(ast), "L", ast);
  print_state_eqns(c->getStateEq(ast), "C", ast);

  // Set the constant value
  // XXX: These should be expression types
  component_map_t<double> consts;
  consts[&bg.getComponentAt(seid)] = 24;
  consts[&bg.getComponentAt(cid)] = 1e-3;
  consts[&bg.getComponentAt(lid)] = 1;
  consts[&bg.getComponentAt(rid)] = 100;

  std::vector<storageVariant> storageComponents{l, c};
  Solver<double> s{ast, std::move(consts), storageComponents};
  print_state_eqns(l->getStateEq(ast), "L", ast);
  print_state_eqns(c->getStateEq(ast), "C", ast);

  // Integrate using euler solver
  integrate(s);
  // Now generate the modellica code
  component_map_t<double> constsM;
  constsM[&bg.getComponentAt(seid)] = 24;
  constsM[&bg.getComponentAt(cid)] = 1e-3;
  constsM[&bg.getComponentAt(lid)] = 1;
  constsM[&bg.getComponentAt(rid)] = 100;

  bg.generateModellica(ast, std::move(constsM));
  return 0;
}
