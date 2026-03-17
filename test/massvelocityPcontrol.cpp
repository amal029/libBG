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

#define N 10000
#define M 1
#define TSTOP 60

void print_state_eqns(const expression_t &res, const char *name,
                      expressionAst &ast) {
  std::cout << std::format("State eq {}: ", name);
  print_expression_t(std::cout, res, ast);
  std::cout << "\n";
}

// Use global variables for making things more efficient.
Solver<double> *gs = nullptr;
std::vector<double> xT(N, 0);

void toIntegrate(double t, double x[], double dxdt[]) {
  std::span<double> ptr(dxdt, gs->getComponentSize());
  std::memcpy(xT.data(), x, M * sizeof(double));
  gs->dxdt(xT, ptr); // getting the derivative
}

void integrate(Solver<double> &s) {
  gs = &s; // Set the global Solver

  std::vector<double> xv;
  // Reserve enough space in input
  xv.reserve(M * (N + 1));

  // Stack allocated
  double t[N + 1] = {0};

  // Call the integrator -- multi-step mode
  double tspan[2] = {0, TSTOP};
  double initial[M] = {0.1};
  euler(toIntegrate, tspan, initial, N, M, t, xv.data());
  std::ofstream file("/tmp/test3.csv");
  assert(file.is_open());
  file << "Time(sec),Velocity (m/sec)\n";
  for (size_t j = 0; j < N + 1; ++j) {
    file << t[j] << "," << xv[j * M] << "\n";
  }
  file.close();
}

int main(void) {
  BondGraph bg("Pcontrol", false);

  size_t mid   = bg.addComponent(Component<ComponentType::L> {"m", bg.getID()});
  size_t velid = bg.addComponent(Component<ComponentType::J1> {"vel", bg.getID()});
  size_t conid = bg.addComponent(Component<ComponentType::J0> {"con", bg.getID()});
  size_t j0id  = bg.addComponent(Component<ComponentType::J0> {"j0", bg.getID()});
  size_t kid   = bg.addComponent(Component<ComponentType::R> {"k", bg.getID()});
  size_t seid  = bg.addComponent(Component<ComponentType::SE> {"se", bg.getID()});
  size_t wid   = bg.addComponent(Component<ComponentType::SF>{"w", bg.getID()});
  size_t yid = bg.addComponent(Component<ComponentType::O>{"y", bg.getID()});

  auto *m = std::get_if<Component<ComponentType::L>>(&bg.getComponentAt(mid));
  auto *vel = std::get_if<Component<ComponentType::J1>>(&bg.getComponentAt(velid));
  auto *con = std::get_if<Component<ComponentType::J0>>(&bg.getComponentAt(conid));
  auto *j0 =
      std::get_if<Component<ComponentType::J0>>(&bg.getComponentAt(j0id));
  auto *k = std::get_if<Component<ComponentType::R>>(&bg.getComponentAt(kid));
  auto *se =
      std::get_if<Component<ComponentType::SE>>(&bg.getComponentAt(seid));
  auto *w = std::get_if<Component<ComponentType::SF>>(&bg.getComponentAt(wid));
  auto *y = std::get_if<Component<ComponentType::O>>(&bg.getComponentAt(yid));

  // Now connect the components
  bg.connect(*vel, *m);
  bg.connect(*vel, *y);
  bg.connect(*y, *j0);
  bg.connect(*se, *j0);
  bg.connect(*con, *vel);
  bg.connect(*w, *con);
  bg.connect(*con, *k);

  // Simplify the graph
  bg.simplify();
  // Causality analysis
  bg.assignCausality();
  // State space representation
  expressionAst ast = bg.generateStateSpace();
  // Print the storage elements
  print_state_eqns(m->getStateEq(ast), "m", ast);

  // Now just plot the thing
  const double mv = 20;
  const double tau = 10;

  component_map_t<double> consts;
  consts[&bg.getComponentAt(seid)] = 0;      // dummy input for effort
  consts[&bg.getComponentAt(wid)] = 2;     // refrence input for the controller (w)
  consts[&bg.getComponentAt(mid)] = mv;       // m value
  consts[&bg.getComponentAt(kid)] = mv / tau; // the P-control gain (kp)

  std::vector<storageVariant> storageComponents{m};
  Solver<double> s{ast, std::move(consts), storageComponents};
  print_state_eqns(m->getStateEq(ast), "m", ast);

  // Integrate using euler solver
  integrate(s);
  return 0;
}
