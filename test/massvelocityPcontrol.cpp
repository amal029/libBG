#include "BondGraph.hpp"
#include "Component.hpp"
#include "Solver.hpp"
#include "euler.hpp"
#include "expression.hpp"
#include <cfloat>
#include <cstring>
#include <fstream>
#include <iostream>
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
  file << "Time(sec), Velocity (m/sec)\n";
  for (size_t j = 0; j < N + 1; ++j) {
    file << t[j] << ", " << xv[j * M] << "\n";
  }
  file.close();
}

int main(void) {
  Component<ComponentType::L> i{"m"};
  Component<ComponentType::J1> vel{"vel"};
  Component<ComponentType::J0> con{"con"};
  Component<ComponentType::J0> j0{"j0"};
  Component<ComponentType::R> r{"k"};
  Component<ComponentType::SE> se{"se"};
  Component<ComponentType::SF> ref{"w"};
  Component<ComponentType::O> y{"y"};

  BondGraph bg;
  bg.addComponent(&i);   // inductor
  bg.addComponent(&vel); // velocity
  bg.addComponent(&j0);  // output connection
  bg.addComponent(&con); // controller
  bg.addComponent(&r);   // P-control
  bg.addComponent(&se);  // 0 volts
  bg.addComponent(&ref); // reference point for p-controller
  bg.addComponent(&y);   // output

  // Now connect the components
  bg.connect(vel, i);
  bg.connect(vel, y);
  bg.connect(y, j0);
  bg.connect(se, j0);
  bg.connect(con, vel);
  bg.connect(ref, con);
  bg.connect(con, r);

  // Simplify the graph
  bg.simplify();
  // Causality analysis
  bg.assignCausality();
  // State space representation
  expressionAst ast = bg.generateStateSpace();
  // Print the storage elements
  print_state_eqns(i.getStateEq(ast), "m", ast);

  // Now just plot the thing
  const double m = 20;
  const double tau = 10;

  component_map_t<double> consts;
  consts[&se] = 0;      // dummy input for effort
  consts[&ref] = 2;     // refrence input for the controller (w)
  consts[&i] = m;       // m value
  consts[&r] = m / tau; // the P-control gain (kp)

  std::vector<storageVariant> storageComponents{&i};
  Solver<double> s{ast, std::move(consts), storageComponents};
  print_state_eqns(i.getStateEq(ast), "m", ast);

  // Integrate using euler solver
  integrate(s);
  return 0;
}
