#include "BondGraph.hpp"
#include "Component.hpp"
#include "Solver.hpp"
#include "expression.hpp"
#include "rkf45.hpp"
#include <cfloat>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <semaphore>
#include <unordered_map>
#include <vector>

void print_state_eqns(const expression_t &res, const char *name,
                      expressionAst &ast) {
  std::cout << std::format("State eq {}: ", name);
  print_expression_t(std::cout, res, ast);
  std::cout << "\n";
}

// Use global variables for making things more efficient.
Solver<double> *gs = nullptr;
std::vector<double> xv;

void toIntegrate(double t, double x[], double dxdt[]) {

  size_t N = gs->getComponentSize();
  memmove(xv.data(), x, N * sizeof(double));
  std::vector<double> res;                            // overhead
  gs->dxdt(xv, res);                                  // getting the derivative
  std::memmove(dxdt, res.data(), N * sizeof(double)); // getting the result back
}

// Declare the function that will be called by the integrator
void integrate(Solver<double> &s) {
  gs = &s; // Set the global Solver
  int neqn = s.getComponentSize();
  // Reserve enough space in input
  xv.reserve(neqn);

  double *Y = new double[neqn];
  double *YP = new double[neqn];
  double t = 0; // The current value of time.
  double tout = 1;
  double abserr = sqrt(DBL_EPSILON);
  double relerr = sqrt(DBL_EPSILON);
  int FLAG = -1;

  // Initial values
  Y[0] = 0;
  Y[1] = 1;
  Y[2] = 2;
  Y[3] = 10;

  // Call the integrator -- one step mode
  while (FLAG != 2) {
    FLAG = r8_rkf45(toIntegrate, neqn, Y, YP, &t, tout, &relerr, abserr, -1);
    std::cout << t << ": ";
    std::cout << "[";
    for (size_t i = 0; i < (size_t)neqn; ++i) {
      std::cout << Y[i] << " ";
    }
    std::cout << "]\n";
  }

  delete[] Y;
  delete[] YP;
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
  Solver<double> s{ast, std::move(consts), storageComponents};
  // Print the things again
  print_state_eqns(res, "l1", ast);
  print_state_eqns(res2, "l2", ast);
  print_state_eqns(resc, "C", ast);
  print_state_eqns(res3, "l3", ast);

  // Integrate using RK45 solver
  integrate(s);
}
