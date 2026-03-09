#include "BondGraph.hpp"
#include "Component.hpp"
#include "Solver.hpp"
#include "expression.hpp"
#include "rkf45.hpp"
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iostream>
#include <span>
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

// We need this function signature, because the library demands it.
void toIntegrate(double t, double x[], double dxdt[]) {

  std::span<double> ptr(dxdt, gs->getComponentSize());
  gs->dxdt(xv, ptr); // getting the derivative
}

// Integrator for the system
void integrate(Solver<double> &s) {
  gs = &s; // Set the global Solver
  int neqn = s.getComponentSize();
  // Reserve enough space in input
  xv.reserve(neqn);

  // This is heap allocated to avoid VLA
  double *YP = new double[neqn];
  double t = 0; // The current value of time.
  double tout = 50;
  double abserr = sqrt(DBL_EPSILON);
  double relerr = sqrt(DBL_EPSILON);
  int FLAG = -1;

  // Initial values
  xv.push_back(0);
  xv.push_back(1);
  xv.push_back(2);
  xv.push_back(10);

  // Write the result to a file
  std::ofstream file("/tmp/test2.csv");
  assert(file.is_open());
  // First write the very first line
  // The header
  file << "Time(sec),L1,L2,C,L3" << "\n";
  file << t << "," << xv[0] << "," << xv[1] << "," << xv[2] << "," << xv[3]
       << "\n";
  // Call the integrator -- one step mode
  while (FLAG != 2) {
    FLAG = r8_rkf45(toIntegrate, neqn, xv.data(), YP, &t, tout, &relerr, abserr,
                    -1);
    file << t << "," << xv[0] << "," << xv[1] << "," << xv[2] << "," << xv[3]
         << "\n";
  }

  file.close();

  delete[] YP;
}

int main() {
  BondGraph bg("hoistingDevice");
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

  // This is the capacitance junction
  size_t v2id = bg.addComponent(Component<ComponentType::J0>{"v2"});
  size_t cid = bg.addComponent(Component<ComponentType::C>{"c"});

  size_t v1id = bg.addComponent(Component<ComponentType::J1>{"v1"});
  size_t se2id = bg.addComponent(Component<ComponentType::SE>{"se2"});
  size_t l3id = bg.addComponent(Component<ComponentType::L>{"l3"});

  // Now add these to the bondgraph
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
  auto *v2 =
      std::get_if<Component<ComponentType::J0>>(&bg.getComponentAt(v2id));
  auto *c = std::get_if<Component<ComponentType::C>>(&bg.getComponentAt(cid));
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

  bg.connect(*tf, *v2);
  bg.connect(*v2, *c);
  bg.connect(*v2, *v1);
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
  const expression_t &resc = c->getStateEq(ast);
  const expression_t &res3 = l3->getStateEq(ast);

  print_state_eqns(res, "l1", ast);
  print_state_eqns(res2, "l2", ast);
  print_state_eqns(resc, "C", ast);
  print_state_eqns(res3, "l3", ast);

  // Get the slope of a given state equation
  // Make the consts first
  component_map_t<double> consts;
  consts[&bg.getComponentAt(seid)] = 1.0;
  consts[&bg.getComponentAt(rid)] = 1;
  consts[&bg.getComponentAt(l1id)] = 2;
  consts[&bg.getComponentAt(gyid)] = 2;
  consts[&bg.getComponentAt(l2id)] = 2;
  consts[&bg.getComponentAt(r2id)] = 1;
  consts[&bg.getComponentAt(tfid)] = -2;
  consts[&bg.getComponentAt(cid)] = 2;
  consts[&bg.getComponentAt(se2id)] = -2;
  consts[&bg.getComponentAt(l3id)] = 2;

  std::vector<storageVariant> storageComponents{l1, l2, c, l3};
  Solver<double> s{ast, std::move(consts), storageComponents};
  // Print the things again
  print_state_eqns(res, "l1", ast);
  print_state_eqns(res2, "l2", ast);
  print_state_eqns(resc, "C", ast);
  print_state_eqns(res3, "l3", ast);

  // Integrate using RK45 solver
  integrate(s);
  // Generate modellica code
  component_map_t<double> constsm;
  constsm[&bg.getComponentAt(seid)] = 1.0;
  constsm[&bg.getComponentAt(rid)] = 1;
  constsm[&bg.getComponentAt(l1id)] = 2;
  constsm[&bg.getComponentAt(gyid)] = 2;
  constsm[&bg.getComponentAt(l2id)] = 2;
  constsm[&bg.getComponentAt(r2id)] = 1;
  constsm[&bg.getComponentAt(tfid)] = -2;
  constsm[&bg.getComponentAt(cid)] = 2;
  constsm[&bg.getComponentAt(se2id)] = -2;
  constsm[&bg.getComponentAt(l3id)] = 2;

  // The initial values for this system
  storage_map_t<double> initialValues{{l1, 0}, {l2, 1}, {c, 1}, {l3, 10}};
  bg.generateModellica(ast, std::move(constsm), std::move(initialValues));
  return 0;
}
