#pragma once

#include "BondGraph.hpp"
#include <cstddef>
#include <vector>

struct Bond {
  const BondGraph *in;
  const BondGraph *out;
  const size_t inport;
  const size_t outport;
};

class HierarchicalBondGraph {
public:
  HierarchicalBondGraph(const char *);
  HierarchicalBondGraph(const HierarchicalBondGraph &) = delete;
  HierarchicalBondGraph(HierarchicalBondGraph &&) = default;
  HierarchicalBondGraph &operator=(const HierarchicalBondGraph &) = delete;
  HierarchicalBondGraph &operator=(HierarchicalBondGraph &&) = default;
  ~HierarchicalBondGraph() {}

  // Method for adding a bond graph
  size_t addBondGraph(BondGraph &&);
  // Method for getting a bond graph
  const BondGraph *getBondGraphAt(size_t i) const;
  // Method to get the bond graph by name
  const BondGraph *getBondGraph(const char *name) const;
  // Add connection between two bondgraphs
  void addBond(const BondGraph *in, size_t inport, const BondGraph *out,
               size_t outport);
  // Get connection between two bondgraphs
  Bond getBondAt(size_t i);
  // This gives the bond from input --> output
  const Bond *getBondForInput(BondGraph *in, size_t inport) const;
  // This gives the reverse edge from output --> input
  const Bond *getBondForOutput(BondGraph *out, size_t outport) const;
  // Traverse the bond graph (DFS)
  template <typename F, bool PRE = false> void applyFunction(F f);
  const char *getName() const { return name; }

private:
  std::vector<BondGraph> bgs;
  std::vector<HierarchicalBondGraph *> hbgs;
  std::vector<Bond> bonds;
  const char *name;
};
