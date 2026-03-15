#include "HierarchicalBondGraph.hpp"
#include "BondGraph.hpp"
#include <cassert>
#include <cstddef>
#include <cstring>
#include <vector>

HierarchicalBondGraph::HierarchicalBondGraph(const char *name) {
  this->name = name;
}

size_t HierarchicalBondGraph::addBondGraph(BondGraph &&b) {
  bgs.emplace_back(std::move(b));
  return bgs.size() - 1;
}

const BondGraph *HierarchicalBondGraph::getBondGraphAt(size_t i) const {
  assert(i > 0 && "index > 0" && i < bgs.size() && "index < max size");
  return &bgs[i];
}

template <typename F, bool PRE> void HierarchicalBondGraph::applyFunction(F f) {
  if (PRE) {
    // Pre order traversal
    for (size_t i = 0; i < bgs.size(); ++i) {
      f(getBondGraphAt(i));
    }
  }

  // Go to children and apply the function to them.
  for (size_t i = 0; i < hbgs.size(); ++i) {
    hbgs[i]->applyFunction(f);
  }

  if (!PRE) {
    // Post order traversal
    for (size_t i = 0; i < bgs.size(); ++i) {
      f(getBondGraphAt(i));
    }
  }
}

const BondGraph *HierarchicalBondGraph::getBondGraph(const char *name) const {
  const BondGraph *toret = nullptr;
  for (size_t j = 0; j < bgs.size(); ++j) {
    if (std::strcmp(bgs[j].getName(), name) == 0) {
      toret = &bgs[j]; // This should not be invalidated
      break;
    }
  }
  if (toret == nullptr) {
    // Now traverse the hbgs to get the required bond graph
    for (size_t j = 0; j < hbgs.size(); ++j) {
      toret = hbgs[j]->getBondGraph(name);
      if (toret != nullptr)
        break;
    }
  }
  return toret;
}

void HierarchicalBondGraph::addBond(const BondGraph *in, size_t inport,
                                    const BondGraph *out, size_t outport) {
  bonds.emplace_back(Bond{in, out, inport, outport});
}

const Bond *HierarchicalBondGraph::getBondForInput(BondGraph *in,
                                                   size_t inport) const {
  const Bond *toret = nullptr;
  for (size_t i = 0; i < bonds.size(); ++i) {
    if (in == bonds[i].in && inport == bonds[i].inport) {
      toret = &bonds[i];
      break;
    }
  }
  return toret;
}

const Bond *HierarchicalBondGraph::getBondForOutput(BondGraph *out,
                                                    size_t outport) const {
  const Bond *toret = nullptr;
  for (size_t i = 0; i < bonds.size(); ++i) {
    if (out == bonds[i].out && outport == bonds[i].outport) {
      toret = &bonds[i];
      break;
    }
  }
  return toret;
}
