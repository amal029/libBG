#pragma once

#include "Component.hpp"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <format>
#include <iostream>
#include <ostream>
#include <sstream>
#include <variant>
#include <vector>

// This is the variant with all the different components
using componentVariant =
    std::variant<Component<ComponentType::C>, Component<ComponentType::L>,
                 Component<ComponentType::J0>, Component<ComponentType::J1>,
                 Component<ComponentType::R>, Component<ComponentType::SE>,
                 Component<ComponentType::SF>, Component<ComponentType::GY>,
                 Component<ComponentType::TF>>;

struct BondGraph {
  // Public methods
  BondGraph() {}
  BondGraph(const BondGraph &) = delete;
  BondGraph(BondGraph &&) = default;
  BondGraph &operator=(const BondGraph &) = delete;
  BondGraph &operator=(BondGraph &&) = default;
  virtual ~BondGraph() {}

  // Adding components to the graph
  template <ComponentType T> void addComponent(Component<T> &&x) {
    // We need to push the pointer to the correct position according to
    // the ID of the component.
    size_t els = x.getID();
    if (components.size() <= els) {
      // Then increase the size of the vector
      components.resize(els + 1);
      // Reserve space for the edges too
      edges.resize(els + 1, {});
      redges.resize(els + 1, {});
      // Reserve space in visited vector too
      visited.resize(els + 1, false);
    }
    components[els] = std::move(x); // add it to the correct position
  }

  // Get the component you want from the Bond Graph
  template <ComponentType T> Component<T> &getComponent(const char *name) {
    bool found = true;
    size_t i = 0;
    for (; i < components.size(); ++i) {
      // Here we need to visit the variant type and get the name of the
      // thing.
      found = std::visit(
          [&name](auto &x) -> bool {
            return (std::strcmp(x.getName(), name) == 0) && (x.getType() == T);
          },
          components[i]);
      if (found)
        break;
    }
    assert(found);
    return std::get<Component<T>>(components[i]);
  }

  // Adding the edge between the components (also add the port for these
  // components)
  template <ComponentType X, ComponentType Y>
  void connect(Component<X> &in, Component<Y> &out) {
    // First assign the port for the in and out
    in.addPort(Port());
    out.addPort(Port());
    // Then update the adjacency list
    edges[in.getID()].push_back(out.getID());
    redges[out.getID()].push_back(in.getID()); // The reverse edge.
  }

  [[nodiscard]]
  // Get the component from the vector
  const componentVariant &getComponentAt(size_t i) const {
    return components[i];
  }

  [[nodiscard]]
  componentVariant &getComponentAt(size_t i) {
    return components[i];
  }

  // Get the size of the component vector
  size_t getNumComponents() const { return components.size(); }

  // Get all the source IDs in the graph
  void getSources(std::vector<size_t> &sources) const {
    using source_e = Component<ComponentType::SE>;
    using source_f = Component<ComponentType::SF>;

    for (size_t i = 0; i < getNumComponents(); ++i) {
      auto &vv = getComponentAt(i);
      if (std::holds_alternative<source_e>(vv)) {
        sources.push_back(std::get<source_e>(vv).getID());
      } else if (std::holds_alternative<source_f>(vv)) {
        sources.push_back(std::get<source_f>(vv).getID());
      }
    }
  }

  // DFS traverse the graph from the sources and apply a function to each of the
  // node
  template <typename Callable, bool PRE = true>
  void dfs(const std::vector<size_t> &sources, Callable &f) {
    for (size_t i : sources) {
      DFS<Callable, PRE>(i, f);
    }
    // Then go through all the nodes that have not been visited yet --
    // possible.
    bool res =
        std::all_of(visited.begin(), visited.end(), [](bool x) { return x; });
    if (!res) {
      size_t i = 0;
      for (bool x : visited) {
        if (!x) {
          DFS<Callable, PRE>(i, f); // Visit if not visited
        }
        ++i;
      }
    }
  }

  // Simplify the graph
  void simplify() {
    std::vector<size_t> done;
    done.reserve(components.size()); // Just to make it faster
    bool madeChange;
    while (true) {
      madeChange = false;
      for (size_t i = 0; i < components.size(); ++i) {
        size_t id =
            std::visit([](const auto &x) { return x.getID(); }, components[i]);
        bool res = std::find(done.begin(), done.end(), id) != done.end();
        if (canElimJunction(id) && res) {
          elimJunction(id);
          madeChange = true;
        }
        if (canContractJunction(id) && res) {
          contractJunction(id);
          madeChange = true;
        }
        // Add id to done
        done.push_back(id);
      }
      // If no change was made in the graph then just break out.
      if (!madeChange)
        break;
    }
  }

  void assignCausality() {
    // First get all the sources
    std::vector<size_t> sources;
    getSources(sources); // These are all the sources
    // Now assign the ports for all the sources and their neighbor
    // junctions for all of these fixed causality sources.

    // There can be only a single output for a given source
    for (const auto &s : sources) {
      if (edges[s].size() != 1) {
        throw NumEdges("Source has incorrect number of ports\n");
      }
      // Now assign the effort and flow causality for this source (also sets
      // assigned for the port)
      std::visit([](auto &x) { x.assignSourceCausality(); }, getComponentAt(s));
      // Now assign the ports for the neighbouring junction
      componentVariant &neighbour = getComponentAt(edges[s][0]);
      ComponentType neighbourType =
          std::visit([](const auto &x) -> ComponentType { return x.getType(); },
                     neighbour);
      if (neighbourType != ComponentType::J0 &&
          neighbourType != ComponentType::J1) {
        std::string message =
            std::format("Source {} not conneted to a Junction\n", s);
        throw InCorrectComponentConnection(message);
      }
      // Now assign causality of the incoming port to the neighbours
      const Port *myPort =
          std::visit([](const auto &x) -> const Port * { return x.getPort(0); },
                     getComponentAt(s));
      assignJunctionCausality(s, edges[s][0], myPort, false);
    }
    // FIXME: Here call Junction causality from every random source for
    // propagation across the graph with propagate = true.
  }

private:
  // This will assign causality for the given port myPort looking at the
  // parent' port assignment.
  void componentAssignAndPropagate(size_t myId, Port *myPort, size_t pid,
                                   std::vector<const Port *> &parentPorts) {
    // The parent should always be a junction This should go back to the
    // junction for assignment as soon as the assignment for this port
    // is over -- call assignJunctioncausality
  }

  void junctionPropagate(size_t id) {
    // Go through all the neighbours and get neighbours that have not
    // yet have a causality assigned

    // The vector of ports that have been assigned already for Junction id
    std::vector<const Port *> assignedPorts;
    for (size_t i = 0; i < edges.size(); ++i) {
      const Port *p = std::visit([&i](const auto &x) { return x.getPort(i); },
                                 getComponentAt(id));
      if (p->getAssigned())
        assignedPorts.push_back(p);
    }

    std::vector<size_t> rs;
    std::vector<Port *> rports;
    // Preferred causality neighbours first then indifferent causality
    for (size_t x : edges[id]) {
      Port *nport = getCausalPort(x, id);
      // If port not yet assigned then
      if (!nport->getAssigned()) {
        // Is the component at x a preferred causality component?
        if (std::visit([](const auto &x)
                           -> bool { return x.getType() != ComponentType::R; },
                       getComponentAt(x))) {
          // Then we want to assign causality to this port of the neighbouring
          // component
          componentAssignAndPropagate(x, nport, id, assignedPorts);
        } else {
          rs.push_back(x); // pushed to Resistor vector
          rports.push_back(nport);
        }
      }
    }
    // Now we assign causality to rs.
    for (size_t i = 0; i < rs.size(); ++i) {
      componentAssignAndPropagate(rs[i], rports[i], id, assignedPorts);
    }
  }

  // This function gives the port of the id connected to parent ID (pid)
  // component
  Port *getCausalPort(size_t id, size_t pid) {
    // Here pid is the id of the parent I should be connected to.
    // Get the index of the pid from redges
    auto res = std::find(redges[id].begin(), redges[id].end(), pid);
    if (res == redges[id].end()) {
      throw InCorrectComponentConnection(
          std::format("Junction {} not connected to {}", id, pid));
    }
    size_t index = res - redges[id].begin();
    // Now index will be the port that will be used to assign causality
    Port *myPort =
        std::visit([&index](auto &x) -> Port * { return x.getPort(index); },
                   getComponentAt(id));
    return myPort;
  }

  // Junction causality assignment
  void assignJunctionCausality(size_t pid, size_t id, const Port *pport,
                               bool propagate = true) {
    Port *myPort = getCausalPort(id, pid);
    // Set the causality of this junction
    myPort->setInCausality(pport->getOutCausality());
    myPort->setOutCausality(pport->getInCausality());
    myPort->setAssigned();
    // Now check that the causality constraints of this junction are
    // satisfied.
    std::visit([](const auto &x) { x.satisfyConstraints(); },
               getComponentAt(id));
    if (propagate) {
      junctionPropagate(id);
    }
  }
  // XXX: Find the matching junction for simplification
  bool canElimJunction(size_t id) {
    bool toret = false;
    // Check if this component is a junction
    auto &component = getComponentAt(id);
    if (std::holds_alternative<Component<ComponentType::J0>>(component) ||
        std::holds_alternative<Component<ComponentType::J1>>(component)) {
      // Check the port size
      size_t numPorts = std::visit(
          [](const auto &x) -> size_t { return x.portSize(); }, component);
      if (numPorts == 2) {
        // Now check that one is an input and one is an output
        toret = (edges[id].size() == 1) && (redges[id].size() == 1);
      }
    }
    return toret;
  }
  // Eliminate the junction
  void elimJunction(size_t id) {
    size_t prevId = redges[id][0];
    size_t nextId = edges[id][0];

    // Get the index of the id in edges of previd
    auto index = std::find(edges[prevId].begin(), edges[prevId].end(), id);
    assert(index != edges[prevId].end());
    // Now replace the index value with nextid
    *index = nextId;

    // Replace nextid' reverse edges id --> previd
    auto index1 = std::find(redges[nextId].begin(), redges[nextId].end(), id);
    assert(index1 != redges[nextId].end());
    *index1 = prevId;

    // Here we set the deleted to true for this component
    std::visit([](auto &x) { x.setDeleted(); }, getComponentAt(id));
  }

  // Contracting junctions
  bool canContractJunction(size_t id) {
    bool toret = false;
    // XXX: A junction with same type connected to another junction of
    // the same type.

    auto &component = getComponentAt(id);
    if (std::holds_alternative<Component<ComponentType::J0>>(component) ||
        std::holds_alternative<Component<ComponentType::J1>>(component)) {
      // Is this junction connected to another junction of the same type
      ComponentType idType =
          std::visit([](const auto &x) -> ComponentType { return x.getType(); },
                     component);
      size_t counter = 0;
      for (size_t nid : edges[id]) {
        ComponentType nType = std::visit(
            [&](const auto &y) -> ComponentType { return y.getType(); },
            getComponentAt(nid));
        counter += (nType == idType) ? 1 : 0;
      }
      toret = counter == 1;
    }
    return toret;
  }

  void contractJunction(size_t id) {
    // Get my type
    ComponentType mType =
        std::visit([](const auto &x) -> ComponentType { return x.getType(); },
                   getComponentAt(id));
    assert(mType == ComponentType::J0 || mType == ComponentType::J1);
    size_t nid = 0;
    // First get the nid with the same type
    for (auto x = edges[id].begin(); x != edges[id].end(); ++x) {
      const componentVariant &comp = getComponentAt(*x);
      bool res = std::visit(
          [&mType](const auto &y) { return y.getType() == mType; }, comp);
      if (res) {
        nid = *x;
        break;
      }
    }
    // Now connect all the inputs from id to nid' input
    for (size_t x : redges[id]) {
      auto rindex = std::find(edges[x].begin(), edges[x].end(), id);
      assert(rindex != edges[x].end());
      *rindex = nid; // reset it to the next same Junction node.
      // Now add 'x' to nid's redges if it is already not there
      auto niter = std::find(redges[nid].begin(), redges[nid].end(), x);
      if (niter == redges[nid].end()) {
        redges[nid].push_back(x);
        // Add port to nid component
        componentVariant &nComponent = getComponentAt(nid);
        std::visit([](auto &c) { c.addPort(Port()); }, nComponent);
      }
    }

    // Now re-attach all the outputs from id to nid
    for (size_t x : edges[id]) {
      if (x == nid)
        continue;
      auto findex = std::find(redges[x].begin(), redges[x].end(), id);
      assert(findex != redges[x].end());
      *findex = nid;
      // Now add a forward edge from nid to x if it is not already there
      auto niter = std::find(edges[nid].begin(), edges[nid].end(), x);
      if (niter == edges[nid].end()) {
        edges[nid].push_back(x);
        // Add port to nid component
        componentVariant &nComponent = getComponentAt(nid);
        std::visit([](auto &c) { c.addPort(Port()); }, nComponent);
      }
    }
    // Set this node to deleted
    std::visit([](auto &x) { x.setDeleted(); }, getComponentAt(id));
  }

  // XXX: Depth first traversal with default pre-order traversal
  template <typename Callable, bool PRE = true>
  void DFS(size_t i, Callable &f) {
    if (visited[i])
      return;
    visited[i] = true;
    // Pre-order traversal
    if (PRE)
      std::visit([&f](auto &x) { f(x); }, components[i]);
    // Go through all the children of these nodes
    for (size_t j : edges[i]) {
      DFS<Callable, PRE>(j, f);
    }
    if (!PRE)
      std::visit([&f](auto &x) { f(x); }, components[i]);
  }

  // The vector of all the components in the graph -- it is closed not an open
  // variant
  std::vector<componentVariant> components;
  // Adjacency list of the graph
  std::vector<std::vector<size_t>> edges;
  std::vector<std::vector<size_t>> redges; // reverse edges
  // The visited node vector
  std::vector<bool> visited;
};

static std::ostream &operator<<(std::ostream &os, BondGraph &g) {
  // First get all the sources
  std::vector<size_t> sources;
  g.getSources(sources);
  // Then visit the graph and apply the print function to everything
  auto visitor = [&os](auto &x) { os << x << "\n"; };
  g.dfs<decltype(visitor), true>(sources, visitor);
  return os;
}
