#pragma once

#include "Component.hpp"
#include <cassert>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <ostream>
#include <variant>
#include <vector>

// This is the variant with all the different components
using componentVariant =
    std::variant<Component<ComponentType::C>, Component<ComponentType::L>,
                 Component<ComponentType::J0>, Component<ComponentType::J1>,
                 Component<ComponentType::R>, Component<ComponentType::SE>,
                 Component<ComponentType::SF>>;

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
  }

  // Get the component from the vector
  const componentVariant &getComponentAt(size_t i) const {
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
  template <typename Callable>
  void dfs(const std::vector<size_t> &sources, Callable &&f) const {
    for (size_t i : sources) {
      DFS(i, f);
    }
  }

private:
  template <typename Callable> void DFS(size_t i, Callable &&f) {
    visited[i] = true;
    // Pre-order traversal
    f(components[i]);
    // Go through all the children of these nodes
    for (size_t j : edges[i]) {
      if (!visited[j])
        DFS(j, f);
    }
  }

  // The vector of all the components in the graph -- it is closed not an open
  // variant
  std::vector<componentVariant> components;
  // Adjacency list of the graph
  std::vector<std::vector<size_t>> edges;
  // The visited node vector
  std::vector<bool> visited;
};

static std::ostream &operator<<(std::ostream &os, const BondGraph &g) {
  for (size_t i = 0; i < g.getNumComponents(); ++i) {
    std::visit([&os](const auto &x) -> void { os << x; }, g.getComponentAt(i));
    if (i < g.getNumComponents() - 1) {
      os << ", ";
    }
    os << "\n";
  }
  return os;
}
