#pragma once

#include "Component.hpp"
#include "expression.hpp"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <format>
#include <iostream>
#include <numeric>
#include <ostream>
#include <stdexcept>
#include <string>
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
    if (!found) {
      throw NotFound(
          std::format("Component named {} not found!", name));
    }
    return std::get<Component<T>>(components[i]);
  }

  // Adding the edge between the components (also add the port for these
  // components)
  template <ComponentType X, ComponentType Y>
  void connect(Component<X> &in, Component<Y> &out) {
    // First assign the port for the in and out
    in.addPort(Port(PortType::OUT));
    out.addPort(Port(PortType::IN));
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
  constexpr componentVariant &getComponentAt(size_t i) {
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
      DFS(i, f);
    }
    // Then go through all the nodes that have not been visited yet --
    // possible.
    size_t i = 0;
    for (bool x : visited) {
      bool deleted = std::visit([](const auto &x) { return x.getDeleted(); },
                                components[i]);
      if (!x && !deleted) {
        DFS<Callable, PRE>(i, f); // Visit if not visited
      }
      ++i;
    }
    resetVisited();
  }

    // Simplify the graph
  void simplify() {
    std::vector<size_t> sources;
    getSources(sources);
    // Eliminate junctions
    auto visitorElim = [&](auto &x) { simplify1(x); };
    dfs(sources, visitorElim);
    // Contract junctions (step 2)
    auto visitorContract = [&](auto &x) { simplify2(x); };
    dfs(sources, visitorContract);
  }

  void assignCausality() {
    // First get all the sources
    std::vector<size_t> sources;
    getSources(sources); // These are all the sources
    // Now assign the ports for all the sources and their neighbor
    // junctions for all of these fixed causality sources.

    // There can be only a single output for a given source
    for (const auto &s : sources) {
      Causality in, out;
      // Now assign the effort and flow causality for this source (also sets
      // assigned for the port)
      ComponentType myT = std::visit([](const auto &x) { return x.getType(); },
                                     getComponentAt(s));
      if (myT == ComponentType::SE) {
        out = Causality::Effort;
        in = Causality::Flow;
      } else if (myT == ComponentType::SF) {
        out = Causality::Flow;
        in = Causality::Effort;
      } else {
        throw std::runtime_error(
            std::format("Source {} has incorrect type", s));
      }

      assignGYTFSourceOutoingCausality(s, out, in, false);
    }

    // Now do junction propagation for each source
    for (const auto &s : sources) {
      junctionPropagate(edges[s][0]);
    }
  }

  // This will generate the state space system for the bond graph
  void generateStateSpace() {
    expressionAst ast;
    // Get all the sources and move along the graph in dfs
    std::vector<size_t> sources;
    getSources(sources);
    auto visitor = [&](auto &x) { addToSpace(ast, x); };
    // This is by default pre-order traversal
    dfs(sources, visitor);
  }

  // Reset the visited flag
  void resetVisited() {
    for (size_t i = 0; i < visited.size(); ++i) {
      visited[i] = false;
    }
  }

  void printEdges() { printEdges(edges); }

private:
  void printEdges(std::vector<std::vector<size_t>> &edges) {
    std::cout << "[";
    for (size_t i = 0; i < edges.size(); ++i) {
      bool isDeleted = std::visit([](const auto &x) { return x.getDeleted(); },
                                  getComponentAt(i));
      if (isDeleted)
        continue;
      std::cout << i << "[";
      for (size_t j = 0; j < edges[i].size(); ++j) {
        std::cout << edges[i][j] << " ";
      }
      std::cout << "]\n";
    }
    std::cout << "]";
  }

  template <ComponentType T> void simplify1(Component<T> &x) {
    if ((T == ComponentType::J0 || T == ComponentType::J1) &&
        canElimJunction(x)) {
      elimJunction(x);
    }
  }

  template <ComponentType T> void simplify2(Component<T> &x) {
    size_t id = x.getID();
    if ((T == ComponentType::J0 || T == ComponentType::J1) &&
        canContractJunction(x)) {
      contractJunction(id);
    }
  }

  template <ComponentType T>
  constexpr void OutputsEqualInputs(expressionAst &space, Component<T> &x) {
    size_t myID = x.getID();
    const size_t numRedges = redges[myID].size();
    for (size_t i = 0; i < edges[myID].size(); ++i) {
      Port *myOutPort = x.getPort(i + numRedges);
      Port *nInPort = getCausalPort(edges[myID][i], myID);
      nInPort->setInExpression(myOutPort->getOutExpression());
      myOutPort->setInExpression(nInPort->getOutExpression());
    }
  }
  constexpr void addToSpace(expressionAst &space,
                            Component<ComponentType::SE> &x) {
    assert(x.portSize() == 1);
    Port *outport = x.getPort(0);
    assert(outport->getPortType() == PortType::OUT &&
           outport->getOutCausality() == Causality::Effort);
    // Now assign the source symbol to this port
    expression_t *p = space.append(make_expr(x.getValue()));
    outport->setOutExpression(p);
    OutputsEqualInputs(space, x);
  }
  void addToSpace(expressionAst &space, Component<ComponentType::SF> &x) {
    assert(x.portSize() == 1);
    Port *outport = x.getPort(0);
    assert(outport->getPortType() == PortType::OUT &&
           outport->getOutCausality() == Causality::Flow);
    // Now assign the source symbol to this port
    expression_t *p = space.append(make_expr(x.getValue()));
    outport->setOutExpression(p);
    OutputsEqualInputs(space, x);
  }
  void addToSpace(expressionAst &space, Component<ComponentType::C> &x) const {
    assert(x.portSize() == 1);
    Port *inport = x.getPort(0);
    assert(inport->getPortType() == PortType::IN);
    bool isIntegral = inport->getOutCausality() == Causality::Effort;
    if (isIntegral) {
      // This has integral causality.
      x.setInternalExpression(inport->getInExpression());
      // Now set the output of inPort
      // First push the Div operation into space.
      expression_t *value =
          space.append(make_expr(x.getValue())); // value copied here
      expression_t *div = space.append(
          make_expr(Expression<EOP::DIV>(inport->getInExpression(), value)));
      inport->setOutExpression(div);
    } else {
      // Differential causality
      throw NotYetImplemented("Differential equality for C");
    }
  }
  void addToSpace(expressionAst &space, Component<ComponentType::L> &x) const {
    assert(x.portSize() == 1);
    Port *inport = x.getPort(0);
    assert(inport->getPortType() == PortType::IN);
    bool isIntegral = inport->getOutCausality() == Causality::Flow;
    if (isIntegral) {
      expression_t *value =
          space.append(make_expr(x.getValue())); // value copied here
      expression_t *div = space.append(
          make_expr(Expression<EOP::DIV>(inport->getInExpression(), value)));
      inport->setOutExpression(div);
    } else {
      throw NotYetImplemented("Differential equality for L");
    }
  }

  // This function is shared between TF/GY for building the state space.
  void addToSpaceTFGY(expressionAst &space, Port *inport, Port *outport,
                      expression_t *value) {
    if (inport->getOutCausality() == Causality::Effort) {
      // outExpression of inport = inExpression of outport * value
      expression_t *res = space.append(
          make_expr(Expression<EOP::MUL>(outport->getInExpression(), value)));
      inport->setOutExpression(res);
      // Now do the flow
      // outputExpression of outport = inExpression of inport * value
      expression_t *res1 = space.append(
          make_expr(Expression<EOP::MUL>(inport->getInExpression(), value)));
      outport->setOutExpression(res1);
    } else {
      // This is the flow out from input..same as above, except div
      expression_t *res = space.append(
          make_expr(Expression<EOP::DIV>(outport->getInExpression(), value)));
      inport->setOutExpression(res);
      // Now do the flow
      // outputExpression of outport = inExpression of inport * value
      expression_t *res1 = space.append(
          make_expr(Expression<EOP::DIV>(inport->getInExpression(), value)));
      outport->setOutExpression(res1);
    }
  }
  void addToSpace(expressionAst &space, Component<ComponentType::TF> &x) {
    assert(x.portSize() == 2);
    Port *inport = x.getPort(0);
    assert(inport->getPortType() == PortType::IN);
    Port *outport = x.getPort(1);
    assert(outport->getPortType() == PortType::OUT);
    assert((inport->getOutCausality() == Causality::Effort &&
            outport->getOutCausality() == Causality::Effort) ||
           (inport->getOutCausality() == Causality::Flow &&
            outport->getOutCausality() == Causality::Flow));
    expression_t *value = space.append(make_expr(x.getValue()));
    addToSpaceTFGY(space, inport, outport, value);
    OutputsEqualInputs(space, x);
  }
  void addToSpace(expressionAst &space, Component<ComponentType::GY> &x) {
    assert(x.portSize() == 2);
    Port *inport = x.getPort(0);
    assert(inport->getPortType() == PortType::IN);
    Port *outport = x.getPort(1);
    assert(outport->getPortType() == PortType::OUT);
    assert((inport->getOutCausality() == Causality::Effort &&
            outport->getOutCausality() == Causality::Flow) ||
           (inport->getOutCausality() == Causality::Flow &&
            outport->getOutCausality() == Causality::Effort));
    expression_t *value = space.append(make_expr(x.getValue()));
    addToSpaceTFGY(space, inport, outport, value);
    OutputsEqualInputs(space, x);
  }
  void addToSpaceJ0J1(expressionAst &space, Port *mainPort,
                      std::vector<Port *> others) {
    for (Port *pp : others) {
      pp->setOutExpression(mainPort->getInExpression());
    }
    // Now handle the sum of flows
    std::vector<PortType> otherPortTypes;
    otherPortTypes.reserve(others.size());
    for (Port *pp : others) {
      otherPortTypes.push_back(pp->getPortType());
    }
    std::vector<expression_t *> nv;
    nv.reserve(others.size());
    // Now if the port type is output then transform its inputExpression
    for (size_t i = 0; i < others.size(); ++i) {
      Port *oport = others[i];
      PortType oport_t = otherPortTypes[i];
      if (oport_t == PortType::OUT) {
        expression_t *ee = space.append(make_expr(Number(-1)));
        expression_t *ee1 = space.append(
            make_expr(Expression<EOP::MUL>(ee, oport->getInExpression())));
        nv.push_back(ee1);
      } else {
        nv.push_back(oport->getInExpression());
      }
    }
    expression_t *res =
        // space[0] always holds a zero number
        std::accumulate(nv.begin(), nv.end(), space[0],
                        [&space](expression_t *init, expression_t *value) {
                          expression_t *r = space.append(
                              make_expr(Expression<EOP::ADD>(init, value)));
                          return r;
                        });
    mainPort->setOutExpression(res);
  }
  void addToSpace(expressionAst &space, Component<ComponentType::J0> &x) {
    // First get the main port with out causality of Effort
    Port *mainPort;
    std::vector<Port *> others;
    for (size_t i = 0; i < x.portSize(); ++i) {
      Port *pp = x.getPort(i);
      if (pp->getInCausality() == Causality::Effort) {
        mainPort = pp;
      } else {
        others.push_back(pp);
      }
    }
    assert(others.size() >=
           2); // There should be at least 2 non main type ports
    assert(mainPort->getOutCausality() == Causality::Flow);
    addToSpaceJ0J1(space, mainPort, others);
    // Now set the equality constraints for all the other outputs
    OutputsEqualInputs(space, x);
  }
  void addToSpace(expressionAst &space, Component<ComponentType::J1> &x) {
    // First get the main port with out causality of Effort
    Port *mainPort;
    std::vector<Port *> others;
    for (size_t i = 0; i < x.portSize(); ++i) {
      Port *pp = x.getPort(i);
      if (pp->getInCausality() == Causality::Flow) {
        mainPort = pp;
      } else {
        others.push_back(pp);
      }
    }
    assert(others.size() >=
           2); // There should be at least 2 non main type ports
    assert(mainPort->getOutCausality() == Causality::Effort);
    addToSpaceJ0J1(space, mainPort, others);
    // Now set the equality constraints for all the other outputs
    OutputsEqualInputs(space, x);
  }
  void addToSpace(expressionAst &space, Component<ComponentType::R> &x) const {
    assert(x.portSize() == 1);
    Port *inport = x.getPort(0);
    assert(inport->getPortType() == PortType::IN);
    bool isIntegral = inport->getOutCausality() == Causality::Effort;
    expression_t *value = space.append(make_expr(x.getValue()));
    expression_t *out;
    if (isIntegral) {
      out = space.append(
          make_expr(Expression<EOP::MUL>(inport->getInExpression(), value)));
    } else {
      out = space.append(
          make_expr(Expression<EOP::DIV>(inport->getInExpression(), value)));
    }
    inport->setOutExpression(out);
  }

  // There the causality is that of the output port
  void assignCausality(Port *out, Port *in, Causality caout, Causality cain) {
    out->setOutCausality(caout);
    out->setInCausality(cain);
    in->setInCausality(out->getOutCausality());
    in->setOutCausality(out->getInCausality());
    out->setAssigned();
    in->setAssigned();
  }
  bool
  isAssignedCausality(Causality x,
                      const std::vector<const Port *> &parentPortsAssigned) {
    bool toret = false;
    for (const Port *pp : parentPortsAssigned) {
      if (pp->getInCausality() == x) {
        toret = true;
        break;
      }
    }
    return toret;
  }
  // This will assign causality for the given port myPort looking at the
  // parent' port assignment.
  void componentAssignAndPropagateC(
      size_t myId, Port *myPort, Port *pPort, size_t pid,
      const std::vector<const Port *> &parentPortsAssigned,
      ComponentType pType) {

    auto integralCausality = [&]() {
      assignCausality(myPort, pPort, Causality::Effort, Causality::Flow);
    };

    auto diffCausality = [&]() {
      assignCausality(myPort, pPort, Causality::Flow, Causality::Effort);
    };

    if (pType == ComponentType::J1) {
      // Here I don't care about the type just assign preferred
      // causality -- effort out and flow in
      integralCausality();
    } else {
      // Now we need to make sure that the assigned causalities do not
      // already have an in Causality of Effort.
      if (isAssignedCausality(Causality::Effort, parentPortsAssigned)) {
        std::cerr << std::format("Assigning differential causality to {}",
                                 myId);
        diffCausality();
      } else {
        integralCausality();
      }
    }
    if (edges[myId].size() > 0) {
      throw NumEdges(
          std::format("Component {} cannot have an outgoing edge\n", myId));
    }
  }

  void componentAssignAndPropagateL(
      size_t myId, Port *myPort, Port *pPort, size_t pid,
      const std::vector<const Port *> &parentPortsAssigned,
      ComponentType pType) {

    auto integralCausality = [&]() {
      assignCausality(myPort, pPort, Causality::Flow, Causality::Effort);
    };

    auto diffCausality = [&]() {
      assignCausality(myPort, pPort, Causality::Effort, Causality::Flow);
    };

    if (pType == ComponentType::J0) {
      // Here I don't care about the type just assign preferred
      // causality -- effort out and flow in
      integralCausality();
    } else {
      // Now we need to make sure that the assigned causalities do not
      // already have an in Causality of Effort.
      if (isAssignedCausality(Causality::Flow, parentPortsAssigned)) {
        std::cerr << std::format("Assigning differential causality to {}",
                                 myId);
        diffCausality();
      } else {
        integralCausality();
      }
    }
    if (edges[myId].size() > 0) {
      throw NumEdges(
          std::format("Component {} cannot have an outgoing edge\n", myId));
    }
  }

  void componentAssignAndPropagateGY(
      size_t myId, Port *myPort, Port *pPort, size_t pid,
      const std::vector<const Port *> &parentPortsAssigned,
      ComponentType pType) {

    auto assignCausalityV = [&](Causality out, Causality in) {
      assignCausality(myPort, pPort, out, in);
    };
    // If pType is 0 then check if it already has an incoming effort. If
    // so, assign this myPort and pPort = flow, else effort.
    Causality in, out;

    if (pType == ComponentType::J0) {
      if (isAssignedCausality(Causality::Effort, parentPortsAssigned)) {
        assignCausalityV(Causality::Flow, Causality::Effort);
        out = Causality::Flow;
        in = Causality::Effort;
      } else {
        assignCausalityV(Causality::Effort, Causality::Flow);
        out = Causality::Effort;
        in = Causality::Flow;
      }
    } else if (pType == ComponentType::J1) {
      if (isAssignedCausality(Causality::Flow, parentPortsAssigned)) {
        assignCausalityV(Causality::Effort, Causality::Flow);
        out = Causality::Effort;
        in = Causality::Flow;
      } else {
        assignCausalityV(Causality::Flow, Causality::Effort);
        out = Causality::Flow;
        in = Causality::Effort;
      }
    } else {
      throw InCorrectComponentConnection(std::format(
          "Input of component {} not connected to a junction {}", myId, pid));
    }

    // Then propogate outwards from here
    assignGYTFSourceOutoingCausality(myId, out, in, true);
  }

  void componentAssignAndPropagateJ(
      size_t myId, Port *myPort, Port *pPort, size_t pid,
      const std::vector<const Port *> &parentPortsAssigned,
      ComponentType pType) {

    auto assignCausalityV = [&](Causality out, Causality in) {
      assignCausality(myPort, pPort, out, in);
    };

    if (pType == ComponentType::J0) {
      if (isAssignedCausality(Causality::Effort, parentPortsAssigned)) {
        assignCausalityV(Causality::Flow, Causality::Effort);
      } else {
        assignCausalityV(Causality::Effort, Causality::Flow);
      }
    } else if (pType == ComponentType::J1) {
      if (isAssignedCausality(Causality::Flow, parentPortsAssigned)) {
        assignCausalityV(Causality::Effort, Causality::Flow);
      } else {
        assignCausalityV(Causality::Flow, Causality::Effort);
      }
    } else {
      throw InCorrectComponentConnection(std::format(
          "Input of component {} not connected to a junction {}", myId, pid));
    }
    // Finally call junction propagate on yourself.
    junctionPropagate(myId);
  }

  void componentAssignAndPropagateTF(
      size_t myId, Port *myPort, Port *pPort, size_t pid,
      const std::vector<const Port *> &parentPortsAssigned,
      ComponentType pType) {

    auto assignCausalityV = [&](Causality out, Causality in) {
      assignCausality(myPort, pPort, out, in);
    };
    // If pType is 0 then check if it already has an incoming effort. If
    // so, assign this myPort and pPort = flow, else effort.
    Causality in, out;

    if (pType == ComponentType::J0) {
      if (isAssignedCausality(Causality::Effort, parentPortsAssigned)) {
        assignCausalityV(Causality::Flow, Causality::Effort);
        out = Causality::Effort;
        in = Causality::Flow;
      } else {
        assignCausalityV(Causality::Effort, Causality::Flow);
        out = Causality::Flow;
        in = Causality::Effort;
      }
    } else if (pType == ComponentType::J1) {
      if (isAssignedCausality(Causality::Flow, parentPortsAssigned)) {
        assignCausalityV(Causality::Effort, Causality::Flow);
        out = Causality::Flow;
        in = Causality::Effort;
      } else {
        assignCausalityV(Causality::Flow, Causality::Effort);
        out = Causality::Effort;
        in = Causality::Flow;
      }
    } else {
      throw InCorrectComponentConnection(std::format(
          "Input of component {} not connected to a junction {}", myId, pid));
    }
    // Then propogate outwards from here
    assignGYTFSourceOutoingCausality(myId, out, in, true);
  }

  void componentAssignAndPropagateR(
      size_t myId, Port *myPort, Port *pPort, size_t pid,
      const std::vector<const Port *> &parentPortsAssigned,
      ComponentType pType) {

    auto integralCausality = [&]() {
      assignCausality(myPort, pPort, Causality::Effort, Causality::Flow);
    };

    auto diffCausality = [&]() {
      assignCausality(myPort, pPort, Causality::Flow, Causality::Effort);
    };

    // If the parent junction is J0 and assigned an input effort then I
    // will give a flow out, else effort out
    if (pType == ComponentType::J0) {
      if (isAssignedCausality(Causality::Effort, parentPortsAssigned))
        diffCausality();
      else
        integralCausality();
    }

    // If the parent junction is J1 and given an input flow then I will
    // give an effort out, else flow out
    if (pType == ComponentType::J1) {
      if (isAssignedCausality(Causality::Flow, parentPortsAssigned))
        integralCausality();
      else
        diffCausality();
    }
    if (edges[myId].size() > 0) {
      throw NumEdges(
          std::format("Component {} cannot have an outgoing edge\n", myId));
    }
  }

  constexpr void junctionPropagate(size_t id) {
    // Go through all the neighbours and get neighbours that have not
    // yet have a causality assigned

    // The vector of ports that have been assigned already for Junction id
    std::vector<const Port *> assignedPorts;
    size_t numPorts = std::visit([](const auto &x) { return x.portSize(); },
                                 getComponentAt(id));
    for (size_t i = 0; i < numPorts; ++i) {
      const Port *p = std::visit([&i](const auto &x) { return x.getPort(i); },
                                 getComponentAt(id));
      if (p->getAssigned()) {
        assignedPorts.push_back(p);
      }
    }

    // If all ports have been assigned just return back
    if (assignedPorts.size() == numPorts)
      return;

    std::vector<size_t> rs;
    std::vector<Port *> rports;
    std::vector<Port *> myRports;

    // GY ports and GYs
    std::vector<size_t> gys;
    std::vector<Port *> gyports;
    std::vector<Port *> mygyports;

    // TFports and TFs
    std::vector<size_t> tfs;
    std::vector<Port *> tfports;
    std::vector<Port *> mytfports;

    // Junction ports
    std::vector<size_t> js;
    std::vector<Port *> jports;
    std::vector<Port *> myjports;

    ComponentType myType = std::visit([](const auto &x) { return x.getType(); },
                                      getComponentAt(id));
    const size_t numRedges = redges[id].size();
    componentVariant &me = getComponentAt(id);
    // Preferred causality neighbours first then indifferent causality
    for (size_t counter = 0; counter < edges[id].size(); ++counter) {
      size_t x = edges[id][counter];
      // IMPORTANT: This means that all reverse edge (incoming) ports
      // come first and then the outgoing (ports) edges come
      Port *myPort =
          std::visit([&counter, &numRedges](
                         auto &x) { return x.getPort(numRedges + counter); },
                     me);
      Port *nport = getCausalPort(x, id);
      // If port not yet assigned then
      if (!nport->getAssigned()) {
        // Is the component at x a preferred causality component?
        ComponentType nType = std::visit(
            [](const auto &x) -> ComponentType { return x.getType(); },
            getComponentAt(x));
        switch (nType) {
        case ComponentType::C: {
          componentAssignAndPropagateC(x, nport, myPort, id, assignedPorts,
                                       myType);
          break;
        }
        case ComponentType::L: {
          componentAssignAndPropagateL(x, nport, myPort, id, assignedPorts,
                                       myType);
          break;
        }
        case ComponentType::GY: {
          gys.push_back(x);
          gyports.push_back(nport);
          mygyports.push_back(myPort);
          break;
        }
        case ComponentType::TF: {
          tfs.push_back(x);
          tfports.push_back(nport);
          mytfports.push_back(myPort);
          break;
        }
        case ComponentType::R: {
          rs.push_back(x); // pushed to Resistor vector
          rports.push_back(nport);
          myRports.push_back(myPort);
          break;
        }
        case ComponentType::SE: {
          throw InCorrectComponentConnection(
              std::format("Outgoing edge from {} to an effort source!", id));
          break;
        }
        case ComponentType::SF: {
          throw InCorrectComponentConnection(
              std::format("Outgoing edge from {} to a flow source!", id));
          break;
        }
        case ComponentType::J0: {
          js.push_back(x);
          jports.push_back(nport);
          myjports.push_back(myPort);
          break;
        }
        case ComponentType::J1: {
          js.push_back(x);
          jports.push_back(nport);
          myjports.push_back(myPort);
          break;
        }
        }
      }
    }

    // The below are giving preference to how to assign causality to
    // different neighbours so that there is a high chance of everyone
    // getting their preferred causality.

    // Now assign to GY first
    for (size_t i = 0; i < gys.size(); ++i) {
      componentAssignAndPropagateGY(gys[i], gyports[i], mygyports[i], id,
                                    assignedPorts, myType);
    }
    // Now assign the TF ports
    for (size_t i = 0; i < tfs.size(); ++i) {
      componentAssignAndPropagateTF(tfs[i], tfports[i], mytfports[i], id,
                                    assignedPorts, myType);
    }
    // Now assign the junction ports
    for (size_t i = 0; i < js.size(); ++i) {
      componentAssignAndPropagateJ(js[i], jports[i], myjports[i], id,
                                   assignedPorts, myType);
    }
    // Now we assign causality to rs.
    for (size_t i = 0; i < rs.size(); ++i) {
      componentAssignAndPropagateR(rs[i], rports[i], myRports[i], id,
                                   assignedPorts, myType);
    }

    // Now check that every port in this junction is assigned and check
    // that constraints are satisfied.
    assignedPorts.clear();
    for (size_t i = 0; i < numPorts; ++i) {
      const Port *p = std::visit([&i](const auto &x) { return x.getPort(i); },
                                 getComponentAt(id));
      if (p->getAssigned()) {
        assignedPorts.push_back(p);
      }
    }
    if (assignedPorts.size() != numPorts) {
      throw JunctionAssignment(
          std::format("All of junction {}' port not assigned", id));
    }
    // All constraints on this junction are satisfied
    std::visit([](const auto &x) { x.satisfyConstraints(); },
               getComponentAt(id));
  }

  // This function gives the port of the id connected to parent ID (pid)
  // component
  constexpr Port *getCausalPort(size_t id, size_t pid) {
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
    assert(myPort->getPortType() == PortType::IN);
    return myPort;
  }

  // The shared function to assign Causality from GY/TF/Source to
  // outgoing junction
  void assignGYTFSourceOutoingCausality(size_t id, Causality out, Causality in,
                                        bool propagate) {
    if (edges[id].size() != 1) {
      throw NumEdges(
          std::format("Component {} has incorrect number of edges\n", id));
    }
    size_t numRedges = redges[id].size();
    // Now assign the given causality to the outgoing port
    Port *myPort =
        std::visit([&numRedges](auto &x) { return x.getPort(numRedges); },
                   getComponentAt(id));
    componentVariant &neighbour = getComponentAt(edges[id][0]);
    ComponentType neighbourType = std::visit(
        [](const auto &x) -> ComponentType { return x.getType(); }, neighbour);
    if (!(neighbourType == ComponentType::J0 ||
          neighbourType == ComponentType::J1)) {
      throw InCorrectComponentConnection(
          std::format("Component {} not connected to a Junction\n", id));
    }
    // Setting the causality of the junction connected to me
    Port *mynPort = getCausalPort(edges[id][0], id);
    // assign causality for me and my neighbour
    assignCausality(myPort, mynPort, out, in);
    if (propagate)
      junctionPropagate(edges[id][0]);

    // assignJunctionCausality(id, edges[id][0], myPort, propagate);
  }

  // XXX: Find the matching junction for simplification
  template <ComponentType T> constexpr bool canElimJunction(Component<T> &x) {
    bool toret = false;
    size_t numPorts = x.portSize();
    if (numPorts == 2)
      toret = (x.getPort(0)->getPortType() == PortType::IN &&
               x.getPort(1)->getPortType() == PortType::OUT) ||
              (x.getPort(0)->getPortType() == PortType::OUT &&
               x.getPort(1)->getPortType() == PortType::IN);
    return toret;
  }
  // Eliminate the junction
  template <ComponentType T> constexpr void elimJunction(Component<T> &x) {
    size_t id = x.getID();
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
    x.setDeleted();
    // std::visit([](auto &x) { x.setDeleted(); }, getComponentAt(id));
  }

  // Contracting junctions
  template <ComponentType T>
  constexpr bool canContractJunction(Component<T> &component) {
    size_t id = component.getID();
    // XXX: A junction with same type connected to another junction of
    // the same type.
    size_t counter = 0;
    for (size_t nid : edges[id]) {
      ComponentType nType = std::visit(
          [&](const auto &y) -> ComponentType { return y.getType(); },
          getComponentAt(nid));
      counter += (nType == T) ? 1 : 0;
    }
    return (counter == 1);
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
        std::visit([](auto &c) { c.addPort(Port(PortType::IN)); }, nComponent);
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
        std::visit([](auto &c) { c.addPort(Port(PortType::OUT)); }, nComponent);
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
  g.dfs(sources, visitor);
  return os;
}
