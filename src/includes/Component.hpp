#pragma once

#include "exception.hpp"
#include "expression.hpp"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <iostream>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

struct Util {
  static size_t getID() {
    static size_t counter = 0;
    return counter++;
  }
};

// The type of components that are allowed in the Bond Graph
enum class ComponentType : std::uint8_t {
  C = 0,
  L,
  R,
  SE,
  SF,
  GY,
  TF,
  J0,
  J1,
  I,
  O
};

enum class Modulated { F = 0, T = 1 };

// The different types of causality
enum class Causality : int { Flow = 0, Effort, ACausal };

// The type of port, IN or OUT
enum class PortType : std::uint8_t { IN = 0, OUT };

struct IO {
  const std::string &output;
  const std::string_view input;
  const ComponentType t;
  const Causality c;
};

// The port structure of the component
struct Port {
  Port(PortType t, size_t ID)
      : in(Causality::ACausal), out(Causality::ACausal), nID(ID), mType(t) {}
  constexpr bool getAssigned() const { return assigned; }
  void setAssigned() { assigned = true; }
  Port(const Port &) = delete;
  Port(Port &&) = default;
  Port &operator=(const Port &) = delete;
  Port &operator=(Port &&) = default;
  ~Port() {}

  // The public functions
  constexpr void setInCausality(Causality c) { in = c; }
  constexpr void setOutCausality(Causality c) { out = c; }
  constexpr Causality getInCausality() const { return in; }
  constexpr Causality getOutCausality() const { return out; }
  constexpr PortType getPortType() const { return mType; }
  constexpr void setOutExpression(expression_t *v) { outx = v; }
  constexpr void setInExpression(expression_t *v) { inx = v; }
  constexpr expression_t *getOutExpression() const { return outx; }
  constexpr expression_t *getInExpression() const { return inx; }
  constexpr size_t getNeighbourID() const { return nID; }
  constexpr void setNeighbourID(size_t nid) { nID = nid; }

  // The below will be called when setting the causality of the port.
  constexpr void setInCausalName(std::string &&in) {
    inCausalName = std::move(in);
  }
  constexpr void setOutCausalName(std::string &&out) {
    outCausalName = std::move(out);
  }
  constexpr std::string_view getInCausalName() const { return inCausalName; }
  constexpr std::string_view getOutCausalName() const { return outCausalName; }

private:
  std::string inCausalName;
  std::string outCausalName;
  // The symbolic values (expressions) each of these ports hold
  expression_t *inx = nullptr;
  expression_t *outx = nullptr;
  Causality in = Causality::ACausal;  // The in causality
  Causality out = Causality::ACausal; // The out causality for this port.
  size_t nID = 0;                     // neighbour that I am connected to
  PortType mType;
  bool assigned = false;
};

// The common Component class
template <ComponentType T, Modulated M = Modulated::F> struct Component {
  constexpr Component() {} // This is for Bond Graph insertion
  constexpr Component(const char *n) : name(n), ID(Util::getID()), myT(T) {
    value = (name + std::string("_") + std::to_string(ID));
  }
  constexpr Component(const Component &) = delete;
  constexpr Component(Component &&) = default;
  constexpr Component &operator=(const Component &) = delete;
  constexpr Component &operator=(Component &&) = default;
  ~Component() {}

  // The public methods
  constexpr const char *getName() const { return name; }
  constexpr size_t getID() const { return ID; }
  constexpr ComponentType getType() const { return myT; }
  constexpr Modulated getModulated() const { return modulatedT; }
  constexpr void getModulated(std::vector<IO> &toret) const {
    if (modulate_signal.has_value()) {
      IO in{modulate_signal.value(), getValue(), T, Causality::ACausal};
      toret.emplace_back(std::move(in));
    }
  }
  constexpr void getIO(std::vector<IO> &toret) const {
    if (effort_signal.has_value()) {
      IO in{effort_signal.value(), getEffort(), T, Causality::Effort};
      toret.emplace_back(std::move(in));
    }
    if (flow_signal.has_value()) {
      IO in{flow_signal.value(), getFlow(), T, Causality::Flow};
      toret.emplace_back(std::move(in));
    }
  }

  constexpr size_t portSize() const { return ports.size(); }
  constexpr std::vector<Port>::iterator portBegin() { return ports.begin(); }
  constexpr std::vector<Port>::iterator portEnd() { return ports.end(); }
  constexpr const Port *getPort(size_t i) const {
    assert(i < ports.size());
    return &ports[i];
  }
  constexpr Port *getPort(size_t i) {
    assert(i < ports.size());
    return &ports[i];
  }
  constexpr std::vector<Port *> getPortWithType(PortType p) {
    std::vector<Port *> toret;
    toret.reserve(ports.size());
    for (size_t i = 0; i < ports.size(); ++i) {
      if (ports[i].getPortType() == p) {
        toret.emplace_back(&ports[i]);
      }
    }
    return toret;
  }
  constexpr Port *getPortWithNeighbourID(size_t nid) {
    Port *toret = nullptr;
    for (size_t i = 0; i < ports.size(); ++i) {
      if (ports[i].getNeighbourID() == nid) {
        toret = &ports[i];
        break;
      }
    }
    return toret;
  }
  constexpr void setPort(size_t i, Port &&p) { ports[i] = std::move(p); }
  constexpr void addPort(Port &&p) { ports.push_back(std::move(p)); }
  constexpr void remPort(size_t nID) {
    auto torem =
        std::find_if(ports.begin(), ports.end(), [&nID](const Port &p) {
          return p.getNeighbourID() == nID;
        });
    assert(torem != ports.end());
    ports.erase(torem);
  }
  // Deleted means deleted from the bond graph
  constexpr void setDeleted() { deleted = true; }
  constexpr bool getDeleted() const { return deleted; }
  constexpr const std::string &getValue() const { return value; }

  // XXX: These should be only for non junctions
  constexpr std::string_view getEffort() const {
    if constexpr (T == ComponentType::C || T == ComponentType::L ||
                  T == ComponentType::R || T == ComponentType::SE ||
                  T == ComponentType::SF) {
      // Get the input port
      return getEffortFlow(Causality::Effort);
    } else if constexpr (T == ComponentType::O) {
      if (portSize() != 1) {
        throw std::runtime_error(std::format(
            "Cannot get effort of connected component {}", getID()));
      } else {
        return getEffortFlow(Causality::Effort);
      }
    } else if constexpr (T == ComponentType::I) {
      if (portSize() != 1) {
        throw std::runtime_error(std::format(
            "Cannot get effort of connected component {}", getID()));
      } else {
        return getEffortFlow(Causality::Effort);
      }
    } else
      throw std::runtime_error(
          std::format("Cannot get effort and flow of component {}", getID()));
  }
  // get the flow variable
  constexpr std::string_view getFlow() const {
    if constexpr (T == ComponentType::C || T == ComponentType::L ||
                  T == ComponentType::R || T == ComponentType::SE ||
                  T == ComponentType::SF) {
      // Get the input port
      return getEffortFlow(Causality::Flow);
    } else if constexpr (T == ComponentType::O) {
      if (portSize() != 1) {
        throw std::runtime_error(std::format(
            "Cannot get effort of connected  component {}", getID()));
      } else {
        return getEffortFlow(Causality::Flow);
      }
    } else if constexpr (T == ComponentType::I) {
      if (portSize() != 1) {
        throw std::runtime_error(std::format(
            "Cannot get effort of connected component {}", getID()));
      } else {
        return getEffortFlow(Causality::Flow);
      }
    } else
      throw std::runtime_error(
          std::format("Cannot get effort and flow of component {}", getID()));
  }

  // XXX: This can be seriously simplified
  constexpr void assignPortName() {
    if constexpr (T == ComponentType::SE) {
      assert(ports.size() == 1);
      ports[0].setInCausalName("f_" + std::to_string(ID));
      ports[0].setOutCausalName("e_" + std::to_string(ID));
    } else if constexpr (T == ComponentType::SF) {
      assert(ports.size() == 1);
      ports[0].setInCausalName("e_" + std::to_string(ID));
      ports[0].setOutCausalName("f_" + std::to_string(ID));
    } else if constexpr (T == ComponentType::C) {
      assert(ports.size() == 1);
      if (ports[0].getOutCausality() == Causality::Effort) {
        // Integral causality
        ports[0].setInCausalName("f_" + std::to_string(ID));
        ports[0].setOutCausalName("e_" + std::to_string(ID));
      } else if (ports[0].getOutCausality() == Causality::Flow) {
        // Differential causality
        ports[0].setInCausalName("e_" + std::to_string(ID));
        ports[0].setOutCausalName("f_" + std::to_string(ID));
      } else {
        throw NotFound(std::format("Component {} not assigned causality", ID));
      }
    } else if constexpr (T == ComponentType::L) {
      assert(ports.size() == 1);
      if (ports[0].getInCausality() == Causality::Flow) {
        // Differential causality
        ports[0].setInCausalName("f_" + std::to_string(ID));
        ports[0].setOutCausalName("e_" + std::to_string(ID));
      } else if (ports[0].getInCausality() == Causality::Effort) {
        // Integral causality
        ports[0].setInCausalName("e_" + std::to_string(ID));
        ports[0].setOutCausalName("f_" + std::to_string(ID));
      } else {
        throw NotFound(std::format("Component {} not assigned causality", ID));
      }
    } else if constexpr (T == ComponentType::GY) {
      assert(ports.size() == 2);
      assert(ports[0].getInCausality() == ports[1].getInCausality());
      assert(ports[0].getOutCausality() == ports[1].getOutCausality());
      if (ports[0].getInCausality() == Causality::Flow) {
        ports[0].setInCausalName("f_" + std::to_string(ID));
        ports[1].setInCausalName("f_" + std::to_string(ID) + "_1");
        ports[0].setOutCausalName("e_" + std::to_string(ID));
        ports[1].setOutCausalName("e_" + std::to_string(ID) + "_1");
      } else if (ports[0].getInCausality() == Causality::Effort) {
        ports[0].setInCausalName("e_" + std::to_string(ID));
        ports[1].setInCausalName("e_" + std::to_string(ID) + "_1");
        ports[0].setOutCausalName("f_" + std::to_string(ID));
        ports[1].setOutCausalName("f_" + std::to_string(ID) + "_1");
      } else {
        throw NotFound(std::format("Component {} not assigned causality", ID));
      }
    } else if constexpr (T == ComponentType::TF) {
      assert(ports.size() == 2);
      assert(ports[0].getInCausality() != ports[1].getInCausality());
      assert(ports[0].getOutCausality() != ports[1].getOutCausality());
      if (ports[0].getInCausality() == Causality::Flow) {
        ports[0].setInCausalName("f_" + std::to_string(ID));
        ports[1].setInCausalName("e_" + std::to_string(ID) + "_1");
        ports[0].setOutCausalName("e_" + std::to_string(ID));
        ports[1].setOutCausalName("f_" + std::to_string(ID) + "_1");
      } else if (ports[0].getInCausality() == Causality::Effort) {
        ports[0].setInCausalName("e_" + std::to_string(ID));
        ports[1].setInCausalName("f_" + std::to_string(ID) + "_1");
        ports[0].setOutCausalName("e_" + std::to_string(ID));
        ports[1].setOutCausalName("f_" + std::to_string(ID) + "_1");
      } else {
        throw NotFound(std::format("Component {} not assigned causality", ID));
      }
    } else if constexpr (T == ComponentType::J0 || T == ComponentType::J1 ||
                         T == ComponentType::I || T == ComponentType::O) {
      for (size_t i = 0; i < ports.size(); ++i) {
        if (ports[i].getInCausality() == Causality::Flow) {
          ports[i].setInCausalName("f_" + std::to_string(ID) + "_" +
                                   std::to_string(i));
          ports[i].setOutCausalName("e_" + std::to_string(ID) + "_" +
                                    std::to_string(i));
        } else if (ports[i].getInCausality() == Causality::Effort) {
          ports[i].setInCausalName("e_" + std::to_string(ID) + "_" +
                                   std::to_string(i));
          ports[i].setOutCausalName("f_" + std::to_string(ID) + "_" +
                                    std::to_string(i));
        } else {
          throw NotFound(
              std::format("Component {} not assigned causality", ID));
        }
      }
    } else if constexpr (T == ComponentType::R) {
      assert(ports.size() == 1);
      if (ports[0].getInCausality() == Causality::Flow) {
        // Mul causality
        ports[0].setInCausalName("f_" + std::to_string(ID));
        ports[0].setOutCausalName("e_" + std::to_string(ID));
      } else if (ports[0].getInCausality() == Causality::Effort) {
        // Div causality
        ports[0].setInCausalName("e_" + std::to_string(ID));
        ports[0].setOutCausalName("f_" + std::to_string(ID));
      } else {
        throw NotFound(std::format("Component {} not assigned causality", ID));
      }
    }
  }

  constexpr void satisfyConstraints() const {
    uint8_t counter = 0;
    for (const Port &x : ports) {
      if (T == ComponentType::J0)
        counter += x.getInCausality() == Causality::Effort ? 1 : 0;
      else
        counter += x.getInCausality() == Causality::Flow ? 1 : 0;
    }
    if (counter != 1) {
      throw JunctionContraintViolated(
          std::format("Junction {} cannot satisfy constraints\n", ID));
    }
  }

  constexpr void setInternalName(std::string &&s) { internal = std::move(s); }
  constexpr const std::string &getInternalName() const { return internal; }
  constexpr std::string &getInternalName() { return internal; }

  // Get the state equation for the given component
  [[nodiscard]]
  constexpr const expression_t &getStateEq(expressionAst &ast) const {
    static_assert(T == ComponentType::L || T == ComponentType::C,
                  "Only C/L are storage components");
    if constexpr (T == ComponentType::L) {
      const Port *p = getPort(0);
      std::vector<size_t> eqs = ast.getEQ();
      if (p->getOutCausality() == Causality::Flow) {
        // std::string ss = "d" + std::string(p->getOutCausalName());
        const std::string &ss = getInternalName();
        return expressionAst::pr_getStateEq(ast, ss, eqs);
      } else {
        std::string_view ss = p->getInCausalName();
        return expressionAst::pr_getStateEq(ast, ss, eqs);
      }
    } else if constexpr (T == ComponentType::C) {
      const Port *p = getPort(0);
      std::vector<size_t> eqs = ast.getEQ();
      if (p->getOutCausality() == Causality::Effort) {
        // std::string ss = "d" + std::string(p->getOutCausalName());
        const std::string &ss = getInternalName();
        return expressionAst::pr_getStateEq(ast, ss, eqs);
      } else {
        std::string_view ss = p->getInCausalName();
        return expressionAst::pr_getStateEq(ast, ss, eqs);
      }
    }
  }

  void signal2ModulatedComponent(std::string &&s) {
    static_assert(M == Modulated::T,
                  "Signals can only influence modulated components");
    modulate_signal = std::move(s);
  }
  void component2Signal(std::string &&s, Causality c) {
    static_assert((T != ComponentType::J0 && T != ComponentType::J1 &&
                   T != ComponentType::GY && T != ComponentType::TF),
                  "Cannot convert non terminal components to signals");
    if (c == Causality::Effort) {
      effort_signal = std::move(s);
    } else if (c == Causality::Flow) {
      flow_signal = std::move(s);
    }
  }

private:
  [[nodiscard]]
  constexpr std::string_view getEffortFlow(Causality c) const {
    const Port *p = getPort(0);
    if (p->getOutCausality() == Causality::ACausal) {
      throw std::runtime_error(
          std::format("Causality not assigned for component {}", getID()));
    } else if (p->getOutCausality() == c) {
      return p->getOutCausalName();
    } else
      return p->getInCausalName();
  }

  std::vector<Port> ports; // The number of ports of this component
  // The symbolic value of this component
  std::string value;
  std::optional<std::string> modulate_signal;
  std::optional<std::string> effort_signal;
  std::optional<std::string> flow_signal;
  const char *name;     // The user name of the component
  std::string internal; // For C and L type components
  size_t ID;            // The unique ID generated internally for each component
  ComponentType myT;
  Modulated modulatedT = M;
  bool deleted = false; // Has this been deleted from the graph
                        // during simplification.
};

// This is the variant with all the different components
using componentVariant =
    std::variant<Component<ComponentType::C, Modulated::T>,
                 Component<ComponentType::C, Modulated::F>,
                 Component<ComponentType::L, Modulated::T>,
                 Component<ComponentType::L, Modulated::F>,
                 Component<ComponentType::J0, Modulated::F>,
                 Component<ComponentType::J0, Modulated::T>,
                 Component<ComponentType::J1, Modulated::T>,
                 Component<ComponentType::J1, Modulated::F>,
                 Component<ComponentType::R, Modulated::T>,
                 Component<ComponentType::R, Modulated::F>,
                 Component<ComponentType::SE, Modulated::T>,
                 Component<ComponentType::SE, Modulated::F>,
                 Component<ComponentType::SF, Modulated::T>,
                 Component<ComponentType::SF, Modulated::F>,
                 Component<ComponentType::GY, Modulated::T>,
                 Component<ComponentType::GY, Modulated::F>,
                 Component<ComponentType::TF, Modulated::T>,
                 Component<ComponentType::TF, Modulated::F>,
                 Component<ComponentType::I, Modulated::T>,
                 Component<ComponentType::I, Modulated::F>,
                 Component<ComponentType::O, Modulated::T>,
                 Component<ComponentType::O, Modulated::F>>;

using componentVariantPtr = componentVariant *;

// The hash required to make a map of Component pointers
struct ComponentHash {
  std::size_t operator()(const componentVariantPtr component) const noexcept {
    return std::hash<size_t>{}(
        std::visit([](const auto &x) { return x.getID(); }, *component));
  }
};

struct ComponentEqual {
  bool operator()(componentVariantPtr lhs,
                  componentVariantPtr rhs) const noexcept {
    size_t lname = std::visit([](const auto &x) { return x.getID(); }, *lhs);
    size_t rname = std::visit([](const auto &x) { return x.getID(); }, *rhs);
    return lname == rname;
  }
};

// This is the variant with all the different components
using storageVariant =
    std::variant<Component<ComponentType::C> *, Component<ComponentType::L> *>;

using IOVariant =
    std::variant<Component<ComponentType::O> *, Component<ComponentType::I> *>;

// The hash required to make a map of Component pointers
struct StorageHash {
  std::size_t operator()(const storageVariant component) const noexcept {
    return std::hash<size_t>{}(
        std::visit([](const auto &x) { return x->getID(); }, component));
  }
};

struct StorageEqual {
  bool operator()(storageVariant lhs, storageVariant rhs) const noexcept {
    size_t lname = std::visit([](const auto &x) { return x->getID(); }, lhs);
    size_t rname = std::visit([](const auto &x) { return x->getID(); }, rhs);
    return lname == rname;
  }
};

template <typename V = double>
using storage_map_t =
    std::unordered_map<storageVariant, V, StorageHash, StorageEqual>;

// Printing the enum
static std::ostream &operator<<(std::ostream &os, const ComponentType &c) {
  switch (c) {
  case ComponentType::C:
    os << "Capacitor";
    break;
  case ComponentType::L:
    os << "Inductor";
    break;
  case ComponentType::R:
    os << "Resistor";
    break;
  case ComponentType::SE:
    os << "Effort Source";
    break;
  case ComponentType::SF:
    os << "Flow Source";
    break;
  case ComponentType::J0:
    os << "0 Junction";
    break;
  case ComponentType::J1:
    os << "1 Junction";
    break;
  case ComponentType::GY:
    os << "Gyrator";
    break;
  case ComponentType::TF:
    os << "Transformer";
    break;
  case ComponentType::I:
    os << "Input";
    break;
  case ComponentType::O:
    os << "Output";
    break;
  }
  return os;
}

// Printing the causality
static std::ostream &operator<<(std::ostream &os, const Causality &c) {
  switch (c) {
  case Causality::Effort:
    os << "Effort";
    break;
  case Causality::ACausal:
    os << "Acausal";
    break;
  case Causality::Flow:
    os << "Flow";
    break;
  }
  return os;
}

// Printing the port
static std::ostream &operator<<(std::ostream &os, const Port &p) {
  os << "{in causality:" << p.getInCausality();
  os << ",";
  os << "in causal var: " << p.getInCausalName() << ", ";
  os << "out causality:" << p.getOutCausality() << ", ";
  os << "out causal var: " << p.getOutCausalName() << ", ";
  if (p.getPortType() == PortType::IN)
    os << "type : IN ";
  else
    os << "type : OUT ";
  os << "neighbour ID: " << p.getNeighbourID();
  os << "}";
  return os;
}

// Operators for printing things
template <ComponentType T, Modulated M>
static std::ostream &operator<<(std::ostream &os, const Component<T, M> &c) {
  if (c.getDeleted())
    throw DeletedException(
        std::format("Accessed a deleted node {}!", c.getID()));
  os << "{";
  os << "Name:" << c.getName() << ", ";
  os << "Type:" << c.getType() << ", ";
  os << "ID:" << c.getID() << ", ";
  os << "value: " << c.getValue() << ", ";
  os << "Ports:{";
  for (size_t i = 0; i < c.portSize(); ++i) {
    os << *c.getPort(i);
    if (i < c.portSize() - 1) {
      os << ", ";
    }
  }
  os << "}}";
  return os;
}
