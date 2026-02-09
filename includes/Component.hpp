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
#include <ostream>
#include <string>
#include <vector>

struct Util {
  static size_t getID() {
    static size_t counter = 0;
    return counter++;
  }
};

// The type of components that are allowed in the Bond Graph
enum class ComponentType : std::uint8_t { C = 0, L, R, SE, SF, GY, TF, J0, J1 };

// The different types of causality
enum class Causality : std::uint8_t { Flow = 0, Effort, ACausal };

// The type of port, IN or OUT
enum class PortType : std::uint8_t { IN = 0, OUT };

// The port structure of the component
struct Port {
  Port(PortType t, size_t ID)
      : in(Causality::ACausal), out(Causality::ACausal), mType(t), nID(ID) {}
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
  void setOutExpression(expression_t *v) { outx = v; }
  void setInExpression(expression_t *v) { inx = v; }
  expression_t *getOutExpression() const { return outx; }
  expression_t *getInExpression() const { return inx; }
  size_t getNeighbourID() const { return nID; }
  void setNeighbourID(size_t nid) { nID = nid; }

private:
  Causality in;  // The in causality
  Causality out; // The out causality for this port.
  bool assigned = false;
  PortType mType;
  size_t nID; // neighbour that I am connected to
  // The symbolic values (expressions) each of these ports hold
  expression_t *inx;
  expression_t *outx;
};

// The common Component class
template <ComponentType T> struct Component {
  constexpr Component() {} // This is for Bond Graph insertion
  constexpr Component(const char *n) : myT(T), name(n), ID(Util::getID()) {
    value = Symbol{(name + std::to_string(ID)).c_str()};
  }
  constexpr Component(const Component &) = delete; // Copy constructor is
                                                   // deleted
  constexpr Component(Component &&) =
      default; // Use the default move constructor
  constexpr Component &
  operator=(const Component &) = delete; // Copy assignment is deleted
  constexpr Component &
  operator=(Component &&) = default; // Use default move assignment operator
  ~Component() {}                    // Destructor

  // The public methods
  constexpr const char *getName() const { return name; }
  constexpr size_t getID() const { return ID; }
  constexpr ComponentType getType() const { return myT; }
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
  constexpr expression_t &&getValue() { return std::move(value); }

  constexpr void satisfyConstraints() const {
    uint8_t counter = 0;
    for (const Port &x : ports) {
      if (T == ComponentType::J0)
        counter += x.getInCausality() == Causality::Effort ? 0 : 1;
      else
        counter += x.getInCausality() == Causality::Flow ? 0 : 1;
    }
    if (counter != 1) {
      throw JunctionContraintViolated(
          std::format("Junction {} cannot satisfy constraints\n", ID));
    }
  }
  // Get Pref Causality
  void setInternalExpression(expression_t *v) { internal = v; }
  expression_t *getInternalExpression() const { return internal; }

private:
  ComponentType myT;
  const char *name; // The user name of the component
  size_t ID;        // The unique ID generated internally for each component
  std::vector<Port> ports; // The number of ports of this component
  bool deleted = false;    // Has this been deleted from the graph
                           // during simplification.
  // The symbolic value of this component
  expression_t value;
  expression_t *internal; // This is only for C/R/L
};

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
  os << "out causality:" << p.getInCausality() << ", ";
  if (p.getPortType() == PortType::IN)
    os << "type : IN ";
  else
    os << "type : OUT ";
  os << "neighbour ID: " << p.getNeighbourID();
  os << "}";
  return os;
}

// Operators for printing things
template <ComponentType T>
static std::ostream &operator<<(std::ostream &os, const Component<T> &c) {
  if (c.getDeleted())
    throw DeletedException(
        std::format("Accessed a deleted node {}!", c.getID()));
  os << "{";
  os << "Name:" << c.getName() << ", ";
  os << "Type:" << c.getType() << ", ";
  os << "ID:" << c.getID() << ", ";
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
