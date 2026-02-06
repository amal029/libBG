#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <ostream>
#include <string>
#include <vector>
#include <format>
#include "exception.hpp"
#include "ginac/symbol.h"
#include <ginac/ginac.h>

struct Util {
  static size_t getID() {
    static size_t counter = 0;
    return counter++;
  }
};

// The type of components that are allowed in the Bond Graph
enum class ComponentType : std::uint8_t { C = 0, L, R, SE, SF, GY, TF, J0, J1 };

// The preferred causality
enum class PrefCausality : std::uint8_t { I = 0, D, N };

// The different types of causality
enum class Causality : std::uint8_t { Flow = 0, Effort, ACausal };

// The port structure of the component
struct Port {
  Port() : in(Causality::ACausal), out(Causality::ACausal) {
    // size_t ID = Util::getID();
    // inx = GiNaC::symbol{"in" + std::to_string(ID)};
    // outx = GiNaC::symbol{"out" + std::to_string(ID)};
  }
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
  constexpr const GiNaC::ex &getInExpression() const { return inx; }
  constexpr const GiNaC::ex &getOutExpression() const { return outx; }

private:
  Causality in;  // The in causality
  Causality out; // The out causality for this port.
  bool assigned = false;
  // The symbolic values (expressions) each of these ports hold
  GiNaC::ex inx;
  GiNaC::ex outx;
};

// The common Component class
template <ComponentType T> struct Component {
  constexpr Component() {} // This is for Bond Graph insertion
  constexpr Component(const char *n, PrefCausality Pref = PrefCausality::N)
      : myT(T), name(n), ID(Util::getID()), mPref(Pref) {
    // Static assert that Pref causality can only be given for components that
    // are not junctions
    assert((T != ComponentType::J0 || Pref == PrefCausality::N) &&
           (T != ComponentType::J1 || Pref == PrefCausality::N));
    value = GiNaC::symbol{name + std::to_string(ID)};
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
  constexpr const Port *getPort(size_t i) const {
    if (i > ports.size()) {
      throw PortIndexOutofBounds("");
    }
    return &ports[i];
  }
  constexpr Port *getPort(size_t i) {
    if (i > ports.size()) {
      throw PortIndexOutofBounds("");
    }
    return &ports[i];
  }
  constexpr void setPort(size_t i, Port &&p) { ports[i] = std::move(p); }
  constexpr void addPort(Port &&p) { ports.push_back(std::move(p)); }
  // Deleted means deleted from the bond graph
  constexpr void setDeleted() { deleted = true; }
  constexpr bool getDeleted() const { return deleted; }
  constexpr const GiNaC::symbol &getValue() const { return value; }

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

private:

  ComponentType myT;
  const char *name; // The user name of the component
  size_t ID;        // The unique ID generated internally for each component
  PrefCausality mPref;
  std::vector<Port> ports; // The number of ports of this component
  bool deleted = false;    // Has this been deleted from the graph
                           // during simplification.
  // The symbolic value of this component
  GiNaC::symbol value;
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
  os << "{in:" << p.getInCausality();
  os << ",";
  os << "out:" << p.getInCausality() << "}";
  return os;
}

// Operators for printing things
template <ComponentType T>
static std::ostream &operator<<(std::ostream &os, const Component<T> &c) {
  if (c.getDeleted())
    throw DeletedException("Accessed a deleted node!");
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
