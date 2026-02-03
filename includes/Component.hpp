#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ostream>
#include <vector>

struct Util {
  static size_t getID() {
    static size_t counter = 0;
    return counter++;
  }
};

// The type of components that are allowed in the Bond Graph
enum class ComponentType : std::uint8_t { C = 0, L, R, SE, SF, J0, J1 };

// The different types of causality
enum class Causality : std::uint8_t { Flow = 0, Effort, ACausal };

// The port structure of the component
struct Port {
  Port() : in(Causality::ACausal), out(Causality::ACausal) {}
  Port(const Port &) = delete;
  Port(Port &&) = default;
  Port &operator=(const Port &) = delete;
  Port &operator=(Port &&) = default;
  virtual ~Port() {}

  // The public functions
  void setInCausality(Causality c) { in = c; }
  void setOutCausality(Causality c) { out = c; }
  Causality getInCausality() const { return in; }
  Causality getOutCausality() const { return out; }

private:
  Causality in;  // The in causality
  Causality out; // The out causality for this port.
};

// The common Component class
template <ComponentType T> struct Component {
  Component() {} // This is for Bond Graph insertion
  Component(const char *n) : myT(T), name(n), ID(Util::getID()) {}
  Component(const Component &) = delete;   // Copy constructor is deleted
  Component(Component &&) = default; // Use the default move constructor
  Component &operator=(const Component &) = delete; // Copy assignment is deleted
  Component &
  operator=(Component &&) = default; // Use default move assignment operator
  virtual ~Component() {}            // Virtual destructor

  // The public methods
  const char *getName() const { return name; }
  size_t getID() const { return ID; }
  ComponentType getType() const { return myT; }
  size_t portSize() const { return ports.size(); }
  const Port &getPort(size_t i) const { return ports[i]; }
  void addPort(Port &&p) { ports.push_back(std::move(p)); }

private:
  ComponentType myT;
  const char *name; // The user name of the component
  size_t ID;        // The unique ID generated internally for each component
  std::vector<Port> ports; // The number of ports of this component
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
  os << "{";
  os << "Name:" << c.getName() << ", ";
  os << "Type:" << c.getType() << ", ";
  os << "ID:" << c.getID() << ", ";
  os << "Ports:{";
  for (size_t i = 0; i < c.portSize(); ++i) {
    os << c.getPort(i);
    if (i < c.portSize() - 1) {
      os << ", ";
    }
  }
  os << "}}";
  return os;
}
