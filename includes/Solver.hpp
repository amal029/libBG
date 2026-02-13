#include "Component.hpp"
#include "exception.hpp"
#include "expression.hpp"
#include "util.hpp"
#include <iostream>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>


template <typename T>
concept NumericType = std::integral<T> || std::floating_point<T>;

template <typename V = double>
using component_map_t =
    std::unordered_map<componentVariant, V, ComponentHash, ComponentEqual>;

template <NumericType T = double> struct Solver {
  explicit Solver(const expressionAst &ast, component_map_t<T> &&consts)
      : _ast(ast) {

    _consts.reserve(consts.size());
    // Convert the component -> value map to string -> value map
    for (const auto &&[k, v] : consts) {
      // First make sure that k is a storage component
      ComponentType cT =
          std::visit([](const auto &x) { return x->getType(); }, k);
      if (cT != ComponentType::C || cT != ComponentType::L) {
        throw IncorrectComponentType("Only storage elements evolve\n");
      }
      std::string &vv =
          std::visit([](const auto &x) { return x->getValue(); }, k);
      _consts[vv] = std::move(v);
    }
  }

  std::vector<T> getSlope(component_map_t<T> &&initialValues, T time) {
    consts_t<T> iValues;
    iValues.reserve(initialValues.size());
    _comps.reserve(initialValues.size());

    // First perform dependence analysis
    for (auto &kv : initialValues) {
      _comps.emplace_back(kv.first);
    }
    // Now perform dependence analysis to order them correctly.
    std::vector<bool> isDeriv = reorder(_comps);

    // First turn the initial values from
    // Component:v --> string:v
    for (const componentVariant k : _comps) {
      // First make sure that k is a storage component
      ComponentType cT =
          std::visit([](const auto &x) { return x->getType(); }, k);
      if (cT != ComponentType::C || cT != ComponentType::L) {
        std::visit(
            [&](const auto &x) {
              std::cerr << *x;
              std::cerr << "\n";
            },
            k);
        throw IncorrectComponentType("Only storage elements evolve\n");
      }
      std::string_view vv =
          std::visit([](const auto &x) { return x->getInternalName(); }, k);
      iValues[vv.substr(1)] = initialValues[k];
    }
    std::vector<T> toret;
    toret.reserve(_comps.size());
    for (size_t counter = 0; counter < _comps.size(); ++counter) {
      if (!isDeriv[counter]) {
        // Then just get the slope/value
        T res = std::visit(
            [&](const auto &x) { return x->eval(_consts, iValues, _ast); },
            _comps[counter]);
        toret.emplace_back(res);

      } else {
        // TODO: For this we first need to calculate the derivative and
        // then integrate.
        throw NotYetImplemented(
            "Expressions with derivatives in expressions not yet implemented");
      }
    }
    return toret;
  }

private:
  // XXX: Reorder so that all the equations that need derivatives come
  // last.
  [[nodiscard]]
  std::vector<bool> reorder(std::vector<componentVariant> &_comps,
                            const component_map_t<T> &iValues) {
    std::vector<bool> toret;
    toret.resize(_comps.size(), false);
    size_t i = 0;
    size_t j = _comps.size() - 1;
    while (i != j || i != _comps.size() - 1) {
      const expression_t &exp = std::visit(
          [&](const auto &x) { x->getStateEq(_ast); }, iValues[_comps[i]]);
      // Does this exp have a derivative on the right?
      bool res = std::visit(
          [&](const auto &x) -> bool { return x.hasDeriv(_ast); }, exp);
      if (res) {
        std::swap(_comps[i], _comps[j]);
        toret[i] = true;
        --j;
      } else {
        ++i;
      }
    }
    return toret;
  }

  consts_t<T> _consts;
  const expressionAst &_ast;
  std::vector<componentVariant> _comps;
};
