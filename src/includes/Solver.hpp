#include "Component.hpp"
#include "exception.hpp"
#include "expression.hpp"
#include "util.hpp"
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

template <typename V = double>
using component_map_t =
    std::unordered_map<componentVariant, V, ComponentHash, ComponentEqual>;

template <typename V = double>
using storage_map_t =
    std::unordered_map<storageVariant, V, StorageHash, StorageEqual>;

template <NumericType T = double> struct Solver {
  explicit Solver(expressionAst &ast, component_map_t<T> &&consts,
                  const std::vector<storageVariant> &comps)
      : _ast(ast), _comps(comps) {

    // Now perform dependence analysis to order them correctly.
    std::vector<bool> isDeriv = reorder(_comps);
    bool res = std::any_of(isDeriv.begin(), isDeriv.end(),
                           [](bool x) { return x == true; });
    if (res) {
      throw NotYetImplemented("DAEs not yet implemented");
    }
    consts_t<T> _consts;
    _consts.reserve(consts.size());
    // Convert the component -> value map to string -> value map
    for (const auto &[k, v] : consts) {
      // XXX: This should do small string optimisation
      std::string vv =
          std::visit([](const auto &x) { return x->getValue(); }, k);
      _consts[vv] = std::move(v);
    }
    // Here replace the constants with their values for all dxdt expressions
    for (const storageVariant &_comp : _comps) {
      const expression_t &ee = std::visit(
          [&](auto &x) -> const expression_t & { return x->getStateEq(_ast); },
          _comp);
      Expression<EOP::EQ> *_ee =
          (Expression<EOP::EQ> *)std::get_if<Expression<EOP::EQ>>(&ee);
      assert(_ee != nullptr);
      bool res = _ast.subs(_ee->getRight(), _consts);
      if (res) {
        // Then the right is guaranteed to be a symbol. Get the symbol from
        // consts and get its value v.
        const Symbol *torep = std::get_if<Symbol>(&_ast[_ee->getRight()]);
        assert(torep != nullptr);
        size_t nindex =
            _ast.append(Number{_consts[std::string(torep->getName())]});
        _ee->setRight(nindex); // replace the right with the new number
      }
    }
  }

  Solver(const Solver &) = delete;
  Solver(Solver &&) = default;
  Solver &operator=(const Solver &) = delete;
  Solver &operator=(Solver &&) = default;

  constexpr void dxdt(const std::vector<T> &xT, std::vector<T> &dxdt) {
    // First turn the initial values from
    // Component:v --> string:v
    for (size_t counter = 0; counter < _comps.size(); ++counter) {
      std::string vv = std::visit(
          [](const auto &x) { return x->getInternalName().substr(1); },
          _comps[counter]);
      iValues[vv] = xT[counter];
    }
    dxdt.reserve(_comps.size());
    for (size_t counter = 0; counter < _comps.size(); ++counter) {
      // Then just get the slope/value
      T res = std::visit(
          [&](const auto &x) {
            const expression_t &ee = x->getStateEq(_ast);
            const Expression<EOP::EQ> *_ee =
                std::get_if<Expression<EOP::EQ>>(&ee);
            return eval(_ast[_ee->getRight()], iValues, _ast);
          },
          _comps[counter]);
      dxdt[counter] = res;
    }
  }

private:
  // XXX: Reorder so that all the equations that need derivatives come
  // last.
  [[nodiscard]]
  std::vector<bool> reorder(std::vector<storageVariant> &_comps) {
    std::vector<bool> toret;
    toret.resize(_comps.size(), false);
    size_t i = 0;
    size_t j = _comps.size() - 1;
    while (i != j || i != _comps.size() - 1) {
      const expression_t &exp = std::visit(
          [&](const auto &x) -> const expression_t & {
            return x->getStateEq(_ast);
          },
          _comps[i]);
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

  consts_t<T> iValues;
  expressionAst &_ast;
  std::vector<storageVariant> _comps;
};
