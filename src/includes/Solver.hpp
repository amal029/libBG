#include "Component.hpp"
#include "exception.hpp"
#include "expression.hpp"
#include "util.hpp"
#include <span>
#include <unordered_map>
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
    if (comps.empty()) {
      throw std::invalid_argument("Solver requires at least one component");
    }
    // Now find if this is a DAE
    bool isDeriv = isDAE(_comps);
    if (isDeriv) {
      throw NotYetImplemented("DAEs not yet implemented");
    }
    consts_t<T> _consts;
    _consts.reserve(consts.size());
    // Convert the component -> value map to string -> value map
    for (const auto &[k, v] : consts) {
      std::string_view vv = std::visit(
          [](const auto &x) -> std::string_view { return x->getValue(); }, k);
      _consts[vv] = v;
    }
    // Here replace the constants with their values for all dxdt expressions
    // Initialize the keys
    iValue_keys.reserve(_comps.size());

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
        // I do this before, because after append _ee might be invalidated.
        _ee->setRight(_ast.size() + 1); // replace the right with the new number
        size_t _ =
            _ast.append(Number{_consts[torep->getName()]});
      }
      std::string_view vv = std::visit(
          [](const auto &x) -> std::string_view {
            return x->getInternalName();
          },
          _comp);
      iValue_keys.push_back(vv.substr(1));
    }
  }

  Solver(const Solver &) = delete;
  Solver(Solver &&) = default;
  Solver &operator=(const Solver &) = delete;
  Solver &operator=(Solver &&) = default;

  [[nodiscard]]
  constexpr size_t getComponentSize() const {
    return _comps.size();
  }

  void dxdt(const std::vector<T> &xT, std::span<T> dxdt) {
    for (size_t counter = 0; counter < _comps.size(); ++counter) {
      iValues[iValue_keys[counter]] = xT[counter];
    }
    for (size_t counter = 0; counter < _comps.size(); ++counter) {
      // Then just get the slope/value
      T res = std::visit(
          [&](const auto &x) {
            const expression_t &ee = x->getStateEq(_ast);
            const Expression<EOP::EQ> *_ee =
                std::get_if<Expression<EOP::EQ>>(&ee);
            T res = eval(_ast[_ee->getRight()], iValues, _ast);
            return res;
          },
          _comps[counter]);
      dxdt[counter] = res;
    }
  }

private:
  [[nodiscard]]
  bool isDAE(const std::vector<storageVariant> &_comps) {
    bool toret = false;
    size_t i = 0;
    while (i < _comps.size()) {
      const expression_t &exp = std::visit(
          [&](const auto &x) -> const expression_t & {
            return x->getStateEq(_ast);
          },
          _comps[i]);
      // Does this exp have a derivative on the right?
      toret = std::visit(
          [&](const auto &x) -> bool { return x.hasDeriv(_ast); }, exp);
      if (toret)
        break;
      ++i;
    }
    return toret;
  }

  std::vector<std::string_view> iValue_keys{};
  consts_t<T> iValues{};
  expressionAst &_ast;
  const std::vector<storageVariant> &_comps;
};
