#pragma once

#include <iostream>
#include <memory>
#include <ostream>
#include <variant>
#include <vector>

#define make_expr(x) std::make_unique<expression_t>(x)

enum class EOP { ADD, MUL, DIV };

struct Symbol {
  const char *name;
};

struct Number {
  double num;
};

// Forward declare a struct
template <EOP op> struct Expression;

// Make a variant of things
using expression_t = std::variant<Symbol, Number, Expression<EOP::ADD>,
                                  Expression<EOP::MUL>, Expression<EOP::DIV>>;

template <EOP op> struct Expression {
  Expression(expression_t *l, expression_t *r) : left(l), right(r) {}
  Expression(const Expression &) = delete;
  Expression &operator=(const Expression &) = delete;
  Expression &operator=(Expression &&) = default;
  Expression(Expression &&) = default;
  expression_t *left;
  expression_t *right;
};

static std::ostream &operator<<(std::ostream &os, const Symbol &in) {
  os << in.name;
  return os;
}

static std::ostream &operator<<(std::ostream &os, const Number &in) {
  os << in.num;
  return os;
}

template <EOP op>
static std::ostream &operator<<(std::ostream &os, const Expression<op> &in) {
  switch (op) {
  case EOP::ADD:
    os << "(" << *(in.left) << " + " << *(in.right) << ")";
    break;
  case EOP::DIV:
    os << "(" << *(in.left) << " / " << *(in.right) << ")";
    break;
  case EOP::MUL:
    os << "(" << *(in.left) << " * " << *(in.right) << ")";
  }
  return os;
}

static std::ostream &operator<<(std::ostream &os, const expression_t &in) {
  std::visit([&os](const auto &x) { os << x; }, in);
  return os;
}

struct expressionAst {
  expressionAst() { arena.push_back(make_expr(Number(0))); }
  [[nodiscard]]
  expression_t *append(std::unique_ptr<expression_t> &&x) {
    arena.emplace_back(std::move(x));
    return arena[arena.size() - 1].get();
  }
  void printExpression(std::ostream &os, const expression_t *p) { os << *p; }
  expression_t *operator[](size_t index) { return arena[index].get(); }
  size_t size() const { return arena.size(); }

private:
  std::vector<std::unique_ptr<expression_t>> arena;
};
