#pragma once

#include <cstdint>
#include <iostream>
#include <ostream>
#include <variant>
#include <vector>

// This is enumeration of all
enum class T : uint8_t { SYM = 0, NUM, ADD, MUL, DIV, SUB, EQ };

enum class EOP : uint8_t { ADD = 0, MUL, DIV, EQ, SUB };

struct Symbol {
  std::string name;
  bool isConst = false;
  T t = T::SYM;
};

struct Number {
  double num;
  T t = T::NUM;
};

// Forward declare a struct
template <EOP op> struct Expression;

// Make a variant of things
using expression_t = std::variant<Symbol, Number, Expression<EOP::ADD>,
                                  Expression<EOP::MUL>, Expression<EOP::DIV>,
                                  Expression<EOP::EQ>, Expression<EOP::SUB>>;

template <EOP op> struct Expression {
  Expression(size_t l, size_t r) : left(l), right(r) {
    if constexpr (op == EOP::ADD) {
      t = T::ADD;
    } else if constexpr (op == EOP::DIV) {
      t = T::DIV;
    } else if constexpr (op == EOP::EQ) {
      t = T::EQ;
    } else if constexpr (op == EOP::MUL) {
      t = T::MUL;
    } else if constexpr (op == EOP::SUB) {
      t = T::SUB;
    }
  }
  Expression(const Expression &) = delete;
  Expression &operator=(const Expression &) = delete;
  Expression &operator=(Expression &&) = default;
  Expression(Expression &&) = default;
  size_t left;
  size_t right;
  T t;
};

struct expressionAst {
  expressionAst() { arena.emplace_back(Number(0)); }
  [[nodiscard]]
  size_t append(expression_t &&x) {
    arena.emplace_back(std::move(x));
    return arena.size() - 1;
  }
  [[nodiscard]]
  const expression_t &operator[](size_t index) const {
    return arena[index];
  }
  [[nodiscard]]
  size_t size() const {
    return arena.size();
  }

private:
  std::vector<expression_t> arena;
};

static void print_expr(std::ostream &os, const Symbol &in,
                       const expressionAst &ast) {
  os << in.name;
}

static void print_expr(std::ostream &os, const Number &in,
                       const expressionAst &ast) {
  os << in.num;
}

static void print_expression_t(std::ostream &os, const expression_t &in,
                               const expressionAst &ast);

template <EOP op>
static void print_expr(std::ostream &os, const Expression<op> &in,
                       const expressionAst &ast) {
  switch (op) {
  case EOP::ADD:
    os << "(";
    print_expression_t(os, ast[in.left], ast);
    os << ")";
    os << " + ";
    os << "(";
    print_expression_t(os, ast[in.right], ast);
    os << ")";
    break;
  case EOP::MUL:
    os << "(";
    print_expression_t(os, ast[in.left], ast);
    os << ")";
    os << " * ";
    os << "(";
    print_expression_t(os, ast[in.right], ast);
    os << ")";
    break;
  case EOP::DIV:
    os << "(";
    print_expression_t(os, ast[in.left], ast);
    os << ")";
    os << " / ";
    os << "(";
    print_expression_t(os, ast[in.right], ast);
    os << ")";
    break;
  case EOP::EQ:
    os << "(";
    print_expression_t(os, ast[in.left], ast);
    os << ")";
    os << " == ";
    os << "(";
    print_expression_t(os, ast[in.right], ast);
    os << ")";
    break;
  case EOP::SUB:
    os << "(";
    print_expression_t(os, ast[in.left], ast);
    os << ")";
    os << " - ";
    os << "(";
    print_expression_t(os, ast[in.right], ast);
    os << ")";
    break;
  }
}

static void print_expression_t(std::ostream &os, const expression_t &in,
                               const expressionAst &ast) {
  std::visit([&os, &ast](const auto &x) { print_expr(os, x, ast); }, in);
}
