#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <ostream>
#include <queue>
#include <stdexcept>
#include <string_view>
#include <variant>
#include <vector>

// This is enumeration of all
enum class T : uint8_t { SYM = 0, NUM, ADD, MUL, DIV, SUB, EQ };

enum class EOP : uint8_t { ADD = 0, MUL, DIV, EQ, SUB };

// Forward declare a struct
template <EOP op> struct Expression;
struct Symbol;
struct Number;
struct expressionAst;

// Make a variant of things
using expression_t = std::variant<Symbol, Number, Expression<EOP::ADD>,
                                  Expression<EOP::MUL>, Expression<EOP::DIV>,
                                  Expression<EOP::EQ>, Expression<EOP::SUB>>;

static void print_expression_t(std::ostream &os, const expression_t &in,
                               const expressionAst &ast);

struct Symbol {
  explicit Symbol(std::string_view n) : name(n) {}
  explicit Symbol(std::string_view n, bool isC) : name(n), isConst(isC) {}
  Symbol(const Symbol &) = delete;
  Symbol(Symbol &&) = default;
  bool pushSymbol(std::queue<size_t *> &q, std::vector<expression_t> &arena) {
    // std::cout << "Checking symbol: " << name << " return: " << (!isConst) <<
    // "\n";
    return (!isConst);
  }
  T getT() const { return t; }
  const std::string_view &getName() const { return name; }
  void print_expr(std::ostream &os, const expressionAst &ast) const {
    os << getName();
  }

private:
  std::string_view name; // change this to string view?
  bool isConst = false;
  T t = T::SYM;
};

struct Number {
  Number(double n) : num(n) {}
  Number(const Number &) = delete;
  Number(Number &&) = default;
  bool pushSymbol(std::queue<size_t *> &q, std::vector<expression_t> &arena) {
    return false;
  }
  T getT() const { return t; }
  double getNum() const { return num; }
  void print_expr(std::ostream &os, const expressionAst &ast) const {
    os << num;
  }

private:
  double num;
  T t = T::NUM;
};

static std::ostream &operator<<(std::ostream &os, const T t) {
  switch (t) {
  case T::SYM:
    os << "SYM";
    break;
  case T::DIV:
    os << "DIV";
    break;
  case T::MUL:
    os << "MUL";
    break;
  case T::ADD:
    os << "ADD";
    break;
  case T::SUB:
    os << "SUB";
    break;
  case T::NUM:
    os << "NUM";
    break;
  case T::EQ:
    os << "EQ";
    break;
  }
  return os;
}

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
  bool pushSymbol(std::queue<size_t *> &q, std::vector<expression_t> &arena) {
    bool res = std::visit([&](auto &x) { return x.pushSymbol(q, arena); },
                          arena[left]);
    if constexpr (op != EOP::EQ) {
      if (res)
        q.push(&left);
    }
    res = std::visit([&](auto &x) { return x.pushSymbol(q, arena); },
                     arena[right]);
    if (res)
      q.push(&right);
    return false;
  }

  Expression(const Expression &) = delete;
  Expression &operator=(const Expression &) = delete;
  Expression &operator=(Expression &&) = default;
  Expression(Expression &&) = default;
  size_t getLeft() { return left; }
  size_t getLeft() const { return left; }
  size_t getRight() { return right; }
  size_t getRight() const { return right; }
  T getT() const { return t; }
  void print_expr(std::ostream &os, const expressionAst &ast) const {
    if constexpr (op != EOP::EQ)
      os << "(";
    print_expression_t(os, ast[getLeft()], ast);
    if constexpr (op == EOP::ADD) {
      os << " + ";
    } else if constexpr (op == EOP::MUL) {
      os << " * ";
    } else if constexpr (op == EOP::DIV) {
      os << " / ";
    } else if constexpr (op == EOP::EQ) {
      os << " = ";
    } else if constexpr (op == EOP::SUB) {
      os << " - ";
    }
    print_expression_t(os, ast[getRight()], ast);
    if constexpr (op != EOP::EQ)
      os << ")";
  }

private:
  size_t left;
  size_t right;
  T t;
};

struct expressionAst {
  explicit expressionAst() noexcept {
    arena.emplace_back(Number{0});
    arena.emplace_back(Symbol{"dt", true});
  };
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

  expressionAst(const expressionAst &) = delete;
  expressionAst(expressionAst &&) = default;
  expressionAst &operator=(const expressionAst &) = delete;
  expressionAst &operator=(expressionAst &&) = default;

  // Get the Equality expressions
  std::vector<size_t> getEQ() {
    std::vector<size_t> toret;
    toret.reserve(arena.size());
    for (size_t i = 0; i < arena.size(); ++i) {
      Expression<EOP::EQ> *pp = std::get_if<Expression<EOP::EQ>>(&arena[i]);
      if (pp != nullptr) {
        toret.push_back(i);
      }
    }
    return toret;
  }
  // Function that collects the non-const symbols from a given expression
  bool getNonConstSymbols(size_t expr, std::queue<size_t *> &q) {
    return std::visit([&](auto &x) { return x.pushSymbol(q, arena); },
                      arena[expr]);
  }

  // Simplification algorithm
  void simplify(size_t eq_index, const std::vector<size_t> &eqs) {
    std::queue<size_t *> q;
    Expression<EOP::EQ> *res =
        std::get_if<Expression<EOP::EQ>>(&arena[eq_index]);
    if (res != nullptr) {
      getNonConstSymbols(eq_index, q);
    } else {
      throw std::runtime_error("Cannot simplify non equality expression\n");
    }
    std::vector<bool> visited;
    visited.resize(eqs.size());
    // Set the current equality to true
    while (!q.empty()) {
      size_t *torep = q.front();
      q.pop();
      // Now get EQ with the left side equal to this symbol
      T t = std::visit([](const auto &x) { return x.getT(); }, arena[*torep]);
      assert(t == T::SYM);
      Symbol *torepsym = (Symbol *)&arena[*torep];
      // Go through the eqs
      size_t counter = 0;
      for (size_t jj : eqs) {
        Expression<EOP::EQ> *x = std::get_if<Expression<EOP::EQ>>(&arena[jj]);
        assert(x != nullptr);
        Symbol *eqls = std::get_if<Symbol>(&arena[x->getLeft()]);
        if (eqls != nullptr && torepsym->getName() == eqls->getName()) {
          if (visited[counter]) {
            x->print_expr(std::cerr, *this);
            std::cerr << "\n";
            throw std::runtime_error("Algebraic Loop");
          }
          visited[counter] = true;
          *torep = x->getRight(); // replaced
          if (getNonConstSymbols(*torep, q)) {
            q.push(torep);
          }
          break;
        } else if (eqls == nullptr) {
          x->print_expr(std::cerr, *this);
          std::cerr << "\n";
          throw std::runtime_error("Left side of equality is not a symbol");
        }
        ++counter;
      }
    }
  }

private:
  std::vector<expression_t> arena;
};

static void print_expression_t(std::ostream &os, const expression_t &in,
                               const expressionAst &ast) {
  std::visit([&os, &ast](const auto &x) { x.print_expr(os, ast); }, in);
}

static std::ostream &operator<<(std::ostream &os, const expressionAst &ast) {
  for (size_t i = 0; i < ast.size(); ++i) {
    print_expression_t(os, ast[i], ast);
    os << "\n";
  }
  return os;
}
