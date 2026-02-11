#include "../includes/expression.hpp"
#include <cstddef>
#include <iostream>
#include <vector>

int main() {
  expressionAst ast;
  size_t l = ast.append(Symbol{"e2"});
  size_t e2 = l;
  size_t r = ast.append(Symbol{"usource", true});
  r = ast.append(Expression<EOP::EQ>(l, r));
  size_t ll = ast.append(Symbol{"df3"});
  size_t dt = ast.append(Symbol{"dt", true});
  l = ast.append(Number{1});
  r = ast.append(Symbol{"L", true});
  l = ast.append(Expression<EOP::DIV>(l, r));
  r = ast.append(Symbol{"e3"});
  size_t e3 = r;
  l = ast.append(Expression<EOP::MUL>(l, r));
  l = ast.append(Expression<EOP::MUL>(l, dt));
  l = ast.append(Expression<EOP::EQ>(ll, l));
  l = ast.append(Symbol{"Rel", true});
  r = ast.append(Symbol{"f4"});
  r = ast.append(Expression<EOP::MUL>{l, r});
  l = ast.append(Symbol{"e4"});
  size_t e4 = l;
  l = ast.append(Expression<EOP::EQ>{l, r});
  r = ast.append(Symbol{"f3"});
  l = ast.append(Symbol{"f2"});
  l = ast.append(Expression<EOP::EQ>{l, r});
  l = ast.append(Symbol{"f4"});
  l = ast.append(Expression<EOP::EQ>{l, r});
  l = ast.append(Symbol{"f5"});
  l = ast.append(Expression<EOP::EQ>{l, r});
  size_t e5 = ast.append(Symbol{"e5"});
  r = ast.append(Expression<EOP::SUB>{e2, e4});
  r = ast.append(Expression<EOP::SUB>{r, e5});
  l = ast.append(Expression<EOP::EQ>{e3, r});
  l = ast.append(Symbol{"K", true});
  r = ast.append(Symbol{"f6"});
  size_t f6 = r;
  r = ast.append(Expression<EOP::MUL>{l, r});
  l = ast.append(Expression<EOP::EQ>{e5, r});
  r = ast.append(Symbol{"f7"});
  l = ast.append(Expression<EOP::EQ>{f6, r});

  // Get the equality expressions
  std::vector<Expression<EOP::EQ> *> eqs = ast.getEQ();
  for (const Expression<EOP::EQ> *x : eqs) {
    x->print_expr(std::cout, ast);
    std::cout << "\n";
  }

  // Simplify an expression
  ast.simplify(1, eqs); // Simplify the index 1 expression
  eqs[1]->print_expr(std::cout, ast);

  return 0;
}
