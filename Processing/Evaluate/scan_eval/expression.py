"""Safe parsing of surface expressions into sympy.

A reference definition carries its surface as a string such as
"c - A*sin(2*pi*x/L)*cos(2*pi*y/L)". sympy's own sympify/parse_expr run the
string through Python's eval, so a definition file could execute arbitrary
code. Instead the string is parsed with the `ast` module and the tree is walked
against a whitelist, building the sympy expression node by node. Nothing the
user wrote is ever evaluated as Python.

Allowed: numbers, the declared variables and parameters, pi and e, + - * / **
%, unary minus, calls to the functions in FUNCTIONS, and (for domain masks)
comparisons combined with `and` / `or` / `not`.
"""

from __future__ import annotations

import ast
from typing import Iterable, Mapping

import numpy as np
import sympy as sp


class ExpressionError(ValueError):
    """An expression that is malformed, unsafe, or refers to unknown names."""


FUNCTIONS = {
    "sin": sp.sin, "cos": sp.cos, "tan": sp.tan,
    "asin": sp.asin, "acos": sp.acos, "atan": sp.atan, "atan2": sp.atan2,
    "sinh": sp.sinh, "cosh": sp.cosh, "tanh": sp.tanh,
    "exp": sp.exp, "log": sp.log, "ln": sp.log, "sqrt": sp.sqrt,
    "abs": sp.Abs, "sign": sp.sign, "min": sp.Min, "max": sp.Max,
    "floor": sp.floor, "ceil": sp.ceiling,
    "hypot": lambda a, b: sp.sqrt(a**2 + b**2),
}

CONSTANTS = {"pi": sp.pi, "e": sp.E}

_MAX_LENGTH = 4000
# A constant exponent above this is almost certainly a typo, and sympy would
# try to evaluate e.g. 9**9**9 exactly, which never finishes.
_MAX_CONSTANT_EXPONENT = 1000

_BINOPS = {
    ast.Add: lambda a, b: a + b,
    ast.Sub: lambda a, b: a - b,
    ast.Mult: lambda a, b: a * b,
    ast.Div: lambda a, b: a / b,
    ast.Pow: lambda a, b: a**b,
    ast.Mod: lambda a, b: sp.Mod(a, b),
}

_COMPARE = {
    ast.Lt: sp.StrictLessThan, ast.LtE: sp.LessThan,
    ast.Gt: sp.StrictGreaterThan, ast.GtE: sp.GreaterThan,
}


def parse_expression(text: str, variables: Iterable[str],
                     params: Mapping[str, float] | None = None,
                     allow_logic: bool = False) -> sp.Expr:
    """Parse `text` into a sympy expression over `variables`.

    Parameters stay symbolic (as sympy Symbols) so the definition hash does not
    depend on how numbers happen to be printed; substitute them with
    `substitute_params` before lambdifying.
    """
    if not isinstance(text, str) or not text.strip():
        raise ExpressionError("expression is empty")
    if len(text) > _MAX_LENGTH:
        raise ExpressionError(f"expression is longer than {_MAX_LENGTH} characters")

    params = dict(params or {})
    names: dict[str, sp.Basic] = {}
    for v in variables:
        names[v] = sp.Symbol(v, real=True)
    for p in params:
        if p in names:
            raise ExpressionError(f"parameter '{p}' has the same name as a variable")
        if p in CONSTANTS or p in FUNCTIONS:
            raise ExpressionError(f"parameter '{p}' shadows a built-in name")
        names[p] = sp.Symbol(p, real=True)
    names.update(CONSTANTS)

    try:
        tree = ast.parse(text.strip(), mode="eval")
    except SyntaxError as exc:
        raise ExpressionError(f"cannot parse '{text}': {exc.msg} (column {exc.offset})") from None

    builder = _Builder(text, names, allow_logic)
    return builder.visit(tree.body)


def substitute_params(expr: sp.Expr, params: Mapping[str, float]) -> sp.Expr:
    return expr.subs({sp.Symbol(k, real=True): sp.Float(v) for k, v in params.items()})


def lambdify(variables: Iterable[str], expr: sp.Expr):
    """Vectorised numpy function of `variables`, always returning an array with
    the broadcast shape of its inputs - lambdify returns a bare scalar for a
    constant expression (e.g. the derivative of a plane), which would otherwise
    break every caller that stacks results."""
    symbols = [sp.Symbol(v, real=True) for v in variables]
    fn = sp.lambdify(symbols, expr, modules="numpy")

    def wrapped(*args):
        args = [np.asarray(a, dtype=float) for a in args]
        shape = np.broadcast_shapes(*(a.shape for a in args))
        with np.errstate(all="ignore"):
            out = fn(*args)
        return np.broadcast_to(np.asarray(out, dtype=float), shape).copy()

    return wrapped


def validate_params(params: Mapping[str, object], where: str) -> dict[str, float]:
    out = {}
    for key, value in params.items():
        if not key.isidentifier():
            raise ExpressionError(f"{where}: parameter name '{key}' is not a valid identifier")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ExpressionError(f"{where}: parameter '{key}' must be a number, got {value!r}")
        if not np.isfinite(value):
            raise ExpressionError(f"{where}: parameter '{key}' is not finite")
        out[key] = float(value)
    return out


class _Builder:
    def __init__(self, text: str, names: dict, allow_logic: bool):
        self.text = text
        self.names = names
        self.allow_logic = allow_logic

    def fail(self, node: ast.AST, message: str):
        col = getattr(node, "col_offset", None)
        where = f" at column {col + 1}" if col is not None else ""
        raise ExpressionError(f"{message}{where} in '{self.text}'")

    def visit(self, node: ast.AST):
        method = getattr(self, "visit_" + type(node).__name__, None)
        if method is None:
            self.fail(node, f"'{type(node).__name__}' is not allowed in a surface expression")
        return method(node)

    def visit_Constant(self, node: ast.Constant):
        if isinstance(node.value, bool) or not isinstance(node.value, (int, float)):
            self.fail(node, f"only numeric constants are allowed, got {node.value!r}")
        return sp.Integer(node.value) if isinstance(node.value, int) else sp.Float(node.value)

    def visit_Name(self, node: ast.Name):
        if node.id in self.names:
            return self.names[node.id]
        if node.id in FUNCTIONS:
            self.fail(node, f"'{node.id}' is a function and must be called, e.g. {node.id}(x)")
        known = ", ".join(sorted(k for k in self.names if k not in CONSTANTS))
        self.fail(node, f"unknown name '{node.id}' (variables and parameters: {known}; "
                        f"constants: pi, e)")

    def visit_BinOp(self, node: ast.BinOp):
        if isinstance(node.op, ast.BitXor):
            self.fail(node, "'^' is not a power operator here - use ** (e.g. x**2)")
        op = _BINOPS.get(type(node.op))
        if op is None:
            self.fail(node, f"operator '{type(node.op).__name__}' is not allowed")
        left, right = self.visit(node.left), self.visit(node.right)
        if isinstance(node.op, ast.Pow) and right.is_number:
            if abs(float(right)) > _MAX_CONSTANT_EXPONENT:
                self.fail(node, f"exponent {right} is unreasonably large")
            if left.is_number and abs(float(left)) > 1e6 and abs(float(right)) > 10:
                self.fail(node, "constant power is unreasonably large")
        return op(left, right)

    def visit_UnaryOp(self, node: ast.UnaryOp):
        operand = self.visit(node.operand)
        if isinstance(node.op, ast.USub):
            return -operand
        if isinstance(node.op, ast.UAdd):
            return operand
        if isinstance(node.op, ast.Not) and self.allow_logic:
            return sp.Not(operand)
        self.fail(node, f"unary operator '{type(node.op).__name__}' is not allowed")

    def visit_Call(self, node: ast.Call):
        if not isinstance(node.func, ast.Name):
            self.fail(node, "only plain function calls like sin(x) are allowed")
        name = node.func.id
        if name not in FUNCTIONS:
            self.fail(node, f"unknown function '{name}' (allowed: {', '.join(sorted(FUNCTIONS))})")
        if node.keywords:
            self.fail(node, f"keyword arguments are not allowed in {name}()")
        if any(isinstance(a, ast.Starred) for a in node.args):
            self.fail(node, "argument unpacking is not allowed")
        args = [self.visit(a) for a in node.args]
        try:
            return FUNCTIONS[name](*args)
        except TypeError as exc:
            self.fail(node, f"wrong number of arguments to {name}(): {exc}")

    def visit_Compare(self, node: ast.Compare):
        if not self.allow_logic:
            self.fail(node, "comparisons are only allowed in a domain mask")
        terms = [self.visit(node.left)] + [self.visit(c) for c in node.comparators]
        parts = []
        for op, a, b in zip(node.ops, terms[:-1], terms[1:]):
            rel = _COMPARE.get(type(op))
            if rel is None:
                self.fail(node, f"comparison '{type(op).__name__}' is not allowed (use < <= > >=)")
            parts.append(rel(a, b))
        return sp.And(*parts) if len(parts) > 1 else parts[0]

    def visit_BoolOp(self, node: ast.BoolOp):
        if not self.allow_logic:
            self.fail(node, "'and'/'or' are only allowed in a domain mask")
        values = [self.visit(v) for v in node.values]
        return sp.And(*values) if isinstance(node.op, ast.And) else sp.Or(*values)
