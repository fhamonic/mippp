# Inside the expression layer

[Expressions and constraints](../modeling/expressions.md) shows *how* to build models; this page explains *what* the expression layer actually is — the concepts it is written against, the ownership rules that make lazy views safe, and the compile-time diagnostics that fire when those rules are broken.

Read it if you want to plug your own expression types into MIP++, write generic code over expressions, or understand a `static_assert` you just hit.

The whole layer lives in four headers:
[`utility/zero.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/utility/zero.hpp),
[`linear_expression.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/linear_expression.hpp),
[`linear_constraint.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/linear_constraint.hpp) and
[`quadratic_expression.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/quadratic_expression.hpp).
None of them knows about any solver.

## Anatomy of a linear expression

`linear_expression` is a duck-typing concept — there is no base class, and no MIP++ type is privileged:

```cpp
template <typename LE>
concept linear_expression =
    linear_term<linear_term_t<LE>> &&
    std::convertible_to<linear_expression_constant_t<LE>,
                        linear_expression_scalar_t<LE>>;
```

Unfolded, a type models it when it has:

- `linear_terms()` — an input range of **terms**, each a tuple-like of size 2 (`std::pair`, `std::tuple`, …) whose first element is a variable handle and second a coefficient;
- `constant()` — a value convertible to the coefficient type.

That is all. A term stream is a *multiset*: repeats are allowed and are summed by whoever consumes them, which is why `x + 2 * x` reaches the solver as the row `3 x` without the expression layer ever building a map.

A minimal custom expression — a dense row over consecutive variable ids:

```cpp
template <typename Model>
struct dense_row {
    std::span<const double> coefficients;  // coefficients[i] belongs to var i

    auto linear_terms() const noexcept {
        return std::views::transform(
            std::views::enumerate(coefficients), [](auto && p) {
                auto && [i, c] = p;
                return std::pair<typename Model::variable, double>(
                    typename Model::variable(static_cast<int>(i)), c);
            });
    }
    zero_t constant() const noexcept { return zero; }
};

static_assert(linear_expression<dense_row<highs_lp>>);
model.add_constraint(dense_row<highs_lp>{coefs} <= 10.0);  // just works
```

Every model function (`set_objective`, `add_constraint`, `add_column`, …) is constrained on the concept, never on a concrete class, so a user-defined type is a first-class citizen — including for the `operators` overloads, which are themselves constrained the same way.

### The trait aliases

Generic code uses these rather than reaching into the types:

| Alias | Yields |
| :--- | :--- |
| `linear_terms_range_t<LE>` | the type of `le.linear_terms()` |
| `linear_term_t<LE>` | the range's value type |
| `linear_expression_variable_t<LE>` | decayed variable type of a term |
| `linear_expression_scalar_t<LE>` | decayed coefficient type of a term |
| `linear_expression_constant_t<LE>` | decayed type of `le.constant()` |
| `linear_term_variable_t<LT>` / `linear_term_scalar_t<LT>` | the same, on a term type |

All of them are spelled on an **lvalue** (`std::declval<LE &>()`), so a conforming type must at least be readable through a non-`const` lvalue.

`compatible_linear_expressions<E1, E2>` — required by every binary operator — asks for **exactly** the same variable *and* scalar types. No implicit conversion is performed: mixing a `float`-coefficient expression with a `double` one is a compile error, not a silent narrowing.

## Constants that occupy no space: `zero_t`

Most expressions have no constant term, and MIP++ makes that fact visible to the compiler.
[`utility/zero.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/utility/zero.hpp) defines an empty tag type `zero_t`, the inline constant `zero`, and the concept `statically_zero`.

```cpp
auto x = model.add_variable();
static_assert(statically_zero<linear_expression_constant_t<decltype(x)>>);
```

`zero_t` converts implicitly to any scalar, so consumers expecting a runtime number keep working, and its arithmetic operators are **hidden friends** — found by ADL only, so they are candidates exactly when an operand *is* a `zero_t` and never pollute unqualified lookup. They propagate the zero-ness through the algebra: `zero + zero`, `-zero`, `zero * s` and `zero / s` are all still `zero_t`, while `zero + s` collapses to plain `s`. Comparison is covered by a single `operator==`: `s == zero` uses the reversed candidate, and `!=` is rewritten from it.

Compound assignment onto a runtime scalar (`s += zero`, `s -= zero`) is provided too, and needs its own hidden friends for a subtle reason: the built-in candidates are `operator+=(S &, R)` for *every* promoted arithmetic `R`, and the conversion operator above — being a template — satisfies all of them equally well. Without an exact match to outrank them the call is ambiguous, even though `s + zero` and `double d = zero` both work. This is what lets `runtime_linear_expression` accumulate constant-free expressions; see below.

Three deliberate omissions: no type in MIP++ declares a conversion **to** `zero_t`, and neither `s / zero` nor `s *= zero` is provided — collapsing a scalar to zero should be spelled out, not implied.

Combined with `[[no_unique_address]]` on the constant member of `linear_expression_view`, an expression with no constant carries **no runtime storage** for it and no addition at consumption time:

```cpp
using terms = std::vector<std::pair<var, double>>;
static_assert(sizeof(linear_expression_view(std::declval<terms &>())) ==
              sizeof(linear_expression_view(std::declval<terms &>(), 1.0)) -
                  sizeof(double));
```

The deduction guides pick the representation automatically: `linear_expression_view(terms)` deduces `zero_t`, and `linear_expression_view(terms, c)` keeps `zero_t` when `c` is itself statically zero, otherwise the scalar type. Downstream, `linear_expression_square` and `linear_expression_mul_view` branch on `statically_zero` to drop whole linear parts at compile time — squaring an expression without constant produces `empty_linear_expression<V, S>`, a real expression with an empty term range.

## Term ranges have two independent properties

Expressions are views, and views differ in what you may do with them. Two orthogonal questions decide it, and each has a concept:

| Concept | Question | Fails when |
| :--- | :--- | :--- |
| `const_readable_linear_terms<E>` | Can the terms be reached through a `const &`, i.e. does reading *not* consume the expression? | the expression **owns** a move-only term range (it was built over an rvalue range, so `views::all` produced an `owning_view`) |
| `multipass_linear_terms<E>` | Const-readable **and** restartable (`forward_range`)? | additionally when the range is single-pass — typically a `join_view` over ranges materialized on the fly, which is exactly what `xsum` produces |

Both are properties of the **type**; references and cv-qualification make no difference. A third, `detail::forwardable_linear_terms`, is the only value-category-dependent one: it constrains the *argument*, since an rvalue may always be gutted whatever its term range looks like.

Four representative expressions separate the axes cleanly (they are the ones the [test suite](https://github.com/fhamonic/mippp/blob/main/test/linear_expression.cpp) pins down):

| Expression | Term range | const-readable | multipass |
| :--- | :--- | :-: | :-: |
| `x * 3.2 - y * 1.5 + 9` | concat of transforms | ✅ | ✅ |
| `xsum(vars)` (named range) | join over temporaries | ✅ | ❌ |
| `linear_expression_view(std::vector<term>{})` | owning view, move-only | ❌ | ❌ |
| `materialize(...)` | `const std::vector &` | ✅ | ✅ |

The practical consequences:

- an expression built over a **named** range only references it — reuse it as often as you like;
- an expression built over an **rvalue** range owns it and is single-use: pass it as an rvalue (`std::move`) at its last use, or hold the range in a named variable instead;
- an `xsum` can be consumed once, in any position that reads its terms a single time — which is every model function — but not by anything that needs a second pass.

## `materialize` — the escape hatch

`runtime_linear_expression<Variable, Scalar, Constant, Container>` is the materialized counterpart of the views: it stores its terms in a container (`std::vector<std::pair<Variable, Scalar>>` by default) and accumulates through `operator+=`, which accepts both expressions and bare scalars.

`materialize(e)` is the one-liner form — it deduces all three of `Variable`, `Scalar` and `Constant` from `e` — and it repairs **both** axes at once: the result is const-readable and random-access whatever `e` was. Use it when:

1. an expression is assembled incrementally, across loops or function boundaries, so no single view can be formed;
2. the same expression is consumed several times (`add_constraint` in a loop, or reused between the objective and a row);
3. a product needs a second pass — see below.

Two materialized expressions add up as cheaply as views do: `a + b` concatenates their containers *by reference* and leaves both operands usable.

When you declare one yourself, `Constant` defaults to `Scalar` and `operator+=` takes both expressions and bare scalars, mixing statically-zero and runtime constants freely:

```cpp
runtime_linear_expression<var, double> row;
for(auto && part : parts) row += weight(part) * X(part);  // zero_t constants
row += 4.5;                                               // runtime constant
model.add_constraint(row <= capacity);
```

Spelling `Constant` as `zero_t` explicitly is worth it for a row you know is constant-free — it keeps the constant out of the object entirely, and turns "someone added an offset to this row" into a compile error.

## Reading the diagnostics

The operations assert their preconditions explicitly, so a misuse produces one sentence instead of a page of concept mismatches:

| Message begins with | Cause | Fix |
| :--- | :--- | :--- |
| *"these expressions use different variable/scalar types"* | `compatible_linear_expressions` violated | cast one side explicitly; check for a stray `float` literal |
| *"this expression owns its term range … and is single-use"* | an lvalue whose terms are move-only was passed to a consuming operation | `std::move` it at its last use, build it over a named range, or `materialize` it |
| *"this expression's term range is single-pass, but a product traverses each operand once per term of the other"* | `square(xsum(...))` or a product with an `xsum` operand | `square(materialize(e))`, `materialize(e1) * materialize(e2)` |
| *"this expression owns a move-only term range, so it cannot back a constraint"* | an expression built over an rvalue range was compared with `<=` / `>=` / `==` | build it over a named range, or `materialize` it |

One more fires from `linear_expressions_sum`: summing expressions whose constants are **runtime** values traverses the range twice (once for the constants, once for the terms) and therefore requires a `forward_range`. Sums of expressions with statically-zero constants — the common case — stay single-pass.

## Operations without the operators

Everything in `namespace mippp::operators` is a thin wrapper over a named free function, so generic code can compose expressions without importing operator overloads into its scope:

| Operator | Free function |
| :--- | :--- |
| `e1 + e2`, `e1 - e2` | `linear_expression_add`, with `linear_expression_negate` |
| `-e` | `linear_expression_negate` |
| `e + c`, `e - c` | `linear_expression_scalar_add` |
| `c * e`, `e / c` | `linear_expression_scalar_mul`, `linear_expression_scalar_div` |
| `xsum(r)`, `xsum(r, f)` | `linear_expressions_sum` (composed with `views::transform`) |

Each one returns a `linear_expression_view` over a standard-library view of its operands — `views::transform` for scalings, `views::join` for `xsum` — and forwards value categories, which is where the ownership rules above come from.

Sums go through `detail::unordered_concat`, a thin wrapper over `std::views::concat` that falls back to a local `concat_view` when the standard one is unavailable (it is a C++26 feature; see [`detail/concat_view.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/detail/concat_view.hpp)). As the name says, it does not promise to keep its operands in order — it may concatenate them the other way round to dodge a libstdc++ 15.1 bug. **Never rely on the order of a term stream**: it is a multiset, and consumers accumulate it.

## Constraints

A constraint is an expression plus a sense and a right-hand side:

```cpp
enum constraint_sense : int { equal = 0, less_equal = -1, greater_equal = 1 };

template <typename T>
concept linear_constraint =
    requires(const T & t) {
        t.linear_terms();
        { t.sense() } -> std::same_as<constraint_sense>;
        t.rhs();
    } && ...;
```

Note the `const T &`: unlike `linear_expression`, a constraint must be readable **without being consumed** — `add_constraint` reads it through a `const &`, and reading it twice must give the same rows. `linear_constraint_view` stores its expression by value and exposes only `const` accessors, so an expression owning a move-only term range cannot back one.

That requirement is enforced on the **class template parameter**, exactly as `linear_expression_square` and `linear_expression_mul_view` do for `multipass_linear_terms`:

```cpp
template <linear_expression LExpr>
    requires const_readable_linear_terms<LExpr>
class linear_constraint_view { /* ... */ };
```

Constraining the type rather than the individual member matters: were the type constructible anyway, asking `linear_constraint<C>` would instantiate `linear_terms()` and fail *outside the immediate context* — the concept would be ill-formed instead of simply `false`, and `std::is_constructible_v` would lie. The comparison operators pair it with a `static_assert` so the error names its cause instead of surfacing as a deduction failure.

The comparison operators never build a two-sided object. They move everything to the left — `e1 <= e2` becomes `linear_constraint_view(e1 + (-e2), less_equal)` — and the view then reports `rhs()` as `-expression.constant()`. So a single normalized form (`terms sense rhs`) reaches the backend, whichever way you spelled the comparison, and scalar-on-the-left forms (`3 <= e`) negate the expression to keep the sense consistent.

Providing your own constraint type is symmetric with expressions: satisfy the concept and `add_constraint` accepts it.

## Quadratic expressions

A **quadratic term** is a tuple-like of size 3 whose first two elements are the *same* variable type, and a `quadratic_expression` exposes two accessors:

```cpp
qe.quadratic_terms();  // range of (variable, variable, coefficient)
qe.linear_part();      // itself a linear_expression (terms + constant)
```

with the variable types matching and the linear scalar convertible to the quadratic one. The corresponding traits mirror the linear ones: `quadratic_term_variable_t`, `quadratic_term_scalar_t`, `quadratic_terms_range_t`, `quadratic_term_t`, `quadratic_expression_variable_t` / `_scalar_t` / `_constant_t`, and `compatible_quadratic_expressions`.

As with linear terms, the stream is a multiset, and the pairs are **unordered**: `square(x1 + x2)` emits all four cartesian products, including both `(x1, x2, 1)` and `(x2, x1, 1)`. Backends fold `(i, j)` and `(j, i)` together and sum duplicates when building the (triangular) Hessian — see [`highs_qp::set_objective`](https://github.com/fhamonic/mippp/blob/main/include/mippp/solvers/highs/v1_10/highs_qp.hpp).

### Products need a second pass

`square(e)` and `e1 * e2` are lazy too — `linear_expression_square` and `linear_expression_mul_view` produce their terms with `views::cartesian_product` — but a cartesian product walks one operand **once per term of the other**. Both therefore require `multipass_linear_terms` on their operands, and enforce it *on the view type itself*, not only in the operator, so that `std::is_constructible_v` and friends do not lie:

```cpp
auto e = xsum(vars);
square(e);               // ✗ single-pass: static_assert with the fix spelled out
square(materialize(e));  // ✓
```

These two views expose **only `const &`** accessors, precisely because they read their operands repeatedly.

!!! warning "Custom quadratic types: the `&&` accessors must partition the state"
    MIP++ forwards the same expression to `quadratic_terms()` *and* `linear_part()` within a single operation (see `quadratic_expression_add`). That is well-defined only if the two accessors consume **disjoint** subobjects: if one `&&` overload may move from a member, no other accessor may read it.

    Providing only `const &` accessors — as `linear_expression_square` and `linear_expression_mul_view` do — satisfies the rule trivially, and is the recommended default for user-defined quadratic expressions.

The named operations are `quadratic_expression_add`, `_negate`, `_scalar_add`, `_scalar_mul`, `_scalar_div`, and `quadratic_expression_lexpr_add` for mixing a quadratic and a linear operand (`qe + le` in either order).

## Evaluating

`evaluate(expr, values_map)` folds an expression against anything indexable by variable handles — a solution, or your own `entity_mapping`:

```cpp
auto sol = model.get_solution();
double lhs = evaluate(2 * x1 + x2, sol);
```

It takes its expression by **forwarding reference** rather than `const &`, on purpose: `linear_expression_view::linear_terms() const &` returns a *copy* of the view, and does not exist at all when the term range is move-only. The same reasoning applies to any generic code you write over expressions — take `E &&`, and forward.

## What's next

The concepts on the model side — what a backend must provide for `add_constraint` and friends to exist at all — are listed in [Model concepts](concepts.md).
