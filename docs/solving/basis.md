# Bases and warm starts

A linear program solved by the simplex method ends on a basis: as many basic entities as there are rows, variables or rows, and every other one nonbasic, at one of its bounds as a rule. MIP++ reads that basis as a snapshot that you query with the handles of your own variables and constraints, and writes a basis back so that the next solve starts from it: the basis of a model solved before, of a twin model, or one of your own making.

## Reading a basis

A workshop makes chairs, tables and desks from 40 boards and 18 hours of labour, plus up to 4 hours of overtime at 3 each, and has orders for at least 3 tables. This program finds the most profitable plan and prints its basis:

```cpp
#include <concepts>
#include <iostream>
#include <string_view>
#include <variant>

--8<-- "test/doc_snippets/basis.cpp:includes"

using namespace mippp;
using namespace mippp::operators;

--8<-- "test/doc_snippets/basis.cpp:describe"

int main() {
    highs_lp model;
    --8<-- "test/doc_snippets/basis.cpp:workshop-model"

    --8<-- "test/doc_snippets/basis.cpp:workshop-basis"
    return 0;
}
```

It prints:

```text
chairs:   basic
tables:   basic
desks:    nonbasic, at its lower side
overtime: basic
wood:     nonbasic, at its upper side
labour:   nonbasic, at its upper side
orders:   nonbasic, at its lower side
```

The plan makes 14 chairs and the 3 tables ordered, uses 2 hours of overtime and no desk. The three variables inside their bounds are basic, one per row; the wood and labour rows are used up, at their upper side, and the orders row is at its lower side.

`get_basis()` returns the snapshot by value: it describes the basis of the last solve, and a later change of the model leaves it as it was. Its `get_status(v)` and `get_status(c)` return a `std::variant` over the tags of namespace `basis_status`, which form a hierarchy:

| Tag | Derives from | Means |
| :--- | :--- | :--- |
| `basic` | | in the basis |
| `nonbasic` | | out of the basis |
| `nonbasic_at_bound` | `nonbasic` | at one of its bounds |
| `nonbasic_at_lower_bound` | `nonbasic_at_bound` | at its lower bound, or a row at its lower side |
| `nonbasic_at_upper_bound` | `nonbasic_at_bound` | at its upper bound, or a row at its upper side |
| `nonbasic_fixed` | `nonbasic_at_bound` | at its two bounds, which are equal, as an `==` row |
| `superbasic` | `nonbasic` | at no bound: strictly inside its bounds, or free |
| `nonbasic_free` | `superbasic` | free, with no finite bound, held at zero |

The side of a row is the side of its activity, the value of its left-hand side: `wood`, whose `2 chairs + 4 tables + 2 desks` reaches 40, reads `nonbasic_at_upper_bound` on every backend. Several solvers name the side of a slack variable instead, which is the other side on some rows, or no side at all, and MIP++ translates their answer.

Each backend reports the most specific tag its solver's answer gives. A fixed entity, a variable with equal bounds or an `==` row, reads `nonbasic_fixed` where the solver has a code for it, and otherwise the side its solver names, which follows the sign of its reduced cost or dual; both derive from `nonbasic_at_bound`. A free nonbasic variable reads `nonbasic_free` where the solver tells it apart, and `superbasic` where one code covers it and the variables stopped strictly inside their bounds, which only a run stopped before the optimum leaves. [Support by model](#support-by-model) lists which backend reports what.

So test a status with `is_a`, at the level your code cares about: `is_a<basis_status::nonbasic_at_bound>(s)` holds for every entity at a bound, fixed ones included, on every backend. `is_a` and `is` compile only for a tag the backend's variant lists or derives from: `basic`, `nonbasic`, `nonbasic_at_bound`, `nonbasic_at_lower_bound`, `nonbasic_at_upper_bound` and `superbasic` compile everywhere, while `nonbasic_fixed` and `nonbasic_free` compile only where the backend reports them. `describe`, above, visits the tag and tests it by derivation, so it compiles on every backend. Code that must know whether an entity is fixed compares its bounds. The types are `model_basis_t<M>` for the snapshot, and `model_variable_basis_status_t<M>` and `model_constraint_basis_status_t<M>` for the two statuses, which may differ.

## Starting a solve from a basis

A new product comes up: a stool, made from one board and one hour, sold at 5. Column generation adds it to the solved model as a column, and starts the next solve from the basis read before:

```cpp
--8<-- "test/doc_snippets/basis.cpp:workshop-stools"
```

It prints:

```text
stools:   nonbasic
profit:   89
```

`set_basis(b)` hands the basis to the solver, and the next `solve()` starts from it. A column the snapshot does not cover, created after it was taken, reads bare `nonbasic`, and a row it does not cover reads `basic`, which is how every solver extends its own basis: the basis stays square, one basic entity per row, and the solve starts one column away from the optimum. A re-solve of the model you changed in place usually warm-starts without `set_basis`, since the solver keeps its own basis; `set_basis` matters when it does not, see [Support by model](#support-by-model), and to start from a basis the solver no longer holds:

- **Another model.** A snapshot is keyed by handle, so it seeds a second model built in the same order, of the same backend or another one: `other.set_basis(model.get_basis())`. Models are move-only, so this is how a second model, built per thread or per node, starts where the first ended. Removals break the correspondence: a target that did not remove the same variables needs a basis of its own making, mapping its handles to the source's.
- **An earlier basis.** After a dive that changed bounds, `set_basis(saved)` takes the solver back to where it was.
- **A basis of your own making.** `set_basis` accepts any type whose `get_status(v)` and `get_status(c)` return a `std::variant` holding a `basis_status::basic` tag and a `basis_status::nonbasic` tag; tags of your own that derive from them write as the tag they derive from. A default-constructed `model_basis_t<M>` is the slack basis, every column nonbasic and every row basic, and `set_status(e, tag)` edits a snapshot, growing it for handles it does not cover: this is how you warm-start a model never solved.

```cpp
--8<-- "test/doc_snippets/basis.cpp:own-start"
```

`set_status` stores the most specific tag the snapshot's status lists: `nonbasic_fixed` written on a backend that reports no fixed entity is stored as bare `nonbasic`, which the solver resolves against the bounds.

A basis is never an error. Every solver snaps a side that the bounds do not have, a variable at an infinite upper bound say, and on an entity with two finite bounds a wrong side costs a bound flip, free under the dual simplex. A basis with the wrong number of basic entries costs more: Gurobi, COPT and GLPK start the solve cold, and the other solvers repair it. Removals are where that happens. A snapshot taken before a basic column was removed is one basic entity short over the model that remains: on Gurobi the next solve starts cold, and the eviction strategies of [column generation](../algorithms/column-generation.md) evict nonbasic columns only, at an optimum. A handle created after a removal may reuse the removed variable's id, and then reads that variable's entry in an older snapshot: a snapshot describes the model as it was.

## When a basis is available

`get_basis()` is defined right after a `solve()` that ran a simplex method, before any other call that changes the model or what the solver holds: it then returns the basis that solve ended with. Where the solver holds none, it throws `mippp::solver_error`: before any solve, after presolve proved the model infeasible or unbounded, after a time limit that stopped the solve before the simplex method started, and after an interior-point run that skipped its crossover, which only native settings can ask for. Which of these each backend reports, and how, is in [Support by model](#support-by-model). A model without variables has one basis, every row basic, which `get_basis()` returns on every backend without a solve.

At any other time its result is unspecified: between a change of the model and the next `solve()`, between `set_basis()` and the next `solve()`, after a `solve()` that MIP++ answers without running the solver, as on a model whose bounds cross, and after `refine_lp_status()`, `compute_iis()` or `compute_iis_by_deletion`. Some backends then throw, others return the basis their next solve would start from.

## The status afterwards

`set_basis()` leaves `get_status()` as it was: the status describes the last solve, as after any other change of the model, and only the next `solve()` replaces it. Gurobi, CPLEX and COPT discard their solution when a basis is written, so on `gurobi_lp`, `cplex_lp` and `copt_lp` the solution getters fail until the next `solve()`, while `get_status()` still reports the solve before.

## Support by model

`has_lp_basis<M>` holds where `get_basis()` exists, and `has_lp_basis_warm_start<M>` where `set_basis()` does. Both are LP features: no `*_milp` model has them, and neither has `highs_qp`, whose quadratic solver neither reports a simplex basis nor starts from one.

| Model | `get_basis()` | `set_basis()` | Fixed entities read | No basis reported |
| :--- | :---: | :---: | :--- | :--- |
| `clp_lp` | ✓ | ✓ | `nonbasic_fixed` | before the first solve |
| `copt_lp` | ✓ | ✓ | `nonbasic_fixed` | when COPT holds none, or one with the wrong number of basic entries, as after `compute_iis()` |
| `cplex_lp` | ✓ | ✓ | a side, `==` rows `nonbasic_fixed` | when CPLEX holds none |
| `glpk_lp` | ✓ | ✓ | `nonbasic_fixed` | never: GLPK always holds a basis |
| `gurobi_lp` | ✓ | ✓ | a side, `==` rows `nonbasic_fixed` | when Gurobi holds none, and after any change of the model |
| `highs_lp` | ✓ | ✓ | a side | when HiGHS holds none, and after any change of the model |
| `mosek_lp` | ✓ | ✓ | `nonbasic_fixed` | when MOSEK's basic solution is undefined or incomplete |
| `soplex_lp` | ✓ | | `nonbasic_fixed` | never: SoPlex always answers |
| `xpress_lp` | ✓ | ✓ | a side | when Xpress holds no basis from its last solve, and after any change of the model |

The backends differ in a few more ways, see [LP bases](../solvers/index.md#limitation-lp-basis):

- `soplex_lp` reads a basis but cannot write one: SoPlex's C interface has no setter.
- `mosek_lp` solves an LP by the interior point, which cannot start from a basis: `set_basis()` makes the next `solve()` run the simplex method, and later solves return to the interior point.
- `glpk_lp` replaces a basis it cannot factorize by its standard basis, so the next solve starts cold.

## Next

[Re-solving and model updates](updates.md) — changing a model between solves without rebuilding it.
