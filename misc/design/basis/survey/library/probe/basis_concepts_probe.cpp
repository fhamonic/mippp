// Probe of the drafted basis concepts of model_concepts.hpp against a
// by-value snapshot built on detail::handle_status_table, as iis_snapshot is.
// Compiled against the repository headers only; no solver involved.
#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

#include "mippp/detail/handle_status_table.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/column_manager.hpp"

using namespace mippp;
namespace bs = mippp::basis_status;

using variable = model_variable<int, double>;
using constraint = model_constraint<int>;

// HiGHS-like: no fixed status (Lower/Basic/Upper/Zero/Nonbasic)
using var_status = std::variant<bs::nonbasic_at_lower_bound, bs::basic,
                                bs::nonbasic_at_upper_bound, bs::nonbasic_free>;
using row_status = std::variant<bs::basic, bs::nonbasic_at_lower_bound,
                                bs::nonbasic_at_upper_bound, bs::nonbasic_free>;
// every leaf the modifiable concept names
using wide_status =
    std::variant<bs::nonbasic_at_lower_bound, bs::basic,
                 bs::nonbasic_at_upper_bound, bs::nonbasic_free,
                 bs::nonbasic_fixed>;

// read-only snapshot, const getters, like iis_snapshot
template <typename VS, typename CS>
struct snapshot {
    detail::handle_status_table<VS> vars;
    detail::handle_status_table<CS> rows;
    VS get_status(variable v) const noexcept { return vars.get(v.uid()); }
    CS get_status(constraint c) const noexcept { return rows.get(c.uid()); }
};

// modifiable snapshot whose set_status forwards unconstrained to the table
template <typename VS, typename CS>
struct modifiable_unconstrained : snapshot<VS, CS> {
    void set_basic(variable v) { this->vars.set(v.uid(), bs::basic{}); }
    void set_basic(constraint c) { this->rows.set(c.uid(), bs::basic{}); }
    void set_nonbasic(variable, double) {}    // cannot snap: no bounds here
    void set_nonbasic(constraint, double) {}  // cannot snap: no row sides
    template <typename Tag>
    void set_status(variable v, Tag t) {
        this->vars.set(v.uid(), t);
    }
    template <typename Tag>
    void set_status(constraint c, Tag t) {
        this->rows.set(c.uid(), t);
    }
};
// same, with set_status constrained on what the table accepts
template <typename VS, typename CS>
struct modifiable_constrained : snapshot<VS, CS> {
    void set_basic(variable v) { this->vars.set(v.uid(), bs::basic{}); }
    void set_basic(constraint c) { this->rows.set(c.uid(), bs::basic{}); }
    void set_nonbasic(variable, double) {}
    void set_nonbasic(constraint, double) {}
    template <typename Tag>
        requires requires(detail::handle_status_table<VS> & t, Tag tag) {
            t.set(0, tag);
        }
    void set_status(variable v, Tag t) {
        this->vars.set(v.uid(), t);
    }
    template <typename Tag>
        requires requires(detail::handle_status_table<CS> & t, Tag tag) {
            t.set(0, tag);
        }
    void set_status(constraint c, Tag t) {
        this->rows.set(c.uid(), t);
    }
};

// a getter that is mutable-only: lp_iis rejects that shape
struct mutable_only_basis {
    var_status get_status(variable);
    row_status get_status(constraint);
};

template <typename B>
struct model_of {
    variable add_variable();
    template <typename LC>
    constraint add_constraint(LC &&);
    B get_basis();
    void set_basis(const B &);
    double add_constraint_dummy;
};

// get_basis spelled with an explicit object parameter, as model_base members are
template <typename B>
struct deducing_this_model {
    variable add_variable();
    template <typename LC>
    constraint add_constraint(LC &&);
    B get_basis(this auto & self);
};

using read_only = snapshot<var_status, row_status>;
using narrow_unconstrained = modifiable_unconstrained<var_status, row_status>;
using narrow_constrained = modifiable_constrained<var_status, row_status>;
using wide_constrained = modifiable_constrained<wide_status, wide_status>;

// 1. a by-value const snapshot satisfies has_lp_basis
static_assert(has_lp_basis<model_of<read_only>>);
// 2. lp_basis accepts a mutable-only getter, lp_iis's const requirement would not
static_assert(lp_basis<mutable_only_basis, model_of<read_only>>);
// 3. warm start: set_basis(const B &) is accepted from a non-const lvalue
static_assert(has_lp_basis_warm_start<model_of<read_only>>);
// 4. modifiable: unconstrained forwarding satisfies the concept on a narrow
//    variant (the deleted table overload is only seen on instantiation)
static_assert(has_modifiable_lp_basis<model_of<narrow_unconstrained>>);
// 5. ... constrained forwarding does not, unless every leaf is listed
static_assert(!has_modifiable_lp_basis<model_of<narrow_constrained>>);
static_assert(has_modifiable_lp_basis<model_of<wide_constrained>>);
// 6. the per-status aliases
static_assert(std::same_as<model_variable_basis_status_t<model_of<read_only>>,
                           var_status>);
static_assert(
    std::same_as<model_constraint_basis_status_t<model_of<read_only>>,
                 row_status>);
// 7. model_basis_t (declval<T &>) works on a deducing-this get_basis ...
static_assert(std::same_as<model_basis_t<deducing_this_model<read_only>>,
                           read_only>);
static_assert(has_lp_basis<deducing_this_model<read_only>>);
// ... while model_variable_basis_status_t (declval<T>) does not
template <typename M>
concept var_status_alias_ok =
    requires { typename model_variable_basis_status_t<M>; };
static_assert(var_status_alias_ok<model_of<read_only>>);
static_assert(!var_status_alias_ok<deducing_this_model<read_only>>);

// 8. the commented column_manager.hpp:170-176 snippet against a real basis type
template <typename M>
auto refresh(M & model, const variable & var) {
    auto basis = model.get_basis();
    return colgen::master_refreshed<model_variable_basis_status_t<M>>{
        -1.0, 2.0, basis.get_status(var)};
}
using refreshed_t =
    decltype(refresh(std::declval<model_of<read_only> &>(), variable(0)));
static_assert(std::same_as<refreshed_t, colgen::master_refreshed<var_status>>);

int main() {
    // 9. runtime: get past the bound reads alternative 0, set past it throws
    read_only b{detail::handle_status_table<var_status>(2),
                detail::handle_status_table<row_status>(1)};
    b.vars.set(1, bs::basic{});
    std::printf("var 1 basic: %d\n", is<bs::basic>(b.get_status(variable(1))));
    std::printf("var 7 (past bound) at_lower: %d\n",
                is<bs::nonbasic_at_lower_bound>(b.get_status(variable(7))));
    std::printf("row 5 (past bound) basic: %d\n",
                is<bs::basic>(b.get_status(constraint(5))));
    try {
        b.vars.set(2, bs::basic{});
        std::printf("set past bound: no throw\n");
    } catch(const std::out_of_range &) {
        std::printf("set past bound: std::out_of_range\n");
    }
    // count_a over ids below the bound, alternative 0 included
    detail::handle_status_table<var_status> t(4);
    t.set(0, bs::basic{});
    std::printf("count_a<nonbasic> over 4 ids, one basic: %zu\n",
                t.count_a<bs::nonbasic>());
    // 10. the narrow unconstrained snapshot: set_status with an unlisted tag
    //     would be a hard error here; left out so the TU compiles.
    narrow_unconstrained n{{detail::handle_status_table<var_status>(1),
                            detail::handle_status_table<row_status>(1)}};
    n.set_status(variable(0), bs::nonbasic_at_upper_bound{});
    std::printf("narrow set listed tag ok: %d\n",
                is<bs::nonbasic_at_upper_bound>(n.get_status(variable(0))));
    return 0;
}
