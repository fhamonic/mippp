#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <numeric>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/handle_status_table.hpp"
#include "mippp/detail/iis_arithmetic.hpp"
#include "mippp/detail/invoke_key.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/iis_outcome.hpp"
#include "mippp/utility/iis_snapshot.hpp"
#include "mippp/utility/variant.hpp"

#include "mippp/solvers/model_base.hpp"
#include "mippp/solvers/xpress/impl/v1/xpress_api.hpp"

namespace mippp {
namespace xpress::impl::v1 {

class xpress_base : protected model_base<int, double> {
protected:
    const xpress_api * XPRS;
    XPRSprob prob;
    double objective_offset;

    std::vector<int> tmp_begins;
    std::vector<char> tmp_types;
    std::vector<double> tmp_rhs;

    void check(const int error) { XPRS->_check(prob, error); }
    // Xpress hands its log to message callbacks and prints nothing itself:
    // this one prints it on stdout, where the other solvers print theirs.
    static void print_message(XPRSprob, void *, const char * msg, int msglen,
                              int msgtype) {
        if(msgtype < 0) {  // the end of a solve
            std::fflush(stdout);
            return;
        }
        if(msg) std::fwrite(msg, 1, static_cast<std::size_t>(msglen), stdout);
        std::fputc('\n', stdout);
    }
    static constexpr char constraint_sense_to_xpress_sense(
        constraint_sense rel) {
        if(rel == constraint_sense::less_equal) return 'L';
        if(rel == constraint_sense::equal) return 'E';
        return 'G';
    }

public:
    // the anchor model_variable_params_t deduces from
    using model_base<int, double>::default_variable_params;
    using model_base<int, double>::variables;
    using model_base<int, double>::constraints;
    double infinity() const noexcept { return XPRS_PLUSINFINITY; }
    using model_base<int, double>::is_infinite;

    [[nodiscard]] explicit xpress_base(const xpress_api & api)
        : model_base<int, double>(), XPRS(&api), objective_offset(0.0) {
        check(XPRS->createprob(&prob));
        check(XPRS->setintcontrol(prob, XPRS_OUTPUTLOG, 0));
        check(XPRS->addcbmessage(prob, print_message, nullptr, 0));
    }
    // check() would throw out of the destructor and terminate the program
    ~xpress_base() {
        if(prob) static_cast<void>(XPRS->destroyprob(prob));
    }

    constexpr xpress_base(const xpress_base &) = delete;
    constexpr xpress_base(xpress_base && other) noexcept
        : model_base<int, double>(std::move(other))
        , XPRS(other.XPRS)
        , prob(other.prob)
        , objective_offset(other.objective_offset)
        , tmp_begins(std::move(other.tmp_begins))
        , tmp_types(std::move(other.tmp_types))
        , tmp_rhs(std::move(other.tmp_rhs)) {
        other.prob = nullptr;
    }

    constexpr xpress_base & operator=(const xpress_base &) = delete;
    constexpr xpress_base & operator=(xpress_base && other) = delete;

    std::size_t num_variables() {
        int num_vars;
        check(XPRS->getintattrib(prob, XPRS_COLS, &num_vars));
        return static_cast<std::size_t>(num_vars);
    }
    std::size_t num_constraints() {
        int num_constrs;
        check(XPRS->getintattrib(prob, XPRS_ROWS, &num_constrs));
        return static_cast<std::size_t>(num_constrs);
    }
    std::size_t num_nonzeros() {
        int num_nonzeros;
        check(XPRS->getintattrib(prob, XPRS_ELEMS, &num_nonzeros));
        return static_cast<std::size_t>(num_nonzeros);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Native handles /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
public:
    const xpress_api & native_api() const noexcept { return *XPRS; }
    XPRSprob native_model() const noexcept { return prob; }
    int native_id(variable v) const noexcept { return v.id(); }
    int native_id(constraint c) const noexcept { return c.id(); }

public:
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Objective ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_maximization() {
        check(XPRS->chgobjsense(prob, XPRS_OBJ_MAXIMIZE));
    }
    void set_minimization() {
        check(XPRS->chgobjsense(prob, XPRS_OBJ_MINIMIZE));
    }

    void set_objective_offset(double constant) { objective_offset = constant; }

    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(XPRS->chgobj(prob, static_cast<int>(num_vars), tmp_indices.data(),
                           tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        set_objective(std::forward<LE>(le));
    }
    void add_to_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        tmp_scalars.resize(num_vars);
        check(XPRS->getobj(prob, tmp_scalars.data(), 0,
                           static_cast<int>(num_vars) - 1));
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(XPRS->chgobj(prob, static_cast<int>(num_vars), tmp_indices.data(),
                           tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }
    template <linear_expression LE>
    void add_to_objective(distinct_variables_t, LE && le) {
        add_to_objective(std::forward<LE>(le));
    }

    double get_objective_offset() { return objective_offset; }
    auto get_objective() {
        const auto num_vars = num_variables();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        check(
            XPRS->getobj(prob, coefs.get(), 0, static_cast<int>(num_vars) - 1));
        return linear_expression_view(
            std::views::transform(
                std::views::iota(index{0}, static_cast<index>(num_vars)),
                [coefs = std::move(coefs)](auto i) {
                    return std::make_pair(variable(i), coefs[i]);
                }),
            get_objective_offset());
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Variables ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
protected:
    void _add_variables(std::size_t offset, std::size_t count,
                        const variable_params & params, char type) {
        std::optional<std::size_t> dbl_offset_1, dbl_offset_2, dbl_offset_3;
        tmp_scalars.resize(0u);
        if(params.obj_coef != 0.0) {
            dbl_offset_1.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, params.obj_coef);
        }
        if(auto lb = params.lower_bound.value_or(XPRS_MINUSINFINITY);
           lb != 0.0) {
            dbl_offset_2.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, lb);
        }
        if(auto ub = params.upper_bound.value_or(XPRS_PLUSINFINITY);
           ub < XPRS_PLUSINFINITY) {
            dbl_offset_3.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, ub);
        }
        check(XPRS->addcols(
            prob, static_cast<int>(count), 0,
            dbl_offset_1.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_1.value()))
                : nullptr,
            nullptr, nullptr, nullptr,
            dbl_offset_2.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_2.value()))
                : nullptr,
            dbl_offset_3.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_3.value()))
                : nullptr));

        if(type != 'C') {
            tmp_indices.resize(count);
            std::iota(tmp_indices.begin(), tmp_indices.end(),
                      static_cast<int>(offset));
            tmp_types.resize(count);
            std::fill(tmp_types.begin(), tmp_types.end(), type);
            check(XPRS->chgcoltype(prob, static_cast<int>(count),
                                   tmp_indices.data(), tmp_types.data()));
        }
    }

public:
    friend model_base<int, double>;
    using model_base<int, double>::add_variable;
    using model_base<int, double>::add_variables;
    using model_base<int, double>::add_named_variable;
    using model_base<int, double>::add_named_variables;

private:
    std::size_t _new_variables(std::size_t count,
                               const variable_params & params,
                               variable_kind kind) {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params,
                       kind == variable_kind::continuous ? 'C'
                       : kind == variable_kind::integer  ? 'I'
                                                         : 'B');
        return offset;
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params) {
        const int var_id = static_cast<int>(num_variables());
        const int cmatbeg = 0;
        _reset_cache();
        _register_constraints_entries<true>(entries);
        const double lb = params.lower_bound.value_or(XPRS_MINUSINFINITY);
        const double ub = params.upper_bound.value_or(XPRS_PLUSINFINITY);
        check(XPRS->addcols(prob, 1, static_cast<int>(tmp_indices.size()),
                            &params.obj_coef, &cmatbeg, tmp_indices.data(),
                            tmp_scalars.data(), &lb, &ub));
        return variable(var_id);
    }

public:
    template <std::ranges::range ER>
    variable add_column(
        ER && entries, const variable_params params = default_variable_params) {
        return _add_column(entries, params);
    }
    variable add_column(
        std::initializer_list<std::pair<constraint, scalar>> entries,
        const variable_params params = default_variable_params) {
        return _add_column(entries, params);
    }

    void set_objective_coefficient(variable v, double c) {
        int var_id = v.id();
        check(XPRS->chgobj(prob, 1, &var_id, &c));
    }
    void set_variable_lower_bound(variable v, double lb) {
        int var_id = v.id();
        char bt = 'L';
        check(XPRS->chgbounds(prob, 1, &var_id, &bt, &lb));
    }
    void set_variable_upper_bound(variable v, double ub) {
        int var_id = v.id();
        char bt = 'U';
        check(XPRS->chgbounds(prob, 1, &var_id, &bt, &ub));
    }
    void set_variable_name(variable v, const std::string & name) {
        check(XPRS->addnames(prob, XPRS_NAMES_COLUMN, name.data(), v.id(),
                             v.id()));
    }

    double get_objective_coefficient(variable v) {
        double coef;
        check(XPRS->getobj(prob, &coef, v.id(), v.id()));
        return coef;
    }
    double get_variable_lower_bound(variable v) {
        double b;
        check(XPRS->getlb(prob, &b, v.id(), v.id()));
        return b;
    }
    double get_variable_upper_bound(variable v) {
        double b;
        check(XPRS->getub(prob, &b, v.id(), v.id()));
        return b;
    }
    std::string get_variable_name(variable v) {
        int nbytes;
        check(XPRS->getnamelist(prob, XPRS_NAMES_COLUMN, nullptr, 0, &nbytes,
                                v.id(), v.id()));
        std::string name(static_cast<std::size_t>(nbytes - 1), '\0');
        check(XPRS->getnamelist(prob, XPRS_NAMES_COLUMN, name.data(), nbytes,
                                nullptr, v.id(), v.id()));
        return name;
    }
    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////////// Constraints ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <bool distinct, linear_constraint LC>
    constraint _add_constraint(LC && lc) {
        int constr_id = static_cast<int>(num_constraints());
        if constexpr(!distinct) _prepare_coalescing(num_variables());
        _reset_cache();
        _register_variables_entries<distinct>(lc.linear_terms());
        int matbegin = 0;
        const double b = lc.rhs();
        const char sense = constraint_sense_to_xpress_sense(lc.sense());
        check(XPRS->addrows(prob, 1, static_cast<int>(tmp_indices.size()),
                            &sense, &b, nullptr, &matbegin, tmp_indices.data(),
                            tmp_scalars.data()));
        return constraint(constr_id);
    }

public:
    template <linear_constraint LC>
    constraint add_constraint(LC && lc) {
        return _add_constraint<false>(std::forward<LC>(lc));
    }
    template <linear_constraint LC>
    constraint add_constraint(distinct_variables_t, LC && lc) {
        return _add_constraint<true>(std::forward<LC>(lc));
    }

private:
    template <bool distinct, linear_constraint LC>
    void _register_constraint(LC && lc) {
        tmp_begins.emplace_back(static_cast<int>(tmp_indices.size()));
        tmp_types.emplace_back(constraint_sense_to_xpress_sense(lc.sense()));
        tmp_rhs.emplace_back(lc.rhs());
        _register_variables_entries<distinct>(lc.linear_terms());
    }
    template <bool distinct, typename Key, typename LastConstrLambda>
        requires linear_constraint<
            detail::key_invoke_result_t<LastConstrLambda &, const Key &>>
    void _register_first_valued_constraint(const Key & key,
                                           LastConstrLambda & lc_lambda) {
        _register_constraint<distinct>(detail::invoke_key(lc_lambda, key));
    }
    template <bool distinct, typename Key, typename OptConstrLambda,
              typename... Tail>
        requires detail::optional_type<detail::key_invoke_result_t<
                     OptConstrLambda &, const Key &>> &&
                 linear_constraint<
                     detail::optional_type_value_t<detail::key_invoke_result_t<
                         OptConstrLambda &, const Key &>>>
    void _register_first_valued_constraint(const Key & key,
                                           OptConstrLambda & opt_lc_lambda,
                                           Tail &... tail) {
        if(const auto & opt_lc = detail::invoke_key(opt_lc_lambda, key)) {
            _register_constraint<distinct>(opt_lc.value());
            return;
        }
        _register_first_valued_constraint<distinct>(key, tail...);
    }
    template <bool distinct, std::ranges::range IR, typename... CL>
    auto _add_constraints(IR && keys, CL &&... constraint_lambdas) {
        if constexpr(!distinct) _prepare_coalescing(num_variables());
        _reset_cache();
        tmp_begins.resize(0);
        tmp_types.resize(0);
        tmp_rhs.resize(0);
        const int offset = static_cast<int>(num_constraints());
        int constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint<distinct>(key,
                                                        constraint_lambdas...);
            ++constr_id;
        }
        check(XPRS->addrows(prob, static_cast<int>(tmp_begins.size()),
                            static_cast<int>(tmp_indices.size()),
                            tmp_types.data(), tmp_rhs.data(), nullptr,
                            tmp_begins.data(), tmp_indices.data(),
                            tmp_scalars.data()));
        return detail::keyed_entities(
            *this, keys, detail::set_constraint_name,
            entity_range(constraint{offset},
                         static_cast<std::size_t>(constr_id - offset)));
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL &&... constraint_lambdas) {
        return _add_constraints<false>(std::forward<IR>(keys),
                                       constraint_lambdas...);
    }
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(distinct_variables_t, IR && keys,
                         CL &&... constraint_lambdas) {
        return _add_constraints<true>(std::forward<IR>(keys),
                                      constraint_lambdas...);
    }

private:
    char _row_type(constraint constr) {
        char type;
        check(XPRS->getrowtype(prob, &type, constr.id(), constr.id()));
        return type;
    }
    double _row_rhs(constraint constr) {
        double rhs;
        check(XPRS->getrhs(prob, &rhs, constr.id(), constr.id()));
        return rhs;
    }
    // the width of a ranged row, whose rhs is its upper side
    double _row_range(constraint constr) {
        double range;
        check(XPRS->getrhsrange(prob, &range, constr.id(), constr.id()));
        return range;
    }
    // Xpress holds a row as a type, a rhs and, on a ranged row, a
    // non-negative width below the rhs: sides that cross have no encoding,
    // and the type goes first because changing it reinterprets the rhs.
    void _set_row_sides(constraint constr, double lower, double upper) {
        if(lower > upper)
            throw std::invalid_argument(
                "mippp: Xpress cannot hold a row whose lower side exceeds "
                "its upper side");
        const int row = constr.id();
        const bool has_lower = lower > -infinity();
        const bool has_upper = upper < infinity();
        const bool ranged = has_lower && has_upper && lower != upper;
        const char type = !has_lower && !has_upper ? 'N'
                          : !has_lower             ? 'L'
                          : !has_upper             ? 'G'
                          : ranged                 ? 'L'
                                                   : 'E';
        const double rhs = has_upper ? upper : has_lower ? lower : 0.0;
        check(XPRS->chgrowtype(prob, 1, &row, &type));
        check(XPRS->chgrhs(prob, 1, &row, &rhs));
        if(ranged) {
            const double range = upper - lower;
            check(XPRS->chgrhsrange(prob, 1, &row, &range));
        }
    }

public:
    double get_constraint_lower_bound(constraint constr) {
        switch(_row_type(constr)) {
            case 'L':
            case 'N':
                return -infinity();
            case 'R':
                return _row_rhs(constr) - _row_range(constr);
            default:
                return _row_rhs(constr);
        }
    }
    double get_constraint_upper_bound(constraint constr) {
        switch(_row_type(constr)) {
            case 'G':
            case 'N':
                return infinity();
            default:
                return _row_rhs(constr);
        }
    }
    void set_constraint_lower_bound(constraint constr, double lower) {
        _set_row_sides(constr, lower, get_constraint_upper_bound(constr));
    }
    void set_constraint_upper_bound(constraint constr, double upper) {
        _set_row_sides(constr, get_constraint_lower_bound(constr), upper);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_feasibility_tolerance(double tol) {
        check(XPRS->setdblcontrol(prob, XPRS_FEASTOL, tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(XPRS->getdblcontrol(prob, XPRS_FEASTOL, &tol));
        return tol;
    }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_time_limit(std::chrono::duration<double> t) {
        check(XPRS->setdblcontrol(prob, XPRS_TIMELIMIT, t.count()));
    }
    auto get_time_limit() {
        double t;
        check(XPRS->getdblcontrol(prob, XPRS_TIMELIMIT, &t));
        return std::chrono::duration<double>(t);
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Verbosity ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_verbose(bool verbose) {
        check(XPRS->setintcontrol(prob, XPRS_OUTPUTLOG, verbose));
    }
    bool is_verbose() {
        int verbose;
        check(XPRS->getintcontrol(prob, XPRS_OUTPUTLOG, &verbose));
        return verbose != 0;
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// IIS ////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // a small subsystem is worth more to its reader than the time the
    // quickest one saves
    static constexpr int _iis_mode_simplest = 1;
    static constexpr int _iis_call_success = 0, _iis_call_feasible = 1,
                         _iis_call_error = 2, _iis_call_stopped = 3;
    // Xpress reads its clock in steps of about 10 ms, so a limit stop can
    // return that much before the limit measured around the call.
    static constexpr double _iis_stop_clock_slack = 0.02;
    // The restriction classes the deletion filter must keep: integrality and
    // the constraint kinds that only native_model() can add, which no MIP++
    // handle names. Delayed rows are ordinary rows and stay candidates. Kept
    // restrictions are still listed in the answer, so the decoder drops
    // their entries.
    static constexpr int _iis_background_ops =
        XPRS_IISOPS_INTEGRALITY | XPRS_IISOPS_GENERAL | XPRS_IISOPS_PWL |
        XPRS_IISOPS_SET | XPRS_IISOPS_INDICATOR;

    class iis_option_guard {
    private:
        const xpress_api & _api;
        XPRSprob _prob;
        int _ops;
        bool _restored = false;

        int _write_back() const noexcept {
            return _api.setintcontrol(_prob, XPRS_IISOPS, _ops);
        }

    public:
        iis_option_guard(const xpress_api & api, XPRSprob prob, int ops)
            : _api(api), _prob(prob) {
            _api._check(_prob, _api.getintcontrol(_prob, XPRS_IISOPS, &_ops));
            _api._check(_prob, _api.setintcontrol(_prob, XPRS_IISOPS, ops));
        }
        iis_option_guard(const iis_option_guard &) = delete;
        iis_option_guard & operator=(const iis_option_guard &) = delete;

        void restore() {
            _restored = true;
            _api._check(_prob, _write_back());
        }
        // a value read back moments ago: the write cannot be rejected
        ~iis_option_guard() {
            if(!_restored) (void)_write_back();
        }
    };

    struct self_infeasible_column {
        std::size_t index;
        detail::iis_column_sides sides;
    };
    // The semi-continuous and partial-integer kinds fall to other, whose
    // admissible values the arithmetic does not decide.
    static constexpr detail::iis_column_kind _iis_column_kind(char type) {
        if(type == 'C') return detail::iis_column_kind::continuous;
        if(type == 'I') return detail::iis_column_kind::integer;
        if(type == 'B') return detail::iis_column_kind::binary;
        return detail::iis_column_kind::other;
    }
    // Xpress keeps a binary column's bounds that exclude [0, 1] as they are,
    // where a bound that leaves room for a value outside it turns the column
    // integer, so the types are read at the call.
    std::optional<self_infeasible_column> _self_infeasible_column(
        std::size_t num_col) {
        if(num_col == 0) return std::nullopt;
        const int last = static_cast<int>(num_col) - 1;
        std::vector<double> lower(num_col), upper(num_col);
        std::vector<char> types(num_col);
        check(XPRS->getlb(prob, lower.data(), 0, last));
        check(XPRS->getub(prob, upper.data(), 0, last));
        check(XPRS->getcoltype(prob, types.data(), 0, last));
        for(std::size_t j = 0; j < num_col; ++j) {
            if(const auto sides = detail::iis_self_infeasible_column(
                   lower[j], upper[j], _iis_column_kind(types[j])))
                return self_infeasible_column{j, *sides};
        }
        return std::nullopt;
    }
    // Without rows, sets, general or PWL constraints, the columns hold any
    // conflict, unless one has a kind whose admissible values the arithmetic
    // does not decide.
    bool _columns_decide_feasibility(std::size_t num_col) {
        int num_sets, num_gencons, num_pwls;
        check(XPRS->getintattrib(prob, XPRS_SETS, &num_sets));
        check(XPRS->getintattrib(prob, XPRS_GENCONS, &num_gencons));
        check(XPRS->getintattrib(prob, XPRS_PWLCONS, &num_pwls));
        if(num_sets > 0 || num_gencons > 0 || num_pwls > 0) return false;
        if(num_col == 0) return true;
        std::vector<char> types(num_col);
        check(XPRS->getcoltype(prob, types.data(), 0,
                               static_cast<int>(num_col) - 1));
        return std::ranges::none_of(types, [](char type) {
            return _iis_column_kind(type) == detail::iis_column_kind::other;
        });
    }

protected:
    using iis_outcome_type =
        std::variant<iis_outcome::incomplete, iis_outcome::irreducible,
                     iis_outcome::feasible, iis_outcome::stopped,
                     iis_outcome::time_limit>;

    template <typename VariableStatus, typename ConstraintStatus>
    iis_snapshot<variable, constraint, VariableStatus, ConstraintStatus,
                 iis_outcome_type>
    _compute_iis(const bool mip) {
        using snapshot = iis_snapshot<variable, constraint, VariableStatus,
                                      ConstraintStatus, iis_outcome_type>;
        iis_option_guard guard(*XPRS, prob, _iis_background_ops);
        // The routine reads the model's TIMELIMIT as a fresh budget of its
        // own, so nothing else is set for the call. Its stop status covers a
        // user interrupt as well and STOPSTATUS does not tell the two apart:
        // the time measured around the call does.
        const double budget = get_time_limit().count();
        const auto start = std::chrono::steady_clock::now();
        int call_status;
        check(XPRS->iisfirst(prob, _iis_mode_simplest, &call_status));
        const double elapsed = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - start)
                                   .count();
        // The answer indexes the original rows and columns, whose counts a
        // problem left presolved through the native handle reads only once the
        // routine has restored it.
        const std::size_t num_col = num_variables();
        const std::size_t num_row = num_constraints();
        detail::handle_status_table<VariableStatus> variable_table(num_col);
        detail::handle_status_table<ConstraintStatus> constraint_table(num_row);
        const auto answer = [&](iis_outcome_type outcome) {
            guard.restore();
            return snapshot(std::move(variable_table),
                            std::move(constraint_table), outcome);
        };
        const auto single_column = [&](const self_infeasible_column & col) {
            variable_table.set(col.index,
                               detail::iis_flagged_status<VariableStatus>(
                                   col.sides.lower, col.sides.upper, false));
            return answer(iis_outcome::irreducible{});
        };
        // On 45.01 the routine stops a feasible MIP without rows on the gap
        // of its internal MIP, where 47.01 answers feasible, so such a model
        // is answered from its columns.
        if(num_row == 0 && _columns_decide_feasibility(num_col)) {
            check(XPRS->iisclear(prob));
            if(const auto col = _self_infeasible_column(num_col))
                return single_column(*col);
            return answer(iis_outcome::feasible{});
        }
        // The routine refuses the whole search on a column whose bounds admit
        // no value, even when the conflict is elsewhere, and
        // XPRSgetlasterror is empty after it: such a column is an IIS by
        // itself, the answer given here.
        if(call_status == _iis_call_error) {
            if(const auto col = _self_infeasible_column(num_col))
                return single_column(*col);
            throw solver_error(
                "mippp: XPRSiisfirst refused the search, as it does on a "
                "column whose bounds admit no value, and no continuous, "
                "integer or binary column has such bounds");
        }
        if(call_status == _iis_call_feasible)
            return answer(iis_outcome::feasible{});
        if(call_status != _iis_call_success && call_status != _iis_call_stopped)
            throw solver_error(
                ("mippp: XPRSiisfirst returned the unknown status " +
                 std::to_string(call_status))
                    .c_str());
        // IISSOLSTATUS alone cannot tell a stop: it reads unstarted when the
        // limit hits the initial LP, and a stop leaves NUMIIS at 0 when it
        // comes before any subsystem.
        int completion, num_iis;
        check(XPRS->getintattrib(prob, XPRS_IISSOLSTATUS, &completion));
        check(XPRS->getintattrib(prob, XPRS_NUMIIS, &num_iis));
        // A limit that strikes before the search starts can also come back
        // as a success with the search unstarted and no subsystem, seen under
        // load. A stop well before the limit had another origin: an
        // interrupt or an iteration limit set through the native handle.
        const bool stopped = call_status == _iis_call_stopped ||
                             (completion == XPRS_IIS_UNSTARTED && num_iis < 1);
        const auto short_of = [&](bool conflict) -> iis_outcome_type {
            if(!stopped) return iis_outcome::incomplete(conflict);
            if(elapsed + _iis_stop_clock_slack >= budget)
                return iis_outcome::time_limit(conflict);
            return iis_outcome::stopped(conflict);
        };
        if(num_iis < 1) return answer(short_of(false));

        int iis_num_row = 0, iis_num_col = 0;
        check(XPRS->getiisdata(prob, 1, &iis_num_row, &iis_num_col, nullptr,
                               nullptr, nullptr, nullptr, nullptr, nullptr,
                               nullptr, nullptr));
        std::vector<int> row_index(static_cast<std::size_t>(iis_num_row)),
            col_index(static_cast<std::size_t>(iis_num_col));
        std::vector<char> row_kind(static_cast<std::size_t>(iis_num_row)),
            col_kind(static_cast<std::size_t>(iis_num_col));
        check(XPRS->getiisdata(prob, 1, &iis_num_row, &iis_num_col,
                               row_index.data(), col_index.data(),
                               row_kind.data(), col_kind.data(), nullptr,
                               nullptr, nullptr, nullptr));
        // An entity listed twice, once per side, needs both, so the sides
        // accumulate rather than the second entry replacing the first.
        std::vector<char> row_lower(num_row, char{0}),
            row_upper(num_row, char{0}), col_lower(num_col, char{0}),
            col_upper(num_col, char{0});
        // A set entry ('1', '2') carries a set index, not a row index, and
        // an indicator entry ('I') its indicator row: neither is a member,
        // and neither may be looked up as a row.
        for(std::size_t k = 0; k < row_index.size(); ++k) {
            const auto row = static_cast<std::size_t>(row_index[k]);
            switch(row_kind[k]) {
                case 'L':
                    row_upper[row] = 1;
                    break;
                case 'G':
                    row_lower[row] = 1;
                    break;
                case 'E':
                    row_lower[row] = row_upper[row] = 1;
                    break;
                default:
                    break;
            }
        }
        // 'F' is the fixing of a column, both bounds at once; 'B', 'I' and
        // the semi-continuous kinds are the kept integrality restrictions
        for(std::size_t k = 0; k < col_index.size(); ++k) {
            const auto col = static_cast<std::size_t>(col_index[k]);
            switch(col_kind[k]) {
                case 'L':
                    col_lower[col] = 1;
                    break;
                case 'U':
                    col_upper[col] = 1;
                    break;
                case 'F':
                    col_lower[col] = col_upper[col] = 1;
                    break;
                default:
                    break;
            }
        }
        // the problem keeps the IIS data until the next search otherwise
        check(XPRS->iisclear(prob));
        // On a MIP the routine lists a ranged row ('R') by one side where
        // integrality needs both, so such a row is reported whole. An
        // equality row gets 'E' there, so its listed side stands.
        std::vector<char> row_type;
        if(mip && num_row > 0) {
            row_type.assign(num_row, char{0});
            check(XPRS->getrowtype(prob, row_type.data(), 0,
                                   static_cast<int>(num_row) - 1));
        }
        for(std::size_t i = 0; i < num_row; ++i) {
            if(!row_lower[i] && !row_upper[i]) continue;
            const bool whole = mip && row_type[i] == 'R';
            constraint_table.set(
                i, detail::iis_flagged_status<ConstraintStatus>(
                       row_lower[i] != 0, row_upper[i] != 0, whole));
        }
        for(std::size_t j = 0; j < num_col; ++j) {
            if(!col_lower[j] && !col_upper[j]) continue;
            variable_table.set(
                j, detail::iis_flagged_status<VariableStatus>(
                       col_lower[j] != 0, col_upper[j] != 0, false));
        }
        if(completion == XPRS_IIS_COMPLETED)
            return answer(iis_outcome::irreducible{});
        return answer(short_of(true));
    }
};

}  // namespace xpress::impl::v1
}  // namespace mippp
