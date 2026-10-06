#pragma once

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/invoke_key.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/solver_exceptions.hpp"

#include "mippp/solvers/cbc/impl/v1/cbc_api.hpp"
#include "mippp/solvers/model_base.hpp"

namespace mippp {
namespace cbc::impl::v1 {

class cbc_milp : protected model_base<int, double> {
private:
    const cbc_api * Cbc;
    Cbc_Model * model;
    double objective_offset;
    double feasibility_tol;
    double objective_sense;
    // Cbc before 3.0 runs branch and bound on the model it is given: that
    // fixes the integer columns of this model at the incumbent and carries
    // the incumbent, unchecked, into the next solve, so each MIP solve runs
    // on a fresh copy, kept until the next solve for its answer.
    bool copy_mip_solves;
    Cbc_Model * mip_copy;
    std::vector<int> mip_start_indices;
    std::vector<double> mip_start_values;
    std::vector<std::pair<std::string, std::string>> parameters;
    // the time limit as set, which Cbc below 3.0 is given with a margin
    std::optional<double> time_limit;

    static constexpr char constraint_sense_to_cbc_sense(constraint_sense rel) {
        if(rel == constraint_sense::less_equal) return 'L';
        if(rel == constraint_sense::equal) return 'E';
        return 'G';
    }

    std::size_t _lazy_num_variables;
    std::size_t _lazy_num_constraints;

public:
    // the anchor model_variable_params_t deduces from
    using model_base<int, double>::default_variable_params;
    using model_base<int, double>::variables;
    using model_base<int, double>::constraints;
    double infinity() const noexcept { return COIN_DBL_MAX; }
    using model_base<int, double>::is_infinite;

    [[nodiscard]] cbc_milp() : cbc_milp(cbc_api::load()) {}
    [[nodiscard]] explicit cbc_milp(const cbc_api & api)
        : model_base<int, double>()
        , Cbc(&api)
        , model(Cbc->newModel())
        , objective_offset(0.0)
        , feasibility_tol(1e-4)
        , objective_sense(1.0)
        , copy_mip_solves(api.library_version() &&
                          api.library_version()->major < 3)
        , mip_copy(nullptr)
        , _lazy_num_variables(0)
        , _lazy_num_constraints(0) {
        Cbc->setLogLevel(model, 0);
        // Below 3.0 the C API bounds a solve by the CPU time of the whole
        // process, which other threads, a BLAS pool's included, run ahead of
        // the wall clock; Cbc's master branch counts wall time by default.
        // Recorded, so that it reaches the private copy MIP solves run on.
        if(api.library_version() && api.library_version()->major < 3)
            _set_parameter("timeMode", "elapsed");
    }
    ~cbc_milp() {
        if(mip_copy) Cbc->deleteModel(mip_copy);
        if(model) Cbc->deleteModel(model);
    }

    constexpr cbc_milp(const cbc_milp &) = delete;
    constexpr cbc_milp(cbc_milp && other) noexcept
        : model_base<int, double>(std::move(other))
        , Cbc(other.Cbc)
        , model(other.model)
        , objective_offset(other.objective_offset)
        , feasibility_tol(other.feasibility_tol)
        , objective_sense(other.objective_sense)
        , copy_mip_solves(other.copy_mip_solves)
        , mip_copy(other.mip_copy)
        , mip_start_indices(std::move(other.mip_start_indices))
        , mip_start_values(std::move(other.mip_start_values))
        , parameters(std::move(other.parameters))
        , time_limit(other.time_limit)
        , _lazy_num_variables(other._lazy_num_variables)
        , _lazy_num_constraints(other._lazy_num_constraints)
        , _status(other._status) {
        other.model = nullptr;
        other.mip_copy = nullptr;
    }

    constexpr cbc_milp & operator=(const cbc_milp &) = delete;
    constexpr cbc_milp & operator=(cbc_milp && other) = delete;

    // counted here rather than asked to Cbc: Cbc_getNumCols/Rows flush the
    // C interface's add buffer, so counting between additions would turn a
    // bulk add into one flush per element
    std::size_t num_variables() const noexcept {
        assert(static_cast<std::size_t>(Cbc->getNumCols(model)) ==
               _lazy_num_variables);
        return _lazy_num_variables;
    }
    std::size_t num_constraints() const noexcept {
        assert(static_cast<std::size_t>(Cbc->getNumRows(model)) ==
               _lazy_num_constraints);
        return _lazy_num_constraints;
    }
    std::size_t num_nonzeros() {
        return static_cast<std::size_t>(Cbc->getNumElements(model));
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Native handles /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
public:
    const cbc_api & native_api() const noexcept { return *Cbc; }
    Cbc_Model * native_model() const noexcept { return model; }
    int native_id(variable v) const noexcept { return v.id(); }
    int native_id(constraint c) const noexcept { return c.id(); }

public:
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Objective ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_maximization() {
        objective_sense = -1.0;
        Cbc->setObjSense(model, objective_sense);
    }
    void set_minimization() {
        objective_sense = 1.0;
        Cbc->setObjSense(model, objective_sense);
    }

    void set_objective_offset(double offset) { objective_offset = offset; }
    void set_objective(linear_expression auto && le) {
        for(auto && v :
            std::views::iota(0, static_cast<int>(_lazy_num_variables))) {
            Cbc->setObjCoeff(model, v, 0.0);
        }
        for(auto && [var, coef] : le.linear_terms()) {
            set_objective_coefficient(var,
                                      get_objective_coefficient(var) + coef);
        }
        set_objective_offset(le.constant());
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        set_objective(std::forward<LE>(le));
    }
    void add_to_objective(linear_expression auto && le) {
        for(auto && [var, coef] : le.linear_terms()) {
            set_objective_coefficient(var,
                                      get_objective_coefficient(var) + coef);
        }
        set_objective_offset(get_objective_offset() + le.constant());
    }
    template <linear_expression LE>
    void add_to_objective(distinct_variables_t, LE && le) {
        add_to_objective(std::forward<LE>(le));
    }
    double get_objective_offset() { return objective_offset; }
    auto get_objective() {
        return linear_expression_view(
            std::views::transform(
                std::views::iota(index{0},
                                 static_cast<index>(_lazy_num_variables)),
                [coefs = Cbc->getObjCoefficients(model)](auto i) {
                    return std::make_pair(variable(i), coefs[i]);
                }),
            get_objective_offset());
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Variables ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    inline void _add_var(const variable_params & p, const bool is_integer,
                         const char * name = "") {
        Cbc->addCol(model, name, p.lower_bound.value_or(-COIN_DBL_MAX),
                    p.upper_bound.value_or(COIN_DBL_MAX), p.obj_coef,
                    is_integer, 0, nullptr, nullptr);
        ++_lazy_num_variables;
    }

public:
    friend model_base<int, double>;
    using model_base<int, double>::add_variable;
    using model_base<int, double>::add_variables;
    using model_base<int, double>::add_named_variable;
    using model_base<int, double>::add_named_variables;
    using model_base<int, double>::add_integer_variable;
    using model_base<int, double>::add_integer_variables;
    using model_base<int, double>::add_binary_variable;
    using model_base<int, double>::add_binary_variables;

private:
    std::size_t _new_variables(std::size_t count,
                               const variable_params & params,
                               variable_kind kind) {
        const std::size_t offset = _lazy_num_variables;
        for(std::size_t i = 0; i < count; ++i)
            _add_var(params, kind != variable_kind::continuous);
        return offset;
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params) {
        _reset_cache();
        _register_constraints_entries<true>(entries);
        Cbc->addCol(model, "", params.lower_bound.value_or(-COIN_DBL_MAX),
                    params.upper_bound.value_or(COIN_DBL_MAX), params.obj_coef,
                    false, static_cast<int>(tmp_indices.size()),
                    tmp_indices.data(), tmp_scalars.data());
        return variable(static_cast<int>(_lazy_num_variables++));
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

    void set_continuous(variable v) noexcept {
        Cbc->setContinuous(model, v.id());
    }
    void set_integer(variable v) noexcept { Cbc->setInteger(model, v.id()); }
    void set_binary(variable v) noexcept {
        set_integer(v);
        set_variable_lower_bound(v, 0);
        set_variable_upper_bound(v, 1);
    }

    void set_objective_coefficient(variable v, double c) {
        Cbc->setObjCoeff(model, v.id(), c);
    }
    void set_variable_lower_bound(variable v, double lb) {
        Cbc->setColLower(model, v.id(), lb);
    }
    void set_variable_upper_bound(variable v, double ub) {
        Cbc->setColUpper(model, v.id(), ub);
    }
    void set_variable_name(variable v, std::string name) {
        Cbc->setColName(model, v.id(), const_cast<char *>(name.c_str()));
    }

    double get_objective_coefficient(variable v) {
        return Cbc->getObjCoefficients(model)[v.id()];
    }
    double get_variable_lower_bound(variable v) {
        return Cbc->getColLower(model)[v.id()];
    }
    double get_variable_upper_bound(variable v) {
        return Cbc->getColUpper(model)[v.id()];
    }
    std::string get_variable_name(variable v) {
        // getNumIntegers flushes Cbc internal buffers to update maxNameLength
        [[maybe_unused]] auto n = Cbc->getNumIntegers(model);
        auto max_length = Cbc->maxNameLength(model);
        std::string name(max_length + 1, '\0');
        Cbc->getColName(model, v.id(), name.data(), max_length + 1);
        name.resize(std::strlen(name.c_str()));
        return name;
    }
    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////////// Constraints ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // Cbc_addRow of Cbc's master branch returns without adding a row that has
    // no terms, which would shift the id of every later row
    void _check_row_added() const {
        if(!tmp_indices.empty()) return;
        const auto num_rows = static_cast<std::size_t>(Cbc->getNumRows(model));
        if(num_rows == _lazy_num_constraints)
            throw solver_error("cbc_milp: this Cbc drops rows without terms");
    }
    template <bool distinct, linear_constraint LC>
    void _add_constraint(LC && lc) {
        _reset_cache();
        _register_variables_entries<distinct>(lc.linear_terms());
        Cbc->addRow(model, "", static_cast<int>(tmp_indices.size()),
                    tmp_indices.data(), tmp_scalars.data(),
                    constraint_sense_to_cbc_sense(lc.sense()), lc.rhs());
        _check_row_added();
        ++_lazy_num_constraints;
    }

public:
    template <linear_constraint LC>
    constraint add_constraint(LC && lc) {
        _prepare_coalescing(_lazy_num_variables);
        int constr_id = static_cast<int>(_lazy_num_constraints);
        _add_constraint<false>(std::forward<LC>(lc));
        return constraint(constr_id);
    }
    template <linear_constraint LC>
    constraint add_constraint(distinct_variables_t, LC && lc) {
        int constr_id = static_cast<int>(_lazy_num_constraints);
        _add_constraint<true>(std::forward<LC>(lc));
        return constraint(constr_id);
    }

private:
    template <bool distinct, typename Key, typename LastConstrLambda>
        requires linear_constraint<
            detail::key_invoke_result_t<LastConstrLambda &, const Key &>>
    void _add_first_valued_constraint(const Key & key,
                                      LastConstrLambda & lc_lambda) {
        _add_constraint<distinct>(detail::invoke_key(lc_lambda, key));
    }
    template <bool distinct, typename Key, typename OptConstrLambda,
              typename... Tail>
        requires detail::optional_type<detail::key_invoke_result_t<
                     OptConstrLambda &, const Key &>> &&
                 linear_constraint<
                     detail::optional_type_value_t<detail::key_invoke_result_t<
                         OptConstrLambda &, const Key &>>>
    void _add_first_valued_constraint(const Key & key,
                                      OptConstrLambda & opt_lc_lambda,
                                      Tail &... tail) {
        if(const auto & opt_lc = detail::invoke_key(opt_lc_lambda, key)) {
            _add_constraint<distinct>(opt_lc.value());
            return;
        }
        _add_first_valued_constraint<distinct>(key, tail...);
    }
    template <bool distinct, std::ranges::range IR, typename... CL>
    auto _add_constraints(IR && keys, CL &... constraint_lambdas) {
        using key_t = std::ranges::range_value_t<IR>;
        if constexpr(!distinct) {
            tmp_entry_index_cache.resize(_lazy_num_variables);
        }
        const int offset = static_cast<int>(_lazy_num_constraints);
        int constr_id = offset;
        for(const key_t & key : keys) {
            _add_first_valued_constraint<distinct>(key, constraint_lambdas...);
            ++constr_id;
        }
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
    template <bool distinct, linear_expression LE>
    constraint _add_ranged_constraint(LE && le, double lb, double ub) {
        const int constr_id = static_cast<int>(_lazy_num_constraints);
        _reset_cache();
        _register_variables_entries<distinct>(le.linear_terms());
        const double c = le.constant();
        Cbc->addRow(model, "", static_cast<int>(tmp_indices.size()),
                    tmp_indices.data(), tmp_scalars.data(), 'L', ub - c);
        _check_row_added();
        Cbc->setRowLower(model, constr_id, lb - c);
        ++_lazy_num_constraints;
        return constraint(constr_id);
    }

public:
    template <linear_expression LE>
    constraint add_ranged_constraint(LE && le, double lb, double ub) {
        _prepare_coalescing(_lazy_num_variables);
        return _add_ranged_constraint<false>(std::forward<LE>(le), lb, ub);
    }
    template <linear_expression LE>
    constraint add_ranged_constraint(distinct_variables_t, LE && le, double lb,
                                     double ub) {
        return _add_ranged_constraint<true>(std::forward<LE>(le), lb, ub);
    }
    auto get_constraint_lhs(constraint constr) {
        const int num_nz = Cbc->getRowNz(model, constr.id());
        const int * ids = Cbc->getRowIndices(model, constr.id());
        const double * coeffs = Cbc->getRowCoeffs(model, constr.id());
        return std::views::transform(
            std::views::iota(0, num_nz), [ids, coeffs](int i) {
                return std::make_pair(variable(ids[i]), coeffs[i]);
            });
    }
    constraint_sense get_constraint_sense(constraint constr) {
        const double lb = Cbc->getRowLower(model)[constr.id()];
        const double ub = Cbc->getRowUpper(model)[constr.id()];
        if(lb == ub) return constraint_sense::equal;
        if(lb == -COIN_DBL_MAX) return constraint_sense::less_equal;
        if(ub == COIN_DBL_MAX) return constraint_sense::greater_equal;
        throw std::runtime_error(
            "Tried to get the sense of a ranged constraint");
    }
    double get_constraint_lower_bound(constraint constr) {
        return Cbc->getRowLower(model)[constr.id()];
    }
    double get_constraint_upper_bound(constraint constr) {
        return Cbc->getRowUpper(model)[constr.id()];
    }
    void set_constraint_lower_bound(constraint constr, double lb) {
        Cbc->setRowLower(model, constr.id(), lb);
    }
    void set_constraint_upper_bound(constraint constr, double ub) {
        Cbc->setRowUpper(model, constr.id(), ub);
    }
    double get_constraint_rhs(constraint constr) {
        if(get_constraint_sense(constr) == constraint_sense::greater_equal)
            return Cbc->getRowLower(model)[constr.id()];
        return Cbc->getRowUpper(model)[constr.id()];
    }
    auto get_constraint(constraint constr) {
        return linear_constraint_view(
            linear_expression_view(get_constraint_lhs(constr),
                                   -get_constraint_rhs(constr)),
            get_constraint_sense(constr));
    }
    void set_constraint_name(constraint constr, const std::string & name) {
        Cbc->setRowName(model, constr.id(), name.c_str());
    }
    auto get_constraint_name(constraint constr) {
        // Cbc_getRowName terminates at name[maxLength - 1]: a buffer of
        // exactly maxNameLength loses the last character of the longest name
        auto max_length = Cbc->maxNameLength(model);
        std::string name(max_length + 1, '\0');
        Cbc->getRowName(model, constr.id(), name.data(), max_length + 1);
        name.resize(std::strlen(name.c_str()));
        return name;
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// MIP start ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <typename ER>
    inline void _add_mip_start(ER && entries) {
        _reset_cache();
        _register_variables_entries<true>(entries);
        Cbc->setMIPStartI(model, static_cast<int>(tmp_indices.size()),
                          tmp_indices.data(), tmp_scalars.data());
        // Swapped rather than assigned: GCC 14 reports a false
        // -Wnull-dereference inside vector::assign here.
        mip_start_indices.swap(tmp_indices);
        mip_start_values.swap(tmp_scalars);
    }

public:
    template <std::ranges::range ER>
    void add_mip_start(ER && entries) {
        _add_mip_start(entries);
    }
    void add_mip_start(
        std::initializer_list<std::pair<variable, scalar>> entries) {
        _add_mip_start(entries);
    }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // Below 3.0 Cbc takes the CPU time its preprocessing used off the limit,
    // although its clock has already counted that time (CbcSolver.cpp, gone
    // from the master branch), so a solve could stop before the limit: it is
    // given 50 ms more, which covers every shortfall measured on small
    // models, and the limit set is read back as set.
    static constexpr double preprocessing_margin = 0.05;
    void set_time_limit(std::chrono::duration<double> t) {
        // Cbc stores a negative limit and stops at once, as under 0.
        if(t.count() < 0) throw solver_error("cbc_milp: negative time limit");
        const auto loaded = Cbc->library_version();
        const bool shortened = loaded && loaded->major < 3;
        Cbc->setMaximumSeconds(
            model, shortened ? t.count() + preprocessing_margin : t.count());
        time_limit = t.count();
    }
    auto get_time_limit() {
        return std::chrono::duration<double>(
            time_limit ? *time_limit : Cbc->getMaximumSeconds(model));
    }
    void set_node_limit(const std::size_t count) {
        Cbc->setMaximumNodes(model, static_cast<int>(count));
    }
    auto get_node_limit() {
        return static_cast<std::size_t>(Cbc->getMaximumNodes(model));
    }
    void set_solution_limit(std::size_t count) {
        Cbc->setMaximumSolutions(model, static_cast<int>(count));
    }
    auto get_solution_limit() {
        return static_cast<std::size_t>(Cbc->getMaximumSolutions(model));
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // recorded because a Cbc_Model gives no read access to its parameters
    void _set_parameter(const std::string & name, const std::string & value) {
        Cbc->setParameter(model, name.c_str(), value.c_str());
        auto it = std::ranges::find(
            parameters, name, &std::pair<std::string, std::string>::first);
        if(it == parameters.end())
            parameters.emplace_back(name, value);
        else
            it->second = value;
    }

public:
    void set_feasibility_tolerance(double tol) {
        feasibility_tol = tol;
        const auto tol_s = std::to_string(tol);
        _set_parameter("primalTolerance", tol_s);
        _set_parameter("dualTolerance", tol_s);
    }
    double get_feasibility_tolerance() { return feasibility_tol; }
    void set_optimality_tolerance(double tol) {
        Cbc->setAllowableFractionGap(model, tol);
    }
    double get_optimality_tolerance() {
        return Cbc->getAllowableFractionGap(model);
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Verbosity ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_verbose(bool verbose) { Cbc->setLogLevel(model, verbose ? 1 : 0); }
    bool is_verbose() { return Cbc->getLogLevel(model) > 0; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown,
            status::optimal,
            status::infeasible,
            status::unbounded,
            status::limit_reached,
            status::time_limit,
            status::node_limit,
            status::solution_limit,
            status::numerical_failure,
            status::interrupted>;

    status_variant _status = status::unknown{};

    // On Cbc's master branch, Cbc_solve returns as for an LP when the
    // relaxation of a MIP is infeasible, unbounded or abandoned, and
    // Cbc_status then aborts the process, hence these queries first.
    // isProvenOptimal leads: on Cbc 2.10 isContinuousUnbounded reads the
    // branch and bound state, which an LP solve leaves as it was.
    status_variant _get_status(Cbc_Model * m) {
        if (Cbc->isProvenOptimal(m)) return status::optimal{};
        if (Cbc->isProvenInfeasible(m)) return status::infeasible{};
        if (Cbc->isContinuousUnbounded(m)) return status::unbounded{};
        if (Cbc->isAbandoned(m)) return status::numerical_failure{};
        if (Cbc->getNumIntegers(m) == 0) return status::unknown{};
        switch (Cbc->status(m)) {
            case 1: {
                // unlike bestSolution(), which keeps an earlier solve's
                // incumbent on Cbc's master branch, this counts this solve's
                const bool has_sol = Cbc->numberSavedSolutions(m) > 0;
                switch (Cbc->secondaryStatus(m)) {
                    case 1: return status::infeasible{};
                    case 3: return status::node_limit{has_sol};
                    case 4: return status::time_limit{has_sol};
                    case 5: return status::interrupted{has_sol};
                    case 6: return status::solution_limit{has_sol};
                    case 7: return status::unbounded{};
                    case 8: return status::limit_reached{has_sol};
                    default: return status::unknown{has_sol};
                }
            }
            case 2: return status::numerical_failure{};
            case 5: return status::interrupted{};
            default: return status::unknown{};
        }
    }
    // clang-format on
public:
    const status_variant & get_status() const { return _status; }
    void reset_status() noexcept { _status = status::unknown{}; }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Solve ///////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    void _copy_into_mip_copy() {
        mip_copy = Cbc->newModel();
        Cbc->setLogLevel(mip_copy, Cbc->getLogLevel(model));
        Cbc->setObjSense(mip_copy, objective_sense);
        const int num_cols = static_cast<int>(_lazy_num_variables);
        const double * col_lb = Cbc->getColLower(model);
        const double * col_ub = Cbc->getColUpper(model);
        const double * obj = Cbc->getObjCoefficients(model);
        for(int j = 0; j < num_cols; ++j)
            Cbc->addCol(mip_copy, "", col_lb[j], col_ub[j], obj[j],
                        static_cast<char>(Cbc->isInteger(model, j)), 0, nullptr,
                        nullptr);
        const int num_rows = static_cast<int>(_lazy_num_constraints);
        const double * row_lb = Cbc->getRowLower(model);
        const double * row_ub = Cbc->getRowUpper(model);
        for(int i = 0; i < num_rows; ++i) {
            Cbc->addRow(mip_copy, "", Cbc->getRowNz(model, i),
                        Cbc->getRowIndices(model, i),
                        Cbc->getRowCoeffs(model, i), 'L', row_ub[i]);
            Cbc->setRowLower(mip_copy, i, row_lb[i]);
        }
        Cbc->setMaximumSeconds(mip_copy, Cbc->getMaximumSeconds(model));
        Cbc->setMaximumNodes(mip_copy, Cbc->getMaximumNodes(model));
        Cbc->setMaximumSolutions(mip_copy, Cbc->getMaximumSolutions(model));
        Cbc->setAllowableFractionGap(mip_copy,
                                     Cbc->getAllowableFractionGap(model));
        for(const auto & [name, value] : parameters)
            Cbc->setParameter(mip_copy, name.c_str(), value.c_str());
        if(!mip_start_indices.empty())
            Cbc->setMIPStartI(
                mip_copy, static_cast<int>(mip_start_indices.size()),
                mip_start_indices.data(), mip_start_values.data());
    }
    Cbc_Model * _solved_model() const noexcept {
        return mip_copy ? mip_copy : model;
    }

public:
    void solve() {
        reset_status();  // a solve that throws reports unknown
        if(mip_copy) {
            Cbc->deleteModel(mip_copy);
            mip_copy = nullptr;
        }
        // Cbc_solve crashes on a model without columns
        if(_lazy_num_variables == 0u) return;
        if(copy_mip_solves && Cbc->getNumIntegers(model) > 0)
            _copy_into_mip_copy();
        Cbc->solve(_solved_model());
        _status = _get_status(_solved_model());
    }
    double get_solution_value() {
        // computed from the solution: Cbc_getObjValue keeps the previous
        // solve's value when a re-solve of a row-less LP takes no iteration
        double value = objective_offset;
        if(_lazy_num_variables == 0u) return value;
        const double * sol = Cbc->bestSolution(_solved_model());
        if(sol == nullptr) sol = Cbc->getColSolution(_solved_model());
        const double * obj = Cbc->getObjCoefficients(model);
        for(std::size_t i = 0; i < _lazy_num_variables; ++i)
            value += obj[i] * sol[i];
        return value;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(_lazy_num_variables);
        if(_lazy_num_variables != 0u) {
            // bestSolution() is null when the model has no integer variable
            const double * sol = Cbc->bestSolution(_solved_model());
            if(sol == nullptr) sol = Cbc->getColSolution(_solved_model());
            std::copy_n(sol, _lazy_num_variables, solution.get());
        }
        return variable_mapping(std::move(solution));
    }
};

}  // namespace cbc::impl::v1
}  // namespace mippp
