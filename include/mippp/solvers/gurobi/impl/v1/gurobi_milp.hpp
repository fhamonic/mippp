#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <memory>
#include <ranges>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/restore_guard.hpp"
#include "mippp/detail/row_preparation.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/gurobi/impl/v1/gurobi_base.hpp"
#include "mippp/solvers/model_base.hpp"

namespace mippp {
namespace gurobi::impl::v1 {

class gurobi_milp : public gurobi_base {
public:
    [[nodiscard]] gurobi_milp() : gurobi_milp(gurobi_api::load()) {}
    [[nodiscard]] explicit gurobi_milp(const gurobi_api & api)
        : gurobi_base(api) {}

    using model_base<int, double>::add_integer_variable;
    using model_base<int, double>::add_integer_variables;
    using model_base<int, double>::add_binary_variable;
    using model_base<int, double>::add_binary_variables;

private:
    inline std::size_t _add_binary_variables(const std::size_t & count) {
        tmp_types.resize(count);
        std::fill(tmp_types.begin(), tmp_types.end(), GRB_BINARY);
        check(GRB->addvars(model, static_cast<int>(count), 0, nullptr, nullptr,
                           nullptr, nullptr, nullptr, nullptr, tmp_types.data(),
                           nullptr));
        if(_remap_ids) _extend_handle_ids_map(count);
        const std::size_t handle_ids_begin =
            _new_var_handle_range(_num_var_native_ids, count);
        _num_var_native_ids += count;
        return handle_ids_begin;
    }

public:
    void set_continuous(variable v) {
        _invalidate_results();
        check(GRB->setcharattrelement(model, GRB_CHAR_ATTR_VTYPE, _native_id(v),
                                      GRB_CONTINUOUS));
    }
    void set_integer(variable v) {
        _invalidate_results();
        check(GRB->setcharattrelement(model, GRB_CHAR_ATTR_VTYPE, _native_id(v),
                                      GRB_INTEGER));
    }
    void set_binary(variable v) {
        _invalidate_results();
        check(GRB->setcharattrelement(model, GRB_CHAR_ATTR_VTYPE, _native_id(v),
                                      GRB_BINARY));
    }
    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////// Special constraints ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <bool distinct, linear_constraint LC>
    void _add_indicator_constraint(variable x, bool val, LC && lc) {
        if constexpr(!distinct) _prepare_coalescing(_num_var_native_ids);
        _reset_cache();
        _register_variables_entries<distinct>(lc.linear_terms());
        check(GRB->addgenconstrIndicator(
            model, nullptr, _native_id(x), static_cast<int>(val),
            static_cast<int>(tmp_indices.size()), tmp_indices.data(),
            tmp_scalars.data(), constraint_sense_to_gurobi_sense(lc.sense()),
            lc.rhs()));
    }

public:
    template <linear_constraint LC>
    void add_indicator_constraint(variable x, bool val, LC && lc) {
        _invalidate_results();
        _add_indicator_constraint<false>(x, val, std::forward<LC>(lc));
    }
    template <linear_constraint LC>
    void add_indicator_constraint(distinct_variables_t, variable x, bool val,
                                  LC && lc) {
        _invalidate_results();
        _add_indicator_constraint<true>(x, val, std::forward<LC>(lc));
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Callbacks ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    class callback_handle_base {
    protected:
        const gurobi_milp & parent;
        GRBmodel * master_model;
        void * cbdata;

        void check(const int error) const {
            parent.GRB->_check(parent.env, error);
        }

        callback_handle_base(const gurobi_milp & parent_,
                             GRBmodel * master_model_, void * cbdata_)
            : parent(parent_), master_model(master_model_), cbdata(cbdata_) {}

    public:
        std::size_t num_variables() {
            int num;
            check(parent.GRB->getintattr(master_model, GRB_INT_ATTR_NUMVARS,
                                         &num));
            return static_cast<std::size_t>(num);
        }
    };

public:
    class candidate_solution_callback_handle
        : public callback_handle_base,
          protected model_base<int, double> {
    private:
        friend gurobi_milp;
        candidate_solution_callback_handle(const gurobi_milp & parent_,
                                           GRBmodel * master_model_,
                                           void * cbdata_)
            : callback_handle_base(parent_, master_model_, cbdata_)
            , model_base<int, double>() {}

        template <bool distinct, linear_constraint LC>
        void _add_lazy_constraint(LC && lc) {
            if constexpr(!distinct) _prepare_coalescing(num_variables());
            _reset_cache();
            const auto native_id = [this](const variable & v) {
                return parent._native_id(v);
            };
            if constexpr(distinct)
                _register_raw_entries(lc.linear_terms(), native_id);
            else
                _register_coalescing_entries(lc.linear_terms(), native_id);
            check(parent.GRB->cblazy(
                cbdata, static_cast<int>(tmp_indices.size()),
                tmp_indices.data(), tmp_scalars.data(),
                constraint_sense_to_gurobi_sense(lc.sense()), lc.rhs()));
        }

    public:
        template <linear_constraint LC>
        void add_lazy_constraint(LC && lc) {
            _add_lazy_constraint<false>(std::forward<LC>(lc));
        }
        template <linear_constraint LC>
        void add_lazy_constraint(distinct_variables_t, LC && lc) {
            _add_lazy_constraint<true>(std::forward<LC>(lc));
        }
        double get_solution_value() {
            double obj;
            check(parent.GRB->cbget(cbdata, GRB_CB_MIPSOL, GRB_CB_MIPSOL_OBJ,
                                    &obj));
            return obj;
        }
        // Captures neither the handle nor the model: a point kept past the
        // call would read a dead frame through the first, and another
        // variable's value through the second once a variable is removed.
        auto get_solution() {
            auto solution =
                std::make_unique_for_overwrite<double[]>(num_variables());
            check(parent.GRB->cbget(cbdata, GRB_CB_MIPSOL, GRB_CB_MIPSOL_SOL,
                                    solution.get()));
            return variable_mapping([native_id = parent._frozen_native_id(),
                                     solution = std::move(solution)](
                                        const variable & x) -> double & {
                return *(solution.get() + native_id(x));
            });
        }
        // `value` reads GRB_INFINITY whatever the point: from MIPSOL Gurobi
        // stores it and tries it later, showing it to this callback again.
        // The return code is dropped: thrown from here, an error would cross
        // GRBoptimize's frames.
        template <detail::point_of<variable, double> Point>
        void add_solution(Point && point) {
            _register_point(point, num_variables(), [this](const int id) {
                return parent._var_handle(id);
            });
            double value;
            parent.GRB->cbsolution(cbdata, tmp_scalars.data(), &value);
        }
    };

    class node_relaxation_callback_handle : public callback_handle_base,
                                            protected model_base<int, double> {
    private:
        std::unique_ptr<double[]> _relaxation;

        friend gurobi_milp;
        node_relaxation_callback_handle(const gurobi_milp & parent_,
                                        GRBmodel * master_model_,
                                        void * cbdata_)
            : callback_handle_base(parent_, master_model_, cbdata_)
            , model_base<int, double>() {}

        // the relaxation point in column order, read once per call
        const double * _relaxation_values() {
            if(!_relaxation) {
                _relaxation =
                    std::make_unique_for_overwrite<double[]>(num_variables());
                check(parent.GRB->cbget(cbdata, GRB_CB_MIPNODE,
                                        GRB_CB_MIPNODE_REL, _relaxation.get()));
            }
            return _relaxation.get();
        }

        template <bool distinct, linear_constraint LC>
        void _add_user_cut(LC && lc) {
            if constexpr(!distinct) _prepare_coalescing(num_variables());
            _reset_cache();
            const auto native_id = [this](const variable & v) {
                return parent._native_id(v);
            };
            if constexpr(distinct)
                _register_raw_entries(lc.linear_terms(), native_id);
            else
                _register_coalescing_entries(lc.linear_terms(), native_id);
            const auto cut = detail::prepare_user_cut(
                tmp_indices, tmp_scalars, lc.sense(),
                static_cast<double>(lc.rhs()), _relaxation_values());
            if(!cut) return;
            // The return code is dropped, as the cut is when Gurobi refuses
            // it: thrown from here, an error would cross GRBoptimize's frames.
            parent.GRB->cbcut(cbdata, static_cast<int>(tmp_indices.size()),
                              tmp_indices.data(), tmp_scalars.data(),
                              constraint_sense_to_gurobi_sense(cut->sense),
                              cut->rhs);
        }

    public:
        template <linear_constraint LC>
        void add_user_cut(LC && lc) {
            _add_user_cut<false>(std::forward<LC>(lc));
        }
        template <linear_constraint LC>
        void add_user_cut(distinct_variables_t, LC && lc) {
            _add_user_cut<true>(std::forward<LC>(lc));
        }
        // Gurobi's callback reads no value for the relaxation point.
        double get_solution_value() {
            const double * point = _relaxation_values();
            double value = parent._relaxation_objective_offset;
            for(std::size_t i = 0; i < parent._relaxation_objective.size(); ++i)
                value += parent._relaxation_objective[i] * point[i];
            return value;
        }
        // Captures neither the handle nor the model: a point kept past the
        // call would read a dead frame through the first, and another
        // variable's value through the second once a variable is removed.
        auto get_solution() {
            const std::size_t num_vars = num_variables();
            auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
            std::copy_n(_relaxation_values(), num_vars, solution.get());
            return variable_mapping([native_id = parent._frozen_native_id(),
                                     solution = std::move(solution)](
                                        const variable & x) -> double & {
                return *(solution.get() + native_id(x));
            });
        }
        // From MIPNODE Gurobi tries the point at once, and shows it to the
        // candidate callback before taking it. The return code is dropped:
        // thrown from here, an error would cross GRBoptimize's frames.
        template <detail::point_of<variable, double> Point>
        void add_solution(Point && point) {
            _register_point(point, num_variables(), [this](const int id) {
                return parent._var_handle(id);
            });
            double value;
            parent.GRB->cbsolution(cbdata, tmp_scalars.data(), &value);
        }
    };

    class progress_callback_handle : private callback_handle_base {
    private:
        friend gurobi_milp;
        using callback_handle_base::callback_handle_base;

        double _read(const int what) const {
            double value;
            this->check(this->parent.GRB->cbget(this->cbdata, GRB_CB_MIP, what,
                                                &value));
            return value;
        }

    public:
        double get_solution_value() {
            return detail::as_infinity(_read(GRB_CB_MIP_OBJBST), GRB_INFINITY);
        }
        double get_dual_bound() {
            return detail::as_infinity(_read(GRB_CB_MIP_OBJBND), GRB_INFINITY);
        }
        std::size_t get_node_count() {
            return static_cast<std::size_t>(_read(GRB_CB_MIP_NODCNT));
        }
        // Not a non-zero return from the callback: GRBoptimize fails on that.
        void interrupt() { this->parent.GRB->terminate(this->master_model); }
    };

private:
    std::function<void(candidate_solution_callback_handle &)>
        candidate_solution_callback;
    std::function<void(node_relaxation_callback_handle &)>
        node_relaxation_callback;
    std::function<void(progress_callback_handle &)> progress_callback;
    // set by the setter, cleared by the solve that honours it
    bool _candidate_callback_is_new = false;
    // the objective as the solve started, in column order
    std::vector<double> _relaxation_objective;
    double _relaxation_objective_offset = 0.0;

    static int main_callback(GRBmodel * master_model, void * cbdata, int where,
                             void * usrdata) {
        const gurobi_milp & parent = *static_cast<gurobi_milp *>(usrdata);
        if((where == GRB_CB_MIPSOL) && parent.candidate_solution_callback) {
            candidate_solution_callback_handle handle(parent, master_model,
                                                      cbdata);
            parent.candidate_solution_callback(handle);
        } else if((where == GRB_CB_MIPNODE) &&
                  parent.node_relaxation_callback) {
            // MIPNODE also fires on nodes whose relaxation is infeasible,
            // unbounded or cut off, where the point it reads is garbage: a
            // separator would still find "violated" cuts in it.
            int status;
            if(parent.GRB->cbget(cbdata, GRB_CB_MIPNODE, GRB_CB_MIPNODE_STATUS,
                                 &status) != 0 ||
               status != GRB_OPTIMAL)
                return 0;
            node_relaxation_callback_handle handle(parent, master_model,
                                                   cbdata);
            parent.node_relaxation_callback(handle);
        } else if((where == GRB_CB_MIP) && parent.progress_callback) {
            progress_callback_handle handle(parent, master_model, cbdata);
            parent.progress_callback(handle);
        }
        return 0;
    }
    // For the duration of a solve only. The user data is this: registered by a
    // setter, it dangles once the model moves and the next solve crashes.
    // PreCrush is written here and not by the setter, which would leave every
    // later solve paying for it: on models with parallel integer columns it
    // cost 100 to 300 times the nodes. Without it Gurobi ignores, with no
    // error, a cut on columns its presolve merged.
    [[nodiscard]] auto _attached_callbacks() {
        const bool cuts = static_cast<bool>(node_relaxation_callback);
        const bool attach =
            candidate_solution_callback || cuts || progress_callback;
        int precrush = 0;
        if(cuts) {
            update_gurobi_model();
            _relaxation_objective.resize(_num_var_native_ids);
            if(_num_var_native_ids > 0)
                check(
                    GRB->getdblattrarray(model, GRB_DBL_ATTR_OBJ, 0,
                                         static_cast<int>(_num_var_native_ids),
                                         _relaxation_objective.data()));
            check(GRB->getdblattr(model, GRB_DBL_ATTR_OBJCON,
                                  &_relaxation_objective_offset));
            check(GRB->getintparam(env, GRB_INT_PAR_PRECRUSH, &precrush));
            check(GRB->setintparam(env, GRB_INT_PAR_PRECRUSH, 1));
        }
        if(attach) check(GRB->setcallbackfunc(model, main_callback, this));
        return detail::restore_guard([this, attach, cuts, precrush] {
            int first_error = 0;
            if(attach)
                first_error = GRB->setcallbackfunc(model, nullptr, nullptr);
            if(cuts) {
                const int error =
                    GRB->setintparam(env, GRB_INT_PAR_PRECRUSH, precrush);
                if(first_error == 0) first_error = error;
            }
            check(first_error);
        });
    }

public:
    template <typename F>
    void set_candidate_solution_callback(F && f) {
        check(GRB->setintparam(env, GRB_INT_PAR_LAZYCONSTRAINTS, 1));
        candidate_solution_callback = std::forward<F>(f);
        _candidate_callback_is_new = true;
    }
    template <typename F>
    void set_node_relaxation_callback(F && f) {
        node_relaxation_callback = std::forward<F>(f);
    }
    auto release_node_relaxation_callback() {
        return std::exchange(node_relaxation_callback, nullptr);
    }
    // Not through the candidate callback's setter: LazyConstraints turns off
    // presolve reductions, and watching a search must not change it.
    template <typename F>
    void set_progress_callback(F && f) {
        progress_callback = std::forward<F>(f);
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// MIP start ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <typename ER>
    inline void _add_mip_start(ER && entries) {
        _reset_cache();
        _register_variables_entries<true>(entries);
        check(GRB->setdblattrlist(model, GRB_DBL_ATTR_START,
                                  static_cast<int>(tmp_indices.size()),
                                  tmp_indices.data(), tmp_scalars.data()));
    }

public:
    template <std::ranges::range ER>
    void add_mip_start(ER && entries) {
        _invalidate_results();
        _add_mip_start(entries);
    }
    void add_mip_start(
        std::initializer_list<std::pair<variable, scalar>> entries) {
        _invalidate_results();
        _add_mip_start(entries);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_optimality_tolerance(double tol) {
        check(GRB->setdblparam(env, GRB_DBL_PAR_MIPGAP, tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(GRB->getdblparam(env, GRB_DBL_PAR_MIPGAP, &tol));
        return tol;
    }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_node_limit(std::size_t n) {
        check(GRB->setdblparam(env, GRB_DBL_PAR_NODELIMIT,
                               static_cast<double>(n)));
    }
    std::size_t get_node_limit() {
        double n;
        check(GRB->getdblparam(env, GRB_DBL_PAR_NODELIMIT, &n));
        return static_cast<std::size_t>(n);
    }
    void set_solution_limit(std::size_t n) {
        check(GRB->setintparam(env, GRB_INT_PAR_SOLUTIONLIMIT,
                               static_cast<int>(n)));
    }
    std::size_t get_solution_limit() {
        int n;
        check(GRB->getintparam(env, GRB_INT_PAR_SOLUTIONLIMIT, &n));
        return static_cast<std::size_t>(n);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown,
            status::optimal,
            status::infeasible_or_unbounded,
            status::infeasible,
            status::unbounded,
            status::limit_reached,
            status::time_limit,
            status::node_limit,
            status::solution_limit,
            status::memory_limit,
            status::failed,
            status::numerical_failure,
            status::interrupted>;

    status_variant _status = status::unknown{};

    status_variant _get_status() {
        using namespace status;
        int status_;
        check(GRB->getintattr(model, GRB_INT_ATTR_STATUS, &status_));
        switch(status_) {
            case GRB_OPTIMAL:         return optimal{};
            case GRB_INF_OR_UNBD:     return infeasible_or_unbounded{};
            case GRB_INFEASIBLE:      return infeasible{};
            case GRB_UNBOUNDED:       return unbounded{};
        }
        int sol_count;
        check(GRB->getintattr(model, GRB_INT_ATTR_SOLCOUNT, &sol_count));
        const bool sol_available = sol_count > 0;
        switch(status_) {
            case GRB_INTERRUPTED:     return interrupted{sol_available};
            case GRB_SUBOPTIMAL:      return failed{sol_available};
            case GRB_NUMERIC:         return numerical_failure{sol_available};
            case GRB_TIME_LIMIT:      return time_limit{sol_available};
            case GRB_NODE_LIMIT:      return node_limit{sol_available};
            case GRB_SOLUTION_LIMIT:  return solution_limit{sol_available};
            case GRB_MEM_LIMIT:       return memory_limit{sol_available};
            case GRB_CUTOFF:
            case GRB_USER_OBJ_LIMIT:
            case GRB_WORK_LIMIT:      return limit_reached{sol_available};
            default:
                return unknown{sol_available}; 
        }
    }
    // clang-format on
public:
    const status_variant & get_status() const {
        return _results_valid ? _status
                              : detail::unknown_status<status_variant>;
    }
    void reset_status() noexcept {
        _status = status::unknown{};
        _invalidate_results();
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        reset_status();  // a solve that throws reports unknown
        // A callback is no change to Gurobi: after a finished solve it hands
        // back the solution it holds, which a candidate callback set since
        // never saw and may forbid. 0 keeps the MIP starts.
        if(std::exchange(_candidate_callback_is_new, false))
            check(GRB->reset(model, 0));
        const auto detach = _attached_callbacks();
        check(GRB->optimize(model));
        _status = _get_status();
        _mark_results_valid();
    }
    // The routine solves when it has no status to trust and overwrites the
    // Status and Runtime attributes, and on a model with indicators the
    // forcing writes discard the held solution: the reported status is reset
    // before the first native call, throw or return.
    auto compute_iis() {
        reset_status();
        return _compute_iis();
    }
    double get_solution_value() {
        _require_valid_results();
        double value;
        check(GRB->getdblattr(model, GRB_DBL_ATTR_OBJVAL, &value));
        return value;
    }
    auto get_solution() {
        _require_valid_results();
        auto solution =
            std::make_unique_for_overwrite<double[]>(_num_var_native_ids);
        check(GRB->getdblattrarray(model, GRB_DBL_ATTR_X, 0,
                                   static_cast<int>(_num_var_native_ids),
                                   solution.get()));
        return variable_mapping(
            [native_id = _frozen_native_id(),
             solution = std::move(solution)](const variable & x) {
                return *(solution.get() + native_id(x));
            });
    }
    double get_dual_bound() {
        _require_valid_results();
        double bound;
        check(GRB->getdblattr(model, GRB_DBL_ATTR_OBJBOUND, &bound));
        return bound;
    }
    std::size_t get_node_count() {
        _require_valid_results();
        double count;
        check(GRB->getdblattr(model, GRB_DBL_ATTR_NODECOUNT, &count));
        return static_cast<std::size_t>(count);
    }
};

}  // namespace gurobi::impl::v1
}  // namespace mippp
