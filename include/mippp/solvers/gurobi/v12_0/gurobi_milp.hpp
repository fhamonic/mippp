#ifndef MIPPP_GUROBI_v12_0_MILP_HPP
#define MIPPP_GUROBI_v12_0_MILP_HPP

#include <functional>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/gurobi/v12_0/gurobi_base.hpp"
#include "mippp/solvers/model_base.hpp"

namespace fhamonic::mippp {
namespace gurobi::v12_0 {

class gurobi_milp : public gurobi_base {
public:
    [[nodiscard]] explicit gurobi_milp(const gurobi_api & api)
        : gurobi_base(api) {}

    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        return _add_variable(params, GRB_INTEGER, NULL);
    }
    auto add_integer_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t handle_ids_begin =
            _add_variables(count, params, GRB_INTEGER);
        return _make_variables_range(handle_ids_begin, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t handle_ids_begin =
            _add_variables(count, params, GRB_INTEGER);
        return _make_indexed_variables_range(handle_ids_begin, count,
                                             std::forward<IL>(id_lambda));
    }

private:
    inline std::size_t _add_binary_variables(const std::size_t & count) {
        const std::size_t offset = _num_var_native_ids;
        tmp_types.resize(count);
        std::fill(tmp_types.begin(), tmp_types.end(), GRB_BINARY);
        check(GRB.addvars(model, static_cast<int>(count), 0, NULL, NULL, NULL,
                          NULL, NULL, NULL, tmp_types.data(), NULL));

        const std::size_t handle_ids_begin =
            _new_var_handle_range(_num_var_native_ids, count);
        _num_var_native_ids += count;
        _var_name_set.resize(_num_var_native_ids, false);
        return handle_ids_begin;
    }

public:
    variable add_binary_variable() {
        return _add_variable(
            {.obj_coef = 0.0, .lower_bound = 0.0, .upper_bound = 1.0},
            GRB_BINARY, NULL);
    }
    auto add_binary_variables(std::size_t count) noexcept {
        const std::size_t handle_ids_begin = _add_binary_variables(count);
        return _make_variables_range(handle_ids_begin, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        const std::size_t handle_ids_begin = _add_binary_variables(count);
        return _make_indexed_variables_range(handle_ids_begin, count,
                                             std::forward<IL>(id_lambda));
    }

    void set_continuous(variable v) noexcept {
        check(GRB.setcharattrelement(model, GRB_CHAR_ATTR_VTYPE, v.id(),
                                     GRB_CONTINUOUS));
    }
    void set_integer(variable v) noexcept {
        check(GRB.setcharattrelement(model, GRB_CHAR_ATTR_VTYPE, v.id(),
                                     GRB_INTEGER));
    }
    void set_binary(variable v) noexcept {
        check(GRB.setcharattrelement(model, GRB_CHAR_ATTR_VTYPE, v.id(),
                                     GRB_BINARY));
    }

    /////////////////////////// Special constraints ///////////////////////////
    // void add_sos1_constraint(VR && variables)
    // void add_sos2_constraint(VR && variables)

    void add_indicator_constraint(variable x, bool val,
                                  linear_constraint auto && lc) {
        _reset_cache(_num_var_native_ids);
        _register_entries(lc.linear_terms());
        check(GRB.addgenconstrIndicator(
            model, NULL, x.id(), static_cast<int>(val),
            static_cast<int>(tmp_indices.size()), tmp_indices.data(),
            tmp_scalars.data(), constraint_sense_to_gurobi_sense(lc.sense()),
            lc.rhs()));
    }

    //////////////////////////////// Callbacks ////////////////////////////////
private:
    class callback_handle_base {
    protected:
        const gurobi_milp & parent;
        GRBmodel * master_model;
        void * cbdata;

    public:
        callback_handle_base(const gurobi_milp & parent_,
                             GRBmodel * master_model_, void * cbdata_)
            : parent(parent_), master_model(master_model_), cbdata(cbdata_) {}

        std::size_t num_variables() {
            int num;
            parent.GRB.getintattr(master_model, GRB_INT_ATTR_NUMVARS, &num);
            return static_cast<std::size_t>(num);
        }
    };

public:
    class candidate_solution_callback_handle : public callback_handle_base,
                                               public model_base<int, double> {
    public:
        candidate_solution_callback_handle(const gurobi_milp & parent_,
                                           GRBmodel * master_model_,
                                           void * cbdata_)
            : callback_handle_base(parent_, master_model_, cbdata_)
            , model_base<int, double>() {}

        std::size_t num_variables() {
            int num;
            parent.GRB.getintattr(master_model, GRB_INT_ATTR_NUMVARS, &num);
            return static_cast<std::size_t>(num);
        }
        void add_lazy_constraint(linear_constraint auto && lc) {
            _reset_cache(num_variables());
            _register_entries(lc.linear_terms());
            parent.GRB.cblazy(cbdata, static_cast<int>(tmp_indices.size()),
                              tmp_indices.data(), tmp_scalars.data(),
                              constraint_sense_to_gurobi_sense(lc.sense()),
                              lc.rhs());
        }
        auto get_solution() {
            auto solution =
                std::make_unique_for_overwrite<double[]>(num_variables());
            parent.GRB.cbget(cbdata, GRB_CB_MIPSOL, GRB_CB_MIPSOL_SOL,
                             solution.get());
            return variable_mapping(
                [this, solution = std::move(solution)](const variable & x) {
                    return *(solution.get() + parent._native_id(x));
                });
        }
    };

private:
    std::function<void(candidate_solution_callback_handle &)>
        candidate_solution_callback;

    static int main_callback(GRBmodel * master_model, void * cbdata, int where,
                             void * usrdata) {
        const gurobi_milp & parent = *static_cast<gurobi_milp *>(usrdata);
        if((where == GRB_CB_MIPSOL) && parent.candidate_solution_callback) {
            candidate_solution_callback_handle handle(parent, master_model,
                                                      cbdata);
            parent.candidate_solution_callback(handle);
        }
        return 0;
    }

    void _enable_callbacks() {
        check(GRB.setintparam(env, GRB_INT_PAR_LAZYCONSTRAINTS, 1));
        check(GRB.setcallbackfunc(model, main_callback, this));
    }

public:
    template <typename F>
    void set_candidate_solution_callback(F && f) {
        _enable_callbacks();
        candidate_solution_callback = std::forward<F>(f);
    }

    //////////////////////////////// MIP start ////////////////////////////////
private:
    template <typename ER>
    inline void _add_mip_start(ER && entries) {
        _reset_raw_cache();
        _register_raw_entries(entries);
        check(GRB.setdblattrlist(model, GRB_DBL_ATTR_START,
                                 static_cast<int>(tmp_indices.size()),
                                 tmp_indices.data(), tmp_scalars.data()));
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

    ////////////////////////// Tolerance parameters ///////////////////////////
    void set_optimality_tolerance(double tol) {
        check(GRB.setdblparam(env, GRB_DBL_PAR_MIPGAP, tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(GRB.getdblparam(env, GRB_DBL_PAR_MIPGAP, &tol));
        return tol;
    }

    ///////////////////////////////// Limits //////////////////////////////////
    void set_time_limit(std::chrono::duration<double> t) {
        check(GRB.setdblparam(env, GRB_DBL_PAR_TIMELIMIT, t.count()));
    }
    auto get_time_limit() {
        double t;
        check(GRB.getdblparam(env, GRB_DBL_PAR_TIMELIMIT, &t));
        return std::chrono::duration<double>(t);
    }

    void set_node_limit(std::size_t n) {
        check(GRB.setdblparam(env, GRB_DBL_PAR_NODELIMIT,
                              static_cast<double>(n)));
    }
    std::size_t get_node_limit() {
        double n;
        check(GRB.getdblparam(env, GRB_DBL_PAR_NODELIMIT, &n));
        return static_cast<std::size_t>(n);
    }

    ////////////////////////////////// Solve //////////////////////////////////
    void solve() { check(GRB.optimize(model)); }

    double get_solution_value() {
        double value;
        check(GRB.getdblattr(model, GRB_DBL_ATTR_OBJVAL, &value));
        return value;
    }

    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(_num_var_native_ids);
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_X, 0,
                                  static_cast<int>(_num_var_native_ids),
                                  solution.get()));
        return variable_mapping(
            [this, solution = std::move(solution)](const variable & x) {
                return *(solution.get() + _native_id(x));
            });
    }

    /////////////////////////// Termination reason ////////////////////////////
    std::variant<reason::optimal, reason::infeasible, reason::unbounded,
                 reason::infeasible_or_unbounded, reason::time_limit,
                 reason::node_limit, reason::numerical_failure, reason::unknown>
    termination_reason() {
        using namespace reason;
        int status;
        check(GRB.getintattr(model, GRB_INT_ATTR_STATUS, &status));
        if(status == GRB_OPTIMAL) return optimal{};
        if(status == GRB_INFEASIBLE) return infeasible{};
        if(status == GRB_UNBOUNDED) return unbounded{};
        if(status == GRB_INF_OR_UNBD) return infeasible_or_unbounded{};
        if(status == GRB_NUMERIC || status == GRB_SUBOPTIMAL)
            return numerical_failure{};
        if(status == GRB_TIME_LIMIT) return time_limit{};
        if(status == GRB_NODE_LIMIT) return node_limit{};
        return unknown{};
    }
};

}  // namespace gurobi::v12_0
}  // namespace fhamonic::mippp

#endif  // MIPPP_GUROBI_v12_0_MILP_HPP