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
        int var_id = static_cast<int>(_lazy_num_variables);
        _add_variable(params, GRB_INTEGER);
        return variable(var_id);
    }
    auto add_integer_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = _lazy_num_variables;
        _add_variables(offset, count, params, GRB_INTEGER);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = _lazy_num_variables;
        _add_variables(offset, count, params, GRB_INTEGER);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

private:
    inline void _add_binary_variables(const std::size_t & offset,
                                      const std::size_t & count) {
        tmp_types.resize(count);
        std::fill(tmp_types.begin(), tmp_types.end(), GRB_BINARY);
        check(GRB.addvars(model, static_cast<int>(count), 0, NULL, NULL, NULL,
                          NULL, NULL, NULL, tmp_types.data(), NULL));
        _lazy_num_variables += count;
        _var_name_set.resize(offset + count, false);
    }

public:
    variable add_binary_variable() {
        int var_id = static_cast<int>(_lazy_num_variables);
        _add_variable({.obj_coef = 0.0, .lower_bound = 0.0, .upper_bound = 1.0},
                      GRB_BINARY);
        return variable(var_id);
    }
    auto add_binary_variables(std::size_t count) noexcept {
        const std::size_t offset = _lazy_num_variables;
        _add_binary_variables(offset, count);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        const std::size_t offset = _lazy_num_variables;
        _add_binary_variables(offset, count);
        return _make_indexed_variables_range(offset, count,
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

    // add_sos1_constraint
    // add_sos2_constraint

    constraint add_indicator_constraint(variable x, bool val,
                                        linear_constraint auto && lc) {
        const int constr_id = static_cast<int>(_lazy_num_constraints++);
        _reset_cache(_lazy_num_variables);
        _register_entries(lc.linear_terms());
        check(GRB.addgenconstrIndicator(
            model, NULL, x.id(), static_cast<int>(val),
            static_cast<int>(tmp_indices.size()), tmp_indices.data(),
            tmp_scalars.data(), constraint_sense_to_gurobi_sense(lc.sense()),
            lc.rhs()));
        return constraint(constr_id);
    }

    class callback_handle : public model_base<int, double> {
    private:
        const gurobi_api & GRB;
        GRBmodel * master_model;
        void * cbdata;
        int where;

    public:
        callback_handle(const gurobi_api & api, GRBmodel * master_model_,
                        void * cbdata_, int where_)
            : model_base<int, double>()
            , GRB(api)
            , master_model(master_model_)
            , cbdata(cbdata_)
            , where(where_) {}

        std::size_t num_variables() {
            int num;
            GRB.getintattr(master_model, GRB_INT_ATTR_NUMVARS, &num);
            return static_cast<std::size_t>(num);
        }

        void add_lazy_constraint(linear_constraint auto && lc) {
            _reset_cache(num_variables());
            _register_entries(lc.linear_terms());
            GRB.cblazy(cbdata, static_cast<int>(tmp_indices.size()),
                       tmp_indices.data(), tmp_scalars.data(),
                       constraint_sense_to_gurobi_sense(lc.sense()), lc.rhs());
        }

        auto get_solution() {
            auto num_vars = num_variables();
            auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
            GRB.cbget(cbdata, where, GRB_CB_MIPSOL_SOL, solution.get());
            return variable_mapping(std::move(solution));
        }
    };

private:
    std::function<void(callback_handle &)> solution_callback;

    static int main_callback(GRBmodel * master_model, void * cbdata, int where,
                             void * usrdata) {
        auto * model = static_cast<gurobi_milp *>(usrdata);
        callback_handle handle(model->GRB, master_model, cbdata, where);
        if((where == GRB_CB_MIPSOL) && model->solution_callback) {
            model->solution_callback(handle);
        }
        return 0;
    }

    void _enable_callbacks() {
        check(GRB.setintparam(env, GRB_INT_PAR_LAZYCONSTRAINTS, 1));
        check(GRB.setcallbackfunc(model, main_callback, this));
    }

public:
    template <typename F>
    void set_solution_callback(F && f) {
        _enable_callbacks();
        solution_callback = std::forward<F>(f);
    }

    void set_optimality_tolerance(double tol) {
        check(GRB.setdblparam(env, GRB_DBL_PAR_MIPGAP, tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(GRB.getdblparam(env, GRB_DBL_PAR_MIPGAP, &tol));
        return tol;
    }

    void solve() { check(GRB.optimize(model)); }

    double get_solution_value() {
        double value;
        check(GRB.getdblattr(model, GRB_DBL_ATTR_OBJVAL, &value));
        return value;
    }

    auto get_solution() {
        auto num_vars = _lazy_num_variables;
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_X, 0,
                                  static_cast<int>(num_vars), solution.get()));
        return variable_mapping(std::move(solution));
    }
};

}  // namespace gurobi::v12_0
}  // namespace fhamonic::mippp

#endif  // MIPPP_GUROBI_v12_0_MILP_HPP