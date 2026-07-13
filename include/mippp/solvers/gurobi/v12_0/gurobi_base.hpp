#ifndef MIPPP_GUROBI_v12_0_BASE_HPP
#define MIPPP_GUROBI_v12_0_BASE_HPP

#include <memory>
#include <numeric>
#include <optional>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/license_error.hpp"

#include "mippp/solvers/gurobi/v12_0/gurobi_api.hpp"
#include "mippp/solvers/remapping_model_base.hpp"

namespace mippp {
namespace gurobi::v12_0 {

class gurobi_base : public remapping_model_base<int, double> {
public:
    using indice = int;
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id>;
    template <typename Map>
    using variable_mapping = entity_mapping<variable, Map>;
    template <typename Map>
    using constraint_mapping = entity_mapping<constraint, Map>;

protected:
    const gurobi_api & GRB;
    GRBenv * env;
    GRBmodel * model;

    void check(const int error) {
        if(error == 0) return;
        if(error == 10009) throw license_error(GRB.geterrormsg(env));
        throw std::runtime_error(std::to_string(error) + " : " +
                                 GRB.geterrormsg(env));
    }
    static constexpr char constraint_sense_to_gurobi_sense(
        constraint_sense rel) {
        if(rel == constraint_sense::less_equal) return GRB_LESS_EQUAL;
        if(rel == constraint_sense::equal) return GRB_EQUAL;
        return GRB_GREATER_EQUAL;
    }
    static constexpr constraint_sense gurobi_sense_to_constraint_sense(
        char sense) {
        if(sense == GRB_LESS_EQUAL) return constraint_sense::less_equal;
        if(sense == GRB_EQUAL) return constraint_sense::equal;
        return constraint_sense::greater_equal;
    }

    std::size_t _num_var_native_ids;
    std::size_t _lazy_num_constraints;

    std::vector<int> tmp_begins;
    std::vector<char> tmp_types;
    std::vector<double> tmp_rhs;

    std::vector<bool> _var_name_set;

public:
    [[nodiscard]] explicit gurobi_base(const gurobi_api & api)
        : remapping_model_base<int, double>()
        , GRB(api)
        , _num_var_native_ids(0)
        , _lazy_num_constraints(0) {
        check(GRB.emptyenvinternal(&env, GRB_VERSION_MAJOR, GRB_VERSION_MINOR,
                                   GRB_VERSION_TECHNICAL));
        check(GRB.startenv(env));
        check(GRB.newmodel(env, &model, "GUROBI", 0, NULL, NULL, NULL, NULL,
                           NULL));
        GRB.freeenv(env);
        env = GRB.getenv(model);
        if(env == NULL)
            throw std::runtime_error(
                "gurobi_base: Could not retrieve model environement.");
    }
    ~gurobi_base() {
        if(model) check(GRB.freemodel(model));
    }

    constexpr gurobi_base(const gurobi_base &) = delete;
    constexpr gurobi_base(gurobi_base && other) noexcept
        : remapping_model_base<int, double>(std::move(other))
        , GRB(other.GRB)
        , env(other.env)
        , model(other.model)
        , _num_var_native_ids(other._num_var_native_ids)
        , _lazy_num_constraints(other._lazy_num_constraints)
        , tmp_begins(std::move(other.tmp_begins))
        , tmp_types(std::move(other.tmp_types))
        , tmp_rhs(std::move(other.tmp_rhs))
        , _var_name_set(std::move(other._var_name_set)) {
        other.model = nullptr;
        other.env = nullptr;
    }

    constexpr gurobi_base & operator=(const gurobi_base &) = delete;
    constexpr gurobi_base & operator=(gurobi_base && other) = delete;

protected:
    void update_gurobi_model() { check(GRB.updatemodel(model)); }

    int _new_var_native_id() {
        if(_remap_ids) _extend_handle_ids_map(1);
        return static_cast<int>(_num_var_native_ids++);
    }

    void _lazily_remove_variables() {
        if(_var_handles_to_delete.empty()) return;
        update_gurobi_model();
        tmp_indices.resize(0);
        for(const variable & var : _var_handles_to_delete)
            tmp_indices.emplace_back(_native_id(var));
        std::ranges::sort(tmp_indices);
        check(GRB.delvars(model, static_cast<int>(tmp_indices.size()),
                          tmp_indices.data()));

        update_gurobi_model();

        const std::size_t new_num_native_ids =
            _num_var_native_ids - tmp_indices.size();
        // Skips remapping if all deletiond are the native ids tail
        if(_remap_ids ||
           static_cast<std::size_t>(tmp_indices.front()) < new_num_native_ids) {
            if(!_remap_ids) {
                _native_ids_map.resize(_num_var_native_ids);
                _handle_ids_map.resize(_num_var_native_ids);
                std::ranges::iota(_native_ids_map, 0);
                std::ranges::iota(_handle_ids_map, 0);
                _remap_ids = true;
            }

            std::size_t offset = 0;
            for(int old_native_id :
                std::views::iota(0, static_cast<int>(_num_var_native_ids))) {
                if(offset < tmp_indices.size() &&
                   old_native_id == tmp_indices[offset]) {
                    ++offset;
                    continue;
                }
                const int handle_id =
                    _handle_ids_map[static_cast<std::size_t>(old_native_id)];
                const int new_native_id =
                    old_native_id - static_cast<int>(offset);
                _handle_ids_map[static_cast<std::size_t>(new_native_id)] =
                    handle_id;
                _native_ids_map[static_cast<std::size_t>(handle_id)] =
                    new_native_id;
            }
            _shrink_handle_ids_map(_var_handles_to_delete.size());
            _free_var_handles.append_range(_var_handles_to_delete);
        }
        _num_var_native_ids = new_num_native_ids;
        _var_handles_to_delete.clear();
    }

public:
    std::size_t num_variables() {
        int num;
        update_gurobi_model();
        check(GRB.getintattr(model, GRB_INT_ATTR_NUMVARS, &num));
        if(static_cast<std::size_t>(num) != _num_var_native_ids)
            throw std::runtime_error(
                "gurobi_base: _num_var_native_ids differs from gurobi "
                "one.");
        return _num_var_native_ids - _var_handles_to_delete.size();
    }
    std::size_t num_constraints() {
        int num;
        update_gurobi_model();
        check(GRB.getintattr(model, GRB_INT_ATTR_NUMCONSTRS, &num));
        if(static_cast<std::size_t>(num) != _lazy_num_constraints)
            throw std::runtime_error(
                "gurobi_base: _lazy_num_constraints differs from "
                "gurobi "
                "one.");
        return _lazy_num_constraints;
    }
    std::size_t num_entries() {
        int num;
        update_gurobi_model();
        check(GRB.getintattr(model, GRB_INT_ATTR_NUMNZS, &num));
        return static_cast<std::size_t>(num);
    }

    void set_maximization() {
        check(GRB.setintattr(model, GRB_INT_ATTR_MODELSENSE, GRB_MAXIMIZE));
    }
    void set_minimization() {
        check(GRB.setintattr(model, GRB_INT_ATTR_MODELSENSE, GRB_MINIMIZE));
    }

    void set_objective_offset(double constant) {
        check(GRB.setdblattr(model, GRB_DBL_ATTR_OBJCON, constant));
    }
    void set_objective(linear_expression auto && le) {
        tmp_scalars.resize(_num_var_native_ids);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[static_cast<std::size_t>(_native_id(var))] += coef;
        }
        check(GRB.setdblattrarray(model, GRB_DBL_ATTR_OBJ, 0,
                                  static_cast<int>(_num_var_native_ids),
                                  tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        _reset_cache(_num_var_native_ids);
        _register_entries(le.linear_terms());
        for(auto && [native_id, coef] :
            std::views::zip(tmp_indices, tmp_scalars)) {
            coef += get_objective_coefficient(_var_handle(native_id));
        }
        check(GRB.setdblattrlist(model, GRB_DBL_ATTR_OBJ,
                                 static_cast<int>(tmp_indices.size()),
                                 tmp_indices.data(), tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }
    double get_objective_offset() {
        double constant;
        update_gurobi_model();
        check(GRB.getdblattr(model, GRB_DBL_ATTR_OBJCON, &constant));
        return constant;
    }
    auto get_objective() {
        auto coefs =
            std::make_shared_for_overwrite<double[]>(_num_var_native_ids);
        update_gurobi_model();
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_OBJ, 0,
                                  static_cast<int>(_num_var_native_ids),
                                  coefs.get()));
        return linear_expression_view(
            std::views::transform(
                std::views::iota(0, static_cast<int>(_num_var_native_ids)),
                [this, coefs = std::move(coefs)](auto && i) {
                    return std::make_pair(_var_handle(i), coefs[i]);
                }),
            get_objective_offset());
    }

protected:
    inline variable _add_variable(const variable_params & params,
                                  const char & type, const char * name_str) {
        check(GRB.addvar(model, 0, NULL, NULL, params.obj_coef,
                         params.lower_bound.value_or(-GRB_INFINITY),
                         params.upper_bound.value_or(GRB_INFINITY), type,
                         name_str));
        return _new_var_handle(_new_var_native_id());
    }
    inline std::size_t _add_variables(const std::size_t & count,
                                      const variable_params & params,
                                      const char & type) {
        if(_remap_ids) _extend_handle_ids_map(count);
        const int new_native_ids_begin = static_cast<int>(_num_var_native_ids);
        const std::size_t handle_ids_begin =
            _new_var_handle_range(_num_var_native_ids, count);
        check(GRB.addvars(model, static_cast<int>(count), 0, NULL, NULL, NULL,
                          NULL, NULL, NULL, NULL, NULL));
        if(double obj = params.obj_coef; obj != 0.0) {
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), obj);
            check(GRB.setdblattrarray(
                model, GRB_DBL_ATTR_OBJ, new_native_ids_begin,
                static_cast<int>(count), tmp_scalars.data()));
        }
        if(double lb = params.lower_bound.value_or(-GRB_INFINITY); lb != 0.0) {
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), lb);
            check(GRB.setdblattrarray(
                model, GRB_DBL_ATTR_LB, new_native_ids_begin,
                static_cast<int>(count), tmp_scalars.data()));
        }
        if(double ub = params.upper_bound.value_or(GRB_INFINITY);
           ub != GRB_INFINITY) {
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), ub);
            check(GRB.setdblattrarray(
                model, GRB_DBL_ATTR_UB, new_native_ids_begin,
                static_cast<int>(count), tmp_scalars.data()));
        }
        if(type != GRB_CONTINUOUS) {
            tmp_types.resize(count);
            std::fill(tmp_types.begin(), tmp_types.end(), type);
            check(GRB.setcharattrarray(
                model, GRB_CHAR_ATTR_VTYPE, new_native_ids_begin,
                static_cast<int>(count), tmp_types.data()));
        }
        _num_var_native_ids += count;
        return handle_ids_begin;
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        return _add_variable(params, GRB_CONTINUOUS, NULL);
    }
    auto add_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t handle_ids_begin =
            _add_variables(count, params, GRB_CONTINUOUS);
        return _make_variables_range(handle_ids_begin, count);
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t handle_ids_begin =
            _add_variables(count, params, GRB_CONTINUOUS);
        return _make_indexed_variables_range(handle_ids_begin, count,
                                             std::forward<IL>(id_lambda));
    }

    variable add_named_variable(
        const std::string & name,
        const variable_params params = default_variable_params) {
        return _add_variable(params, GRB_CONTINUOUS, name.c_str());
    }
    template <typename NL>
    auto add_named_variables(
        std::size_t count, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t handle_ids_begin =
            _add_variables(count, params, GRB_CONTINUOUS);
        return _make_named_variables_range(handle_ids_begin, count,
                                           std::forward<NL>(name_lambda), this);
    }
    template <typename IL, typename NL>
    auto add_named_variables(
        std::size_t count, IL && id_lambda, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t handle_ids_begin =
            _add_variables(count, params, GRB_CONTINUOUS);
        return _make_indexed_named_variables_range(
            handle_ids_begin, count, std::forward<IL>(id_lambda),
            std::forward<NL>(name_lambda), this);
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params,
                                const char & type) {
        _reset_raw_cache();
        _register_raw_entries(entries);
        check(
            GRB.addvar(model, static_cast<int>(tmp_indices.size()),
                       tmp_indices.data(), tmp_scalars.data(), params.obj_coef,
                       params.lower_bound.value_or(-GRB_INFINITY),
                       params.upper_bound.value_or(GRB_INFINITY), type, NULL));
        return _new_var_handle(_new_var_native_id());
    }

public:
    template <std::ranges::range ER>
    variable add_column(
        ER && entries, const variable_params params = default_variable_params) {
        return _add_column(entries, params, GRB_CONTINUOUS);
    }
    variable add_column(
        std::initializer_list<std::pair<constraint, scalar>> entries,
        const variable_params params = default_variable_params) {
        return _add_column(entries, params, GRB_CONTINUOUS);
    }

    void remove_variable(variable v) {
        _var_handles_to_delete.emplace_back(v);
        _lazily_remove_variables();
    }
    template <std::ranges::range VR>
    void remove_variables(VR && variables) {
        _var_handles_to_delete.append_range(variables);
        _lazily_remove_variables();
    }

    void set_objective_coefficient(variable v, double c) {
        check(GRB.setdblattrelement(model, GRB_DBL_ATTR_OBJ, _native_id(v), c));
    }
    void set_variable_lower_bound(variable v, double lb) {
        check(GRB.setdblattrelement(model, GRB_DBL_ATTR_LB, _native_id(v), lb));
    }
    void set_variable_upper_bound(variable v, double ub) {
        check(GRB.setdblattrelement(model, GRB_DBL_ATTR_UB, _native_id(v), ub));
    }
    void set_variable_name(variable v, const char * name_ptr) {
        check(GRB.setstrattrelement(model, GRB_STR_ATTR_VARNAME, _native_id(v),
                                    name_ptr));
    }
    void set_variable_name(variable v, const std::string & name) {
        set_variable_name(v, name.c_str());
    }

    double get_objective_coefficient(variable v) {
        double coef;
        update_gurobi_model();
        check(GRB.getdblattrelement(model, GRB_DBL_ATTR_OBJ, _native_id(v),
                                    &coef));
        return coef;
    }
    double get_variable_lower_bound(variable v) {
        double lb;
        update_gurobi_model();
        check(
            GRB.getdblattrelement(model, GRB_DBL_ATTR_LB, _native_id(v), &lb));
        return lb;
    }
    double get_variable_upper_bound(variable v) {
        double ub;
        update_gurobi_model();
        check(
            GRB.getdblattrelement(model, GRB_DBL_ATTR_UB, _native_id(v), &ub));
        return ub;
    }
    std::string get_variable_name(variable v) {
        char * name;
        update_gurobi_model();
        check(GRB.getstrattrelement(model, GRB_STR_ATTR_VARNAME, _native_id(v),
                                    &name));
        return std::string(name);
    }

    constraint add_constraint(linear_constraint auto && lc) {
        const int constr_id = static_cast<int>(_lazy_num_constraints++);
        _reset_cache(_num_var_native_ids);
        _register_entries(lc.linear_terms());
        check(GRB.addconstr(model, static_cast<int>(tmp_indices.size()),
                            tmp_indices.data(), tmp_scalars.data(),
                            constraint_sense_to_gurobi_sense(lc.sense()),
                            lc.rhs(), NULL));
        return constraint(constr_id);
    }

private:
    template <linear_constraint LC>
    void _register_constraint(const LC & lc) {
        ++register_count;
        tmp_begins.emplace_back(static_cast<int>(tmp_indices.size()));
        tmp_types.emplace_back(constraint_sense_to_gurobi_sense(lc.sense()));
        tmp_rhs.emplace_back(lc.rhs());
        _register_entries(lc.linear_terms());
    }
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    void _register_first_valued_constraint(const Key & key,
                                           const LastConstrLambda & lc_lambda) {
        _register_constraint(lc_lambda(key));
    }
    template <typename Key, typename OptConstrLambda, typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    void _register_first_valued_constraint(
        const Key & key, const OptConstrLambda & opt_lc_lambda,
        const Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            _register_constraint(opt_lc.value());
            return;
        }
        _register_first_valued_constraint(key, tail...);
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL... constraint_lambdas) {
        _reset_cache(_num_var_native_ids);
        tmp_begins.resize(0);
        tmp_types.resize(0);
        tmp_rhs.resize(0);
        const int offset = static_cast<int>(_lazy_num_constraints);
        int constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint(key, constraint_lambdas...);
            ++constr_id;
        }
        check(GRB.addconstrs(
            model, constr_id - offset, static_cast<int>(tmp_indices.size()),
            tmp_begins.data(), tmp_indices.data(), tmp_scalars.data(),
            tmp_types.data(), tmp_rhs.data(), NULL));
        _lazy_num_constraints += static_cast<std::size_t>(constr_id - offset);
        return constraints_range(
            std::forward<IR>(keys),
            std::views::transform(std::views::iota(offset, constr_id),
                                  [](auto && i) { return constraint{i}; }));
    }

    void set_constraint_rhs(constraint constr, double rhs) {
        check(GRB.setdblattrelement(model, GRB_DBL_ATTR_RHS, constr.id(), rhs));
    }
    void set_constraint_sense(constraint constr, constraint_sense r) {
        check(GRB.setcharattrelement(model, GRB_CHAR_ATTR_SENSE, constr.id(),
                                     constraint_sense_to_gurobi_sense(r)));
    }
    // adds an equality constraint with a slack variable bounded in [0, ub-lb]
    constraint add_ranged_constraint(linear_expression auto && le, double lb,
                                     double ub) {
        int constr_id = static_cast<int>(_lazy_num_constraints++);
        _reset_cache(_num_var_native_ids);
        _register_entries(le.linear_terms());
        check(GRB.addrangeconstr(model, static_cast<int>(tmp_indices.size()),
                                 tmp_indices.data(), tmp_scalars.data(), lb, ub,
                                 NULL));
        _new_var_handle(_new_var_native_id());  // added_slack variable
        return constraint(constr_id);
    }
    // void set_constraint_name(constraint constr, auto && name);

    auto get_constraint_lhs(constraint constr) {
        int num_nz, beg;
        update_gurobi_model();
        check(GRB.getconstrs(model, &num_nz, NULL, NULL, NULL, constr.id(), 1));
        auto indices = std::make_shared_for_overwrite<int[]>(
            static_cast<std::size_t>(num_nz));
        auto coefs = std::make_shared_for_overwrite<double[]>(
            static_cast<std::size_t>(num_nz));
        check(GRB.getconstrs(model, &num_nz, &beg, indices.get(), coefs.get(),
                             constr.id(), 1));
        return std::views::transform(
            std::views::iota(0, num_nz), [this, indices = std::move(indices),
                                          coefs = std::move(coefs)](int i) {
                return std::make_pair(_var_handle(indices.get()[i]),
                                      coefs.get()[i]);
            });
    }
    double get_constraint_rhs(constraint constr) {
        double rhs;
        update_gurobi_model();
        check(
            GRB.getdblattrelement(model, GRB_DBL_ATTR_RHS, constr.id(), &rhs));
        return rhs;
    }
    constraint_sense get_constraint_sense(constraint constr) {
        char sense;
        update_gurobi_model();
        check(GRB.getcharattrelement(model, GRB_CHAR_ATTR_SENSE, constr.id(),
                                     &sense));
        return gurobi_sense_to_constraint_sense(sense);
    }
    auto get_constraint(constraint constr) {
        return linear_constraint_view(
            linear_expression_view(get_constraint_lhs(constr),
                                   -get_constraint_rhs(constr)),
            get_constraint_sense(constr));
    }

    auto get_constraint_name(constraint constr) {
        char * name;
        update_gurobi_model();
        check(GRB.getstrattrelement(model, GRB_STR_ATTR_CONSTRNAME, constr.id(),
                                    &name));
        return std::string(name);
    }

    void set_feasibility_tolerance(double tol) {
        check(GRB.setdblparam(env, GRB_DBL_PAR_FEASIBILITYTOL, tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(GRB.getdblparam(env, GRB_DBL_PAR_FEASIBILITYTOL, &tol));
        return tol;
    }
};

}  // namespace gurobi::v12_0
}  // namespace mippp

#endif  // MIPPP_GUROBI_v12_0_BASE_HPP