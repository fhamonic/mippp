#ifndef MIPPP_CPLEX_v22_12_BASE_MODEL_HPP
#define MIPPP_CPLEX_v22_12_BASE_MODEL_HPP

#include <cassert>
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

#include "mippp/solvers/cplex/v22_12/cplex_api.hpp"
#include "mippp/solvers/remapping_model_base.hpp"

namespace mippp {
namespace cplex::v22_12 {

class cplex_base : public remapping_model_base<int, double> {
public:
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
    int cplex_status;
    const cplex_api & CPX;
    CPXENVptr env;
    CPXLPptr lp;

    std::vector<int> tmp_begins;
    std::vector<char> tmp_types;
    std::vector<double> tmp_rhs;

    void check(const int error) {
        if(error == 0) return;
        char errmsg[CPXMESSAGEBUFSIZE];
        CPX.geterrorstring(env, error, errmsg);
        if(error == 1016)
            throw license_error(errmsg);
        throw std::runtime_error(errmsg);
    }
    static constexpr char constraint_sense_to_cplex_sense(
        constraint_sense rel) {
        if(rel == constraint_sense::less_equal) return 'L';
        if(rel == constraint_sense::equal) return 'E';
        return 'G';
    }
    static constexpr constraint_sense cplex_sense_to_constraint_sense(
        char sense) {
        if(sense == 'L') return constraint_sense::less_equal;
        if(sense == 'E') return constraint_sense::equal;
        return constraint_sense::greater_equal;
    }

public:
    [[nodiscard]] explicit cplex_base(const cplex_api & api)
        : remapping_model_base<int, double>()
        , CPX(api)
        , env(CPX.openCPLEX(&cplex_status))
        , lp(CPX.createprob(env, &cplex_status, "cplex_base")) {}
    ~cplex_base() {
        if(lp) check(CPX.freeprob(env, &lp));
    }

    constexpr cplex_base(const cplex_base &) = delete;
    constexpr cplex_base(cplex_base && other) noexcept
        : remapping_model_base<int, double>(std::move(other))
        , CPX(other.CPX)
        , env(other.env)
        , lp(other.lp)
        , tmp_begins(std::move(other.tmp_begins))
        , tmp_types(std::move(other.tmp_types))
        , tmp_rhs(std::move(other.tmp_rhs)) {
        other.env = nullptr;
        other.lp = nullptr;
    }

    constexpr cplex_base & operator=(const cplex_base &) = delete;
    constexpr cplex_base & operator=(cplex_base && other) = delete;

protected:
    int _new_var_native_id() {
        if(_remap_ids) _extend_handle_ids_map(1);
        return CPX.getnumcols(env, lp);
    }
    std::size_t _num_var_native_ids() {
        return static_cast<std::size_t>(CPX.getnumcols(env, lp));
    }

    void _lazily_remove_variables() {
        if(_var_handles_to_delete.empty()) return;
        const std::size_t old_num_variables =
            static_cast<std::size_t>(CPX.getnumcols(env, lp));
        tmp_indices.resize(old_num_variables);
        std::fill(tmp_indices.begin(), tmp_indices.end(), 0);
        for(const variable & var : _var_handles_to_delete) {
            tmp_indices[static_cast<std::size_t>(_native_id(var))] = 1;
        }
        check(CPX.delsetcols(env, lp, tmp_indices.data()));

        if(!_remap_ids) {
            _native_ids_map.resize(old_num_variables);
            _handle_ids_map.resize(old_num_variables);
            std::ranges::iota(_native_ids_map, 0);
            std::ranges::iota(_handle_ids_map, 0);
            _remap_ids = true;
        }
        for(std::size_t old_native_id :
            std::views::iota(std::size_t{0}, old_num_variables)) {
            const int new_native_id = tmp_indices[old_native_id];
            if(new_native_id == -1) continue;
            _native_ids_map[static_cast<std::size_t>(
                _handle_ids_map[old_native_id])] = new_native_id;
            // (new_native_id <= old_native_id) is guaranteed
            _handle_ids_map[static_cast<std::size_t>(new_native_id)] =
                _handle_ids_map[old_native_id];
        }
        _shrink_handle_ids_map(_var_handles_to_delete.size());
        _free_var_handles.append_range(_var_handles_to_delete);
        _var_handles_to_delete.clear();
    }

public:
    std::size_t num_variables() {
        return static_cast<std::size_t>(CPX.getnumcols(env, lp)) -
               _var_handles_to_delete.size();
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(CPX.getnumrows(env, lp));
    }
    std::size_t num_entries() {
        return static_cast<std::size_t>(CPX.getnumnz(env, lp));
    }

    void set_maximization() { check(CPX.chgobjsen(env, lp, CPX_MAX)); }
    void set_minimization() { check(CPX.chgobjsen(env, lp, CPX_MIN)); }

    void set_objective_offset(double constant) {
        check(CPX.chgobjoffset(env, lp, constant));
    }

    void set_objective(linear_expression auto && le) {
        const std::size_t num_vars = _num_var_native_ids();
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[static_cast<std::size_t>(_native_id(var))] += coef;
        }
        check(CPX.chgobj(env, lp, static_cast<int>(num_vars),
                         tmp_indices.data(), tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        const std::size_t num_vars = _num_var_native_ids();
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        tmp_scalars.resize(num_vars);
        check(CPX.getobj(env, lp, tmp_scalars.data(), 0,
                         static_cast<int>(num_vars) - 1));
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[static_cast<std::size_t>(_native_id(var))] += coef;
        }
        check(CPX.chgobj(env, lp, static_cast<int>(num_vars),
                         tmp_indices.data(), tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }

    double get_objective_offset() {
        double objective_offset;
        check(CPX.getobjoffset(env, lp, &objective_offset));
        return objective_offset;
    }
    auto get_objective() {
        const std::size_t num_vars = _num_var_native_ids();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        check(CPX.getobj(env, lp, coefs.get(), 0,
                         static_cast<int>(num_vars) - 1));
        return linear_expression_view(
            std::views::transform(
                std::views::iota(0, static_cast<int>(num_vars)),
                [this, coefs = std::move(coefs)](auto i) {
                    return std::make_pair(_var_handle(i), coefs[i]);
                }),
            get_objective_offset());
    }

protected:
    variable _add_variable(const variable_params & params, char type,
                           char * name = NULL) {
        const int var_id = _new_var_native_id();
        const double lb = params.lower_bound.value_or(-CPX_INFBOUND);
        const double ub = params.upper_bound.value_or(CPX_INFBOUND);
        check(CPX.newcols(env, lp, 1, &params.obj_coef, &lb, &ub,
                          (type != CPX_CONTINUOUS) ? &type : NULL, &name));
        return _new_var_handle(var_id);
    }
    std::size_t _add_variables(std::size_t count,
                               const variable_params & params, char type) {
        if(_remap_ids) _extend_handle_ids_map(count);
        const std::size_t handle_ids_begin =
            _new_var_handle_range(_num_var_native_ids(), count);
        std::optional<std::size_t> dbl_offset_1, dbl_offset_2, dbl_offset_3;
        tmp_scalars.resize(0u);
        if(params.obj_coef != 0.0) {
            dbl_offset_1.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, params.obj_coef);
        }
        if(auto lb = params.lower_bound.value_or(-CPX_INFBOUND); lb != 0.0) {
            dbl_offset_2.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, lb);
        }
        if(auto ub = params.upper_bound.value_or(CPX_INFBOUND);
           ub != CPX_INFBOUND) {
            dbl_offset_3.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, ub);
        }
        if(type != CPX_CONTINUOUS) {
            tmp_types.resize(count);
            std::fill(tmp_types.begin(), tmp_types.end(), type);
        }
        check(CPX.newcols(
            env, lp, static_cast<int>(count),
            dbl_offset_1.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_1.value()))
                : NULL,
            dbl_offset_2.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_2.value()))
                : NULL,
            dbl_offset_3.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_3.value()))
                : NULL,
            (type != CPX_CONTINUOUS) ? tmp_types.data() : NULL, NULL));
        return handle_ids_begin;
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        return _add_variable(params, CPX_CONTINUOUS);
    }
    auto add_variables(std::size_t count,
                       variable_params params = {
                           .obj_coef = 0,
                           .lower_bound = 0,
                           .upper_bound = std::nullopt}) noexcept {
        const std::size_t handle_ids_begin =
            _add_variables(count, params, CPX_CONTINUOUS);
        return _make_variables_range(handle_ids_begin, count);
    }
    template <typename IL>
    auto add_variables(std::size_t count, IL && id_lambda,
                       variable_params params = {
                           .obj_coef = 0,
                           .lower_bound = 0,
                           .upper_bound = std::nullopt}) noexcept {
        const std::size_t handle_ids_begin =
            _add_variables(count, params, CPX_CONTINUOUS);
        return _make_indexed_variables_range(handle_ids_begin, count,
                                             std::forward<IL>(id_lambda));
    }

    variable add_named_variable(
        const std::string & name,
        const variable_params params = default_variable_params) {
        return _add_variable(params, CPX_CONTINUOUS, std::string(name).data());
    }
    variable add_named_variable(
        std::string && name,
        const variable_params params = default_variable_params) {
        return _add_variable(params, CPX_CONTINUOUS, name.data());
    }
    template <typename NL>
    auto add_named_variables(
        std::size_t count, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t handle_ids_begin =
            _add_variables(count, params, CPX_CONTINUOUS);
        return _make_named_variables_range(handle_ids_begin, count,
                                           std::forward<NL>(name_lambda), this);
    }
    template <typename IL, typename NL>
    auto add_named_variables(
        std::size_t count, IL && id_lambda, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t handle_ids_begin =
            _add_variables(count, params, CPX_CONTINUOUS);
        return _make_indexed_named_variables_range(
            handle_ids_begin, count, std::forward<IL>(id_lambda),
            std::forward<NL>(name_lambda), this);
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params) {
        const int var_id = _new_var_native_id();
        const int cmatbeg = 0;
        _reset_raw_cache();
        _register_raw_entries(entries);
        const double lb = params.lower_bound.value_or(-CPX_INFBOUND);
        const double ub = params.upper_bound.value_or(CPX_INFBOUND);
        check(CPX.addcols(env, lp, 1, static_cast<int>(tmp_indices.size()),
                          &params.obj_coef, &cmatbeg, tmp_indices.data(),
                          tmp_scalars.data(), &lb, &ub, NULL));
        return _new_var_handle(var_id);
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
        int var_id = _native_id(v);
        check(CPX.chgobj(env, lp, 1, &var_id, &c));
    }
    void set_variable_lower_bound(variable v, double lb) noexcept {
        int var_id = _native_id(v);
        char lu = 'L';
        check(CPX.chgbds(env, lp, 1, &var_id, &lu, &lb));
    }
    void set_variable_upper_bound(variable v, double ub) noexcept {
        int var_id = _native_id(v);
        char lu = 'U';
        check(CPX.chgbds(env, lp, 1, &var_id, &lu, &ub));
    }
    void set_variable_name(variable v, const std::string & name) noexcept {
        int var_id = _native_id(v);
        char * col_name = const_cast<char *>(name.c_str());
        check(CPX.chgcolname(env, lp, 1, &var_id, &col_name));
    }

    double get_objective_coefficient(variable v) {
        const int var_id = _native_id(v);
        double coef;
        check(CPX.getobj(env, lp, &coef, var_id, var_id));
        return coef;
    }
    double get_variable_lower_bound(variable v) noexcept {
        const int var_id = _native_id(v);
        double b;
        check(CPX.getlb(env, lp, &b, var_id, var_id));
        return b;
    }
    double get_variable_upper_bound(variable v) noexcept {
        const int var_id = _native_id(v);
        double b;
        check(CPX.getub(env, lp, &b, var_id, var_id));
        return b;
    }
    std::string get_variable_name(variable v) noexcept {
        const int var_id = _native_id(v);
        std::string name;
        name.resize(name.capacity());
        char * subptr;
        int surplus = 0;
        if(CPX.getcolname(env, lp, &subptr, name.data(),
                          static_cast<int>(name.size()), &surplus, var_id,
                          var_id) == 1207) {
            name.resize(name.size() - static_cast<std::size_t>(surplus));
            check(CPX.getcolname(env, lp, &subptr, name.data(),
                                 static_cast<int>(name.size()), &surplus,
                                 var_id, var_id));
        }
        name.resize(name.size() - static_cast<std::size_t>(surplus + 1));
        return name;
    }

    constraint add_constraint(linear_constraint auto && lc) {
        int constr_id = static_cast<int>(num_constraints());
        _reset_cache(_num_var_native_ids());
        _register_entries(lc.linear_terms());
        int matbegin = 0;
        const double b = lc.rhs();
        const char sense = constraint_sense_to_cplex_sense(lc.sense());
        check(CPX.addrows(env, lp, 0, 1, static_cast<int>(tmp_indices.size()),
                          &b, &sense, &matbegin, tmp_indices.data(),
                          tmp_scalars.data(), NULL, NULL));
        return constraint(constr_id);
    }

private:
    template <linear_constraint LC>
    void _register_constraint(const int & constr_id, const LC & lc) {
        tmp_begins.emplace_back(static_cast<int>(tmp_indices.size()));
        tmp_types.emplace_back(constraint_sense_to_cplex_sense(lc.sense()));
        tmp_rhs.emplace_back(lc.rhs());
        _register_entries(lc.linear_terms());
    }
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    void _register_first_valued_constraint(const int & constr_id,
                                           const Key & key,
                                           const LastConstrLambda & lc_lambda) {
        _register_constraint(constr_id, lc_lambda(key));
    }
    template <typename Key, typename OptConstrLambda, typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    void _register_first_valued_constraint(
        const int & constr_id, const Key & key,
        const OptConstrLambda & opt_lc_lambda, const Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            _register_constraint(constr_id, opt_lc.value());
            return;
        }
        _register_first_valued_constraint(constr_id, key, tail...);
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL... constraint_lambdas) {
        _reset_cache(_num_var_native_ids());
        tmp_begins.resize(0);
        tmp_types.resize(0);
        tmp_rhs.resize(0);
        const int offset = static_cast<int>(num_constraints());
        int constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint(constr_id, key,
                                              constraint_lambdas...);
            ++constr_id;
        }
        check(CPX.addrows(env, lp, 0, static_cast<int>(tmp_begins.size()),
                          static_cast<int>(tmp_indices.size()), tmp_rhs.data(),
                          tmp_types.data(), tmp_begins.data(),
                          tmp_indices.data(), tmp_scalars.data(), NULL, NULL));
        return constraints_range(
            std::forward<IR>(keys),
            std::views::transform(std::views::iota(offset, constr_id),
                                  [](auto && i) { return constraint{i}; }));
    }

    void set_constraint_rhs(constraint constr, double rhs) {
        int constr_id = constr.id();
        check(CPX.chgrhs(env, lp, 1, &constr_id, &rhs));
    }
    void set_constraint_sense(constraint constr, constraint_sense r) {
        int constr_id = constr.id();
        char sense = constraint_sense_to_cplex_sense(r);
        check(CPX.chgsense(env, lp, 1, &constr_id, &sense));
    }

    auto get_constraint_lhs(constraint constr) {
        int palceholder, surplus, beg;
        if(int error = CPX.getrows(env, lp, &palceholder, NULL, NULL, NULL, 0,
                                   &surplus, constr.id(), constr.id());
           error != 1207 && error != 0)
            throw std::runtime_error("CPLEX: error " + std::to_string(error));

        const int num_nz = -surplus;
        auto indices = std::make_shared_for_overwrite<int[]>(
            static_cast<std::size_t>(num_nz));
        auto coefs = std::make_shared_for_overwrite<double[]>(
            static_cast<std::size_t>(num_nz));
        check(CPX.getrows(env, lp, &palceholder, &beg, indices.get(),
                          coefs.get(), num_nz, &surplus, constr.id(),
                          constr.id()));
        return std::views::transform(
            std::views::iota(0, num_nz), [this, indices = std::move(indices),
                                          coefs = std::move(coefs)](int i) {
                return std::make_pair(_var_handle(indices.get()[i]),
                                      coefs.get()[i]);
            });
    }
    double get_constraint_rhs(constraint constr) {
        double rhs;
        check(CPX.getrhs(env, lp, &rhs, constr.id(), constr.id()));
        return rhs;
    }
    constraint_sense get_constraint_sense(constraint constr) {
        char sense;
        check(CPX.getsense(env, lp, &sense, constr.id(), constr.id()));
        return cplex_sense_to_constraint_sense(sense);
    }
    auto get_constraint(constraint constr) {
        return linear_constraint_view(
            linear_expression_view(get_constraint_lhs(constr),
                                   -get_constraint_rhs(constr)),
            get_constraint_sense(constr));
    }
};

}  // namespace cplex::v22_12
}  // namespace mippp

#endif  // MIPPP_CPLEX_v22_12_BASE_MODEL_HPP