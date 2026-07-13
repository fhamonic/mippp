#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

// Fuzzy differential testing of LP models against the trivially correct
// dumb_lp reference implementation.
//
// lp_fuzzy_state_machine maintains an instance of the tested model and an
// instance of dumb_lp, applies the same randomly generated operations to
// both and periodically (after a random number of operations) tests, in a
// randomized order, that all the getters have matching outputs.
//
// The machine is event based: each event has a name, an applicability guard
// (enough variables/constraints in the current state) and an action. Events
// are of two kinds:
//   - mutation events ("add variable", "remove variable", ...) that apply
//     the same operation to both models,
//   - check events ("test matching get constraint", ...) that compare the
//     outputs of one getter on both models.
// Events requiring optional capabilities are only registered when the tested
// model satisfies the corresponding concept. New events can be added with
// register_mutation() / register_check().
//
// Because models may recycle entity ids differently after removals, the
// machine tracks (tested, reference) handle pairs, labeled x0, x1, ... and
// c0, c1, ... in traces and failure messages, and compares expressions
// factorized through this pairing. Solutions are compared by optimality
// status and objective value only (LPs may have several optimal solutions);
// the tested solution is additionally checked to be feasible for the
// reference model's data.
//
// Every executed operation is recorded with its generated values; when a
// failure occurs, the whole trace of operations that led to it is printed.
//
// Environment variables:
//   MIPPP_FUZZ_SEED     reproduce a run (the seed is printed on each run)
//   MIPPP_FUZZ_ROUNDS   number of rounds (default 40)
//   MIPPP_FUZZ_VERBOSE  if set, stream each operation to stderr as it is
//                       executed (useful when the tested solver crashes
//                       before the failure trace can be printed)

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <format>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <random>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"

#include "dumb_lp.hpp"

namespace fhamonic::mippp {

template <typename Tested>
class lp_fuzzy_state_machine {
public:
    static constexpr double infinity_threshold = 1e20;
    static constexpr double data_tolerance = 1e-9;
    static constexpr double solution_tolerance = 1e-5;

    using tested_variable = typename Tested::variable;
    using tested_constraint = typename Tested::constraint;
    using reference_variable = dumb_lp::variable;
    using reference_constraint = dumb_lp::constraint;

private:
    struct variable_entry {
        tested_variable tested;
        reference_variable reference;
        std::string label;
        bool named;
    };
    struct constraint_entry {
        tested_constraint tested;
        reference_constraint reference;
        std::string label;
    };
    struct event {
        std::string name;
        std::function<bool()> applicable;
        std::function<void()> action;
    };

    unsigned long _seed;
    std::mt19937 _rng;
    Tested _tested;
    dumb_lp _reference;
    std::vector<variable_entry> _variables;
    std::vector<constraint_entry> _constraints;
    std::size_t _variable_counter = 0;
    std::size_t _constraint_counter = 0;
    std::size_t _name_counter = 0;
    std::vector<event> _mutation_events;
    std::vector<event> _check_events;
    std::vector<std::string> _trace;
    bool _verbose;

public:
    lp_fuzzy_state_machine(Tested && tested, dumb_lp && reference,
                           unsigned long seed)
        : _seed(seed)
        , _rng(static_cast<std::mt19937::result_type>(seed))
        , _tested(std::move(tested))
        , _reference(std::move(reference))
        , _verbose(std::getenv("MIPPP_FUZZ_VERBOSE") != nullptr) {
        _register_default_events();
    }

    Tested & tested() { return _tested; }
    dumb_lp & reference() { return _reference; }
    const std::vector<std::string> & trace() const { return _trace; }

    template <typename Guard, typename Action>
    void register_mutation(std::string name, Guard && guard, Action && action) {
        _mutation_events.emplace_back(std::move(name),
                                      std::forward<Guard>(guard),
                                      std::forward<Action>(action));
    }
    template <typename Guard, typename Action>
    void register_check(std::string name, Guard && guard, Action && action) {
        _check_events.emplace_back(std::move(name), std::forward<Guard>(guard),
                                   std::forward<Action>(action));
    }

    ////////////////////////////////////////////////////////////////////////////
    // Operations trace
    ////////////////////////////////////////////////////////////////////////////

private:
    void _log(std::string line) {
        if(_verbose) std::cout << "[   FUZZ   ] " << line << std::endl;
        _trace.emplace_back(std::move(line));
    }
    void _print_trace() const {
        std::cout << "[   FUZZ   ] failure with MIPPP_FUZZ_SEED=" << _seed
                  << " after the following operations:\n";
        for(const std::string & line : _trace)
            std::cout << "[   FUZZ   ]     " << line << '\n';
        std::cout << std::flush;
    }

    std::string _new_variable_label() {
        return std::format("x{}", _variable_counter++);
    }
    std::string _new_constraint_label() {
        return std::format("c{}", _constraint_counter++);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Random generation helpers
    ////////////////////////////////////////////////////////////////////////////

    int _rand_int(int min, int max) {
        return std::uniform_int_distribution<int>(min, max)(_rng);
    }
    std::size_t _rand_index(std::size_t size) {
        return static_cast<std::size_t>(
            _rand_int(0, static_cast<int>(size) - 1));
    }
    bool _rand_bool(double proba) {
        return std::bernoulli_distribution(proba)(_rng);
    }
    double _rand_coef() {
        static constexpr double choices[] = {-3.0, -2.5, -2.0, -1.0, -0.5,
                                             0.5,  1.0,  2.0,  2.5,  3.0};
        return choices[_rand_index(std::size(choices))];
    }
    double _rand_obj_coef() { return _rand_bool(0.2) ? 0.0 : _rand_coef(); }
    double _rand_rhs() { return static_cast<double>(_rand_int(-10, 10)); }
    constraint_sense _rand_sense() {
        switch(_rand_int(0, 2)) {
            case 0:
                return constraint_sense::less_equal;
            case 1:
                return constraint_sense::greater_equal;
            default:
                return constraint_sense::equal;
        }
    }

    struct params_spec {
        double obj_coef;
        std::optional<double> lower_bound;
        std::optional<double> upper_bound;
    };
    // mostly finite bounds so that generated problems are mostly bounded
    params_spec _rand_params() {
        params_spec spec{.obj_coef = _rand_obj_coef(),
                         .lower_bound = std::nullopt,
                         .upper_bound = std::nullopt};
        if(!_rand_bool(0.15))
            spec.lower_bound = static_cast<double>(_rand_int(-5, 3));
        if(!_rand_bool(0.15))
            spec.upper_bound = spec.lower_bound.value_or(-5.0) +
                               static_cast<double>(_rand_int(0, 8));
        return spec;
    }
    template <typename M>
    static typename M::variable_params _to_params(const params_spec & spec) {
        return {.obj_coef = spec.obj_coef,
                .lower_bound = spec.lower_bound,
                .upper_bound = spec.upper_bound};
    }
    static std::string _params_str(const params_spec & spec) {
        return std::format(
            "{{.obj_coef={}, .lower_bound={}, .upper_bound={}}}", spec.obj_coef,
            spec.lower_bound.has_value() ? std::format("{}", *spec.lower_bound)
                                         : "-inf",
            spec.upper_bound.has_value() ? std::format("{}", *spec.upper_bound)
                                         : "+inf");
    }

    // terms are (index in _variables, coefficient); indices may repeat in
    // order to exercise the merging of duplicated terms
    struct expr_spec {
        std::vector<std::pair<std::size_t, double>> terms;
        double constant;
    };
    expr_spec _rand_expr(std::size_t max_num_terms) {
        expr_spec spec{.terms = {}, .constant = _rand_rhs()};
        if(_variables.empty()) return spec;
        const int num_terms =
            _rand_int(0, static_cast<int>(std::min<std::size_t>(
                             max_num_terms, _variables.size() + 1)));
        for(int i = 0; i < num_terms; ++i)
            spec.terms.emplace_back(_rand_index(_variables.size()),
                                    _rand_coef());
        return spec;
    }
    struct constraint_spec {
        expr_spec lhs;
        constraint_sense sense;
        double rhs;
    };
    constraint_spec _rand_constraint() {
        constraint_spec spec{
            .lhs = _rand_expr(4u), .sense = _rand_sense(), .rhs = _rand_rhs()};
        spec.lhs.constant = 0.0;
        if(spec.lhs.terms.empty())
            spec.lhs.terms.emplace_back(_rand_index(_variables.size()),
                                        _rand_coef());
        return spec;
    }

    std::vector<std::pair<tested_variable, double>> _tested_terms(
        const expr_spec & spec) {
        std::vector<std::pair<tested_variable, double>> terms;
        for(const auto & [index, coef] : spec.terms)
            terms.emplace_back(_variables[index].tested, coef);
        return terms;
    }
    std::vector<std::pair<reference_variable, double>> _reference_terms(
        const expr_spec & spec) {
        std::vector<std::pair<reference_variable, double>> terms;
        for(const auto & [index, coef] : spec.terms)
            terms.emplace_back(_variables[index].reference, coef);
        return terms;
    }

    static const char * _sense_str(constraint_sense sense) {
        switch(sense) {
            case constraint_sense::less_equal:
                return "<=";
            case constraint_sense::greater_equal:
                return ">=";
            default:
                return "==";
        }
    }
    std::string _expr_str(const expr_spec & spec) const {
        std::string str;
        for(const auto & [index, coef] : spec.terms) {
            if(!str.empty()) str += " + ";
            str += std::format("{}*{}", coef, _variables[index].label);
        }
        if(spec.constant != 0.0 || str.empty()) {
            if(!str.empty()) str += " + ";
            str += std::format("{}", spec.constant);
        }
        return str;
    }
    std::string _constraint_str(const constraint_spec & spec) const {
        return std::format("{} {} {}", _expr_str(spec.lhs),
                           _sense_str(spec.sense), spec.rhs);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Comparison helpers
    ////////////////////////////////////////////////////////////////////////////

    std::map<int, std::size_t> _tested_variable_index() const {
        std::map<int, std::size_t> index_of;
        for(std::size_t i = 0; i < _variables.size(); ++i)
            index_of[static_cast<int>(_variables[i].tested.id())] = i;
        return index_of;
    }
    std::map<int, std::size_t> _reference_variable_index() const {
        std::map<int, std::size_t> index_of;
        for(std::size_t i = 0; i < _variables.size(); ++i)
            index_of[static_cast<int>(_variables[i].reference.id())] = i;
        return index_of;
    }

    // factorizes terms as a map from index in _variables to coefficient;
    // terms referencing unknown variables (e.g. dense representations
    // including removed columns) must have a zero coefficient
    template <typename Terms>
    std::map<std::size_t, double> _factorize_terms(
        Terms && terms, const std::map<int, std::size_t> & index_of,
        const char * side) {
        std::map<std::size_t, double> factorized;
        for(auto && [var, coef] : terms) {
            const auto it = index_of.find(static_cast<int>(var.id()));
            if(it == index_of.end()) {
                EXPECT_NEAR(static_cast<double>(coef), 0.0, data_tolerance)
                    << side << " expression references the unknown variable id "
                    << var.id() << " with a nonzero coefficient";
                continue;
            }
            factorized[it->second] += static_cast<double>(coef);
        }
        return factorized;
    }
    template <typename TestedTerms, typename ReferenceTerms>
    void _expect_matching_terms(TestedTerms && tested_terms,
                                ReferenceTerms && reference_terms) {
        const auto tested =
            _factorize_terms(tested_terms, _tested_variable_index(), "tested");
        const auto reference = _factorize_terms(
            reference_terms, _reference_variable_index(), "reference");
        for(const auto & [index, tested_coef] : tested) {
            const double reference_coef =
                reference.contains(index) ? reference.at(index) : 0.0;
            EXPECT_NEAR(tested_coef, reference_coef, data_tolerance)
                << "coefficient of " << _variables[index].label;
        }
        for(const auto & [index, reference_coef] : reference) {
            if(tested.contains(index)) continue;
            EXPECT_NEAR(0.0, reference_coef, data_tolerance)
                << "coefficient of " << _variables[index].label;
        }
    }

    static double _clamp_infinite(double value) {
        if(value >= infinity_threshold)
            return std::numeric_limits<double>::infinity();
        if(value <= -infinity_threshold)
            return -std::numeric_limits<double>::infinity();
        return value;
    }
    static void _expect_matching_bound(double tested, double reference) {
        tested = _clamp_infinite(tested);
        reference = _clamp_infinite(reference);
        if(std::isinf(tested) || std::isinf(reference)) {
            EXPECT_EQ(tested, reference);
            return;
        }
        EXPECT_NEAR(tested, reference, data_tolerance);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Default events
    ////////////////////////////////////////////////////////////////////////////

    bool _has_variables() const { return !_variables.empty(); }
    bool _has_constraints() const { return !_constraints.empty(); }

    void _register_default_events() {
        _register_default_mutations();
        _register_default_checks();
    }

    void _register_default_mutations() {
        register_mutation(
            "add variable", [] { return true; },
            [this] {
                const params_spec spec = _rand_params();
                const std::string label = _new_variable_label();
                _log(std::format("{} = add_variable({})", label,
                                 _params_str(spec)));
                _variables.emplace_back(
                    _tested.add_variable(_to_params<Tested>(spec)),
                    _reference.add_variable(_to_params<dumb_lp>(spec)), label,
                    false);

                // std::cout << "add_variable -> " <<
                // _variables.back().tested.id()
                //           << " : " << _variables.back().reference.id()
                //           << std::endl;
            });
        register_mutation(
            "add variables", [] { return true; },
            [this] {
                const std::size_t count =
                    static_cast<std::size_t>(_rand_int(0, 3));
                const params_spec spec = _rand_params();
                std::string labels;
                std::vector<std::string> new_labels;
                for(std::size_t i = 0; i < count; ++i) {
                    new_labels.emplace_back(_new_variable_label());
                    if(!labels.empty()) labels += ", ";
                    labels += new_labels.back();
                }
                _log(std::format("{}{}add_variables({}, {})", labels,
                                 count == 0 ? "" : " = ", count,
                                 _params_str(spec)));
                auto tested_vars =
                    _tested.add_variables(count, _to_params<Tested>(spec));
                auto reference_vars =
                    _reference.add_variables(count, _to_params<dumb_lp>(spec));
                for(auto && [tested_var, reference_var, label] :
                    std::views::zip(tested_vars, reference_vars, new_labels))
                    _variables.emplace_back(tested_var, reference_var, label,
                                            false);
            });
        if constexpr(has_named_variables<Tested>) {
            register_mutation(
                "add named variable", [] { return true; },
                [this] {
                    const params_spec spec = _rand_params();
                    const std::string label = _new_variable_label();
                    const std::string name =
                        std::format("v{}", _name_counter++);
                    _log(std::format("{} = add_named_variable(\"{}\", {})",
                                     label, name, _params_str(spec)));
                    _variables.emplace_back(
                        _tested.add_named_variable(name,
                                                   _to_params<Tested>(spec)),
                        _reference.add_named_variable(
                            name, _to_params<dumb_lp>(spec)),
                        label, true);
                });
            register_mutation(
                "set variable name", [this] { return _has_variables(); },
                [this] {
                    variable_entry & entry =
                        _variables[_rand_index(_variables.size())];
                    const std::string name =
                        std::format("v{}", _name_counter++);
                    _log(std::format("set_variable_name({}, \"{}\")",
                                     entry.label, name));
                    _tested.set_variable_name(entry.tested, name);
                    _reference.set_variable_name(entry.reference, name);
                    entry.named = true;
                });
        }
        if constexpr(has_modifiable_variables_bounds<Tested>) {
            register_mutation(
                "set variable bounds", [this] { return _has_variables(); },
                [this] {
                    const variable_entry & entry =
                        _variables[_rand_index(_variables.size())];
                    const double lb = static_cast<double>(_rand_int(-5, 3));
                    const double ub = lb + static_cast<double>(_rand_int(0, 8));
                    _log(
                        std::format("set_variable_lower_bound({0}, {1}) + "
                                    "set_variable_upper_bound({0}, {2})",
                                    entry.label, lb, ub));
                    _tested.set_variable_lower_bound(entry.tested, lb);
                    _tested.set_variable_upper_bound(entry.tested, ub);
                    _reference.set_variable_lower_bound(entry.reference, lb);
                    _reference.set_variable_upper_bound(entry.reference, ub);
                });
        }
        register_mutation(
            "set objective", [] { return true; },
            [this] {
                const expr_spec spec = _rand_expr(6u);
                _log(std::format("set_objective({})", _expr_str(spec)));
                const auto tested_terms = _tested_terms(spec);
                const auto reference_terms = _reference_terms(spec);
                _tested.set_objective(
                    linear_expression_view(tested_terms, spec.constant));
                _reference.set_objective(
                    linear_expression_view(reference_terms, spec.constant));
            });
        register_mutation(
            "set objective offset", [] { return true; },
            [this] {
                const double offset = _rand_rhs();
                _log(std::format("set_objective_offset({})", offset));
                _tested.set_objective_offset(offset);
                _reference.set_objective_offset(offset);
            });
        if constexpr(has_modifiable_objective<Tested>) {
            register_mutation(
                "add objective", [] { return true; },
                [this] {
                    const expr_spec spec = _rand_expr(4u);
                    _log(std::format("add_objective({})", _expr_str(spec)));
                    const auto tested_terms = _tested_terms(spec);
                    const auto reference_terms = _reference_terms(spec);
                    _tested.add_objective(
                        linear_expression_view(tested_terms, spec.constant));
                    _reference.add_objective(
                        linear_expression_view(reference_terms, spec.constant));
                });
            register_mutation(
                "set objective coefficient",
                [this] { return _has_variables(); },
                [this] {
                    const variable_entry & entry =
                        _variables[_rand_index(_variables.size())];
                    const double coef = _rand_obj_coef();
                    _log(std::format("set_objective_coefficient({}, {})",
                                     entry.label, coef));
                    _tested.set_objective_coefficient(entry.tested, coef);
                    _reference.set_objective_coefficient(entry.reference, coef);
                });
        }
        register_mutation(
            "add constraint", [this] { return _has_variables(); },
            [this] {
                const constraint_spec spec = _rand_constraint();
                const std::string label = _new_constraint_label();
                _log(std::format("{} = add_constraint({})", label,
                                 _constraint_str(spec)));
                const auto tested_terms = _tested_terms(spec.lhs);
                const auto reference_terms = _reference_terms(spec.lhs);
                _constraints.emplace_back(
                    _tested.add_constraint(linear_constraint_view(
                        linear_expression_view(tested_terms, -spec.rhs),
                        spec.sense)),
                    _reference.add_constraint(linear_constraint_view(
                        linear_expression_view(reference_terms, -spec.rhs),
                        spec.sense)),
                    label);
            });
        register_mutation(
            "add constraints", [this] { return _has_variables(); },
            [this] {
                const int count = _rand_int(1, 3);
                std::vector<constraint_spec> specs;
                std::vector<std::vector<std::pair<tested_variable, double>>>
                    tested_terms;
                std::vector<std::vector<std::pair<reference_variable, double>>>
                    reference_terms;
                std::vector<std::string> labels;
                std::string labels_str, specs_str;
                for(int i = 0; i < count; ++i) {
                    specs.emplace_back(_rand_constraint());
                    tested_terms.emplace_back(_tested_terms(specs.back().lhs));
                    reference_terms.emplace_back(
                        _reference_terms(specs.back().lhs));
                    labels.emplace_back(_new_constraint_label());
                    if(i > 0) {
                        labels_str += ", ";
                        specs_str += "; ";
                    }
                    labels_str += labels.back();
                    specs_str += _constraint_str(specs.back());
                }
                _log(std::format("{} = add_constraints({{{}}})", labels_str,
                                 specs_str));
                auto keys = std::views::iota(0, count);
                auto tested_constrs = _tested.add_constraints(keys, [&](int i) {
                    return linear_constraint_view(
                        linear_expression_view(
                            tested_terms[static_cast<std::size_t>(i)],
                            -specs[static_cast<std::size_t>(i)].rhs),
                        specs[static_cast<std::size_t>(i)].sense);
                });
                auto reference_constrs =
                    _reference.add_constraints(keys, [&](int i) {
                        return linear_constraint_view(
                            linear_expression_view(
                                reference_terms[static_cast<std::size_t>(i)],
                                -specs[static_cast<std::size_t>(i)].rhs),
                            specs[static_cast<std::size_t>(i)].sense);
                    });
                for(auto && [tested_constr, reference_constr, label] :
                    std::views::zip(tested_constrs, reference_constrs, labels))
                    _constraints.emplace_back(tested_constr, reference_constr,
                                              label);
            });
        if constexpr(has_add_column<Tested>) {
            register_mutation(
                "add column", [this] { return _has_constraints(); },
                [this] {
                    const params_spec spec = _rand_params();
                    // distinct constraints: some models overwrite instead of
                    // summing duplicated column entries
                    std::vector<std::size_t> indices(_constraints.size());
                    std::ranges::iota(indices, std::size_t{0});
                    std::ranges::shuffle(indices, _rng);
                    indices.resize(static_cast<std::size_t>(
                        _rand_int(1, static_cast<int>(std::min<std::size_t>(
                                         3u, indices.size())))));
                    std::vector<std::pair<tested_constraint, double>>
                        tested_entries;
                    std::vector<std::pair<reference_constraint, double>>
                        reference_entries;
                    std::string entries_str;
                    for(const std::size_t i : indices) {
                        const double coef = _rand_coef();
                        tested_entries.emplace_back(_constraints[i].tested,
                                                    coef);
                        reference_entries.emplace_back(
                            _constraints[i].reference, coef);
                        if(!entries_str.empty()) entries_str += ", ";
                        entries_str += std::format("{{{}, {}}}",
                                                   _constraints[i].label, coef);
                    }
                    const std::string label = _new_variable_label();
                    _log(std::format("{} = add_column({{{}}}, {})", label,
                                     entries_str, _params_str(spec)));
                    _variables.emplace_back(
                        _tested.add_column(tested_entries,
                                           _to_params<Tested>(spec)),
                        _reference.add_column(reference_entries,
                                              _to_params<dumb_lp>(spec)),
                        label, false);

                    // std::cout << "add_column -> "
                    //           << _variables.back().tested.id() << " : "
                    //           << _variables.back().reference.id() <<
                    //           std::endl;
                });
        }
        if constexpr(has_remove_variable<Tested>) {
            register_mutation(
                "remove variable", [this] { return _has_variables(); },
                [this] {
                    const std::size_t index = _rand_index(_variables.size());
                    _log(std::format("remove_variable({})",
                                     _variables[index].label));
                    _tested.remove_variable(_variables[index].tested);
                    _reference.remove_variable(_variables[index].reference);
                    _variables.erase(_variables.begin() +
                                     static_cast<std::ptrdiff_t>(index));
                });
        }
        if constexpr(has_modifiable_constraint_rhs<Tested>) {
            register_mutation(
                "set constraint rhs", [this] { return _has_constraints(); },
                [this] {
                    const constraint_entry & entry =
                        _constraints[_rand_index(_constraints.size())];
                    const double rhs = _rand_rhs();
                    _log(std::format("set_constraint_rhs({}, {})", entry.label,
                                     rhs));
                    _tested.set_constraint_rhs(entry.tested, rhs);
                    _reference.set_constraint_rhs(entry.reference, rhs);
                });
        }
        if constexpr(has_modifiable_constraint_sense<Tested>) {
            register_mutation(
                "set constraint sense", [this] { return _has_constraints(); },
                [this] {
                    const constraint_entry & entry =
                        _constraints[_rand_index(_constraints.size())];
                    const constraint_sense sense = _rand_sense();
                    _log(std::format("set_constraint_sense({}, {})",
                                     entry.label, _sense_str(sense)));
                    _tested.set_constraint_sense(entry.tested, sense);
                    _reference.set_constraint_sense(entry.reference, sense);
                });
        }
        if constexpr(has_modifiable_constraint_lhs<Tested>) {
            register_mutation(
                "set constraint lhs",
                [this] { return _has_variables() && _has_constraints(); },
                [this] {
                    const constraint_entry & entry =
                        _constraints[_rand_index(_constraints.size())];
                    const constraint_spec spec = _rand_constraint();
                    _log(std::format("set_constraint_lhs({}, {})", entry.label,
                                     _expr_str(spec.lhs)));
                    const auto tested_terms = _tested_terms(spec.lhs);
                    const auto reference_terms = _reference_terms(spec.lhs);
                    _tested.set_constraint_lhs(entry.tested, tested_terms);
                    _reference.set_constraint_lhs(entry.reference,
                                                  reference_terms);
                });
        }
        if constexpr(has_lp_status<Tested>) {
            register_mutation(
                "solve and compare solutions",
                [this] { return _has_variables(); },  //
                [this] { _solve_and_compare(); });
        }
    }

    template <typename M>
    static std::string _status_str(M & model) {
        if(model.proven_optimal())
            return std::format("optimal (value={})",
                               model.get_solution_value());
        if(model.proven_infeasible()) return "infeasible";
        if(model.proven_unbounded()) return "unbounded";
        return "unknown";
    }

    void _solve_and_compare() {
        _log("solve()");
        _tested.solve();
        _reference.solve();
        _log(std::format("    -> tested: {}, reference: {}",
                         _status_str(_tested), _status_str(_reference)));
        EXPECT_EQ(_tested.proven_optimal(), _reference.proven_optimal());
        // infeasible vs unbounded is not compared strictly: solvers and
        // presolves may not always distinguish the two
        if(!_tested.proven_optimal() || !_reference.proven_optimal()) return;
        const double reference_value = _reference.get_solution_value();
        const double value_tolerance =
            solution_tolerance * (1.0 + std::fabs(reference_value));
        EXPECT_NEAR(_tested.get_solution_value(), reference_value,
                    value_tolerance);
        // the primal solutions themselves may differ (multiple optima), but
        // the tested one must be feasible for the reference model's data
        auto tested_solution = _tested.get_solution();
        std::vector<double> values;
        for(const variable_entry & entry : _variables)
            values.emplace_back(tested_solution[entry.tested]);
        std::map<int, std::size_t> reference_index =
            _reference_variable_index();
        for(std::size_t i = 0; i < _variables.size(); ++i) {
            EXPECT_GE(values[i] + solution_tolerance,
                      _clamp_infinite(_reference.get_variable_lower_bound(
                          _variables[i].reference)))
                << "lower bound of " << _variables[i].label;
            EXPECT_LE(values[i] - solution_tolerance,
                      _clamp_infinite(_reference.get_variable_upper_bound(
                          _variables[i].reference)))
                << "upper bound of " << _variables[i].label;
        }
        for(std::size_t i = 0; i < _constraints.size(); ++i) {
            const reference_constraint constr = _constraints[i].reference;
            double lhs = 0.0;
            for(const auto & [var, coef] :
                _reference.get_constraint_lhs(constr))
                lhs += coef * values[reference_index.at(var.id())];
            const double rhs = _reference.get_constraint_rhs(constr);
            const double tolerance =
                solution_tolerance * (1.0 + std::fabs(rhs));
            switch(_reference.get_constraint_sense(constr)) {
                case constraint_sense::equal:
                    EXPECT_NEAR(lhs, rhs, tolerance)
                        << "constraint " << _constraints[i].label
                        << " violated";
                    break;
                case constraint_sense::less_equal:
                    EXPECT_LE(lhs - tolerance, rhs)
                        << "constraint " << _constraints[i].label
                        << " violated";
                    break;
                case constraint_sense::greater_equal:
                    EXPECT_GE(lhs + tolerance, rhs)
                        << "constraint " << _constraints[i].label
                        << " violated";
                    break;
            }
        }
    }

    void _register_default_checks() {
        register_check(
            "test matching num variables", [] { return true; },
            [this] {
                EXPECT_EQ(_tested.num_variables(), _variables.size());
                EXPECT_EQ(_reference.num_variables(), _variables.size());
            });
        register_check(
            "test matching num constraints", [] { return true; },
            [this] {
                EXPECT_EQ(_tested.num_constraints(), _constraints.size());
                EXPECT_EQ(_reference.num_constraints(), _constraints.size());
            });
        if constexpr(has_readable_objective<Tested>) {
            register_check(
                "test matching objective offset", [] { return true; },
                [this] {
                    EXPECT_NEAR(_tested.get_objective_offset(),
                                _reference.get_objective_offset(),
                                data_tolerance);
                });
            register_check(
                "test matching objective coefficients", [] { return true; },
                [this] {
                    for(std::size_t i = 0; i < _variables.size(); ++i)
                        EXPECT_NEAR(_tested.get_objective_coefficient(
                                        _variables[i].tested),
                                    _reference.get_objective_coefficient(
                                        _variables[i].reference),
                                    data_tolerance)
                            << "objective coefficient of "
                            << _variables[i].label;
                });
            register_check(
                "test matching get objective", [] { return true; },
                [this] {
                    auto tested_objective = _tested.get_objective();
                    auto reference_objective = _reference.get_objective();
                    _expect_matching_terms(tested_objective.linear_terms(),
                                           reference_objective.linear_terms());
                    EXPECT_NEAR(tested_objective.constant(),
                                reference_objective.constant(), data_tolerance);
                });
        }
        if constexpr(has_readable_variables_bounds<Tested>) {
            register_check(
                "test matching variables bounds", [] { return true; },
                [this] {
                    for(std::size_t i = 0; i < _variables.size(); ++i) {
                        SCOPED_TRACE(
                            std::format("bounds of {}", _variables[i].label));
                        _expect_matching_bound(
                            _tested.get_variable_lower_bound(
                                _variables[i].tested),
                            _reference.get_variable_lower_bound(
                                _variables[i].reference));
                        _expect_matching_bound(
                            _tested.get_variable_upper_bound(
                                _variables[i].tested),
                            _reference.get_variable_upper_bound(
                                _variables[i].reference));
                    }
                });
        }
        if constexpr(has_named_variables<Tested>) {
            register_check(
                "test matching variable names", [] { return true; },
                [this] {
                    for(std::size_t i = 0; i < _variables.size(); ++i) {
                        if(!_variables[i].named) continue;
                        EXPECT_EQ(
                            _tested.get_variable_name(_variables[i].tested),
                            _reference.get_variable_name(
                                _variables[i].reference))
                            << "name of " << _variables[i].label;
                    }
                });
        }
        if constexpr(has_readable_constraint_rhs<Tested>) {
            register_check(
                "test matching constraints rhs", [] { return true; },
                [this] {
                    for(std::size_t i = 0; i < _constraints.size(); ++i)
                        EXPECT_NEAR(
                            _tested.get_constraint_rhs(_constraints[i].tested),
                            _reference.get_constraint_rhs(
                                _constraints[i].reference),
                            data_tolerance)
                            << "rhs of " << _constraints[i].label;
                });
        }
        if constexpr(has_readable_constraint_sense<Tested>) {
            register_check(
                "test matching constraints sense", [] { return true; },
                [this] {
                    for(std::size_t i = 0; i < _constraints.size(); ++i)
                        EXPECT_EQ(_tested.get_constraint_sense(
                                      _constraints[i].tested),
                                  _reference.get_constraint_sense(
                                      _constraints[i].reference))
                            << "sense of " << _constraints[i].label;
                });
        }
        if constexpr(has_readable_constraint_lhs<Tested>) {
            register_check(
                "test matching constraints lhs", [] { return true; },
                [this] {
                    for(std::size_t i = 0; i < _constraints.size(); ++i) {
                        SCOPED_TRACE(
                            std::format("lhs of {}", _constraints[i].label));
                        _expect_matching_terms(
                            _tested.get_constraint_lhs(_constraints[i].tested),
                            _reference.get_constraint_lhs(
                                _constraints[i].reference));
                    }
                });
        }
        if constexpr(has_readable_constraints<Tested>) {
            register_check(
                "test matching get constraint", [] { return true; },
                [this] {
                    for(std::size_t i = 0; i < _constraints.size(); ++i) {
                        SCOPED_TRACE(std::format("constraint {}",
                                                 _constraints[i].label));
                        auto tested_constr =
                            _tested.get_constraint(_constraints[i].tested);
                        auto reference_constr = _reference.get_constraint(
                            _constraints[i].reference);
                        _expect_matching_terms(tested_constr.linear_terms(),
                                               reference_constr.linear_terms());
                        EXPECT_EQ(tested_constr.sense(),
                                  reference_constr.sense());
                        EXPECT_NEAR(tested_constr.rhs(), reference_constr.rhs(),
                                    data_tolerance);
                    }
                });
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Main loop
    ////////////////////////////////////////////////////////////////////////////

public:
    void run(std::size_t num_rounds,
             std::size_t max_mutations_per_round = 16u) {
        SCOPED_TRACE(std::format("fuzzy run with MIPPP_FUZZ_SEED={}", _seed));
        std::size_t step = 0;
        std::vector<std::size_t> applicable;
        std::vector<std::size_t> check_order(_check_events.size());
        std::ranges::iota(check_order, std::size_t{0});
        for(std::size_t round = 0; round < num_rounds; ++round) {
            const int num_mutations =
                _rand_int(1, static_cast<int>(max_mutations_per_round));
            for(int i = 0; i < num_mutations; ++i) {
                applicable.clear();
                for(std::size_t e = 0; e < _mutation_events.size(); ++e)
                    if(_mutation_events[e].applicable())
                        applicable.emplace_back(e);
                if(applicable.empty()) break;
                const event & ev = _mutation_events[applicable[_rand_index(
                    applicable.size())]];
                SCOPED_TRACE(std::format("round {} step {} mutation '{}'",
                                         round, step, ev.name));
                ev.action();
                ++step;
                if(::testing::Test::HasFailure()) {
                    _print_trace();
                    return;
                }
            }
            // periodic getter checks, in randomized order
            _log("-- CROSS CHECK --");
            std::ranges::shuffle(check_order, _rng);
            for(const std::size_t e : check_order) {
                const event & ev = _check_events[e];
                if(!ev.applicable()) continue;
                SCOPED_TRACE(
                    std::format("round {} check '{}'", round, ev.name));
                ev.action();
                if(::testing::Test::HasFailure()) {
                    _log(std::format("FAILED check '{}'", ev.name));
                    _print_trace();
                    return;
                }
            }
        }
    }
};

////////////////////////////////////////////////////////////////////////////////
// Test suite
////////////////////////////////////////////////////////////////////////////////

namespace fuzzy_detail {
inline unsigned long env_or(const char * name, unsigned long default_value) {
    if(const char * str = std::getenv(name))
        return std::strtoul(str, nullptr, 10);
    return default_value;
}
}  // namespace fuzzy_detail

template <typename T>
struct LpFuzzyTest : public T {
    using typename T::model_type;
    static_assert(lp_model<model_type>);
};
TYPED_TEST_SUITE_P(LpFuzzyTest);

TYPED_TEST_P(LpFuzzyTest, random_operations) {
    this->SkipOnLicenseError([this]() {
        std::optional<clp_api> reference_api;
        try {
            reference_api.emplace();
        } catch(const std::exception & e) {
            GTEST_SKIP() << "the dumb_lp reference model requires Clp: "
                         << e.what();
        }
        const unsigned long seed =
            fuzzy_detail::env_or("MIPPP_FUZZ_SEED", std::random_device{}());
        const std::size_t num_rounds =
            fuzzy_detail::env_or("MIPPP_FUZZ_ROUNDS", 40ul);
        ::testing::Test::RecordProperty("fuzz_seed", std::to_string(seed));
        std::cout << "[   FUZZ   ] MIPPP_FUZZ_SEED=" << seed
                  << " MIPPP_FUZZ_ROUNDS=" << num_rounds << std::endl;
        lp_fuzzy_state_machine<typename TestFixture::model_type> machine(
            this->new_model(), dumb_lp(reference_api.value()), seed);
        machine.run(num_rounds);
    });
}

REGISTER_TYPED_TEST_SUITE_P(LpFuzzyTest, random_operations);

}  // namespace fhamonic::mippp
