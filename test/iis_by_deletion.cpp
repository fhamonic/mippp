#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <ostream>
#include <ranges>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/iis_arithmetic.hpp"
#include "mippp/detail/invoke_key.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/quadratic_expression.hpp"
#include "mippp/solvers/model_base.hpp"
#include "mippp/utility/entity_range.hpp"
#include "mippp/utility/iis_by_deletion.hpp"
#include "mippp/utility/iis_outcome.hpp"
#include "mippp/utility/status.hpp"
#include "mippp/utility/variant.hpp"

#include "assert_helper.hpp"
#include "iis_outcome_assert.hpp"
#include "test_suites/iis_oracle.hpp"

using namespace mippp;
using namespace std::chrono_literals;

namespace {

using seconds = std::chrono::duration<double>;
using membership = iis_oracle::membership;

using iis_stub_status = std::variant<
    status::unknown, status::optimal, status::optimal_face_unbounded,
    status::optimal_infeasible_unscaled, status::infeasible_or_unbounded,
    status::infeasible, status::primal_and_dual_infeasible, status::unbounded,
    status::failed, status::numerical_failure, status::interrupted,
    status::limit_reached, status::time_limit>;

struct iis_stub_failure : std::runtime_error {
    using runtime_error::runtime_error;
};

enum class stub_side { variable_lower, variable_upper, row_lower, row_upper };

struct stub_write {
    stub_side kind;
    int id;
    double value;
    friend bool operator==(const stub_write &, const stub_write &) = default;
};
void PrintTo(const stub_write & w, std::ostream * os) {
    *os << "{side " << static_cast<int>(w.kind) << ", id " << w.id << ", "
        << w.value << "}";
}

// what a trial saw, recorded by solve()
struct stub_data {
    std::vector<std::pair<double, double>> variable_bounds;  // removed included
    std::vector<std::pair<double, double>> row_bounds;
    std::vector<std::map<int, double>> row_terms;
    std::vector<double> objective;
    double offset = 0.0;
    bool maximize = false;
    std::optional<double> time_limit;  // nullopt on an untimed stub
    std::vector<std::tuple<int, int, double>> quadratic_terms;
    friend bool operator==(const stub_data &, const stub_data &) = default;
};
void PrintTo(const stub_data & d, std::ostream * os) {
    *os << "variables:";
    for(const auto & [lb, ub] : d.variable_bounds)
        *os << " [" << lb << ", " << ub << "]";
    *os << " rows:";
    for(std::size_t i = 0; i < d.row_bounds.size(); ++i) {
        *os << " [" << d.row_bounds[i].first << ", " << d.row_bounds[i].second
            << "]{";
        for(const auto & [id, coef] : d.row_terms[i])
            *os << " x" << id << ":" << coef;
        *os << " }";
    }
    *os << " objective:";
    for(const double c : d.objective) *os << " " << c;
    *os << " offset " << d.offset << (d.maximize ? " max" : " min");
    if(d.time_limit) *os << " time_limit " << *d.time_limit;
    *os << " quadratic terms " << d.quadratic_terms.size();
}

// A scripted lp_model: bounds and two-sided rows in plain arrays, a lazy
// objective view like the Clp one, and a solve() that either consumes a
// scripted status or decides by exact interval arithmetic on the models it
// accepts, so that the unscripted cases run a genuine monotone oracle.
template <bool Timed, bool Quadratic>
class iis_stub_model : protected model_base<int, double> {
private:
    using base = model_base<int, double>;

    struct column {
        double lower;
        double upper;
        double obj_coef;
        bool removed;
    };
    struct row {
        std::map<int, double> terms;
        double lower;
        double upper;
    };

    std::vector<column> _cols;
    std::vector<row> _rows;
    bool _maximize = false;
    double _offset = 0.0;
    std::vector<std::tuple<int, int, double>> _quadratic_terms;
    iis_stub_status _status = status::unknown{};

public:
    using base::default_variable_params;
    using base::is_infinite;
    // finite, so that a side at the wrong infinity or beyond the threshold is
    // expressible with plain doubles
    double infinity() const noexcept { return 1e20; }

    // recording and scripting, public since it is a test double
    std::deque<std::optional<iis_stub_status>> script;
    std::size_t solves = 0;
    std::vector<stub_data> trials;
    std::vector<stub_write> side_writes;
    std::size_t objective_writes = 0;
    std::size_t offset_writes = 0;
    std::size_t variables_calls = 0;
    std::vector<double> time_limit_writes;
    std::size_t time_limit_reads = 0;
    double time_limit_value = std::numeric_limits<double>::infinity();
    std::optional<std::pair<stub_side, int>> failing_side;
    std::size_t failed_writes = 0;
    std::function<void(iis_stub_model &, std::size_t)> on_solve;

    void reset_recording() {
        solves = 0;
        trials.clear();
        side_writes.clear();
        objective_writes = 0;
        offset_writes = 0;
        variables_calls = 0;
        time_limit_writes.clear();
        time_limit_reads = 0;
        failed_writes = 0;
    }
    // the probe snapshots the built model here; nothing to do on the stub
    void record_construction() {}

    std::size_t num_variables() {
        return static_cast<std::size_t>(std::ranges::count_if(
            _cols, [](const column & col) { return !col.removed; }));
    }
    std::size_t num_constraints() { return _rows.size(); }
    std::vector<variable> variables() {
        ++variables_calls;
        std::vector<variable> live;
        for(std::size_t i = 0; i < _cols.size(); ++i)
            if(!_cols[i].removed) live.emplace_back(static_cast<int>(i));
        return live;
    }
    using base::constraints;

    void set_maximization() { _maximize = true; }
    void set_minimization() { _maximize = false; }
    bool is_maximization() const noexcept { return _maximize; }

    ////////////////////////////////////////////////////////////////////////////
    // Objective
    ////////////////////////////////////////////////////////////////////////////

    void set_objective_offset(double constant) {
        ++offset_writes;
        _offset = constant;
    }
    double get_objective_offset() { return _offset; }
    // replaces the whole objective, the quadratic part included, as highs_qp
    // does
    void set_objective(linear_expression auto && le) {
        ++objective_writes;
        for(column & col : _cols) col.obj_coef = 0.0;
        for(auto && [var, coef] : le.linear_terms())
            _cols[var.uid()].obj_coef += coef;
        set_objective_offset(le.constant());
        _quadratic_terms.clear();
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        set_objective(std::forward<LE>(le));
    }
    double get_objective_coefficient(variable v) {
        return _cols[v.uid()].obj_coef;
    }
    auto get_objective() {
        return linear_expression_view(
            std::views::transform(
                std::views::filter(
                    std::views::iota(0, static_cast<int>(_cols.size())),
                    [this](int i) {
                        return !_cols[static_cast<std::size_t>(i)].removed;
                    }),
                [this](int i) {
                    return std::make_pair(
                        variable(i),
                        _cols[static_cast<std::size_t>(i)].obj_coef);
                }),
            _offset);
    }

    template <quadratic_expression QE>
    void set_quadratic_objective(QE && qe)
        requires Quadratic
    {
        set_objective(qe.linear_part());
        for(auto && [v1, v2, coef] : qe.quadratic_terms())
            _quadratic_terms.emplace_back(v1.id(), v2.id(), coef);
    }
    template <quadratic_expression QE>
    void set_quadratic_objective(distinct_variables_t, QE && qe)
        requires Quadratic
    {
        set_quadratic_objective(std::forward<QE>(qe));
    }
    auto get_quadratic_objective()
        requires Quadratic
    {
        return quadratic_expression_view(
            std::views::transform(
                _quadratic_terms,
                [](const std::tuple<int, int, double> & t) {
                    return std::tuple(variable(std::get<0>(t)),
                                      variable(std::get<1>(t)), std::get<2>(t));
                }),
            get_objective());
    }

    ////////////////////////////////////////////////////////////////////////////
    // Variables
    ////////////////////////////////////////////////////////////////////////////

private:
    std::size_t _new_variables(std::size_t count,
                               const variable_params & params, variable_kind) {
        const std::size_t offset = _cols.size();
        for(std::size_t i = 0; i < count; ++i)
            _cols.push_back(
                column{.lower = params.lower_bound.value_or(-infinity()),
                       .upper = params.upper_bound.value_or(infinity()),
                       .obj_coef = params.obj_coef,
                       .removed = false});
        return offset;
    }

    void _record(stub_side kind, int id, double value) {
        if(failing_side && failing_side->first == kind &&
           failing_side->second == id) {
            ++failed_writes;
            throw iis_stub_failure("failing side");
        }
        side_writes.push_back({kind, id, value});
    }

public:
    friend base;
    using base::add_variable;
    using base::add_variables;

    // ids are never recycled, so a handle created after a removal has a
    // fresh id
    void remove_variable(variable v) {
        _cols[v.uid()] = column{
            .lower = 0.0, .upper = 0.0, .obj_coef = 0.0, .removed = true};
        for(row & r : _rows) r.terms.erase(v.id());
    }

    void set_variable_lower_bound(variable v, double lb) {
        _record(stub_side::variable_lower, v.id(), lb);
        _cols[v.uid()].lower = lb;
    }
    void set_variable_upper_bound(variable v, double ub) {
        _record(stub_side::variable_upper, v.id(), ub);
        _cols[v.uid()].upper = ub;
    }
    double get_variable_lower_bound(variable v) { return _cols[v.uid()].lower; }
    double get_variable_upper_bound(variable v) { return _cols[v.uid()].upper; }

    ////////////////////////////////////////////////////////////////////////////
    // Constraints
    ////////////////////////////////////////////////////////////////////////////

private:
    template <typename Terms>
    static std::map<int, double> _terms_of(Terms && terms) {
        std::map<int, double> merged;
        for(auto && [var, coef] : terms) merged[var.id()] += coef;
        // a zero coefficient is not a term: the default rule must see a
        // term-less row as such
        std::erase_if(merged, [](const auto & e) { return e.second == 0.0; });
        return merged;
    }
    constraint _add_row(std::map<int, double> terms, double lower,
                        double upper) {
        _rows.push_back(row{std::move(terms), lower, upper});
        return constraint(static_cast<int>(_rows.size() - 1));
    }

public:
    constraint add_constraint(linear_constraint auto && lc) {
        const double rhs = lc.rhs();
        auto terms = _terms_of(lc.linear_terms());
        switch(lc.sense()) {
            case constraint_sense::less_equal:
                return _add_row(std::move(terms), -infinity(), rhs);
            case constraint_sense::greater_equal:
                return _add_row(std::move(terms), rhs, infinity());
            case constraint_sense::equal:
                break;
        }
        return _add_row(std::move(terms), rhs, rhs);
    }
    template <linear_constraint LC>
    constraint add_constraint(distinct_variables_t, LC && lc) {
        return add_constraint(std::forward<LC>(lc));
    }
    template <std::ranges::range IR, typename CL>
    auto add_constraints(IR && keys, CL constraint_lambda) {
        const std::size_t offset = _rows.size();
        for(auto && key : keys)
            add_constraint(detail::invoke_key(constraint_lambda, key));
        return entity_range(constraint(static_cast<int>(offset)),
                            _rows.size() - offset);
    }
    template <std::ranges::range IR, typename CL>
    auto add_constraints(distinct_variables_t, IR && keys,
                         CL constraint_lambda) {
        return add_constraints(std::forward<IR>(keys),
                               std::move(constraint_lambda));
    }
    template <linear_expression LE>
    constraint add_ranged_constraint(LE && le, double lb, double ub) {
        auto terms = _terms_of(le.linear_terms());
        const double constant = le.constant();
        return _add_row(std::move(terms), lb - constant, ub - constant);
    }
    template <linear_expression LE>
    constraint add_ranged_constraint(distinct_variables_t, LE && le, double lb,
                                     double ub) {
        return add_ranged_constraint(std::forward<LE>(le), lb, ub);
    }

    void set_constraint_lower_bound(constraint c, double lb) {
        _record(stub_side::row_lower, c.id(), lb);
        _rows[c.uid()].lower = lb;
    }
    void set_constraint_upper_bound(constraint c, double ub) {
        _record(stub_side::row_upper, c.id(), ub);
        _rows[c.uid()].upper = ub;
    }
    double get_constraint_lower_bound(constraint c) {
        return _rows[c.uid()].lower;
    }
    double get_constraint_upper_bound(constraint c) {
        return _rows[c.uid()].upper;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Time limit
    ////////////////////////////////////////////////////////////////////////////

    void set_time_limit(std::chrono::duration<double> t)
        requires Timed
    {
        time_limit_value = t.count();
        time_limit_writes.push_back(t.count());
    }
    std::chrono::duration<double> get_time_limit()
        requires Timed
    {
        ++time_limit_reads;
        return std::chrono::duration<double>(time_limit_value);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Solve
    ////////////////////////////////////////////////////////////////////////////

private:
    // exact interval arithmetic over bounds and single-variable unit rows,
    // anything else must be scripted
    iis_stub_status _default_verdict() const {
        std::vector<std::pair<double, double>> box;
        for(const column & col : _cols) box.emplace_back(col.lower, col.upper);
        for(const row & r : _rows) {
            if(r.terms.empty()) {
                if(!(r.lower <= 0.0 && 0.0 <= r.upper))
                    return status::infeasible{};
                continue;
            }
            const auto & [id, coef] = *r.terms.begin();
            const auto j = static_cast<std::size_t>(id);
            if(r.terms.size() != 1 || coef != 1.0 || _cols[j].removed)
                throw std::logic_error("iis_stub_model: script this model");
            box[j].first = std::max(box[j].first, r.lower);
            box[j].second = std::min(box[j].second, r.upper);
        }
        for(std::size_t j = 0; j < _cols.size(); ++j) {
            if(_cols[j].removed) continue;
            const auto [lb, ub] = box[j];
            // a value at or beyond the threshold is not one the column can
            // take, so x >= +infinity() is infeasible even with a free upper
            if(!(lb <= ub && lb < infinity() && ub > -infinity()))
                return status::infeasible{};
        }
        return status::optimal{};
    }

public:
    iis_stub_status get_status() const { return _status; }
    void reset_status() noexcept { _status = status::unknown{}; }
    void solve() {
        const std::size_t i = solves++;
        trials.push_back(data());
        if(on_solve) on_solve(*this, i);
        if(!script.empty()) {
            const auto s = script.front();
            script.pop_front();
            if(!s) throw iis_stub_failure("scripted failure");
            _status = *s;
            return;
        }
        _status = _default_verdict();
    }
    double get_solution_value() { return 0.0; }
    auto get_solution() {
        return variable_mapping(std::vector<double>(_cols.size(), 0.0));
    }

    ////////////////////////////////////////////////////////////////////////////
    // Inspection
    ////////////////////////////////////////////////////////////////////////////

    std::vector<double> objective_coefficients() const {
        std::vector<double> coefs;
        for(const column & col : _cols) coefs.push_back(col.obj_coef);
        return coefs;
    }
    std::vector<std::pair<double, double>> live_variable_bounds() const {
        std::vector<std::pair<double, double>> bounds;
        for(const column & col : _cols)
            if(!col.removed) bounds.emplace_back(col.lower, col.upper);
        return bounds;
    }
    std::vector<std::pair<double, double>> row_bounds() const {
        std::vector<std::pair<double, double>> bounds;
        for(const row & r : _rows) bounds.emplace_back(r.lower, r.upper);
        return bounds;
    }
    std::vector<std::map<int, double>> row_terms() const {
        std::vector<std::map<int, double>> terms;
        for(const row & r : _rows) terms.push_back(r.terms);
        return terms;
    }
    const std::vector<std::tuple<int, int, double>> & quadratic_terms_storage()
        const noexcept {
        return _quadratic_terms;
    }
    double current_time_limit() const noexcept { return time_limit_value; }

    stub_data data() const {
        stub_data d;
        for(const column & col : _cols)
            d.variable_bounds.emplace_back(col.lower, col.upper);
        d.row_bounds = row_bounds();
        d.row_terms = row_terms();
        d.objective = objective_coefficients();
        d.offset = _offset;
        d.maximize = _maximize;
        if constexpr(Timed) d.time_limit = time_limit_value;
        d.quadratic_terms = _quadratic_terms;
        return d;
    }
};

// Deduced as the model of the run, so its solve() runs in every trial and
// checks what the trial sees.
template <bool Timed, bool Quadratic>
struct iis_stub_probe : iis_stub_model<Timed, Quadratic> {
    using base = iis_stub_model<Timed, Quadratic>;

    std::optional<double> caller_time_limit;
    std::optional<std::vector<std::map<int, double>>> row_terms_at_construction;
    bool maximize_at_construction = false;

    void record_construction() {
        row_terms_at_construction = this->row_terms();
        maximize_at_construction = this->is_maximization();
    }

    void solve() {
        if(!row_terms_at_construction) record_construction();
        for(const double c : this->objective_coefficients()) EXPECT_EQ(c, 0.0);
        EXPECT_EQ(this->get_objective_offset(), 0.0);
        EXPECT_EQ(this->is_maximization(), maximize_at_construction);
        if constexpr(Quadratic) {
            EXPECT_TRUE(this->quadratic_terms_storage().empty());
        }
        // the solver never receives a crossed pair
        for(const auto & [lb, ub] : this->live_variable_bounds())
            EXPECT_LE(lb, ub);
        for(const auto & [lo, hi] : this->row_bounds()) EXPECT_LE(lo, hi);
        EXPECT_EQ(this->row_terms(), *row_terms_at_construction);
        if constexpr(Timed) {
            if(caller_time_limit && !std::isnan(*caller_time_limit)) {
                EXPECT_LE(this->current_time_limit(), *caller_time_limit);
            }
        }
        base::solve();
    }
    // once the model is built, the run must not reach the sense
    void set_minimization() {
        if(row_terms_at_construction) ADD_FAILURE() << "sense touched";
        base::set_minimization();
    }
    void set_maximization() {
        if(row_terms_at_construction) ADD_FAILURE() << "sense touched";
        base::set_maximization();
    }
};

struct stub_without_reset : iis_stub_model<false, false> {
    void reset_status() noexcept = delete;
};
struct stub_without_row_setters : iis_stub_model<false, false> {
    void set_constraint_lower_bound(constraint, double) = delete;
};
struct qp_stub_without_readable_quadratic : iis_stub_model<false, true> {
    void get_quadratic_objective() = delete;
};

// Rows held as a sense and an rhs, as Gurobi holds them: no row-bound setter
// and no ranged row. The sides stay in the base's arrays, which the trials
// and the default rule read, and row_writes records each call of the two
// setters with the row's state after it. The sense is held apart, as Gurobi
// holds it: read back from the sides, a >= row whose rhs is still infinite
// between two writes would read as ==.
struct sense_rhs_stub : iis_stub_model<false, false> {
    using base = iis_stub_model<false, false>;

    enum class row_setter { sense, rhs };
    struct row_write {
        int id;
        row_setter setter;
        constraint_sense sense;
        double rhs;
        friend bool operator==(const row_write &, const row_write &) = default;
    };
    std::vector<row_write> row_writes;

    void set_constraint_lower_bound(constraint, double) = delete;
    void set_constraint_upper_bound(constraint, double) = delete;
    template <typename... Args>
    constraint add_ranged_constraint(Args &&...) = delete;

    // a row never written has the finite rhs it was built with
    constraint_sense get_constraint_sense(constraint c) {
        if(const auto it = _written.find(c.id()); it != _written.end())
            return it->second.first;
        const double lb = get_constraint_lower_bound(c);
        if(lb == get_constraint_upper_bound(c)) return constraint_sense::equal;
        return lb > -infinity() ? constraint_sense::greater_equal
                                : constraint_sense::less_equal;
    }
    double get_constraint_rhs(constraint c) {
        if(const auto it = _written.find(c.id()); it != _written.end())
            return it->second.second;
        return get_constraint_sense(c) == constraint_sense::greater_equal
                   ? get_constraint_lower_bound(c)
                   : get_constraint_upper_bound(c);
    }
    void set_constraint_sense(constraint c, constraint_sense sense) {
        _write(c, row_setter::sense, sense, get_constraint_rhs(c));
    }
    void set_constraint_rhs(constraint c, double rhs) {
        _write(c, row_setter::rhs, get_constraint_sense(c), rhs);
    }

    [[nodiscard]] bool sense_written(constraint c) const {
        return std::ranges::any_of(row_writes, [&](const row_write & w) {
            return w.id == c.id() && w.setter == row_setter::sense;
        });
    }

private:
    std::map<int, std::pair<constraint_sense, double>> _written;

    void _write(constraint c, row_setter setter, constraint_sense sense,
                double rhs) {
        row_writes.push_back({c.id(), setter, sense, rhs});
        base::set_constraint_lower_bound(
            c, sense == constraint_sense::less_equal ? -infinity() : rhs);
        base::set_constraint_upper_bound(
            c, sense == constraint_sense::greater_equal ? infinity() : rhs);
        _written[c.id()] = {sense, rhs};
    }
};
void PrintTo(const sense_rhs_stub::row_write & w, std::ostream * os) {
    *os << "{row " << w.id << ", "
        << (w.setter == sense_rhs_stub::row_setter::sense ? "sense" : "rhs")
        << " setter, sense " << static_cast<int>(w.sense) << ", " << w.rhs
        << "}";
}

// a ranged row would need two distinct sides, which no sense and rhs express
struct ranged_stub_with_sense_and_rhs : iis_stub_model<false, false> {
    void set_constraint_lower_bound(constraint, double) = delete;
    void set_constraint_upper_bound(constraint, double) = delete;
    constraint_sense get_constraint_sense(constraint) {
        return constraint_sense::equal;
    }
    void set_constraint_sense(constraint, constraint_sense) {}
    void set_constraint_rhs(constraint, double) {}
};
// the writer reads the sense, to write it only when it changes
struct sense_rhs_stub_without_readable_sense : sense_rhs_stub {
    void get_constraint_sense(constraint) = delete;
};
// HiGHS and CPLEX: a sense and an rhs beside the row-bound setters, which
// range a row, so the filter writes the sides
struct bounds_stub_with_sense_and_rhs : iis_stub_model<false, false> {
    template <typename... Args>
    constraint add_ranged_constraint(Args &&...) = delete;
    constraint_sense get_constraint_sense(constraint) {
        return constraint_sense::equal;
    }
    void set_constraint_sense(constraint, constraint_sense) {}
    void set_constraint_rhs(constraint, double) {}
};

}  // namespace

///////////////////////////////////////////////////////////////////////////////
//////////////////////////// Compile-time checks //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static_assert(iis_by_deletion_model<iis_stub_model<false, false>>);
static_assert(iis_by_deletion_model<iis_stub_model<true, false>>);
static_assert(iis_by_deletion_model<iis_stub_model<false, true>>);
static_assert(iis_by_deletion_model<iis_stub_model<true, true>>);
static_assert(iis_by_deletion_model<iis_stub_probe<false, false>>);
static_assert(iis_by_deletion_model<iis_stub_probe<true, false>>);
static_assert(iis_by_deletion_model<iis_stub_probe<true, true>>);
static_assert(!iis_by_deletion_model<stub_without_reset>);
static_assert(!iis_by_deletion_model<stub_without_row_setters>);
static_assert(iis_by_deletion_model<sense_rhs_stub>);
static_assert(!has_modifiable_constraint_bounds<sense_rhs_stub> &&
              !has_ranged_constraints<sense_rhs_stub> &&
              has_readable_constraint_bounds<sense_rhs_stub>);
static_assert(detail::iis_rows_as_sense_and_rhs<sense_rhs_stub>);
static_assert(!iis_by_deletion_model<ranged_stub_with_sense_and_rhs> &&
              has_ranged_constraints<ranged_stub_with_sense_and_rhs> &&
              has_readable_constraint_sense<ranged_stub_with_sense_and_rhs> &&
              has_modifiable_constraint_sense<ranged_stub_with_sense_and_rhs>);
static_assert(
    !iis_by_deletion_model<sense_rhs_stub_without_readable_sense> &&
    has_modifiable_constraint_sense<sense_rhs_stub_without_readable_sense> &&
    has_modifiable_constraint_rhs<sense_rhs_stub_without_readable_sense>);
static_assert(
    !detail::iis_rows_as_sense_and_rhs<bounds_stub_with_sense_and_rhs> &&
    !has_ranged_constraints<bounds_stub_with_sense_and_rhs> &&
    iis_by_deletion_model<bounds_stub_with_sense_and_rhs>);
// the conditional requirement bites on a qp_model only
static_assert(qp_model<qp_stub_without_readable_quadratic>);
static_assert(!iis_by_deletion_model<qp_stub_without_readable_quadratic>);
static_assert(iis_by_deletion_model<iis_stub_model<false, true>> &&
              qp_model<iis_stub_model<false, true>>);
static_assert(!qp_model<iis_stub_model<false, false>>);
static_assert(has_time_limit<iis_stub_model<true, false>> &&
              !has_time_limit<iis_stub_model<false, false>>);
static_assert(has_ranged_constraints<iis_stub_model<false, false>>);
static_assert(!has_iis<iis_stub_model<false, false>>);
static_assert(std::same_as<decltype(compute_iis_by_deletion(
                               std::declval<iis_stub_model<false, false> &>())),
                           iis_by_deletion_t<iis_stub_model<false, false>>>);
static_assert(lp_iis<iis_by_deletion_t<iis_stub_model<false, false>>,
                     iis_stub_model<false, false>>);
static_assert(lp_iis<iis_by_deletion_t<iis_stub_probe<true, true>>,
                     iis_stub_probe<true, true>>);
static_assert(
    std::same_as<
        deletion_filter_outcome,
        std::variant<iis_outcome::incomplete, iis_outcome::irreducible,
                     iis_outcome::feasible, iis_outcome::inconclusive_trial,
                     iis_outcome::interrupted, iis_outcome::time_limit,
                     iis_outcome::solve_limit>>);
static_assert(lp_iis_outcome<deletion_filter_outcome>);
static_assert(std::same_as<decltype(std::declval<const iis_by_deletion_t<
                                        iis_stub_model<false, false>> &>()
                                        .get_outcome()),
                           deletion_filter_outcome>);
static_assert(
    lp_iis_status<detail::iis_sided_status> &&
    std::same_as<std::variant_alternative_t<0, detail::iis_sided_status>,
                 iis_status::absent>);

// the classifier table
static_assert(detail::classify_deletion_trial(iis_stub_status(
                  status::infeasible{})) == deletion_verdict::infeasible);
static_assert(detail::classify_deletion_trial(
                  iis_stub_status(status::primal_and_dual_infeasible{})) ==
              deletion_verdict::infeasible);
static_assert(detail::classify_deletion_trial(
                  iis_stub_status(status::infeasible_or_unbounded{})) ==
              deletion_verdict::infeasible);
static_assert(detail::classify_deletion_trial(
                  iis_stub_status(status::optimal_infeasible_unscaled{})) ==
              deletion_verdict::inconclusive);
static_assert(detail::classify_deletion_trial(iis_stub_status(
                  status::unbounded{})) == deletion_verdict::inconclusive);
static_assert(detail::classify_deletion_trial(iis_stub_status(status::failed{
                  false})) == deletion_verdict::inconclusive);
static_assert(detail::classify_deletion_trial(iis_stub_status(status::failed{
                  true})) == deletion_verdict::inconclusive);
static_assert(detail::classify_deletion_trial(
                  iis_stub_status(status::numerical_failure{true})) ==
              deletion_verdict::inconclusive);
static_assert(detail::classify_deletion_trial(iis_stub_status(
                  status::time_limit{true})) == deletion_verdict::feasible);
static_assert(detail::classify_deletion_trial(iis_stub_status(
                  status::interrupted{true})) == deletion_verdict::feasible);
static_assert(detail::classify_deletion_trial(iis_stub_status(
                  status::limit_reached{true})) == deletion_verdict::feasible);
static_assert(detail::classify_deletion_trial(iis_stub_status(
                  status::optimal{})) == deletion_verdict::feasible);
static_assert(detail::classify_deletion_trial(
                  iis_stub_status(status::optimal_face_unbounded{})) ==
              deletion_verdict::feasible);
static_assert(detail::classify_deletion_trial(
                  iis_stub_status(status::time_limit{false})) ==
              deletion_verdict::inconclusive);
static_assert(detail::classify_deletion_trial(
                  iis_stub_status(status::interrupted{false})) ==
              deletion_verdict::inconclusive);
static_assert(detail::classify_deletion_trial(
                  iis_stub_status(status::limit_reached{false})) ==
              deletion_verdict::inconclusive);
static_assert(detail::classify_deletion_trial(iis_stub_status(
                  status::unknown{})) == deletion_verdict::inconclusive);

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Fixture ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

struct iis_by_deletion : ::testing::Test {
    using stub = iis_stub_model<false, false>;
    using timed_stub = iis_stub_model<true, false>;
    using qp_stub = iis_stub_model<false, true>;
    using probe = iis_stub_probe<false, false>;
    using timed_probe = iis_stub_probe<true, false>;
    using variable = model_variable<int, double>;
    using constraint = model_constraint<int>;

    static constexpr double inf = 1e20;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    static constexpr double positive_infinity =
        std::numeric_limits<double>::infinity();

    struct conflict_handles {
        variable x, y;
        constraint r0, r1, r2;
    };
    struct crossed_handles {
        variable x, y;
        constraint r;
    };

    // x in [0, 10], y in [0, 1], x >= 12, x <= 20, y >= 0.5, objective
    // 3x - 2y + 7. Candidates: 0 x lower, 1 x upper, 2 y lower, 3 y upper,
    // 4 r0 lower, 5 r1 upper, 6 r2 lower; the only conflict is {1, 4}.
    template <typename M>
    static conflict_handles conflict_model(M & model, bool maximize = false) {
        using namespace operators;
        auto x = model.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
        auto y = model.add_variable({.lower_bound = 0.0, .upper_bound = 1.0});
        auto r0 = model.add_constraint(x >= 12);
        auto r1 = model.add_constraint(x <= 20);
        auto r2 = model.add_constraint(y >= 0.5);
        if(maximize)
            model.set_maximization();
        else
            model.set_minimization();
        model.set_objective(3 * x - 2 * y + 7);
        model.record_construction();
        model.reset_recording();
        return {x, y, r0, r1, r2};
    }
    // x in [3, 1], y in [0, 5], y >= 1. Candidates: 0 x lower, 1 x upper,
    // 2 y lower, 3 y upper, 4 row lower; the crossed pair is {0, 1}.
    template <typename M>
    static crossed_handles crossed_variable_model(M & model) {
        using namespace operators;
        auto x = model.add_variable({.lower_bound = 3.0, .upper_bound = 1.0});
        auto y = model.add_variable({.lower_bound = 0.0, .upper_bound = 5.0});
        auto r = model.add_constraint(y >= 1);
        model.record_construction();
        model.reset_recording();
        return {x, y, r};
    }
    template <typename M>
    static variable bounded_variable(M & model, double lower, double upper) {
        return model.add_variable({.lower_bound = lower, .upper_bound = upper});
    }
    template <typename M>
    static constraint add_row(M & model, double lower, double upper) {
        return model.add_ranged_constraint(
            empty_linear_expression<variable, double>, lower, upper);
    }

    template <typename Iis, typename Handle>
    static membership status_of(const Iis & iis, Handle handle) {
        return iis_oracle::membership_of(iis.get_status(handle));
    }
    template <typename M>
    static void expect_untouched(const M & model, const stub_data & before) {
        EXPECT_EQ(model.data(), before);
        EXPECT_TRUE(model.side_writes.empty());
        EXPECT_EQ(model.objective_writes, 0u);
        EXPECT_EQ(model.time_limit_reads, 0u);
    }
    template <typename M>
    static void expect_restored(const M & model, const stub_data & before) {
        EXPECT_EQ(model.data(), before);
    }
    // the base's solve(): a probe primed this way runs no trial check
    template <bool Timed, bool Quadratic>
    static void prime_optimal(iis_stub_model<Timed, Quadratic> & model) {
        model.script.push_back(status::optimal{});
        model.solve();
        model.reset_recording();
    }

    template <typename Tag>
    static void expect_scripted_single_variable(
        std::deque<std::optional<iis_stub_status>> script, Tag expected,
        membership x_status) {
        stub model;
        const auto x = bounded_variable(model, 0.0, 10.0);
        const std::size_t expected_solves = script.size();
        model.script = std::move(script);
        const auto iis = compute_iis_by_deletion(model);
        EXPECT_TRUE(
            outcome_is<Tag>(iis.get_outcome(), expected.conflict_available));
        EXPECT_EQ(model.solves, expected_solves);
        EXPECT_EQ(status_of(iis, x), x_status);
        EXPECT_EQ(iis.num_variable_members(),
                  x_status == membership::absent ? 0u : 1u);
        EXPECT_EQ(iis.num_constraint_members(), 0u);
        EXPECT_TRUE(is<status::unknown>(model.get_status()));
    }

    template <typename M>
    static void budget_sweep(seconds time_limit) {
        for(std::size_t b = 0; b < 10; ++b) {
            M model;
            const auto h = conflict_model(model);
            const auto before = model.data();
            const auto iis = compute_iis_by_deletion(
                model, iis_limits{.max_solves = b, .time_limit = time_limit});
            EXPECT_LE(model.solves, b);
            if(b == 0) {
                EXPECT_TRUE(outcome_is<iis_outcome::solve_limit>(
                    iis.get_outcome(), false));
                expect_untouched(model, before);
                continue;
            }
            expect_restored(model, before);
            EXPECT_TRUE(is<status::unknown>(model.get_status()));
            if(b < 8) {
                EXPECT_TRUE(outcome_is<iis_outcome::solve_limit>(
                    iis.get_outcome(), true));
                // the conflict is never dropped
                const auto x_status = status_of(iis, h.x);
                EXPECT_TRUE(x_status == membership::upper ||
                            x_status == membership::both)
                    << "b = " << b;
                const auto r0_status = status_of(iis, h.r0);
                EXPECT_TRUE(r0_status == membership::lower ||
                            r0_status == membership::both)
                    << "b = " << b;
                continue;
            }
            EXPECT_TRUE(
                outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
            EXPECT_EQ(model.solves, 8u);
            EXPECT_EQ(status_of(iis, h.x), membership::upper);
            EXPECT_EQ(status_of(iis, h.y), membership::absent);
            EXPECT_EQ(status_of(iis, h.r0), membership::lower);
            EXPECT_EQ(status_of(iis, h.r1), membership::absent);
            EXPECT_EQ(status_of(iis, h.r2), membership::absent);
        }
    }

    template <typename M>
    static void expect_column_less_answer(M & model, const stub_data & before,
                                          constraint r0, constraint r1,
                                          constraint r2,
                                          const iis_limits & limits) {
        const auto iis = compute_iis_by_deletion(model, limits);
        EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        EXPECT_EQ(status_of(iis, r0), membership::absent);
        EXPECT_EQ(status_of(iis, r1), membership::lower);
        EXPECT_EQ(status_of(iis, r2), membership::absent);
        EXPECT_EQ(iis.num_variable_members(), 0u);
        EXPECT_EQ(iis.num_constraint_members(), 1u);
        EXPECT_EQ(model.solves, 0u);
        expect_untouched(model, before);
        EXPECT_TRUE(is<status::optimal>(model.get_status()));
    }

    template <typename M, typename Stop>
    static void expect_crossed_pair_at_zero_budget(const iis_limits & limits) {
        M model;
        const auto [x, y, r] = crossed_variable_model(model);
        prime_optimal(model);
        const auto before = model.data();
        const auto iis = compute_iis_by_deletion(model, limits);
        EXPECT_TRUE(outcome_is<Stop>(iis.get_outcome(), true));
        EXPECT_EQ(status_of(iis, x), membership::both);
        EXPECT_EQ(status_of(iis, y), membership::absent);
        EXPECT_EQ(status_of(iis, r), membership::absent);
        EXPECT_EQ(iis.num_variable_members(), 1u);
        EXPECT_EQ(iis.num_constraint_members(), 0u);
        EXPECT_EQ(model.solves, 0u);
        expect_untouched(model, before);
        EXPECT_TRUE(is<status::optimal>(model.get_status()));
    }
};

///////////////////////////////////////////////////////////////////////////////
////////////////////// Classification through the function ////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST_F(iis_by_deletion, failed_with_a_solution_is_inconclusive) {
    expect_scripted_single_variable({status::failed{true}},
                                    iis_outcome::inconclusive_trial(false),
                                    membership::absent);
}

TEST_F(iis_by_deletion, numerical_failure_is_inconclusive) {
    expect_scripted_single_variable({status::numerical_failure{true}},
                                    iis_outcome::inconclusive_trial(false),
                                    membership::absent);
}

TEST_F(iis_by_deletion, time_limit_with_a_solution_proves_feasibility) {
    expect_scripted_single_variable({status::time_limit{true}},
                                    iis_outcome::feasible{},
                                    membership::absent);
}

TEST_F(iis_by_deletion, time_limit_without_a_solution_is_inconclusive) {
    expect_scripted_single_variable({status::time_limit{false}},
                                    iis_outcome::inconclusive_trial(false),
                                    membership::absent);
}

TEST_F(iis_by_deletion, infeasible_or_unbounded_proves_infeasibility) {
    expect_scripted_single_variable({status::infeasible_or_unbounded{},
                                     status::optimal{}, status::optimal{}},
                                    iis_outcome::irreducible{},
                                    membership::both);
}

TEST_F(iis_by_deletion, primal_and_dual_infeasible_proves_infeasibility) {
    expect_scripted_single_variable({status::primal_and_dual_infeasible{},
                                     status::optimal{}, status::optimal{}},
                                    iis_outcome::irreducible{},
                                    membership::both);
}

TEST_F(iis_by_deletion, optimal_infeasible_unscaled_is_inconclusive) {
    expect_scripted_single_variable({status::optimal_infeasible_unscaled{}},
                                    iis_outcome::inconclusive_trial(false),
                                    membership::absent);
}

TEST_F(iis_by_deletion, unbounded_is_inconclusive) {
    expect_scripted_single_variable({status::unbounded{}},
                                    iis_outcome::inconclusive_trial(false),
                                    membership::absent);
}

TEST_F(iis_by_deletion, unknown_is_inconclusive) {
    expect_scripted_single_variable({status::unknown{}},
                                    iis_outcome::inconclusive_trial(false),
                                    membership::absent);
}

// drop lower -> trial({upper}) inconclusive keeps it; drop upper ->
// trial({lower}) feasible keeps it
TEST_F(iis_by_deletion, inconclusive_singleton_keeps_the_member) {
    expect_scripted_single_variable(
        {status::infeasible{}, status::failed{true}, status::optimal{}},
        iis_outcome::inconclusive_trial(true), membership::both);
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// Outcomes on the default rule //////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST_F(iis_by_deletion, feasible_model_makes_one_solve) {
    using namespace operators;
    stub model;
    const auto x = bounded_variable(model, 0.0, 1.0);
    model.add_constraint(x <= 5);
    const auto before = model.data();
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::feasible>(iis.get_outcome()));
    EXPECT_EQ(model.solves, 1u);
    EXPECT_EQ(iis.num_variable_members(), 0u);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
    expect_restored(model, before);
    ASSERT_EQ(model.trials.size(), 1u);
    for(const double c : model.trials[0].objective) EXPECT_EQ(c, 0.0);
}

TEST_F(iis_by_deletion, background_alone_infeasible_has_no_member) {
    stub model;
    model.add_variable({.lower_bound = -model.infinity()});
    model.script = {status::infeasible{}};
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(iis.num_variable_members(), 0u);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_EQ(model.solves, 1u);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}

TEST_F(iis_by_deletion, bound_and_row_conflict_is_irreducible) {
    probe model;
    const auto h = conflict_model(model);
    const auto before = model.data();
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, h.x), membership::upper);
    EXPECT_EQ(status_of(iis, h.y), membership::absent);
    EXPECT_EQ(status_of(iis, h.r0), membership::lower);
    EXPECT_EQ(status_of(iis, h.r1), membership::absent);
    EXPECT_EQ(status_of(iis, h.r2), membership::absent);
    EXPECT_EQ(iis.num_variable_members(), 1u);
    EXPECT_EQ(iis.num_constraint_members(), 1u);
    EXPECT_EQ(model.solves, 8u);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
    expect_restored(model, before);
}

TEST_F(iis_by_deletion, ranged_row_sides_are_separate_candidates) {
    stub model;
    const auto x = bounded_variable(model, 9.0, 12.0);
    const auto r = model.add_ranged_constraint(x, 2.0, 8.0);
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, x), membership::lower);
    EXPECT_EQ(status_of(iis, r), membership::upper);
    EXPECT_EQ(model.solves, 5u);
}

// lower 1e20, upper 1e20: not crossed, the lower side is a candidate and the
// upper is not
TEST_F(iis_by_deletion, wrong_infinity_side_is_a_candidate) {
    using namespace operators;
    stub model;
    const auto x = bounded_variable(model, 0.0, 1.0);
    const auto r = model.add_constraint(x >= model.infinity());
    EXPECT_EQ(model.get_constraint_lower_bound(r), inf);
    EXPECT_EQ(model.get_constraint_upper_bound(r), inf);
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, r), membership::lower);
    EXPECT_EQ(status_of(iis, x), membership::absent);
    EXPECT_EQ(model.solves, 4u);
}

// The single pass swaps the last member into the tested slot, so the trials
// test candidates 0, 6, 5, 4, 1, 2, 3; within a trial the applier walks the
// candidates in ascending index. If either order changes, update this case.
TEST_F(iis_by_deletion, every_finite_side_is_enumerated_in_id_order) {
    stub model;
    const auto [x, y, r0, r1, r2] = conflict_model(model);
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    const std::vector<stub_write> expected = {
        // initial trial: no side written
        {stub_side::variable_lower, x.id(), -inf},  // candidate 0, dropped
        {stub_side::row_lower, r2.id(), -inf},      // candidate 6, dropped
        {stub_side::row_upper, r1.id(), inf},       // candidate 5, dropped
        {stub_side::row_lower, r0.id(), -inf},      // candidate 4, kept
        {stub_side::variable_upper, x.id(), inf},   // candidate 1, kept
        {stub_side::row_lower, r0.id(), 12.0},
        {stub_side::variable_upper, x.id(), 10.0},  // candidate 2, dropped
        {stub_side::variable_lower, y.id(), -inf},
        {stub_side::variable_upper, y.id(), inf},  // candidate 3, dropped
        // restore, in candidate order over the sides still relaxed
        {stub_side::variable_lower, x.id(), 0.0},
        {stub_side::variable_lower, y.id(), 0.0},
        {stub_side::variable_upper, y.id(), 1.0},
        {stub_side::row_upper, r1.id(), 20.0},
        {stub_side::row_lower, r2.id(), 0.5},
    };
    EXPECT_EQ(model.side_writes, expected);
}

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Guard ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST_F(iis_by_deletion, data_is_restored_after_a_complete_run) {
    stub model;
    conflict_model(model, true);
    const auto before = model.data();
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    expect_restored(model, before);
    EXPECT_TRUE(model.is_maximization());
    EXPECT_EQ(model.objective_writes, 2u);
    EXPECT_GE(model.offset_writes, 1u);
}

TEST_F(iis_by_deletion, lazy_objective_is_restored) {
    stub model;
    const auto h = conflict_model(model);
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(model.get_objective_coefficient(h.x), 3.0);
    EXPECT_EQ(model.get_objective_coefficient(h.y), -2.0);
    EXPECT_EQ(model.get_objective_offset(), 7.0);
    ASSERT_EQ(model.trials.size(), 8u);
    for(const stub_data & trial : model.trials) {
        for(const double c : trial.objective) EXPECT_EQ(c, 0.0);
        EXPECT_EQ(trial.offset, 0.0);
    }
}

TEST_F(iis_by_deletion, quadratic_objective_is_restored) {
    using namespace operators;
    qp_stub model;
    const auto x = bounded_variable(model, 0.0, 10.0);
    const auto y = bounded_variable(model, 0.0, 1.0);
    model.add_constraint(x >= 12);
    model.set_quadratic_objective(x * x + 2 * x * y + 3 * x + 1);
    const auto before = model.data();
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    ASSERT_QUAD_TERMS(model.get_quadratic_objective().quadratic_terms(),
                      {{x, x, 1.0}, {x, y, 2.0}});
    EXPECT_EQ(model.get_objective_coefficient(x), 3.0);
    EXPECT_EQ(model.get_objective_coefficient(y), 0.0);
    EXPECT_EQ(model.get_objective_offset(), 1.0);
    ASSERT_FALSE(model.trials.empty());
    for(const stub_data & trial : model.trials)
        EXPECT_TRUE(trial.quadratic_terms.empty());
    expect_restored(model, before);
}

TEST_F(iis_by_deletion, throw_at_trial_k_restores_and_resets_status) {
    for(const std::size_t k : {0u, 1u, 3u}) {
        stub model;
        conflict_model(model);
        prime_optimal(model);
        const auto before = model.data();
        model.on_solve = [k](stub &, std::size_t i) {
            if(i == k) throw iis_stub_failure("trial");
        };
        EXPECT_THROW((void)compute_iis_by_deletion(model), iis_stub_failure);
        expect_restored(model, before);
        EXPECT_TRUE(is<status::unknown>(model.get_status()));
        EXPECT_EQ(model.solves, k + 1);
    }
}

TEST_F(iis_by_deletion, throw_during_restore_still_restores_the_rest) {
    using namespace operators;
    stub model;
    const auto [x, y, r] = crossed_variable_model(model);
    model.set_objective(2 * x + 1);
    model.reset_recording();
    const auto x_id = x.id();
    model.on_solve = [x_id](stub & m, std::size_t i) {
        if(i == 1) m.failing_side = std::pair{stub_side::variable_upper, x_id};
    };
    EXPECT_THROW((void)compute_iis_by_deletion(model), iis_stub_failure);
    EXPECT_EQ(model.solves, 2u);
    // attempted once: the destructor did not retry
    EXPECT_EQ(model.failed_writes, 1u);
    EXPECT_EQ(model.get_variable_lower_bound(x), 3.0);
    EXPECT_EQ(model.get_variable_upper_bound(x), inf);
    EXPECT_EQ(model.get_variable_lower_bound(y), 0.0);
    EXPECT_EQ(model.get_variable_upper_bound(y), 5.0);
    EXPECT_EQ(model.get_constraint_lower_bound(r), 1.0);
    EXPECT_EQ(model.get_constraint_upper_bound(r), inf);
    EXPECT_EQ(model.get_objective_coefficient(x), 2.0);
    EXPECT_EQ(model.get_objective_coefficient(y), 0.0);
    EXPECT_EQ(model.get_objective_offset(), 1.0);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}

TEST_F(iis_by_deletion, nothing_is_written_before_the_first_trial) {
    {
        stub model;
        conflict_model(model);
        const auto before = model.data();
        const auto iis =
            compute_iis_by_deletion(model, iis_limits{.max_solves = 0});
        EXPECT_TRUE(
            outcome_is<iis_outcome::solve_limit>(iis.get_outcome(), false));
        EXPECT_EQ(model.solves, 0u);
        expect_untouched(model, before);
    }
    {
        timed_stub model;
        conflict_model(model);
        const auto before = model.data();
        const auto iis = compute_iis_by_deletion(
            model, iis_limits{.max_solves = 0, .time_limit = 3600s});
        EXPECT_TRUE(
            outcome_is<iis_outcome::solve_limit>(iis.get_outcome(), false));
        EXPECT_EQ(model.solves, 0u);
        expect_untouched(model, before);
        EXPECT_EQ(model.time_limit_reads, 0u);
    }
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Prechecks //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST_F(iis_by_deletion, column_less_model_is_decided_without_a_solve) {
    stub model;
    const auto r0 = add_row(model, -inf, 5.0);
    const auto r1 = add_row(model, 1.0, inf);
    const auto r2 = add_row(model, -inf, -3.0);
    prime_optimal(model);
    const auto before = model.data();
    expect_column_less_answer(model, before, r0, r1, r2, iis_limits{});
    // the precheck costs no solve, so every budget gets the same answer
    expect_column_less_answer(model, before, r0, r1, r2,
                              iis_limits{.max_solves = 0});
    expect_column_less_answer(model, before, r0, r1, r2,
                              iis_limits{.time_limit = 0s});
    std::stop_source source;
    source.request_stop();
    expect_column_less_answer(model, before, r0, r1, r2,
                              iis_limits{.stop_token = source.get_token()});
}

TEST_F(iis_by_deletion, column_less_upper_side_below_zero) {
    stub model;
    const auto r = add_row(model, -inf, -3.0);
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, r), membership::upper);
    EXPECT_EQ(model.solves, 0u);
}

TEST_F(iis_by_deletion, column_less_first_violated_side_wins) {
    stub model;
    const auto r0 = add_row(model, -inf, -1.0);
    const auto r1 = add_row(model, 2.0, inf);
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, r0), membership::upper);
    EXPECT_EQ(status_of(iis, r1), membership::absent);
    EXPECT_EQ(iis.num_constraint_members(), 1u);
    EXPECT_EQ(model.solves, 0u);
}

TEST_F(iis_by_deletion, column_less_precheck_compares_exactly_with_zero) {
    {
        stub model;
        add_row(model, 0.0, inf);
        add_row(model, -inf, 0.0);
        add_row(model, 0.0, 0.0);
        const auto iis = compute_iis_by_deletion(model);
        EXPECT_TRUE(outcome_is<iis_outcome::feasible>(iis.get_outcome()));
        EXPECT_EQ(iis.num_constraint_members(), 0u);
        EXPECT_EQ(model.solves, 0u);
    }
    {
        stub model;
        const auto r = add_row(model, 1e-300, inf);
        const auto iis = compute_iis_by_deletion(model);
        EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        EXPECT_EQ(status_of(iis, r), membership::lower);
    }
    {
        // a lower side the backend holds at the wrong infinity stays a
        // candidate, which 0 violates
        stub model;
        const auto r = add_row(model, inf, inf);
        const auto iis = compute_iis_by_deletion(model);
        EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        EXPECT_EQ(status_of(iis, r), membership::lower);
    }
}

TEST_F(iis_by_deletion, column_less_after_removing_every_variable) {
    using namespace operators;
    stub model;
    const auto x = model.add_variable();
    const auto r = model.add_constraint(x >= 1);
    model.remove_variable(x);
    EXPECT_TRUE(model.variables().empty());
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, r), membership::lower);
    EXPECT_EQ(status_of(iis, x), membership::absent);
    EXPECT_EQ(model.solves, 0u);
}

// The lower side wins when 0 violates both; the comparison is exact.
static_assert(detail::iis_side_violated_by_zero(1., 2.) == std::optional(true));
static_assert(detail::iis_side_violated_by_zero(-2., -1.) ==
              std::optional(false));
static_assert(detail::iis_side_violated_by_zero(1., -1.) ==
              std::optional(true));
static_assert(!detail::iis_side_violated_by_zero(0., 0.));
static_assert(!detail::iis_side_violated_by_zero(
    -std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity()));

namespace {
std::string self_infeasible_sides(double lower, double upper,
                                  detail::iis_column_kind kind) {
    const auto sides = detail::iis_self_infeasible_column(lower, upper, kind);
    if(!sides) return "none";
    if(sides->lower && sides->upper) return "both";
    return sides->lower ? "lower" : "upper";
}
}  // namespace

// The native wrappers whose routines fail on such a column answer it from
// this arithmetic, which no backend in CI reaches.
TEST(iis_arithmetic, column_whose_bounds_admit_no_value) {
    using kind = detail::iis_column_kind;
    constexpr double inf = std::numeric_limits<double>::infinity();
    EXPECT_EQ(self_infeasible_sides(1., 0., kind::continuous), "both");
    EXPECT_EQ(self_infeasible_sides(0.25, 0.75, kind::continuous), "none");
    EXPECT_EQ(self_infeasible_sides(-inf, inf, kind::continuous), "none");
    EXPECT_EQ(self_infeasible_sides(0.25, 0.75, kind::integer), "both");
    EXPECT_EQ(self_infeasible_sides(-0.5, 0.5, kind::integer), "none");
    EXPECT_EQ(self_infeasible_sides(-inf, inf, kind::integer), "none");
    EXPECT_EQ(self_infeasible_sides(2., 3., kind::binary), "lower");
    EXPECT_EQ(self_infeasible_sides(2., 1., kind::binary), "lower");
    EXPECT_EQ(self_infeasible_sides(-3., -2., kind::binary), "upper");
    EXPECT_EQ(self_infeasible_sides(0., -2., kind::binary), "upper");
    EXPECT_EQ(self_infeasible_sides(0.25, 0.75, kind::binary), "both");
    EXPECT_EQ(self_infeasible_sides(-3., 3., kind::binary), "none");
    EXPECT_EQ(self_infeasible_sides(1., 1., kind::binary), "none");
    // never decided, crossed or not
    EXPECT_EQ(self_infeasible_sides(2., 1., kind::other), "none");
}

static_assert(detail::iis_column_kind_of<'C', 'I', 'B'>('B') ==
              detail::iis_column_kind::binary);
static_assert(detail::iis_column_kind_of<'C', 'I', 'B'>('S') ==
              detail::iis_column_kind::other);

// Column 0, semi-continuous, is never decided; columns 2 and 3 both admit no
// value.
TEST(iis_arithmetic, first_self_infeasible_column_by_type_code) {
    const std::vector<double> lower{2., 0.25, 1., 2.};
    const std::vector<double> upper{1., 0.75, 0., 3.};
    const std::vector<char> types{'S', 'C', 'I', 'B'};
    const auto first = detail::iis_first_self_infeasible_column<'C', 'I', 'B'>(
        lower, upper, types);
    ASSERT_TRUE(first);
    EXPECT_EQ(first->index, 2u);
    EXPECT_TRUE(first->sides.lower && first->sides.upper);
    EXPECT_FALSE((detail::iis_first_self_infeasible_column<'C', 'I', 'B'>(
        lower, upper, std::span(types).first(2))));
    // the codes are the backend's: here 'S' spells continuous
    const auto continuous =
        detail::iis_first_self_infeasible_column<'S', 'I', 'B'>(lower, upper,
                                                                types);
    ASSERT_TRUE(continuous);
    EXPECT_EQ(continuous->index, 0u);
}

TEST_F(iis_by_deletion, empty_model_is_feasible) {
    stub model;
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::feasible>(iis.get_outcome()));
    EXPECT_EQ(iis.num_variable_members(), 0u);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_EQ(model.solves, 0u);
}

TEST_F(iis_by_deletion, crossed_pair_at_zero_budget_is_the_proven_pair) {
    expect_crossed_pair_at_zero_budget<stub, iis_outcome::solve_limit>(
        iis_limits{.max_solves = 0});
    expect_crossed_pair_at_zero_budget<stub, iis_outcome::time_limit>(
        iis_limits{.time_limit = 0s});
    std::stop_source source;
    source.request_stop();
    expect_crossed_pair_at_zero_budget<stub, iis_outcome::interrupted>(
        iis_limits{.stop_token = source.get_token()});
}

TEST_F(iis_by_deletion, crossed_variable_is_decided_by_two_trials) {
    probe model;
    const auto [x, y, r] = crossed_variable_model(model);
    const auto before = model.data();
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, x), membership::both);
    EXPECT_EQ(status_of(iis, y), membership::absent);
    EXPECT_EQ(status_of(iis, r), membership::absent);
    ASSERT_EQ(model.solves, 2u);
    ASSERT_EQ(model.trials.size(), 2u);
    // the upper side alone, everything else relaxed
    EXPECT_EQ(model.trials[0].variable_bounds[x.uid()], std::pair(-inf, 1.0));
    EXPECT_EQ(model.trials[0].variable_bounds[y.uid()], std::pair(-inf, inf));
    EXPECT_EQ(model.trials[0].row_bounds[r.uid()], std::pair(-inf, inf));
    // then the lower side alone
    EXPECT_EQ(model.trials[1].variable_bounds[x.uid()], std::pair(3.0, inf));
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
    expect_restored(model, before);
}

TEST_F(iis_by_deletion, crossed_variable_side_infeasible_alone) {
    stub model;
    const auto [x, y, r] = crossed_variable_model(model);
    model.script = {status::infeasible{}, status::optimal{}};
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, x), membership::upper);
    EXPECT_EQ(status_of(iis, y), membership::absent);
    EXPECT_EQ(status_of(iis, r), membership::absent);
    EXPECT_EQ(model.solves, 2u);
}

// 0 <= 1 holds, 0 >= 2 fails
TEST_F(iis_by_deletion, crossed_row_without_terms_needs_one_side) {
    stub model;
    const auto x = bounded_variable(model, 0.0, 1.0);
    const auto r = add_row(model, 2.0, 1.0);
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, r), membership::lower);
    EXPECT_EQ(status_of(iis, x), membership::absent);
    EXPECT_EQ(model.solves, 2u);
}

TEST_F(iis_by_deletion, crossed_row_with_terms_needs_both_sides) {
    stub model;
    const auto x = bounded_variable(model, 0.0, 10.0);
    const auto r = model.add_ranged_constraint(x, 7.0, 3.0);
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, r), membership::both);
    EXPECT_EQ(status_of(iis, x), membership::absent);
    EXPECT_EQ(model.solves, 2u);
}

TEST_F(iis_by_deletion, first_crossed_pair_in_enumeration_order_is_used) {
    stub model;
    const auto x = bounded_variable(model, 3.0, 1.0);
    const auto y = bounded_variable(model, 5.0, 2.0);
    const auto r = add_row(model, 2.0, 1.0);
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, x), membership::both);
    EXPECT_EQ(status_of(iis, y), membership::absent);
    EXPECT_EQ(status_of(iis, r), membership::absent);
    ASSERT_EQ(model.solves, 2u);
    // the later crossed pairs are ordinary candidates, relaxed in both trials
    EXPECT_EQ(model.trials[0].variable_bounds[y.uid()], std::pair(-inf, inf));
    EXPECT_EQ(model.trials[1].variable_bounds[y.uid()], std::pair(-inf, inf));
    EXPECT_EQ(model.trials[0].row_bounds[r.uid()], std::pair(-inf, inf));
}

TEST_F(iis_by_deletion, crossed_pair_under_one_solve) {
    stub model;
    const auto [x, y, r] = crossed_variable_model(model);
    const auto before = model.data();
    const auto iis =
        compute_iis_by_deletion(model, iis_limits{.max_solves = 1});
    EXPECT_TRUE(outcome_is<iis_outcome::solve_limit>(iis.get_outcome(), true));
    EXPECT_EQ(status_of(iis, x), membership::both);
    EXPECT_EQ(model.solves, 1u);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
    expect_restored(model, before);
}

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Status ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST_F(iis_by_deletion, status_is_unknown_after_a_run_that_solved) {
    {
        stub model;
        bounded_variable(model, 0.0, 1.0);
        prime_optimal(model);
        model.script = {status::optimal{}};
        const auto iis = compute_iis_by_deletion(model);
        EXPECT_TRUE(outcome_is<iis_outcome::feasible>(iis.get_outcome()));
        EXPECT_TRUE(is<status::unknown>(model.get_status()));
    }
    {
        stub model;
        conflict_model(model);
        prime_optimal(model);
        const auto iis = compute_iis_by_deletion(model);
        EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        EXPECT_TRUE(is<status::unknown>(model.get_status()));
    }
}

TEST_F(iis_by_deletion, status_is_kept_when_no_solve_ran) {
    stub model;
    conflict_model(model);
    prime_optimal(model);
    const auto iis =
        compute_iis_by_deletion(model, iis_limits{.max_solves = 0});
    EXPECT_TRUE(outcome_is<iis_outcome::solve_limit>(iis.get_outcome(), false));
    EXPECT_TRUE(is<status::optimal>(model.get_status()));
}

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Limits ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST_F(iis_by_deletion, budget_sweep_keeps_a_valid_answer) {
    budget_sweep<stub>(seconds(positive_infinity));
    // forwarding under a solve limit
    budget_sweep<timed_stub>(3600s);
}

TEST_F(iis_by_deletion, stop_requested_beforehand) {
    std::stop_source source;
    source.request_stop();
    stub model;
    conflict_model(model);
    const auto before = model.data();
    const auto iis = compute_iis_by_deletion(
        model, iis_limits{.stop_token = source.get_token()});
    EXPECT_TRUE(outcome_is<iis_outcome::interrupted>(iis.get_outcome(), false));
    EXPECT_EQ(model.solves, 0u);
    expect_untouched(model, before);
}

TEST_F(iis_by_deletion, zero_time_budget) {
    stub model;
    conflict_model(model);
    const auto before = model.data();
    const auto iis =
        compute_iis_by_deletion(model, iis_limits{.time_limit = 0s});
    EXPECT_TRUE(outcome_is<iis_outcome::time_limit>(iis.get_outcome(), false));
    EXPECT_EQ(model.solves, 0u);
    expect_untouched(model, before);
}

TEST_F(iis_by_deletion,
       nan_or_negative_time_limit_throws_before_reading_the_model) {
    for(const double limit : {nan, -1.0, -positive_infinity}) {
        stub model;
        conflict_model(model);
        const auto before = model.data();
        EXPECT_THROW((void)compute_iis_by_deletion(
                         model, iis_limits{.time_limit = seconds(limit)}),
                     std::invalid_argument);
        EXPECT_EQ(model.variables_calls, 0u);
        EXPECT_EQ(model.solves, 0u);
        expect_untouched(model, before);
    }
}

TEST_F(iis_by_deletion, stop_requested_during_a_trial_keeps_the_proven_set) {
    std::stop_source source;
    stub model;
    const auto h = conflict_model(model);
    const auto before = model.data();
    model.on_solve = [&source](stub &, std::size_t i) {
        if(i == 0) source.request_stop();
    };
    const auto iis = compute_iis_by_deletion(
        model, iis_limits{.stop_token = source.get_token()});
    EXPECT_TRUE(outcome_is<iis_outcome::interrupted>(iis.get_outcome(), true));
    EXPECT_EQ(status_of(iis, h.x), membership::both);
    EXPECT_EQ(status_of(iis, h.y), membership::both);
    EXPECT_EQ(status_of(iis, h.r0), membership::lower);
    EXPECT_EQ(status_of(iis, h.r1), membership::upper);
    EXPECT_EQ(status_of(iis, h.r2), membership::lower);
    EXPECT_EQ(iis.num_variable_members(), 2u);
    EXPECT_EQ(iis.num_constraint_members(), 3u);
    EXPECT_EQ(model.solves, 1u);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
    expect_restored(model, before);
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Forwarding /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST_F(iis_by_deletion, default_limits_never_touch_the_time_limit) {
    timed_stub model;
    conflict_model(model);
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(model.time_limit_reads, 0u);
    EXPECT_TRUE(model.time_limit_writes.empty());
}

// the deadline saturates to no deadline
TEST_F(iis_by_deletion, infinite_time_limit_never_touches_the_time_limit) {
    for(const seconds limit : {seconds(positive_infinity), seconds::max()}) {
        timed_stub model;
        conflict_model(model);
        const auto iis =
            compute_iis_by_deletion(model, iis_limits{.time_limit = limit});
        EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        EXPECT_EQ(model.time_limit_reads, 0u);
        EXPECT_TRUE(model.time_limit_writes.empty());
    }
}

TEST_F(iis_by_deletion, finite_deadline_forwards_min_of_remaining_and_saved) {
    for(const double saved :
        {positive_infinity, std::numeric_limits<double>::max(), 1e100, 1e75,
         1e20, 2.0, 0.0, nan}) {
        timed_probe model;
        model.time_limit_value = saved;
        model.caller_time_limit = saved;
        conflict_model(model);
        const auto iis =
            compute_iis_by_deletion(model, iis_limits{.time_limit = 3600s});
        EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        EXPECT_EQ(model.time_limit_reads, 1u);
        const auto & writes = model.time_limit_writes;
        // one per trial plus the restore
        ASSERT_EQ(writes.size(), model.solves + 1) << "saved = " << saved;
        for(std::size_t i = 0; i + 1 < writes.size(); ++i) {
            const double w = writes[i];
            EXPECT_LE(w, 3600.0) << "saved = " << saved;
            if(!std::isnan(saved)) {
                EXPECT_LE(w, saved) << "saved = " << saved;
            }
            if(std::isnan(saved) || saved >= 3600.0) {
                // remaining, and a NaN saved limit forwards remaining too
                EXPECT_GT(w, 3590.0) << "saved = " << saved;
            } else {
                EXPECT_EQ(w, saved);
            }
        }
        const double last = writes.back();
        if(std::isnan(saved)) {
            EXPECT_TRUE(std::isnan(last));
        } else {
            EXPECT_EQ(last, saved);
        }
    }
}

TEST_F(iis_by_deletion, caller_limit_shorter_than_saved) {
    timed_probe model;
    model.time_limit_value = 2.0;
    model.caller_time_limit = 2.0;
    conflict_model(model);
    const auto iis =
        compute_iis_by_deletion(model, iis_limits{.time_limit = 1s});
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    const auto & writes = model.time_limit_writes;
    ASSERT_EQ(writes.size(), model.solves + 1);
    for(std::size_t i = 0; i + 1 < writes.size(); ++i) {
        EXPECT_GT(writes[i], 0.0);
        EXPECT_LE(writes[i], 1.0);
    }
    EXPECT_EQ(writes.back(), 2.0);
}

TEST_F(iis_by_deletion, time_limit_is_restored_after_a_throw) {
    timed_stub model;
    model.time_limit_value = 2.0;
    conflict_model(model);
    model.on_solve = [](timed_stub &, std::size_t i) {
        if(i == 1) throw iis_stub_failure("trial");
    };
    EXPECT_THROW(
        (void)compute_iis_by_deletion(model, iis_limits{.time_limit = 3600s}),
        iis_stub_failure);
    EXPECT_EQ(model.time_limit_reads, 1u);
    ASSERT_FALSE(model.time_limit_writes.empty());
    EXPECT_EQ(model.time_limit_writes.back(), 2.0);
    EXPECT_EQ(model.time_limit_value, 2.0);
}

TEST_F(iis_by_deletion, untimed_model_ignores_a_finite_deadline) {
    stub model;
    const auto h = conflict_model(model);
    const auto iis =
        compute_iis_by_deletion(model, iis_limits{.time_limit = 3600s});
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, h.x), membership::upper);
    EXPECT_EQ(status_of(iis, h.r0), membership::lower);
    EXPECT_EQ(model.solves, 8u);
}

TEST_F(iis_by_deletion, no_time_limit_is_read_when_no_trial_runs) {
    timed_stub model;
    conflict_model(model);
    const auto iis = compute_iis_by_deletion(
        model, iis_limits{.max_solves = 0, .time_limit = 3600s});
    EXPECT_TRUE(outcome_is<iis_outcome::solve_limit>(iis.get_outcome(), false));
    EXPECT_EQ(model.time_limit_reads, 0u);
    EXPECT_TRUE(model.time_limit_writes.empty());
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Snapshot keys /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST_F(iis_by_deletion, handles_the_enumeration_skips_read_absent) {
    stub model;
    const auto x0 = bounded_variable(model, 0.0, 1.0);
    const auto x1 = bounded_variable(model, 2.0, 1.0);
    model.remove_variable(x0);
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, x1), membership::both);
    EXPECT_EQ(status_of(iis, x0), membership::absent);
    EXPECT_EQ(iis.num_variable_members(), 1u);
    for(const stub_write & w : model.side_writes) EXPECT_NE(w.id, x0.id());
    // past the bound of the table
    const auto x2 = model.add_variable();
    EXPECT_EQ(x2.uid(), 2u);
    EXPECT_EQ(status_of(iis, x2), membership::absent);
}

TEST_F(iis_by_deletion, answer_is_keyed_by_id_after_a_later_removal) {
    stub model;
    const auto h = conflict_model(model);
    const auto iis = compute_iis_by_deletion(model);
    model.remove_variable(h.y);
    EXPECT_EQ(status_of(iis, h.x), membership::upper);
    EXPECT_EQ(status_of(iis, h.y), membership::absent);
    EXPECT_EQ(status_of(iis, h.r0), membership::lower);
    EXPECT_EQ(iis.num_variable_members(), 1u);
    EXPECT_EQ(iis.num_constraint_members(), 1u);
}

///////////////////////////////////////////////////////////////////////////////
////////////////////// Rows written as a sense and an rhs /////////////////////
///////////////////////////////////////////////////////////////////////////////

// x in [0, 1], x == 5 and x <= 3: the == row conflicts through its lower
// side, and the <= row is redundant.
TEST_F(iis_by_deletion, rows_without_bound_setters_take_a_sense_and_an_rhs) {
    using namespace operators;
    sense_rhs_stub model;
    const auto x = bounded_variable(model, 0.0, 1.0);
    const auto equal = model.add_constraint(x == 5);
    const auto at_most = model.add_constraint(x <= 3);
    model.reset_recording();
    const auto before = model.data();
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, x), membership::upper);
    EXPECT_EQ(status_of(iis, equal), membership::lower);
    EXPECT_EQ(status_of(iis, at_most), membership::absent);
    EXPECT_EQ(iis.num_variable_members(), 1u);
    EXPECT_EQ(iis.num_constraint_members(), 1u);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
    expect_restored(model, before);
    // every state of the == row is one of its sides, both or neither; the
    // trials test its upper side before its lower one, so one of them sees
    // its lower side alone
    bool lower_side_alone = false;
    for(const stub_data & trial : model.trials) {
        const auto [lo, hi] = trial.row_bounds[equal.uid()];
        EXPECT_TRUE((lo == 5.0 || lo == -inf) && (hi == 5.0 || hi == inf))
            << lo << ", " << hi;
        lower_side_alone |= (lo == 5.0 && hi == inf);
    }
    EXPECT_TRUE(lower_side_alone);
    // the restore writes the == row back as one, not as two sides, and its
    // sense moved while the <= row's never did
    using row_write = sense_rhs_stub::row_write;
    std::vector<row_write> writes_of_equal;
    std::ranges::copy_if(
        model.row_writes, std::back_inserter(writes_of_equal),
        [&](const row_write & w) { return w.id == equal.id(); });
    ASSERT_FALSE(writes_of_equal.empty());
    EXPECT_EQ(writes_of_equal.back(),
              (row_write{equal.id(), sense_rhs_stub::row_setter::rhs,
                         constraint_sense::equal, 5.0}));
    EXPECT_TRUE(model.sense_written(equal));
    EXPECT_FALSE(model.sense_written(at_most));
}

// x and y in [0, 1], x >= 2, x <= 3 and y >= -10: the first row conflicts
// with the upper bound of x, and the two others are redundant. Each row keeps
// its sense through the run, relaxed to an infinite rhs and restored through
// its rhs.
TEST_F(iis_by_deletion, one_sided_rows_are_relaxed_through_their_rhs_alone) {
    using namespace operators;
    sense_rhs_stub model;
    const auto x = bounded_variable(model, 0.0, 1.0);
    const auto at_least = model.add_constraint(x >= 2);
    const auto at_most = model.add_constraint(x <= 3);
    const auto y = bounded_variable(model, 0.0, 1.0);
    const auto loose = model.add_constraint(y >= -10);
    model.reset_recording();
    const auto before = model.data();
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, x), membership::upper);
    EXPECT_EQ(status_of(iis, y), membership::absent);
    EXPECT_EQ(status_of(iis, at_least), membership::lower);
    EXPECT_EQ(status_of(iis, at_most), membership::absent);
    EXPECT_EQ(status_of(iis, loose), membership::absent);
    expect_restored(model, before);
    ASSERT_FALSE(model.row_writes.empty());
    for(const auto & w : model.row_writes) {
        EXPECT_EQ(w.setter, sense_rhs_stub::row_setter::rhs) << w.id;
        EXPECT_EQ(w.sense, w.id == at_most.id()
                               ? constraint_sense::less_equal
                               : constraint_sense::greater_equal)
            << w.id;
    }
    // the redundant rows were free in some trial, the >= one at -infinity()
    for(const auto c : {at_most, loose})
        EXPECT_TRUE(std::ranges::any_of(model.trials, [&](const stub_data & t) {
            return t.row_bounds[c.uid()] == std::pair{-inf, inf};
        })) << c.id();
    const sense_rhs_stub::row_write loose_freed{
        loose.id(), sense_rhs_stub::row_setter::rhs,
        constraint_sense::greater_equal, -inf};
    EXPECT_NE(std::ranges::find(model.row_writes, loose_freed),
              model.row_writes.end());
}

// x in [0, 1], x >= 2 and x == 0.5: both sides of the == row are dropped, so
// a trial sees it free, and the restore makes it an == row again.
TEST_F(iis_by_deletion, a_row_without_its_sides_is_free_then_restored_whole) {
    using namespace operators;
    sense_rhs_stub model;
    const auto x = bounded_variable(model, 0.0, 1.0);
    const auto at_least = model.add_constraint(x >= 2);
    const auto equal = model.add_constraint(x == 0.5);
    model.reset_recording();
    const auto before = model.data();
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(status_of(iis, x), membership::upper);
    EXPECT_EQ(status_of(iis, at_least), membership::lower);
    EXPECT_EQ(status_of(iis, equal), membership::absent);
    expect_restored(model, before);
    EXPECT_EQ(model.get_constraint_sense(equal), constraint_sense::equal);
    EXPECT_EQ(model.get_constraint_rhs(equal), 0.5);
    EXPECT_TRUE(std::ranges::any_of(model.trials, [&](const stub_data & t) {
        return t.row_bounds[equal.uid()] == std::pair{-inf, inf};
    }));
    EXPECT_FALSE(model.sense_written(at_least));
}

// The trials test candidates 0 (x lower) then 4 (the upper side of the ==
// row): the write that relaxes it throws, and so does its restore. The
// error propagates and the bound already relaxed is written back.
TEST_F(iis_by_deletion, a_throwing_row_write_leaves_the_rest_restored) {
    using namespace operators;
    sense_rhs_stub model;
    const auto x = bounded_variable(model, 0.0, 1.0);
    model.add_constraint(x >= 2);
    const auto equal = model.add_constraint(x == 0.5);
    model.reset_recording();
    const auto equal_id = equal.id();
    model.on_solve = [equal_id](iis_stub_model<false, false> & m,
                                std::size_t i) {
        if(i == 1) m.failing_side = std::pair{stub_side::row_lower, equal_id};
    };
    EXPECT_THROW((void)compute_iis_by_deletion(model), iis_stub_failure);
    EXPECT_EQ(model.solves, 2u);
    EXPECT_EQ(model.failed_writes, 2u);
    EXPECT_EQ(model.get_variable_lower_bound(x), 0.0);
    EXPECT_EQ(model.get_variable_upper_bound(x), 1.0);
    EXPECT_EQ(model.get_constraint_lower_bound(equal), 0.5);
    EXPECT_EQ(model.get_constraint_upper_bound(equal), 0.5);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Probe invariants ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST_F(iis_by_deletion,
       every_trial_sees_a_zero_objective_and_the_callers_sense) {
    probe model;
    conflict_model(model, true);
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    ASSERT_EQ(model.solves, 8u);
    ASSERT_EQ(model.trials.size(), 8u);
    ASSERT_TRUE(model.row_terms_at_construction.has_value());
    for(const stub_data & trial : model.trials) {
        EXPECT_TRUE(trial.maximize);
        for(const double c : trial.objective) EXPECT_EQ(c, 0.0);
        EXPECT_EQ(trial.offset, 0.0);
        EXPECT_EQ(trial.row_terms, *model.row_terms_at_construction);
    }
    EXPECT_TRUE(model.is_maximization());
}
