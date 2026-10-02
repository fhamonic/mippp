// The code of docs/solving/infeasibility.md. The page shows the sections
// between the --8<-- markers, so they stay plain user code; the tests below
// them check what the page says about their output and outcome.

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stop_token>
#include <streambuf>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "dumb_lp.hpp"
#include "mippp/solvers/clp/all.hpp"
// --8<-- [start:includes]
#include "mippp/solvers/highs/all.hpp"
#include "mippp/utility/iis_by_deletion.hpp"
// --8<-- [end:includes]

using namespace mippp;

#include "test_suites/all.hpp"

namespace infeasibility_page {

// here rather than in each function: on MSVC a block-scope using-directive
// does not reach into a generic lambda
using namespace mippp::operators;

// --8<-- [start:print-member]
// Prints a member with the side it conflicts through.
template <typename Status>
void print_member(std::string_view name, const Status & status) {
    if(!is_a<iis_status::member>(status)) return;
    if(is_a<iis_status::member_lower>(status))
        std::cout << name << ": lower side\n";
    else if(is_a<iis_status::member_upper>(status))
        std::cout << name << ": upper side\n";
    else
        std::cout << name << ": member\n";  // both sides, or a side not named
}
// --8<-- [end:print-member]

// What the workshop's model reports around the analysis, which the page
// describes and the tests check.
template <typename Model>
struct workshop_run {
    model_status_t<Model> first_solve;
    model_status_t<Model> after_analysis;
    model_status_t<Model> after_fix;
    double overtime_after_fix = 0.;
};

// The page's example as one program: Analyze stands for the lines of the page
// that compute the IIS, so that each path runs on the same model.
template <typename lp_type, typename Analyze>
workshop_run<lp_type> workshop(Analyze && analyze) {
    // --8<-- [start:workshop-model]
    lp_type model;
    auto chairs = model.add_variable();  // >= 0, as are tables and desks
    auto tables = model.add_variable();
    auto desks = model.add_variable();
    // in a params list, an omitted bound is unbounded
    auto overtime =
        model.add_variable({.obj_coef = 1, .lower_bound = 0, .upper_bound = 1});
    auto wood = model.add_constraint(2 * chairs + 5 * tables + 4 * desks <= 60);
    auto labour =
        model.add_constraint(chairs + 3 * tables + 2 * desks - overtime <= 30);
    auto order_chairs = model.add_constraint(chairs == 10);
    auto order_tables = model.add_constraint(tables == 6);
    auto order_desks = model.add_constraint(desks == 2);

    model.solve();  // infeasible
    // --8<-- [end:workshop-model]
    workshop_run<lp_type> run{model.get_status(), {}, {}};

    // --8<-- [start:workshop-report]
    auto print_conflict = [&](const auto & iis) {
        print_member("wood", iis.get_status(wood));
        print_member("labour", iis.get_status(labour));
        print_member("order chairs", iis.get_status(order_chairs));
        print_member("order tables", iis.get_status(order_tables));
        print_member("order desks", iis.get_status(order_desks));
        print_member("chairs", iis.get_status(chairs));
        print_member("tables", iis.get_status(tables));
        print_member("desks", iis.get_status(desks));
        print_member("overtime", iis.get_status(overtime));
    };
    // --8<-- [end:workshop-report]
    analyze(model, print_conflict);
    run.after_analysis = model.get_status();

    // --8<-- [start:workshop-fix]
    model.set_variable_upper_bound(overtime, 2);
    model.solve();  // optimal
    std::cout << "overtime: " << model.get_solution()[overtime] << " hours\n";
    // --8<-- [end:workshop-fix]
    run.after_fix = model.get_status();
    if(status::solution_available(run.after_fix))
        run.overtime_after_fix = model.get_solution()[overtime];
    return run;
}

template <typename Model, typename Print>
void deletion_path(Model & model, Print & print_conflict) {
    // --8<-- [start:workshop-deletion]
    const auto iis = compute_iis_by_deletion(model);
    if(iis.get_outcome() == iis_outcome::irreducible) print_conflict(iis);
    // --8<-- [end:workshop-deletion]
}

template <typename Model, typename Print>
void native_path(Model & model, Print & print_conflict) {
    // --8<-- [start:workshop-native]
    print_conflict(model.compute_iis());
    // --8<-- [end:workshop-native]
}

// --8<-- [start:diagnose]
// Calls report with an IIS of the model: the native routine's where the model
// has one, the deletion filter's otherwise, or when the native call fails, as
// it does on HiGHS before 1.14.
template <typename Model, typename Report>
    requires has_iis<Model> || iis_by_deletion_model<Model>
void diagnose(Model & model, Report && report) {
    if constexpr(has_iis<Model>) {
        std::optional<model_iis_t<Model>> native;
        try {
            native.emplace(model.compute_iis());
        } catch(const solver_error &) {
            if constexpr(!iis_by_deletion_model<Model>) throw;
        }
        if(native) {
            report(*native);
            return;
        }
    }
    if constexpr(iis_by_deletion_model<Model>)
        report(compute_iis_by_deletion(model));
}
// --8<-- [end:diagnose]

template <typename Model, typename Print>
void bounded_path(Model & model, Print & print_conflict, std::stop_token stop) {
    // --8<-- [start:limits]
    using namespace std::chrono_literals;
    const auto iis = compute_iis_by_deletion(
        model, {.max_solves = 20, .time_limit = 10s, .stop_token = stop});
    if(iis.get_outcome() == iis_outcome::irreducible ||
       iis.get_outcome() == iis_outcome::not_proven_minimal)
        print_conflict(iis);
    // --8<-- [end:limits]
}

// --8<-- [start:side-name]
// Every tag has an overload, so the visit compiles on the variant of any path,
// and the most derived one wins: plain member is told from member_both.
std::string_view side_of(iis_status::absent) { return "absent"; }
std::string_view side_of(iis_status::member) { return "side not named"; }
std::string_view side_of(iis_status::member_lower) { return "lower side"; }
std::string_view side_of(iis_status::member_upper) { return "upper side"; }
std::string_view side_of(iis_status::member_both) { return "both sides"; }

template <typename Status>
std::string_view side_name(const Status & status) {
    return std::visit([](auto tag) { return side_of(tag); }, status);
}
// --8<-- [end:side-name]

// --8<-- [start:member-rows]
template <typename Model, typename Iis>
std::vector<model_constraint_t<Model>> member_rows(Model & model,
                                                   const Iis & iis) {
    std::vector<model_constraint_t<Model>> rows;
    for(auto c : model.constraints())
        if(is_a<iis_status::member>(iis.get_status(c))) rows.push_back(c);
    return rows;
}
// --8<-- [end:member-rows]

// --8<-- [start:relax-members]
struct named_sides {
    bool lower = false;
    bool upper = false;
};

// plain member cannot say which side conflicts, so both are relaxed, as for
// member_both
struct sides_named {
    named_sides operator()(iis_status::absent) const { return {}; }
    named_sides operator()(iis_status::member) const { return {true, true}; }
    named_sides operator()(iis_status::member_lower) const {
        return {.lower = true};
    }
    named_sides operator()(iis_status::member_upper) const {
        return {.upper = true};
    }
    named_sides operator()(iis_status::member_both) const {
        return {true, true};
    }
};

// Relaxes the sides s names of row c. A model without row-bound setters, such
// as Gurobi's, holds a row as a sense and an rhs and has no ranged row: an ==
// row losing one side keeps the other through its sense, and any other
// relaxation frees the row through an infinite rhs.
template <typename Model>
void relax_row(Model & model, model_constraint_t<Model> c, named_sides s) {
    const auto inf = model.infinity();
    if constexpr(has_modifiable_constraint_bounds<Model>) {
        if(s.lower) model.set_constraint_lower_bound(c, -inf);
        if(s.upper) model.set_constraint_upper_bound(c, inf);
    } else {
        const constraint_sense sense = model.get_constraint_sense(c);
        if(sense == constraint_sense::equal) {
            if(s.lower && s.upper) {
                model.set_constraint_sense(c, constraint_sense::less_equal);
                model.set_constraint_rhs(c, inf);
            } else if(s.lower) {
                model.set_constraint_sense(c, constraint_sense::less_equal);
            } else if(s.upper) {
                model.set_constraint_sense(c, constraint_sense::greater_equal);
            }
        } else if(sense == constraint_sense::greater_equal ? s.lower
                                                           : s.upper) {
            model.set_constraint_rhs(
                c, sense == constraint_sense::greater_equal ? -inf : inf);
        }
    }
}

template <typename Model, typename Iis>
void relax_members(Model & model, const Iis & iis) {
    const auto inf = model.infinity();
    for(auto v : model.variables()) {
        const named_sides s = std::visit(sides_named{}, iis.get_status(v));
        if(s.lower) model.set_variable_lower_bound(v, -inf);
        if(s.upper) model.set_variable_upper_bound(v, inf);
    }
    for(auto c : model.constraints())
        relax_row(model, c, std::visit(sides_named{}, iis.get_status(c)));
}
// --8<-- [end:relax-members]

// --8<-- [start:repair-loop]
template <typename Model, typename Report>
iis_outcome relax_until_feasible(Model & model, Report && report) {
    while(true) {
        const auto iis = compute_iis_by_deletion(model);
        // an answer without members leaves nothing to relax: the model is
        // feasible, the run proved nothing, or the background conflicts on
        // its own
        if(iis.num_variable_members() + iis.num_constraint_members() == 0)
            return iis.get_outcome();
        report(iis);
        relax_members(model, iis);
    }
}
// --8<-- [end:repair-loop]

// --8<-- [start:narrow-partial]
template <typename Model, typename Iis>
auto rerun_on_members(Model & model, const Iis & partial,
                      const iis_limits & limits = {}) {
    const auto inf = model.infinity();
    std::vector<std::tuple<model_variable_t<Model>, double, double>> bounds;
    // a row as the model holds it: its two sides, or its sense and rhs
    using row_data = std::conditional_t<has_modifiable_constraint_bounds<Model>,
                                        std::pair<double, double>,
                                        std::pair<constraint_sense, double>>;
    std::vector<std::pair<model_constraint_t<Model>, row_data>> rows;
    // a side at infinity is no candidate, so relaxing every side the partial
    // answer does not name leaves only its members to the filter
    for(auto v : model.variables()) {
        bounds.emplace_back(v, model.get_variable_lower_bound(v),
                            model.get_variable_upper_bound(v));
        const named_sides kept =
            std::visit(sides_named{}, partial.get_status(v));
        if(!kept.lower) model.set_variable_lower_bound(v, -inf);
        if(!kept.upper) model.set_variable_upper_bound(v, inf);
    }
    for(auto c : model.constraints()) {
        if constexpr(has_modifiable_constraint_bounds<Model>)
            rows.emplace_back(c, row_data{model.get_constraint_lower_bound(c),
                                          model.get_constraint_upper_bound(c)});
        else
            rows.emplace_back(c, row_data{model.get_constraint_sense(c),
                                          model.get_constraint_rhs(c)});
        const named_sides kept =
            std::visit(sides_named{}, partial.get_status(c));
        relax_row(model, c, {.lower = !kept.lower, .upper = !kept.upper});
    }
    auto restore = [&] {
        for(const auto & [v, lower, upper] : bounds) {
            model.set_variable_lower_bound(v, lower);
            model.set_variable_upper_bound(v, upper);
        }
        for(const auto & [c, data] : rows) {
            if constexpr(has_modifiable_constraint_bounds<Model>) {
                model.set_constraint_lower_bound(c, data.first);
                model.set_constraint_upper_bound(c, data.second);
            } else {
                model.set_constraint_sense(c, data.first);
                model.set_constraint_rhs(c, data.second);
            }
        }
    };
    try {
        auto narrowed = compute_iis_by_deletion(model, limits);
        restore();
        return narrowed;
    } catch(...) {
        restore();
        throw;
    }
}
// --8<-- [end:narrow-partial]

template <typename Model>
struct repair_run {
    iis_outcome last;
    int rounds;
    model_status_t<Model> after_repair;
};

// The workshop again, with one team per product instead of shared labour:
// the chairs and the desks each hold a conflict of their own.
template <typename lp_type>
repair_run<lp_type> teams_workshop() {
    // --8<-- [start:teams-model]
    lp_type model;
    auto chairs = model.add_variable();
    auto tables = model.add_variable();
    auto desks = model.add_variable();
    auto team_chairs = model.add_constraint(chairs <= 8);  // 1 hour a chair
    auto team_tables = model.add_constraint(3 * tables <= 20);
    auto team_desks = model.add_constraint(2 * desks <= 3);
    auto order_chairs = model.add_constraint(chairs == 10);
    auto order_tables = model.add_constraint(tables == 6);
    auto order_desks = model.add_constraint(desks == 2);
    // --8<-- [end:teams-model]

    // --8<-- [start:teams-repair]
    int round = 0;
    const iis_outcome last = relax_until_feasible(model, [&](const auto & iis) {
        std::cout << "conflict " << ++round << '\n';
        print_member("team chairs", iis.get_status(team_chairs));
        print_member("order chairs", iis.get_status(order_chairs));
        print_member("team tables", iis.get_status(team_tables));
        print_member("order tables", iis.get_status(order_tables));
        print_member("team desks", iis.get_status(team_desks));
        print_member("order desks", iis.get_status(order_desks));
    });
    model.solve();  // optimal
    // --8<-- [end:teams-repair]
    return {last, round, model.get_status()};
}

template <typename milp_type>
auto truck_loading() {
    // --8<-- [start:trucks]
    milp_type model;
    auto trucks =
        model.add_integer_variable({.lower_bound = 0, .upper_bound = 10});
    auto load = model.add_constraint(3 * trucks == 10);  // tonnes, full trucks
    const auto iis = compute_iis_by_deletion(model);
    // --8<-- [end:trucks]
    return std::pair{iis.get_outcome(),
                     std::pair{iis.get_status(trucks), iis.get_status(load)}};
}

}  // namespace infeasibility_page

namespace {

using namespace infeasibility_page;

// The page includes the same file under the example's code.
std::string page_output(const char * name) {
    std::ifstream file(std::string(MIPPP_DOC_SNIPPETS_DIR "/") + name);
    // std::string(istreambuf_iterator...) draws a false -Wnull-dereference
    // from gcc 15
    std::ostringstream text;
    text << file.rdbuf();
    return text.str();
}

// The page's code prints to std::cout, which a test reads back through this.
class cout_capture {
public:
    cout_capture() : previous(std::cout.rdbuf(text.rdbuf())) {}
    ~cout_capture() { std::cout.rdbuf(previous); }
    cout_capture(const cout_capture &) = delete;
    cout_capture & operator=(const cout_capture &) = delete;
    std::string str() const { return text.str(); }

private:
    std::ostringstream text;
    std::streambuf * previous;
};

constexpr solver_version highs_native_iis_floor{1, 14, 0};

struct infeasibility_page_highs_lp : model_test<highs_api, highs_lp> {
    static void SetUpTestSuite() { construct_api("HIGHS"); }
};
struct infeasibility_page_highs_milp : model_test<highs_api, highs_milp> {
    static void SetUpTestSuite() { construct_api("HIGHS"); }
};
struct infeasibility_page_clp_lp : model_test<clp_api, clp_lp> {
    static void SetUpTestSuite() { construct_api("CLP"); }
};
// A model without row-bound setters, as Gurobi's, which CI runs on Clp: the
// repair code takes its sense-and-rhs branch there.
struct infeasibility_page_dumb_lp : model_test<clp_api, dumb_lp> {
    static void SetUpTestSuite() { construct_api("CLP"); }
};
static_assert(!has_modifiable_constraint_bounds<dumb_lp> &&
              iis_by_deletion_model<dumb_lp>);

struct run_deletion_path {
    template <typename Model, typename Print>
    void operator()(Model & model, Print & print) const {
        deletion_path(model, print);
    }
};
struct run_native_path {
    template <typename Model, typename Print>
    void operator()(Model & model, Print & print) const {
        native_path(model, print);
    }
};
struct run_diagnose {
    template <typename Model, typename Print>
    void operator()(Model & model, Print & print) const {
        diagnose(model, print);
    }
};
struct run_bounded_path {
    std::stop_token stop;
    template <typename Model, typename Print>
    void operator()(Model & model, Print & print) const {
        bounded_path(model, print, stop);
    }
};
// A budget too small for the workshop: the members printed are the last
// subset proven infeasible.
struct run_three_solves {
    iis_outcome * outcome;
    std::optional<iis_reason> * reason;
    template <typename Model, typename Print>
    void operator()(Model & model, Print & print) const {
        const auto iis = compute_iis_by_deletion(model, {.max_solves = 3});
        *outcome = iis.get_outcome();
        *reason = iis.get_reason();
        print(iis);
    }
};
struct run_member_rows {
    std::size_t * num_rows;
    std::size_t * num_variables;
    template <typename Model, typename Print>
    void operator()(Model & model, Print &) const {
        const auto iis = compute_iis_by_deletion(model);
        const auto rows = member_rows(model, iis);
        *num_rows = rows.size();
        *num_variables = iis.num_variable_members();
        EXPECT_EQ(rows.size(), iis.num_constraint_members());
        for(auto c : rows)
            EXPECT_TRUE(is_a<iis_status::member>(iis.get_status(c)));
    }
};

// The side of every entity, through the visitor of Reading the answer.
struct run_side_names {
    std::vector<std::string_view> * variables;
    std::vector<std::string_view> * rows;
    template <typename Model, typename Print>
    void operator()(Model & model, Print &) const {
        const auto iis = compute_iis_by_deletion(model);
        for(auto v : model.variables())
            variables->push_back(side_name(iis.get_status(v)));
        for(auto c : model.constraints())
            rows->push_back(side_name(iis.get_status(c)));
    }
};

struct run_relax_members {
    template <typename Model, typename Print>
    void operator()(Model & model, Print &) const {
        relax_members(model, compute_iis_by_deletion(model));
    }
};

template <typename Model>
std::vector<std::pair<double, double>> all_sides(Model & model) {
    std::vector<std::pair<double, double>> sides;
    for(auto v : model.variables())
        sides.emplace_back(model.get_variable_lower_bound(v),
                           model.get_variable_upper_bound(v));
    for(auto c : model.constraints())
        sides.emplace_back(model.get_constraint_lower_bound(c),
                           model.get_constraint_upper_bound(c));
    return sides;
}

// The three-solve answer, then a run whose only candidates are its members.
struct run_on_the_members {
    std::size_t * candidates;
    std::size_t * solves;
    iis_outcome * outcome;
    template <typename Model, typename Print>
    void operator()(Model & model, Print & print) const {
        const auto partial = compute_iis_by_deletion(model, {.max_solves = 3});
        ASSERT_EQ(partial.get_outcome(), iis_outcome::not_proven_minimal);
        *candidates = 0;
        for(auto v : model.variables()) {
            const auto kept = std::visit(sides_named{}, partial.get_status(v));
            *candidates += kept.lower + kept.upper;
        }
        for(auto c : model.constraints()) {
            const auto kept = std::visit(sides_named{}, partial.get_status(c));
            *candidates += kept.lower + kept.upper;
        }
        const auto before = all_sides(model);
        const std::size_t solves_before = model.solves;
        const auto iis = rerun_on_members(model, partial);
        *solves = model.solves - solves_before;
        *outcome = iis.get_outcome();
        EXPECT_EQ(all_sides(model), before);
        print(iis);
    }
};

// Removing any one member of the IIS from the whole model makes it feasible,
// so every infeasible subsystem holds all of them: the IIS is the only one.
struct run_single_removals {
    // a named template rather than a generic lambda, into which MSVC can miss
    // the using-directives
    template <typename Model, typename Relax, typename Restore>
    static void expect_feasible_without(Model & model, Relax relax,
                                        Restore restore) {
        relax();
        model.solve();
        EXPECT_TRUE(is_a<status::optimal>(model.get_status()));
        restore();
    }
    template <typename Model, typename Print>
    void operator()(Model & model, Print &) const {
        const auto iis = compute_iis_by_deletion(model);
        ASSERT_EQ(iis.get_outcome(), iis_outcome::irreducible);
        const double inf = model.infinity();
        std::size_t num_sides = 0;
        for(auto v : model.variables()) {
            const auto s = iis.get_status(v);
            const double lb = model.get_variable_lower_bound(v);
            const double ub = model.get_variable_upper_bound(v);
            if(is_a<iis_status::member_lower>(s) ||
               is_a<iis_status::member_both>(s)) {
                expect_feasible_without(
                    model, [&] { model.set_variable_lower_bound(v, -inf); },
                    [&] { model.set_variable_lower_bound(v, lb); });
                ++num_sides;
            }
            if(is_a<iis_status::member_upper>(s) ||
               is_a<iis_status::member_both>(s)) {
                expect_feasible_without(
                    model, [&] { model.set_variable_upper_bound(v, inf); },
                    [&] { model.set_variable_upper_bound(v, ub); });
                ++num_sides;
            }
        }
        for(auto c : model.constraints()) {
            const auto s = iis.get_status(c);
            const double lb = model.get_constraint_lower_bound(c);
            const double ub = model.get_constraint_upper_bound(c);
            if(is_a<iis_status::member_lower>(s) ||
               is_a<iis_status::member_both>(s)) {
                expect_feasible_without(
                    model, [&] { model.set_constraint_lower_bound(c, -inf); },
                    [&] { model.set_constraint_lower_bound(c, lb); });
                ++num_sides;
            }
            if(is_a<iis_status::member_upper>(s) ||
               is_a<iis_status::member_both>(s)) {
                expect_feasible_without(
                    model, [&] { model.set_constraint_upper_bound(c, inf); },
                    [&] { model.set_constraint_upper_bound(c, ub); });
                ++num_sides;
            }
        }
        EXPECT_EQ(num_sides, 5u);
        model.reset_status();
    }
};

template <typename Model>
void expect_page_run(const workshop_run<Model> & run) {
    EXPECT_TRUE(is_a<status::infeasible>(run.first_solve));
    EXPECT_TRUE(is<status::unknown>(run.after_analysis));
    EXPECT_TRUE(is_a<status::optimal>(run.after_fix));
    EXPECT_NEAR(run.overtime_after_fix, 2., TEST_EPSILON);
}

}  // namespace

TEST_F(infeasibility_page_highs_lp, workshop_prints_the_page_output) {
    cout_capture out;
    expect_page_run(workshop<highs_lp>(run_deletion_path{}));
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

TEST_F(infeasibility_page_clp_lp, workshop_prints_the_page_output) {
    cout_capture out;
    expect_page_run(workshop<clp_lp>(run_deletion_path{}));
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

TEST_F(infeasibility_page_dumb_lp, workshop_prints_the_page_output) {
    cout_capture out;
    expect_page_run(workshop<dumb_lp>(run_deletion_path{}));
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

// The workshop has a single IIS, so the native routine finds the same one.
TEST_F(infeasibility_page_highs_lp, workshop_native_answer_is_the_same) {
    const auto loaded = api->library_version();
    if(loaded && *loaded < highs_native_iis_floor)
        GTEST_SKIP() << "Highs_getIis needs HiGHS "
                     << to_string(highs_native_iis_floor) << ", "
                     << api->library_path() << " is " << to_string(*loaded);
    cout_capture out;
    expect_page_run(workshop<highs_lp>(run_native_path{}));
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

TEST_F(infeasibility_page_highs_lp, compute_iis_below_the_floor_throws) {
    const auto loaded = api->library_version();
    if(!loaded || *loaded >= highs_native_iis_floor)
        GTEST_SKIP() << "the loaded HiGHS has the native routine";
    cout_capture out;
    EXPECT_THROW(workshop<highs_lp>(run_native_path{}), solver_error);
}

// Native above the floor, the deletion filter below it: the same answer.
TEST_F(infeasibility_page_highs_lp, diagnose_prints_the_page_output) {
    cout_capture out;
    expect_page_run(workshop<highs_lp>(run_diagnose{}));
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

TEST_F(infeasibility_page_clp_lp, diagnose_runs_the_deletion_filter) {
    static_assert(!has_iis<clp_lp> && iis_by_deletion_model<clp_lp>);
    cout_capture out;
    expect_page_run(workshop<clp_lp>(run_diagnose{}));
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

TEST_F(infeasibility_page_highs_lp, bounded_run_completes_on_the_workshop) {
    std::stop_source stop;
    cout_capture out;
    expect_page_run(workshop<highs_lp>(run_bounded_path{stop.get_token()}));
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

// A run stopped before its first solve leaves the status of the solve that
// found the model infeasible, and prints no member.
TEST_F(infeasibility_page_highs_lp, cancelled_run_prints_no_member) {
    std::stop_source stop;
    stop.request_stop();
    cout_capture out;
    const auto run = workshop<highs_lp>(run_bounded_path{stop.get_token()});
    EXPECT_EQ(out.str(), "overtime: 2 hours\n");  // the remedy's line only
    EXPECT_TRUE(is_a<status::infeasible>(run.after_analysis));
    EXPECT_TRUE(is_a<status::optimal>(run.after_fix));
}

TEST_F(infeasibility_page_highs_lp, solve_limit_keeps_a_conflicting_subset) {
    iis_outcome outcome = iis_outcome::irreducible;
    std::optional<iis_reason> reason;
    cout_capture out;
    expect_page_run(workshop<highs_lp>(run_three_solves{&outcome, &reason}));
    EXPECT_EQ(outcome, iis_outcome::not_proven_minimal);
    EXPECT_EQ(reason, iis_reason::solve_limit);
    // every entity of the irreducible answer, among others
    std::istringstream expected(page_output("infeasibility_workshop.txt"));
    for(std::string line; std::getline(expected, line);) {
        const std::string name = line.substr(0, line.find(':') + 1);
        EXPECT_NE(out.str().find(name), std::string::npos) << name;
    }
    EXPECT_NE(out.str(), page_output("infeasibility_workshop.txt"));
}

TEST_F(infeasibility_page_highs_lp, workshop_has_a_single_iis) {
    cout_capture out;
    expect_page_run(workshop<highs_lp>(run_single_removals{}));
}

TEST_F(infeasibility_page_highs_lp, member_rows_lists_the_conflicting_rows) {
    std::size_t num_rows = 0;
    std::size_t num_variables = 0;
    cout_capture out;
    expect_page_run(
        workshop<highs_lp>(run_member_rows{&num_rows, &num_variables}));
    EXPECT_EQ(num_rows, 4u);       // labour and the three orders
    EXPECT_EQ(num_variables, 1u);  // overtime
}

TEST_F(infeasibility_page_highs_lp, side_name_reads_the_side_of_each_member) {
    std::vector<std::string_view> variables;
    std::vector<std::string_view> rows;
    cout_capture out;
    expect_page_run(workshop<highs_lp>(run_side_names{&variables, &rows}));
    using names = std::vector<std::string_view>;
    // chairs, tables, desks and overtime
    EXPECT_EQ(variables, (names{"absent", "absent", "absent", "upper side"}));
    // wood, labour and the three orders
    EXPECT_EQ(rows, (names{"absent", "upper side", "lower side", "lower side",
                           "lower side"}));
}

// The members of the partial answer hold the whole IIS, found again with one
// solve per member side plus one, and every side is written back.
TEST_F(infeasibility_page_highs_lp,
       rerun_on_members_completes_a_partial_answer) {
    std::size_t candidates = 0;
    std::size_t solves = 0;
    iis_outcome outcome = iis_outcome::undetermined;
    cout_capture out;
    expect_page_run(workshop<iis_trial_probe<highs_lp>>(
        run_on_the_members{&candidates, &solves, &outcome}));
    EXPECT_EQ(outcome, iis_outcome::irreducible);
    EXPECT_EQ(candidates, 11u);  // of the workshop's 13 finite sides
    EXPECT_EQ(solves, candidates + 1);
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

TEST_F(infeasibility_page_dumb_lp,
       rerun_on_members_completes_a_partial_answer) {
    std::size_t candidates = 0;
    std::size_t solves = 0;
    iis_outcome outcome = iis_outcome::undetermined;
    cout_capture out;
    expect_page_run(workshop<iis_trial_probe<dumb_lp>>(
        run_on_the_members{&candidates, &solves, &outcome}));
    EXPECT_EQ(outcome, iis_outcome::irreducible);
    EXPECT_EQ(candidates, 11u);
    EXPECT_EQ(solves, candidates + 1);
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

// Relaxing the whole conflict frees the orders, so the fix that follows needs
// no overtime at all.
TEST_F(infeasibility_page_highs_lp, relaxing_the_members_repairs_the_workshop) {
    cout_capture out;
    const auto run = workshop<highs_lp>(run_relax_members{});
    EXPECT_TRUE(is<status::unknown>(run.after_analysis));
    EXPECT_TRUE(is_a<status::optimal>(run.after_fix));
    EXPECT_NEAR(run.overtime_after_fix, 0., TEST_EPSILON);
}

TEST_F(infeasibility_page_dumb_lp, relaxing_the_members_repairs_the_workshop) {
    cout_capture out;
    const auto run = workshop<dumb_lp>(run_relax_members{});
    EXPECT_TRUE(is<status::unknown>(run.after_analysis));
    EXPECT_TRUE(is_a<status::optimal>(run.after_fix));
    EXPECT_NEAR(run.overtime_after_fix, 0., TEST_EPSILON);
}

TEST_F(infeasibility_page_highs_lp, repair_loop_explains_one_conflict_a_round) {
    cout_capture out;
    const auto run = teams_workshop<highs_lp>();
    EXPECT_EQ(run.last, iis_outcome::feasible);
    EXPECT_EQ(run.rounds, 2);
    EXPECT_TRUE(is_a<status::optimal>(run.after_repair));
    EXPECT_EQ(out.str(), page_output("infeasibility_repair.txt"));
}

TEST_F(infeasibility_page_clp_lp, repair_loop_explains_one_conflict_a_round) {
    cout_capture out;
    const auto run = teams_workshop<clp_lp>();
    EXPECT_EQ(run.last, iis_outcome::feasible);
    EXPECT_EQ(run.rounds, 2);
    EXPECT_TRUE(is_a<status::optimal>(run.after_repair));
    EXPECT_EQ(out.str(), page_output("infeasibility_repair.txt"));
}

TEST_F(infeasibility_page_dumb_lp, repair_loop_explains_one_conflict_a_round) {
    cout_capture out;
    const auto run = teams_workshop<dumb_lp>();
    EXPECT_EQ(run.last, iis_outcome::feasible);
    EXPECT_EQ(run.rounds, 2);
    EXPECT_TRUE(is_a<status::optimal>(run.after_repair));
    EXPECT_EQ(out.str(), page_output("infeasibility_repair.txt"));
}

// Integrality is background: only the row's two sides are members, and the
// same row over a continuous variable is feasible.
TEST_F(infeasibility_page_highs_milp, trucks_row_needs_both_sides) {
    const auto [outcome, statuses] = truck_loading<highs_milp>();
    EXPECT_EQ(outcome, iis_outcome::irreducible);
    EXPECT_TRUE(is<iis_status::absent>(statuses.first));
    EXPECT_TRUE(is<iis_status::member_both>(statuses.second));
    EXPECT_EQ(side_name(statuses.second), "both sides");
}

TEST_F(infeasibility_page_highs_lp, trucks_row_is_feasible_on_an_lp) {
    using namespace operators;
    highs_lp model;
    auto trucks = model.add_variable({.lower_bound = 0, .upper_bound = 10});
    model.add_constraint(3 * trucks == 10);
    EXPECT_EQ(compute_iis_by_deletion(model).get_outcome(),
              iis_outcome::feasible);
}
