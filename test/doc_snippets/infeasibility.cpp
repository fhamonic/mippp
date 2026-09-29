// The code of docs/solving/infeasibility.md. The page shows the sections
// between the --8<-- markers, so they stay plain user code; the tests below
// them check what the page says about their output and outcome.

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <fstream>
#include <map>
#include <optional>
#include <ostream>
#include <sstream>
#include <stop_token>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/solvers/clp/all.hpp"
#include "mippp/solvers/highs/all.hpp"
#include "mippp/utility/iis_by_deletion.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

namespace infeasibility_page {

// here rather than in each function: on MSVC a block-scope using-directive
// does not reach into a generic lambda
using namespace mippp::operators;

// --8<-- [start:print-member]
// The status variant of a path lists only the tags it reports, so the side is
// read by overload: the most derived tag wins, and a routine that cannot name
// the side of a row reports plain member.
struct member_side {
    std::string_view operator()(iis_status::absent) const { return "absent"; }
    std::string_view operator()(iis_status::member) const {
        return "side not named";
    }
    std::string_view operator()(iis_status::member_lower) const {
        return "lower side";
    }
    std::string_view operator()(iis_status::member_upper) const {
        return "upper side";
    }
    std::string_view operator()(iis_status::member_both) const {
        return "both sides";
    }
};

template <typename Status>
void print_member(std::ostream & out, std::string_view name,
                  const Status & status) {
    if(is_a<iis_status::member>(status))
        out << name << ": " << std::visit(member_side{}, status) << '\n';
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

// The page's example as one program: Analyze stands for the line of the page
// that computes the IIS, so that each path runs on the same model.
template <typename lp_type, typename Analyze>
workshop_run<lp_type> workshop(std::ostream & out, Analyze && analyze) {
    // --8<-- [start:workshop-model]
    const std::vector<std::string> products = {"chairs", "tables", "desks"};
    const std::map<std::string, double> boards = {
        {"chairs", 2}, {"tables", 5}, {"desks", 4}};
    const std::map<std::string, double> hours = {
        {"chairs", 1}, {"tables", 3}, {"desks", 2}};
    const std::map<std::string, double> ordered = {
        {"chairs", 10}, {"tables", 6}, {"desks", 2}};

    lp_type model;
    auto make = model.add_variables(products);  // make(p) >= 0
    auto overtime =
        model.add_variable({.obj_coef = 1, .lower_bound = 0, .upper_bound = 1});
    auto boards_used = [&](const std::string & p) {
        return boards.at(p) * make(p);
    };
    auto hours_used = [&](const std::string & p) {
        return hours.at(p) * make(p);
    };
    auto wood = model.add_constraint(xsum(products, boards_used) <= 60);
    auto labour =
        model.add_constraint(xsum(products, hours_used) - overtime <= 30);
    auto orders = model.add_constraints(products, [&](const std::string & p) {
        return make(p) == ordered.at(p);
    });

    model.solve();  // infeasible
    // --8<-- [end:workshop-model]
    workshop_run<lp_type> run{model.get_status(), {}, {}};

    // --8<-- [start:workshop-report]
    auto print_conflict = [&](const auto & iis) {
        print_member(out, "wood", iis.get_status(wood));
        print_member(out, "labour", iis.get_status(labour));
        for(const std::string & p : products) {
            print_member(out, "order " + p, iis.get_status(orders(p)));
            print_member(out, "make " + p, iis.get_status(make(p)));
        }
        print_member(out, "overtime", iis.get_status(overtime));
    };
    // --8<-- [end:workshop-report]
    analyze(model, print_conflict);
    run.after_analysis = model.get_status();

    // --8<-- [start:workshop-fix]
    model.set_variable_upper_bound(overtime, 2);
    model.solve();  // optimal, with 2 hours of overtime
    // --8<-- [end:workshop-fix]
    run.after_fix = model.get_status();
    if(status::solution_available(run.after_fix))
        run.overtime_after_fix = model.get_solution()[overtime];
    return run;
}

template <typename Model, typename Print>
void deletion_path(Model & model, Print & print_conflict) {
    // --8<-- [start:workshop-deletion]
    print_conflict(compute_iis_by_deletion(model));
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
    std::ostringstream out;
    expect_page_run(workshop<highs_lp>(out, run_deletion_path{}));
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

TEST_F(infeasibility_page_clp_lp, workshop_prints_the_page_output) {
    std::ostringstream out;
    expect_page_run(workshop<clp_lp>(out, run_deletion_path{}));
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

// The workshop has a single IIS, so the native routine finds the same one.
TEST_F(infeasibility_page_highs_lp, workshop_native_answer_is_the_same) {
    const auto loaded = api->library_version();
    if(loaded && *loaded < highs_native_iis_floor)
        GTEST_SKIP() << "Highs_getIis needs HiGHS "
                     << to_string(highs_native_iis_floor) << ", "
                     << api->library_path() << " is " << to_string(*loaded);
    std::ostringstream out;
    expect_page_run(workshop<highs_lp>(out, run_native_path{}));
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

TEST_F(infeasibility_page_highs_lp, compute_iis_below_the_floor_throws) {
    const auto loaded = api->library_version();
    if(!loaded || *loaded >= highs_native_iis_floor)
        GTEST_SKIP() << "the loaded HiGHS has the native routine";
    std::ostringstream out;
    EXPECT_THROW(workshop<highs_lp>(out, run_native_path{}), solver_error);
}

// Native above the floor, the deletion filter below it: the same answer.
TEST_F(infeasibility_page_highs_lp, diagnose_prints_the_page_output) {
    std::ostringstream out;
    expect_page_run(workshop<highs_lp>(out, run_diagnose{}));
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

TEST_F(infeasibility_page_clp_lp, diagnose_runs_the_deletion_filter) {
    static_assert(!has_iis<clp_lp> && iis_by_deletion_model<clp_lp>);
    std::ostringstream out;
    expect_page_run(workshop<clp_lp>(out, run_diagnose{}));
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

TEST_F(infeasibility_page_highs_lp, bounded_run_completes_on_the_workshop) {
    std::stop_source stop;
    std::ostringstream out;
    expect_page_run(
        workshop<highs_lp>(out, run_bounded_path{stop.get_token()}));
    EXPECT_EQ(out.str(), page_output("infeasibility_workshop.txt"));
}

// A run stopped before its first solve leaves the status of the solve that
// found the model infeasible.
TEST_F(infeasibility_page_highs_lp, cancelled_run_prints_nothing) {
    std::stop_source stop;
    stop.request_stop();
    std::ostringstream out;
    const auto run =
        workshop<highs_lp>(out, run_bounded_path{stop.get_token()});
    EXPECT_EQ(out.str(), "");
    EXPECT_TRUE(is_a<status::infeasible>(run.after_analysis));
    EXPECT_TRUE(is_a<status::optimal>(run.after_fix));
}

TEST_F(infeasibility_page_highs_lp, solve_limit_keeps_a_conflicting_subset) {
    iis_outcome outcome = iis_outcome::irreducible;
    std::optional<iis_reason> reason;
    std::ostringstream out;
    expect_page_run(
        workshop<highs_lp>(out, run_three_solves{&outcome, &reason}));
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
    std::ostringstream out;
    expect_page_run(workshop<highs_lp>(out, run_single_removals{}));
}

TEST_F(infeasibility_page_highs_lp, member_rows_lists_the_conflicting_rows) {
    std::size_t num_rows = 0;
    std::size_t num_variables = 0;
    std::ostringstream out;
    expect_page_run(
        workshop<highs_lp>(out, run_member_rows{&num_rows, &num_variables}));
    EXPECT_EQ(num_rows, 4u);       // labour and the three orders
    EXPECT_EQ(num_variables, 1u);  // overtime
}

// Integrality is background: only the row's two sides are members, and the
// same row over a continuous variable is feasible.
TEST_F(infeasibility_page_highs_milp, trucks_row_needs_both_sides) {
    const auto [outcome, statuses] = truck_loading<highs_milp>();
    EXPECT_EQ(outcome, iis_outcome::irreducible);
    EXPECT_TRUE(is<iis_status::absent>(statuses.first));
    EXPECT_TRUE(is<iis_status::member_both>(statuses.second));
}

TEST_F(infeasibility_page_highs_lp, trucks_row_is_feasible_on_an_lp) {
    using namespace operators;
    highs_lp model;
    auto trucks = model.add_variable({.lower_bound = 0, .upper_bound = 10});
    model.add_constraint(3 * trucks == 10);
    EXPECT_EQ(compute_iis_by_deletion(model).get_outcome(),
              iis_outcome::feasible);
}
