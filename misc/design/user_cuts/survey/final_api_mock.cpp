// Shape check of the proposed user-cut concepts (user_cuts_plan.md:75-104)
// against generic user code. -fsyntax-only; nothing runs.
#include <cstddef>
#include <functional>
#include <mutex>
#include <optional>
#include <set>
#include <vector>

#include "mippp/solvers/gurobi/all.hpp"

namespace proposed {
using namespace mippp;
// Copied from user_cuts_plan.md:75-104, in another namespace because
// model_concepts.hpp:831 already defines the old has_node_relaxation_callback.
template <typename T>
using node_relaxation_callback_handle_t =
    typename T::node_relaxation_callback_handle;

template <typename T>
concept has_node_relaxation_callback =
    requires(T & model, node_relaxation_callback_handle_t<T> & handle) {
        { model.set_node_relaxation_callback(
                [](node_relaxation_callback_handle_t<T> &) {}) };
        { handle.get_solution_value() } -> std::same_as<model_scalar_t<T>>;
        { handle.get_solution() }
                -> input_mapping_of<model_variable_t<T>, model_scalar_t<T>>;
    };

template <typename T, typename M = T>
concept has_user_cuts = requires(T & handle) {
    { handle.add_user_cut(archetype::linear_constraint<M>()) };
    { handle.add_user_cut(distinct_variables,
                          archetype::linear_constraint<M>()) };
};
}  // namespace proposed

namespace user {
using namespace mippp;
using namespace mippp::operators;
// using-declarations in this namespace hide mippp's old concept of the same name
using proposed::has_node_relaxation_callback;
using proposed::has_user_cuts;
using proposed::node_relaxation_callback_handle_t;

// A real model, so that model_variable_t deduces as on a backend, plus the
// proposed member types.
struct mock_model : gurobi_milp {
    using variable_t = model_variable_t<gurobi_milp>;
    class node_relaxation_callback_handle {
    public:
        struct point {
            double operator[](const variable_t &) const { return 0.0; }
        };
        double get_solution_value() { return 0.0; }
        point get_solution() { return {}; }
        template <linear_constraint LC>
        void add_user_cut(LC &&) {}
        template <linear_constraint LC>
        void add_user_cut(distinct_variables_t, LC &&) {}
    };
    std::function<void(node_relaxation_callback_handle &)> relaxation_callback;
    template <typename F>
    void set_node_relaxation_callback(F && f) {
        relaxation_callback = std::forward<F>(f);
    }
};

// A handle without the distinct_variables overload.
struct half_handle_model : gurobi_milp {
    using variable_t = model_variable_t<gurobi_milp>;
    struct node_relaxation_callback_handle {
        struct point {
            double operator[](const variable_t &) const { return 0.0; }
        };
        double get_solution_value() { return 0.0; }
        point get_solution() { return {}; }
        template <linear_constraint LC>
        void add_user_cut(LC &&) {}
    };
    template <typename F>
    void set_node_relaxation_callback(F &&) {}
};

using relax_h = node_relaxation_callback_handle_t<mock_model>;
using cand_h = candidate_solution_callback_handle_t<mock_model>;

static_assert(milp_model<mock_model>);
static_assert(has_node_relaxation_callback<mock_model>);
static_assert(has_user_cuts<relax_h, mock_model>);
static_assert(!has_user_cuts<cand_h, mock_model>);
static_assert(!has_lazy_constraints<relax_h, mock_model>);
static_assert(!has_user_cuts<mock_model>);  // M = T: the model has none
static_assert(!has_enumerable_variables<relax_h, mock_model>);
static_assert(!has_enumerable_constraints<relax_h, mock_model>);
static_assert(!has_user_cuts<
              node_relaxation_callback_handle_t<half_handle_model>,
              half_handle_model>);
static_assert(!has_node_relaxation_callback<gurobi_milp>);
// The shipped concept still holds on the mock, with or without reads.
static_assert(mippp::has_node_relaxation_callback<mock_model>);
static_assert(mippp::has_node_relaxation_callback<half_handle_model>);

///////////////////////////////////////////////////////////////////////////////
// 1. DFJ branch-and-cut, lazy and user cuts sharing one separator.
///////////////////////////////////////////////////////////////////////////////
struct digraph {
    std::size_t n;
    std::vector<std::pair<std::size_t, std::size_t>> arcs;
};
// stub: the vertex sets S with x(delta+(S)) < 1
template <typename Point, typename X>
std::vector<std::vector<std::size_t>> violated_subtours(const digraph &,
                                                        const Point &, X &&) {
    return {};
}

template <typename Model>
    requires milp_model<Model> && has_candidate_solution_callback<Model> &&
             has_lazy_constraints<candidate_solution_callback_handle_t<Model>,
                                  Model> &&
             has_node_relaxation_callback<Model> &&
             has_user_cuts<node_relaxation_callback_handle_t<Model>, Model>
double solve_atsp(Model & model, const digraph & g,
                  const std::vector<double> & cost) {
    auto X = model.add_binary_variables(g.arcs.size());
    const auto arc_ids = std::views::iota(std::size_t{0}, g.arcs.size());
    model.set_objective(xsum(arc_ids, [&](auto a) { return cost[a] * X(a); }));
    // degree rows only: the subtours are excluded by the lazy family
    const auto leaving = [&](const std::vector<std::size_t> & S) {
        std::vector<std::size_t> out;
        for(auto a : arc_ids) {
            const bool in_s = std::ranges::contains(S, g.arcs[a].first);
            const bool head_in_s = std::ranges::contains(S, g.arcs[a].second);
            if(in_s && !head_in_s) out.push_back(a);
        }
        return out;
    };
    // One separator, both handles: the point reads the same way.
    auto separate = [&](auto & handle) {
        auto x = handle.get_solution();
        for(const auto & S : violated_subtours(g, x, X)) {
            auto cut = xsum(leaving(S), X) >= 1;
            using H = std::remove_cvref_t<decltype(handle)>;
            if constexpr(has_user_cuts<H, Model>)
                handle.add_user_cut(cut);
            else
                handle.add_lazy_constraint(cut);
        }
    };
    model.set_candidate_solution_callback(separate);
    model.set_node_relaxation_callback(separate);
    // the shorthand, checked type first
    model.set_node_relaxation_callback(
        [&](has_user_cuts<Model> auto & handle) { separate(handle); });
    model.solve();
    return model.get_solution_value();
}

///////////////////////////////////////////////////////////////////////////////
// 2. Benders: eta >= Q(xbar) + g'(x - xbar) on a fractional xbar.
///////////////////////////////////////////////////////////////////////////////
struct subproblem_answer {
    double value;
    std::vector<double> subgradient;
};
subproblem_answer solve_subproblem(const std::vector<double> &) { return {}; }

template <typename Model>
    requires milp_model<Model> && has_node_relaxation_callback<Model> &&
             has_user_cuts<node_relaxation_callback_handle_t<Model>, Model>
void benders(Model & model, std::size_t n) {
    auto x = model.add_binary_variables(n);
    auto eta = model.add_variable();
    const auto ids = std::views::iota(std::size_t{0}, n);
    model.set_objective(xsum(ids, [&](auto i) { return 1.0 * x(i); }) + eta);
    model.set_node_relaxation_callback([&, x, eta](auto & handle) {
        auto point = handle.get_solution();
        std::vector<double> xbar(n);
        for(auto i : ids) xbar[i] = point[x(i)];
        const auto [q, g] = solve_subproblem(xbar);
        double gx = 0;
        for(auto i : ids) gx += g[i] * xbar[i];
        if(point[eta] >= q - 1e-9) return;
        // distinct: eta is not among the x
        handle.add_user_cut(
            distinct_variables,
            eta - xsum(ids, [&](auto i) { return g[i] * x(i); }) >= q - gx);
        // the node's LP value, for a user's own tailing-off test
        [[maybe_unused]] const double z = handle.get_solution_value();
    });
}

///////////////////////////////////////////////////////////////////////////////
// 3. A separator with shared state, used by both callbacks.
///////////////////////////////////////////////////////////////////////////////
template <typename Model>
struct cover_separator {
    using variable = model_variable_t<Model>;
    std::vector<variable> items;
    std::vector<double> weights;
    double capacity;
    std::set<std::vector<std::size_t>> offered;  // shared across calls
    std::mutex mutex;  // the user's own: both callbacks may touch `offered`
    std::size_t calls = 0;

    template <typename Handle>
    void operator()(Handle & handle) {
        auto x = handle.get_solution();
        std::vector<std::size_t> cover;  // stub: greedy on 1 - x
        double w = 0;
        for(std::size_t i = 0; i < items.size() && w <= capacity; ++i)
            if(x[items[i]] > 1e-6) {
                cover.push_back(i);
                w += weights[i];
            }
        if(w <= capacity) return;
        const std::scoped_lock lock(mutex);
        ++calls;
        offered.insert(cover);
        auto cut = xsum(cover, [&](auto i) { return 1.0 * items[i]; }) <=
                   static_cast<double>(cover.size()) - 1;
        if constexpr(has_user_cuts<Handle, Model>)
            handle.add_user_cut(cut);
        else
            handle.add_lazy_constraint(cut);
    }
};

template <typename Model>
    requires has_candidate_solution_callback<Model> &&
             has_lazy_constraints<candidate_solution_callback_handle_t<Model>,
                                  Model> &&
             has_node_relaxation_callback<Model> &&
             has_user_cuts<node_relaxation_callback_handle_t<Model>, Model>
void attach_covers(Model & model, cover_separator<Model> & sep) {
    model.set_candidate_solution_callback([&](auto & h) { sep(h); });
    model.set_node_relaxation_callback([&](auto & h) { sep(h); });
}

///////////////////////////////////////////////////////////////////////////////
// 4. Generic code over either handle: the reads have the same names.
///////////////////////////////////////////////////////////////////////////////
template <typename Handle, typename Model>
double objective_gap_of_point(Handle & handle, double incumbent) {
    return incumbent - handle.get_solution_value();
}

void instantiate(mock_model & m, const digraph & g) {
    solve_atsp(m, g, {});
    benders(m, 3);
    cover_separator<mock_model> sep{{}, {}, 1.0, {}, {}, 0};
    attach_covers(m, sep);
    relax_h * h = nullptr;
    (void)objective_gap_of_point<relax_h, mock_model>(*h, 0);
}

#ifdef EXPECT_FAIL_REAL_BACKEND
// Fails: no backend has the reads, so the requires-clause rejects it.
void on_gurobi(gurobi_milp & m, const digraph & g) { solve_atsp(m, g, {}); }
#endif
#ifdef EXPECT_FAIL_SHORTHAND_ON_CANDIDATE
void wrong_handle(mock_model & m) {
    m.set_candidate_solution_callback(
        [&](has_user_cuts<mock_model> auto & h) { (void)h; });
}
#endif

}  // namespace user
int main() {}
