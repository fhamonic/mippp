#include <cstdlib>
#include <exception>
#include <iostream>
#include <string_view>

#include "mippp/solvers/highs/all.hpp"

int main(int argc, char ** argv) {
    // Implement the discovery protocol to exercise GoogleTest.cmake without
    // requiring the GoogleTest library for this build-system regression test.
    if(argc > 1 && std::string_view(argv[1]) == "--gtest_list_tests") {
        std::cout << "HiGHS.\n  Runtime\n";
        return 0;
    }
    const char * required = std::getenv("MIPPP_REQUIRED_SOLVERS");
#ifdef EXPECT_HIGHS
    if(!required || std::string_view(required) != "CLP;HIGHS") return 1;
    const char * library = std::getenv("MIPPP_HIGHS_LIBRARY");
    if(!library || !*library) return 2;
    try {
        mippp::highs_lp model;
        model.set_maximization();
        model.add_variable(
            {.obj_coef = 1., .lower_bound = 0., .upper_bound = 2.});
        model.solve();
        if(!mippp::is<mippp::status::optimal>(model.get_status())) return 3;
        if(model.get_solution_value() != 2.) return 4;
        std::cout << "Loaded HiGHS from " << library << '\n';
    } catch(const std::exception & error) {
        std::cerr << error.what() << '\n';
        return 5;
    }
#else
    if(!required || std::string_view(required) != "CLP") return 6;
#endif
    return 0;
}
