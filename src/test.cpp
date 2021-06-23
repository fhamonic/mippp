#include <iostream>
#include <filesystem>
#include <fstream>

#include <boost/iterator/zip_iterator.hpp>

#include "solver_builder.hpp"

using namespace SolverBuilder_Utils;

std::ostream& operator<<(std::ostream& os, const LinearExpression & linear_expr) {
    std::for_each(
        boost::make_zip_iterator(boost::make_tuple(
            linear_expr.getCoefficients().begin(),
            linear_expr.getIndices().begin()
        )),
        boost::make_zip_iterator(boost::make_tuple(
            linear_expr.getCoefficients().end(),
            linear_expr.getIndices().end()
        )),
        [&os](const auto & t) {
            os << t.template get<0>() << "*x" << t.template get<1>() << " + ";
        }
    );
    return os << linear_expr.getConstant();
}

std::ostream& operator<<(std::ostream& os, const QuadraticExpression & quad_expr) {
    std::for_each(
        boost::make_zip_iterator(boost::make_tuple(
            quad_expr.getQuadCoefficients().begin(),
            quad_expr.getQuadIndices1().begin(),
            quad_expr.getQuadIndices2().begin()
        )),
        boost::make_zip_iterator(boost::make_tuple(
            quad_expr.getQuadCoefficients().end(),
            quad_expr.getQuadIndices1().end(),
            quad_expr.getQuadIndices2().end()
        )),
        [&os](const auto & t) {
            os << t.template get<0>() << "*x" << t.template get<1>() << "x" << t.template get<2>() << " + ";
        }
    );
    return os << quad_expr.getLineraExpression();
}

int main() {
    quadratic_ineq_constraint_lhs_easy_init constraint;

    std::cout << constraint(23.0)(0, 2.0)(1, 1.0)(1, 2, 3.0).less()(4, 3.0)(1.0).take_data().quadratic_expression << std::endl;

    return EXIT_SUCCESS;
}