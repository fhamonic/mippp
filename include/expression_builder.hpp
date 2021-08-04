/**
 * @file solver_builder.hpp
 * @author François Hamonic (francois.hamonic@gmail.com)
 * @brief OSI_Builder class declaration
 * @version 0.1
 * @date 2021-08-4
 * 
 * @copyright Copyright (c) 2020
 */
#ifndef EXPRESSION_BUILDER_HPP
#define EXPRESSION_BUILDER_HPP

#include <numeric>
#include <vector>

#include "expressions/linear_expression.hpp"
#include "expressions/quadratic_expression.hpp"

#include "constraints/linear_constraints.hpp"
#include "constraints/quadratic_constraints.hpp"

namespace Builder {
    enum OptimizationSense { MIN=-1, MAX=1 };
    constexpr double INFTY = std::numeric_limits<double>::max();

    
    class linear_ineq_constraint_rhs_easy_init {
    private:
        LinearIneqConstraint constraint_data;
    public:
        linear_ineq_constraint_rhs_easy_init(LinearIneqConstraint&& data) 
                : constraint_data(std::move(data)) {}
        linear_ineq_constraint_rhs_easy_init& operator()(double c) { 
            constraint_data.linear_expression.add(-c);
            return *this;
        }
        linear_ineq_constraint_rhs_easy_init& operator()(int id
                , double coef=1) {
            constraint_data.linear_expression.add(id, -coef);
            return *this;
        }    
    };

    class linear_ineq_constraint_lhs_easy_init {
    private:
        LinearIneqConstraint constraint_data;
    public:
        linear_ineq_constraint_lhs_easy_init() {}
        linear_ineq_constraint_lhs_easy_init& operator()(double c) { 
            constraint_data.linear_expression.add(c);
            return *this;
        }
        linear_ineq_constraint_lhs_easy_init& operator()(int id
                , double coef=1) { 
            constraint_data.linear_expression.add(id, coef);
            return *this;
        }

        linear_ineq_constraint_rhs_easy_init less() { 
            constraint_data.sense = LESS;
            return linear_ineq_constraint_rhs_easy_init(
                    std::move(constraint_data));
        }
        linear_ineq_constraint_rhs_easy_init equal() {
            constraint_data.sense = EQUAL;
            return linear_ineq_constraint_rhs_easy_init(
                    std::move(constraint_data));
        }
        linear_ineq_constraint_rhs_easy_init greater() { 
            constraint_data.sense = GREATER;
            return linear_ineq_constraint_rhs_easy_init(
                    std::move(constraint_data));
        }
    };




    class linear_range_constraint_lhs_easy_init {
    private:
        LinearRangeConstraint constraint_data;
    public:
        linear_range_constraint_lhs_easy_init() {}
        linear_range_constraint_lhs_easy_init& lower(double c) { 
            constraint_data.lower_bound = c;
            return *this;
        }
        linear_range_constraint_lhs_easy_init& upper(double c) { 
            constraint_data.upper_bound = c;
            return *this;
        }
        linear_range_constraint_lhs_easy_init& operator()(int id,
                double coef=1) { 
            constraint_data.linear_expression.add(id, coef);
            return *this;
        }

        LinearRangeConstraint&& take_data()
            { return std::move(constraint_data); }
    };



    
    class quadratic_ineq_constraint_rhs_easy_init {
    private:
        QuadraticIneqConstraint constraint_data;
    public:
        quadratic_ineq_constraint_rhs_easy_init(
                QuadraticIneqConstraint&& data) 
                : constraint_data(std::move(data)) {}
        quadratic_ineq_constraint_rhs_easy_init& operator()(double c) {
            constraint_data.quadratic_expression.add(-c);
            return *this;
        }
        quadratic_ineq_constraint_rhs_easy_init& operator()(int id,
                double coef=1) { 
            constraint_data.quadratic_expression.add(id, -coef);
            return *this;
        }
        quadratic_ineq_constraint_rhs_easy_init& operator()(int id_1, int id_2,
                double coef=1) { 
            constraint_data.quadratic_expression.add(id_1, id_2, -coef);
            return *this;
        }

        QuadraticIneqConstraint&& take_data()
            { return std::move(constraint_data); }
    };

    class quadratic_ineq_constraint_lhs_easy_init {
    private:
        QuadraticIneqConstraint constraint_data;
    public:
        quadratic_ineq_constraint_lhs_easy_init() {}
        quadratic_ineq_constraint_lhs_easy_init& operator()(double c) {
            constraint_data.quadratic_expression.add(c);
            return *this;
        }
        quadratic_ineq_constraint_lhs_easy_init& operator()(int id, double coef=1) { 
            constraint_data.quadratic_expression.add(id, coef);
            return *this;
        }
        quadratic_ineq_constraint_lhs_easy_init& operator()(int id_1, int id_2, double coef=1) { 
            constraint_data.quadratic_expression.add(id_1, id_2, coef);
            return *this;
        }

        quadratic_ineq_constraint_rhs_easy_init less() { 
            constraint_data.sense = LESS;
            return quadratic_ineq_constraint_rhs_easy_init(std::move(constraint_data));
        }
        quadratic_ineq_constraint_rhs_easy_init equal() {
            constraint_data.sense = EQUAL;
            return quadratic_ineq_constraint_rhs_easy_init(std::move(constraint_data));
        }
        quadratic_ineq_constraint_rhs_easy_init greater() { 
            constraint_data.sense = GREATER;
            return quadratic_ineq_constraint_rhs_easy_init(std::move(constraint_data));
        }
    };

} //SolverBuilder_Utils

#endif //EXPRESSION_BUILDER_HPP