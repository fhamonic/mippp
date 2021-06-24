/**
 * @file solver_builder.hpp
 * @author François Hamonic (francois.hamonic@gmail.com)
 * @brief OSI_Builder class declaration
 * @version 0.1
 * @date 2020-10-27
 * 
 * @copyright Copyright (c) 2020
 */
#ifndef SOLVER_BUILDER_HPP
#define SOLVER_BUILDER_HPP

#include <cmath>
#include <functional>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <boost/iterator/zip_iterator.hpp>
#include <boost/range/combine.hpp>
#include <boost/range/algorithm.hpp>

#include <range/v3/all.hpp>

namespace SolverBuilder_Utils {
    enum InequalitySense { LESS=-1, EQUAL=0, GREATER=1 };
    enum OptimizationSense { MIN=-1, MAX=1};
    constexpr double INFTY = std::numeric_limits<double>::max();

    class LinearExpression {
    private:
        double _constant;
        std::vector<int> _indices;
        std::vector<double> _coefficients;
    public:
        LinearExpression() : _constant{0} {}

        void add(double c) {
            _constant += c;
        }
        void add(int id, double coef=1) {
            _indices.push_back(id);
            _coefficients.push_back(coef);
        }

        double getConstant() const { return _constant; }
        int getNbTerms() const { return _indices.size(); }
        int * getIndicesData() { return _indices.data(); }
        double * getCoefficientsData() { 
            return _coefficients.data();
        }
        const std::vector<int> & getIndices() const { return _indices; }
        const std::vector<double> & getCoefficients() const { 
            return _coefficients;
        }

        LinearExpression & simplify() {
            auto zip_view = ranges::view::zip(_indices, _coefficients);
            ranges::sort(zip_view, [](auto p1, auto p2){ return p1.first < p2.first; }); 
        
            const auto begin = zip_view.begin();
            auto first = begin;
            const auto end = zip_view.end();
            for(auto next = first+1; next != end; ++next) {
                if((*first).first != (*next).first) {
                    if((*first).second != 0.0)
                        ++first;
                    *first = *next;
                    continue;
                }
                (*first).second += (*next).second;               
            }
            const size_t new_length = std::distance(begin, first+1);
            _indices.resize(new_length);
            _coefficients.resize(new_length);
            return *this;
        }
    };


    struct linear_ineq_constraint {
        InequalitySense sense;
        LinearExpression linear_expression;
        linear_ineq_constraint() = default;
    };

    
    class linear_ineq_constraint_rhs_easy_init {
    private:
        linear_ineq_constraint constraint_data;
    public:
        linear_ineq_constraint_rhs_easy_init(linear_ineq_constraint&& data) 
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
        linear_ineq_constraint constraint_data;
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



    struct linear_range_constraint {
        double lower_bound, upper_bound;
        LinearExpression linear_expression;
        linear_range_constraint()
                : lower_bound{std::numeric_limits<double>::min()}
                , upper_bound{std::numeric_limits<double>::max()} {}
    };

    class linear_range_constraint_lhs_easy_init {
    private:
        linear_range_constraint constraint_data;
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

        linear_range_constraint take_data()
            { return std::move(constraint_data); }
    };



    class QuadraticExpression {
    private:
        LinearExpression _linear_expression;
        std::vector<int> _quad_indices_1;
        std::vector<int> _quad_indices_2;
        std::vector<double> quad_coefficients;
    public:
        void add(double c) { _linear_expression.add(c); }
        void add(int id, double coef=1) {
            _linear_expression.add(id, coef);
        }
        void add(int id_1, int id_2, double coef=1) {
            _quad_indices_1.push_back(id_1);
            _quad_indices_2.push_back(id_2);
            quad_coefficients.push_back(coef);
        }

        const LinearExpression & getLineraExpression() const { return _linear_expression; }

        double getConstant() const { return _linear_expression.getConstant(); }

        int getNbLinearTerms() { return _linear_expression.getNbTerms(); }
        int * getLinearIndicesData() { return _linear_expression.getIndicesData(); }
        double * getLinearCoefficientsData() { return _linear_expression.getCoefficientsData(); }
        const std::vector<int> & getLinearIndices() const { return _linear_expression.getIndices(); }
        const std::vector<double> & getLinearCoefficients() const { return _linear_expression.getCoefficients(); }

        int getNbQuadTerms() { return _quad_indices_1.size(); }
        int * getQuadIndices1Data() { return _quad_indices_1.data(); }
        int * getQuadIndices2Data() { return _quad_indices_2.data(); }
        double * getQuadCoefficientsData() { return quad_coefficients.data(); }
        const std::vector<int> & getQuadIndices1() const { return _quad_indices_1; }
        const std::vector<int> & getQuadIndices2() const { return _quad_indices_2; }
        const std::vector<double> & getQuadCoefficients() const { return quad_coefficients; }

        bool isLinear() { return _quad_indices_1.empty(); }

        QuadraticExpression & simplify() {
            _linear_expression.simplify();
            return *this;
        }
    };

    struct quadratic_ineq_constraint {
        InequalitySense sense;
        QuadraticExpression quadratic_expression;
        quadratic_ineq_constraint() = default;
    };

    
    class quadratic_ineq_constraint_rhs_easy_init {
    private:
        quadratic_ineq_constraint constraint_data;
    public:
        quadratic_ineq_constraint_rhs_easy_init(
                quadratic_ineq_constraint&& data) 
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

        quadratic_ineq_constraint take_data()
            { return std::move(constraint_data); }
    };

    class quadratic_ineq_constraint_lhs_easy_init {
    private:
        quadratic_ineq_constraint constraint_data;
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



// using namespace SolverBuilder_Utils;


// /**
//  * @brief A practical class for building OsiSolver instances 
//  */
// class SolverBuilder {
//     public:
//         class VarType {
//             protected:
//                 int begin_id;
//                 int end_id;
//                 double _default_lb;
//                 double _default_ub;
//                 bool _integer;
//                 VarType(int number, double lb=0, double ub=INFTY, bool integer=false) : begin_id(0), end_id(number), _default_lb(lb), _default_ub(ub), _integer(integer) {}
//                 VarType() : VarType(0) {}
//             public:
//                 void offsetIds(int offset) { begin_id += offset; end_id+= offset; }
//                 int getNumber() const { return end_id - begin_id; }
//                 double getDefaultLB() { return _default_lb; }
//                 double getDefaultUB() { return _default_ub; }
//                 bool isInteger() { return _integer; }
//         };
//     private:
//         int nb_vars;        
//         std::vector<VarType*> varTypes;

//         std::unique_ptr<double[]> objective;
//         std::unique_ptr<double[]> col_lb;
//         std::unique_ptr<double[]> col_ub;
//         std::unique_ptr<std::string[]> col_names;

//         std::vector<int> starts;
//         std::vector<int> indices;
//         std::vector<double> coefficients;

//         std::vector<double> row_lb;
//         std::vector<double> row_ub;

//         std::vector<int> integers_variables;
//     public:
//         // OSI_Builder();
//         // ~OSI_Builder();
        
//         // OSI_Builder & addVarType(VarType * var_type);
//         // void init();
//         // OSI_Builder & setObjective(int  var_id, double coef);
//         // OSI_Builder & setBounds(int  var_id, double lb, double ub);
//         // OSI_Builder & buffEntry(int  var_id, double coef);
//         // OSI_Builder & popEntryBuffer();
//         // OSI_Builder & clearEntryBuffer();
//         // OSI_Builder & pushRowWithoutClearing(double lb, double ub);
//         // OSI_Builder & pushRow(double lb, double ub);
//         // OSI_Builder & setColName(int var_id, std::string name);

//         // OSI_Builder & setContinuous(int var_id);
//         // OSI_Builder & setInteger(int var_id);
        
//         template <class OsiSolver>
//         OsiSolver * buildSolver(int sense, bool relaxed=false) {
//             OsiSolver * solver = new OsiSolver();
//             solver->loadProblem(*matrix, col_lb, col_ub, objective, row_lb.data(), row_ub.data());
//             solver->setObjSense(sense);
//             if(relaxed)
//                 return solver;
//             for(int i : integers_variables) 
//                 solver->setInteger(i);
//             solver->setColNames(colNames, 0, nb_vars, 0);
//             return solver;
//         }

//         static int nb_pairs(int n) {
//             return n*(n-1)/2;
//         };
//         static int nb_couples(int n) {
//             return 2*nb_pairs(n);
//         };

//         static int compose_pair(int i, int j) {
//             assert(i != j);
//             if(i > j) std::swap(i,j);
//             return nb_pairs(j)+i;
//         };
//         static std::pair<int,int> retrieve_pair(int id) {
//             const int j = std::round(std::sqrt(2*id+1));
//             const int i = id - nb_pairs(j);
//             return std::make_pair(i,j);
//         };
//         static int compose_couple(int i, int j) {
//             return 2*compose_pair(i, j) + (i < j ? 0 : 1);
//         };
//         static std::pair<int,int> retrieve_couple(int id) {
//             std::pair<int,int> p = retrieve_pair(id / 2);
//             if(id % 2 == 1) std::swap(p.first, p.second);
//             return p;
//         };

        

//         const std::vector<VarType*> & getVarTypes() { return varTypes; }

//         int getNbVars() const { return nb_vars; };

//         int getNbNonZeroVars() const {
//             std::vector<int> non_zero(nb_vars, 0);
//             const int * indices = matrix->getIndices();
//             for(int i=0; i<matrix->getNumElements(); ++i)
//                 non_zero[indices[i]] = 1;      
//             return std::accumulate(non_zero.begin(), non_zero.end(), 0);
//         };
//         int getNbConstraints() const { return matrix->getNumRows(); };

//         int getNbElems() const { return nb_entries; };

//         double * getObjective() { return objective; }
//         CoinPackedMatrix * getMatrix() { return matrix; }

//         double * getColLB() { return col_lb; }
//         double * getColUB() { return col_ub; }

//         double * getRowLB() { return row_lb.data(); }
//         double * getRowUB() { return row_ub.data(); }
// };

#endif //OSICLPSOLVER_BUILDER_HPP