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