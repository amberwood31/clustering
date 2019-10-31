//
// Created by amber on 2019-09-06.
//
#pragma once
#ifndef SLAM_PLUS_PLUS_OPTIMIZERSLAMPP_H
#define SLAM_PLUS_PLUS_OPTIMIZERSLAMPP_H

//

//#include "slam_pp/SE2_Types.h" // SE(2) types


/**
 *	@brief SLAM++ slam optimizer interface
 */

/**
 *	@def WANT_DEEP_VERTEX_ACCESS
 *	@brief if defined, it is possible to access optimizer vertex objects;
 *		otherwise only vertex state vectors can be accessed
 *	@note If not defined, the optimizer does not need to be included,
 *		resulting in much faster compilation times of files including this.
 */
//NO_DEEP_VERTEX_ACCESS

/**
 *	@brief SLAM optimizer
 *	@note The purpose is not to have any interaction with CSystemType or CNonlinearSolverType
 *		in the header, in order to move the slow building of SLAM++ to Optimizer.cpp, which
 *		is rarely changed throughout the development process.
 */
class CSLAMOptimizer {
public:

    class CSLAMOptimizerCore; // forward declaration

protected:
    CSLAMOptimizerCore *m_p_optimizer; // PIMPL

public:
    CSLAMOptimizer(bool b_verbose = false, bool b_use_schur = true, int max_iteration = 5, double error_tolerance = 1e-5); // throw(srd::bad_alloc) // todo - add more optimization settings as required
    ~CSLAMOptimizer();

    size_t n_Vertex_Num() const;

    Eigen::Map<Eigen::VectorXd> r_Vertex_State(size_t n_index); // returns map of vertex state only

    void Delay_Optimization();
    void Enable_Optimization(); // throw(srd::bad_alloc, std::runtime_error)

    void Optimize(size_t n_max_iteration_num = 5, double f_min_dx_norm = .01); // throw(srd::bad_alloc, std::runtime_error)

    bool Load_New_Edge(int id_from, int id_to, double x, double y, double rot, Eigen::Matrix3d information); // load a new_edge from the file into m_new_edge
    void Increment_Once(); // increment once with the current m_new_edge
    double Calculate_Ofc(FILE *full_analysis_file); // predict objective function change for the current m_new_edge

    void Show_Stats() const;

    bool Dump_State(const char *p_s_filename) const;

    void Generate_plot() const;


/**
 *	@brief prints all the important compiler / optimization switches this app was built with
 */
    void DisplaySwitches();



private:
    CSLAMOptimizer(const CSLAMOptimizer &r_other); // no copy
    CSLAMOptimizer &operator =(const CSLAMOptimizer &r_other); // no copy
};


#endif //SLAM_PLUS_PLUS_OPTIMIZERSLAMPP_H
