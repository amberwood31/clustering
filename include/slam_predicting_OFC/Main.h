//
// Created by amber on 2019-08-21.
//
#pragma once
#ifndef SLAM_PLUS_PLUS_MAIN_H
#define SLAM_PLUS_PLUS_MAIN_H


#include "slam/LinearSolver_UberBlock.h"
#include "slam/LinearSolver_Schur.h"
//#include "slam/LinearSolver_CholMod.h" // linear solvers (only one is required)
#include "slam/ConfigSolvers.h" // nonlinear graph solvers
#include "slam/SE2_Types.h" // SE(2) types
#include "slam/SE3_Types.h"
#include "slam_app/ParsePrimitives.h"
#include <list>

/*#include "rtabmap/utilite/ULogger.h"
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>

#include "rtabmap/utilite/ULogger.h"
*/ // TODO_LOCAL figure out how to integrate utilite library


/**
 *	@brief a helper function that adds a bunch of edges
 *
 *	@tparam CSystemType is type of the system, which is defined inside main()
 *
 *	@param[out] system is system to add the edges to
 *	@param[in] information is the information matrix for the edges
 *
 *	@note The data are taken from Olson's Manhattan dataset.
 */
template <class CSystemType>
void Add_More_ManhattanEdges(CSystemType &system, Eigen::Matrix3d information);

std::string uReplaceChar(const std::string & str, char before, char after);
double uStr2Double(const std::string & str);
std::list<std::string> uSplit(const std::string & str, char separator = ' ');
template<class V>
std::vector<V> uListToVector(const std::list<V> & list);

/**
 *	@brief structure, containing values of all the commandline arguments
 */
struct TCommandLineArgs {
    ENonlinearSolverType n_solver_choice; /**< @brief nonlinear solver selector */
    bool b_write_bitmaps; /**< @brief bitmaps write flag */
    bool b_write_solution; /**< @brief initial / solution write flag */
    bool b_xz_plots; /**< @brief x-z bitmaps orientation flag */
    bool b_write_system_matrix; /**< @brief matrix write flag */
    bool b_no_show; /**< @brief bitmaps show flag (only on windows) */
    bool b_show_commandline; /**< @brief commandline repeat flag */
    bool b_show_flags; /**< @brief show build flags flag */
    bool b_show_detailed_timing; /**< @brief show detailed timing flag */
    bool b_verbose; /**< @brief verbosity flag; true means verbose, false means silent */
    bool b_use_schur; /**< @brief use Schur component flag */
    bool b_run_matrix_benchmarks; /**< @brief run block matrix benchmarks flag */
    bool b_run_matrix_unit_tests; /**< @brief run block matrix unit tests flag */
    bool b_use_old_system; /**< @brief old system flag (deprecated) */
    bool b_pose_only; /**< @brief optimize pose-only problems */bool b_use_SE3; /**< @brief process SE3 system @note This is not overriden in commandline but detected in peek-parsing. */
    const char *p_s_input_file; /**< @brief path to the data file */
    int n_max_lines_to_process; /**< @brief maximal number of lines to process */
    size_t n_linear_solve_each_n_steps; /**< @brief linear solve period, in steps (0 means disabled) */
    size_t n_nonlinear_solve_each_n_steps; /**< @brief nonlinear solve period, in steps (0 means disabled) */
    size_t n_max_nonlinear_solve_iteration_num; /**< @brief maximal number of iterations in nonlinear solve step */
    double f_nonlinear_solve_error_threshold; /**< @brief error threshold for nonlinear solve */
    size_t n_max_final_optimization_iteration_num; /**< @brief number of nonlinear solver iterations */ // as many other solvers
    double f_final_optimization_threshold; /**< @brief final optimization threshold */
    const char *p_s_bench_name; /**< @brief benchmark file name (only if b_run_matrix_benchmarks is set) */
    const char *p_s_bench_type; /**< @brief benchmark type (only if b_run_matrix_benchmarks is set) */
    size_t n_omp_threads; /**< @brief OpenMP number of threads override */
    bool b_omp_dynamic; /**< @brief OpenMP dynamic scheduling override enable flag */
    bool b_do_marginals; /**< @brief marginal covariance calculation enable flag */
    /**
     *	@brief selects default values for commandline args
     */
    void Defaults();

    /**
     *	@brief parse commandline arguments
     *
     *	@param[in] n_arg_num is number of commandline arguments
     *	@param[in] p_arg_list is the list of commandline arguments
     *
     *	@return Returns true on success, false on failure.
     */
    bool Parse(int n_arg_num, const char **p_arg_list);

};


typedef MakeTypelist(CVertexPose2D) TVertexTypelist;
typedef MakeTypelist(CEdgePose2D) TEdgeTypelist;

typedef CFlatSystem<CVertexPose2D, TVertexTypelist, CEdgePose2D, TEdgeTypelist> CSystemType;

typedef CLinearSolver_UberBlock<CSystemType::_TyHessianMatrixBlockList> CLinearSolverType;

template <class CSystemType>
bool load_graph(const std::string &fileName, CSystemType &system);


#endif //SLAM_PLUS_PLUS_MAIN_H