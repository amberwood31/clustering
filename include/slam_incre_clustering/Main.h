//
// Created by amber on 2019-08-21.
//
#pragma once
#ifndef SLAM_PLUS_PLUS_MAIN_H
#define SLAM_PLUS_PLUS_MAIN_H

#include "slam/ConfigSolvers.h" // nonlinear graph solvers
//#include "slam/SE2_Types.h" // SE(2) types
//#include "slam/SE3_Types.h"
#include "rrr/types.hpp"

/**
 *	@brief structure, containing values of all the commandline arguments
 */
struct TCommandLineArgs {
    bool b_show_commandline; /**< @brief commandline repeat flag */
    bool b_show_flags; /**< @brief show build flags flag */
    bool b_show_detailed_timing; /**< @brief show detailed timing flag */
    bool b_verbose; /**< @brief verbosity flag; true means verbose, false means silent */
    bool b_use_schur; /**< @brief use Schur component flag */
    bool b_use_old_system; /**< @brief old system flag (deprecated) */
    bool b_pose_only; /**< @brief optimize pose-only problems */
    bool b_use_SE3; /**< @brief process SE3 system @note This is not overriden in commandline but detected in peek-parsing. */
    const char *p_s_input_file; /**< @brief path to the data file */
    int n_max_lines_to_process; /**< @brief maximal number of lines to process */
    int n_spatial_clustering_threshold;
    float f_chi2_dif_test_threshold;
    size_t n_linear_solve_each_n_steps; /**< @brief linear solve period, in steps (0 means disabled) */
    size_t n_nonlinear_solve_each_n_steps; /**< @brief nonlinear solve period, in steps (0 means disabled) */
    size_t n_max_nonlinear_solve_iteration_num; /**< @brief maximal number of iterations in nonlinear solve step */
    double f_nonlinear_solve_error_threshold; /**< @brief error threshold for nonlinear solve */
    size_t n_max_final_optimization_iteration_num; /**< @brief number of nonlinear solver iterations */ // as many other solvers
    double f_final_optimization_threshold; /**< @brief final optimization threshold */
    size_t n_omp_threads; /**< @brief OpenMP number of threads override */
    bool b_omp_dynamic; /**< @brief OpenMP dynamic scheduling override enable flag */
    //bool b_do_marginals; /**< @brief marginal covariance calculation enable flag */
    // this would always be on since this is needed to predict objective value changes
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

bool LoadLoopClosures(const char* file_name, IntPairSet& loops);

/**
 *	@brief prints all the important compiler / optimization switches this app was built with
 */
void DisplaySwitches();


#endif //SLAM_PLUS_PLUS_MAIN_H
