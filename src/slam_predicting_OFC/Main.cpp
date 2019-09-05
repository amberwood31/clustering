//
// Created by amber on 2019-08-21.
//
int n_dummy_param = 0;
/**< @brief a dummy parameter, used as a convenient commandline input, intended for debugging / testing */
#include "slam_predicting_OFC/Main.h"

#include <stdio.h> // printf
#include <iostream>
#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP


/**
 *	@brief main
 *
 *	@param[in] n_arg_num is number of commandline arguments
 *	@param[in] p_arg_list is the list of commandline arguments
 *
 *	@return Returns 0 on success, -1 on failure.
 */
int main(int UNUSED(n_arg_num), const char **UNUSED(p_arg_list))
{
    CTimer t;
    double start, end;
    start = t.f_Time();

    TCommandLineArgs t_cmd_args;
    t_cmd_args.Defaults(); // set defaults
    if(!t_cmd_args.Parse(n_arg_num, p_arg_list))
        return -1;
    // parse commandline


    if(t_cmd_args.b_show_commandline) {
        printf("> ./SLAM_plus_plus");
        for(int i = 1; i < n_arg_num; ++ i)
            printf(" %s", p_arg_list[i]);
        printf("\n");
    }
    // display commandline

    {

        FILE *p_fr;
        /*
        if((p_fr = fopen(t_cmd_args.p_s_input_file, "rb")))
            fclose(p_fr);
        else {
            fprintf(stderr, "error: can't open input file \'%s\'\n", t_cmd_args.p_s_input_file);
            return -1;
        }
        if((p_fr = fopen(t_cmd_args.p_s_inlier_file, "rb")))
            fclose(p_fr);
        else {
            fprintf(stderr, "error: can't open inlier file \'%s\'\n", t_cmd_args.p_s_inlier_file);
            return -1;
        }
         */
        if((p_fr = fopen(t_cmd_args.p_s_outlier_file, "rb")))
            fclose(p_fr);
        else {
            fprintf(stderr, "error: can't open outlier file \'%s\'\n", t_cmd_args.p_s_outlier_file);
            return -1;
        }
    }
    // see if the input is there; otherwise will get some misleading errors about peek-parsing
    // and potentially about SE(2) (or other default solver) being disabled (if unlucky)

    if(t_cmd_args.b_use_old_system) {
        fprintf(stderr, "error: the legacy solver was not compiled\n");
        return -1;
    } else {
    }



    //typedef CLinearSolver_CholMod CLinearSolverType; // or cholmod

    CSystemType system;
    TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting();
    TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy( true, frequency::Never(), EBlockMatrixPart(mpart_Nothing), EBlockMatrixPart(mpart_LastColumn + mpart_Diagonal), mpart_Nothing);

    CNonlinearSolver_Lambda<CSystemType, CLinearSolverType> solver(system, t_incremental_config, t_marginals_config, t_cmd_args.b_verbose);

    /*
    std::string file_path = "/home/amber/SLAM_plus_plus_v2.30/slam/data/manhattan_odometry.g2o";
    if(t_cmd_args.b_verbose)
        fprintf(stderr, "Loading graph\n");
    load_graph(t_cmd_args.p_s_input_file, system);
    system.Plot2D("resultUnoptim.tga", plot_quality::plot_Printing); // plot in print quality

    if(t_cmd_args.b_verbose)
        fprintf(stderr, "Optimizing\n");
    solver.Optimize();
    // optimize the system

    solver.r_MarginalCovariance().Dump_Diagonal();
    system.Plot2D("result.tga", plot_quality::plot_Printing); // plot in print quality
*/
    std::string outlier_file = "/home/amber/SLAM_plus_plus_v2.30/slam/data/outlier.g2o";
    FILE * file = 0;
    file = fopen(t_cmd_args.p_s_outlier_file, "r");

    const char *save_file_name = "outlier_analysis.txt";
    FILE * save_file = fopen(save_file_name, "w");

    const char *real_ofc_name = "real_ofc.txt";
    FILE * real_ofc_file = fopen(real_ofc_name, "w");

    const char *full_analysis_name = "full_analysis.txt";
    FILE * full_analysis_file = fopen(full_analysis_name, "w");


    if(file){

        analyze_edge_set(file, system, solver, 1, save_file, real_ofc_file, full_analysis_file, t_cmd_args.b_verbose);

    }// load one outlier and predict the objective function change
    fclose(file);
    fclose(save_file);
    fclose(real_ofc_file);
    fclose(full_analysis_file);

    end= t.f_Time();
    printf("\nthe whole process took %f sec\n", end-start);


    system.Plot2D("result.tga", plot_quality::plot_Printing); // plot in print quality
/*
    std::string inlier_file = "/home/amber/SLAM_plus_plus_v2.30/slam/data/inlier.g2o";
    file = 0;
    file = fopen(t_cmd_args.p_s_inlier_file, "r");

    save_file_name = "inlier_analysis.txt";
    save_file = fopen(save_file_name, "w");
    if(file){

        analyze_edge_set(file, system, solver, 0, save_file);

    }// load one outlier and predict the objective function change
    fclose(file);
    fclose(save_file);
    */


    solver.Dump(); // show some stats

    return 0;
}



void TCommandLineArgs::Defaults()
{
    n_solver_choice = nlsolver_Lambda; /**< @brief nonlinear solver selector */
    // solver selection

    b_write_bitmaps = true;
    b_write_solution = true;
    b_xz_plots = false;
    b_write_system_matrix = false;
    b_no_show = false;
    b_show_commandline = true;
    b_show_flags = true;
    b_show_detailed_timing = true;
    b_verbose = false;
    // verbosity

    b_use_schur = false;

    b_run_matrix_benchmarks = false;
    b_run_matrix_unit_tests = false;
    b_use_old_system = false; // t_odo - make this commandline
    b_pose_only = false;
    b_use_SE3 = false; // note this is not overriden in commandline but detected in peek-parsing

    p_s_input_file = 0; /** <@brief path to the data file */
    p_s_inlier_file = 0; /** <@brief path to the inlier file */
    p_s_outlier_file = 0; /** <@brief path to the outlier file */
    n_max_lines_to_process = 0; /** <@brief maximal number of lines to process */

    n_linear_solve_each_n_steps = 0; /**< @brief linear solve period, in steps (0 means disabled) */
    n_nonlinear_solve_each_n_steps = 0; /**< @brief nonlinear solve period, in steps (0 means disabled) */
    n_max_nonlinear_solve_iteration_num = 10; /**< @brief maximal number of iterations in nonlinear solve step */
    f_nonlinear_solve_error_threshold = 20; /**< @brief error threshold for nonlinear solve */
    n_max_final_optimization_iteration_num = 5; // as many other solvers
    f_final_optimization_threshold = 0.01;
    // optimization mode for slam

    p_s_bench_name = 0;
    p_s_bench_type = "all";

    n_omp_threads = size_t(-1);
    b_omp_dynamic = false;

    b_do_marginals = false;

}

void PrintHelp()
{
    printf("General use:\n"
           "    ./SLAM_plus_plus -i <filename> --no-detailed-timing\n"
           "\n"
           "To run the pose-only datasets more quickly:\n"
           "    ./SLAM_plus_plus -i <filename> --pose-only --no-detailed-timing\n"
           "\n"
           "To run incrementally:\n"
           "    ./SLAM_plus_plus -nsp <optimize-each-N-verts> -fL -i <filename> --no-detailed-timing\n"
           "\n"
           "This generates initial.txt and initial.tga, a description and image of the\n"
           "system before the final optimization, and solution.txt and solution.tga, a\n"
           "description and image of the final optimized system (unless --no-solution\n"
           "or --no-bitmaps are specified, respectively).\n"
           "\n"
           "--help|-h         displays this help screen\n"
           "--verbose|-v      displays verbose output while running (may slow down,\n"
           "                  especially in windows and if running incrementally)\n"
           "--silent|-s       suppresses displaying verbose output\n"
           "--no-show|-ns     does not show output image (windows only)\n"
           "--no-commandline|-nc    does not echo command line\n"
           "--no-flags|-nf    does not show compiler flags\n"
           "--no-detailed-timing|-ndt    does not show detailed timing breakup (use this, lest\n"
           "                  you will get confused)\n"
           "--no-bitmaps|-nb  does not write bitmaps initial.tga and solution.tga (does not\n"
           "                  affect the text files)\n"
           "--no-solution|-ns does not write text files initial.txt and solution.txt\n"
           "--xz-plots|-xz    turns bitmaps initial.tga and solution.tga into the X-Z plane\n"
           "--dump-system-matrix|-dsm    writes system matrix as system.mtx (matrix market)\n"
           "                  and system.bla (block layout) before optimization\n"
           "--pose-only|-po   enables optimisation for pose-only slam (will warn and ignore\n"
           "                  on datasets with landmarks (only the first 1000 lines checked,\n"
           "                  in case there are landmarks later, it would segfault))\n"
           "--use-old-code|-uogc    uses the old CSparse code (no block matrices in it)\n"
           "--a-solver|-A     uses A solver\n"
           "--lambda|-,\\      uses lambda solver (default, preferred batch solver)\n"
           "--lambda-lm|-,\\lm uses lambda solver with Levenberg Marquardt (default for BA)\n"
           "--lambda-dl|-,\\dl uses lambda solver with Dogleg and fluid relinearization\n"
           "--l-solver|-L     uses L solver\n"
           "--fast-l|-fL      uses the new fast L solver (preferred incremental solver)\n"
           "--use-schur|-us   uses Schur complement to accelerate linear solving\n"
           "--do-marginals|-dm enables marginal covariance calculation (experimental)\n"
           "--infile|-i <filename>    specifies input file <filename>; it can cope with\n"
           "                  many file types and conventions\n"
           "--parse-lines-limit|-pll <N>    sets limit of lines read from the input file\n"
           "                  (handy for testing), note this does not set limit of vertices\n"
           "                  nor edges!\n"
           "--linear-solve-period|-lsp <N>    sets period for incrementally running linear\n"
           "                  solver (default 0: disabled)\n"
           "--nonlinear-solve-period|-nsp <N>    sets period for incrementally running\n"
           "                  non-linear solver (default 0: disabled)\n"
           "--max-nonlinear-solve-iters|-mnsi <N>    sets maximal number of nonlinear\n"
           "                  solver iterations (default 10)\n"
           "--nonlinear-solve-error-thresh|-nset <f>    sets nonlinear solve error threshold\n"
           "                  (default 20)\n"
           "--max-final-nonlinear-solve-iters|-mfnsi <N>    sets maximal number of final\n"
           "                  optimization iterations (default 5)\n"
           "--final-nonlinear-solve-error-thresh|-fnset <f>    sets final nonlinear solve\n"
           "                  error threshold (default 0.01)\n"
           "--run-matrix-benchmarks|-rmb <benchmark-name> <benchmark-type>    runs block\n"
           "                  matrix benchmarks (benchmark-name is name of a folder with\n"
           "                  UFLSMC benchmark, benchmark-type is one of alloc, factor, all)\n"
           "--run-matrix-unit-tests|-rmut    runs block matrix unit tests\n"
           "--omp-set-num-threads <N>    sets number of threads to N (default is to use as\n"
           "                  many threads as there are CPU cores)\n"
           "--omp-set-dynamic <N>    enables dynamic adjustment of the number of threads is N\n"
           "                  is nonzero, disables if zero (disabled by default)\n");
}

template <class CSystemType>
bool load_graph(const char *fileName, CSystemType &system){

    FILE * file = 0;
    file = fopen(fileName, "r");

    if(file)
    {
        char line[400];
        while ( fgets (line , 400 , file) != NULL )
        {
            std::vector<std::string> strList = uListToVector(uSplit(uReplaceChar(line, '\n', ' '), ' '));
            if(strList.size() == 9)
            {
                //VERTEX_SE3:QUAT

            }
            else if(strList.size() == 5)
            {
                //VERTEX_SE2

            }
            else if(strList.size() == 30)
            {
                //EDGE_SE3:QUAT
                /*int idFrom = atoi(strList[1].c_str());
                int idTo = atoi(strList[2].c_str());
                float x = uStr2Float(strList[3]);
                float y = uStr2Float(strList[4]);
                float z = uStr2Float(strList[5]);
                float roll = uStr2Float(strList[6]);
                float pitch = uStr2Float(strList[7]);
                float yaw = uStr2Float(strList[8]);
                Eigen::Matrix<double, 6, 1> edge;
                edge << x, y, z, roll, pitch, yaw;

                system.r_Add_Edge(CEdgePose3D(idFrom, idTo, edge, information, system));
                 */
            }
            else if(strList.size() == 12)
            {
                //EDGE_SE2
                int idFrom = atoi(strList[1].c_str());
                int idTo = atoi(strList[2].c_str());
                float x = uStr2Double(strList[3]);
                float y = uStr2Double(strList[4]);
                float rot = uStr2Double(strList[5]);

                Eigen::Matrix3d information;
                information << uStr2Double(strList[6]), uStr2Double(strList[7]), uStr2Double(strList[8]),
                                uStr2Double(strList[7]), uStr2Double(strList[9]), uStr2Double(strList[10]),
                                uStr2Double(strList[8]), uStr2Double(strList[10]), uStr2Double(strList[11]);

                system.r_Add_Edge(CEdgePose2D(idFrom, idTo, Eigen::Vector3d(x, y, rot), information, system));
            }
            else if(strList.size())
            {
                fprintf(stderr, "Error parsing graph file %s on line \"%s\" (strList.size()=%d)", fileName, line, (int)strList.size());
            }
        }

        fclose(file);
    }
    else
    {
        fprintf(stderr, "Cannot open file %s", fileName);
        return false;
    }
    return true;

}

void zero_offdiagonal(Eigen::MatrixXd &square_mat, int mat_size)
{
    for (int i=0; i < mat_size; i++)
    {
        for (int j=0;  j< mat_size; j++)
        {
            if (i!=j)
                square_mat(i, j) = 0.0;
        }

    }


}

template<class CEdgeType, class CSolverType>
void calculate_ofc( CEdgeType &new_edge, Eigen::MatrixXd &information, CSolverType &solver, int vertex_from, int vertex_to, FILE * full_analysis_file, double &del_obj_function, double &mi_gain)
{
    Eigen::Matrix3d r_t_jacobian0, r_t_jacobian1, cov_inv, innovation_cov;
    Eigen::Vector3d r_v_expectation, r_v_error;
    new_edge.Calculate_Jacobians_Expectation_Error(r_t_jacobian0, r_t_jacobian1, r_v_expectation,r_v_error);
    // optimize the system

    Eigen::MatrixXd joined_matrix(3,6);
    joined_matrix << - r_t_jacobian0, - r_t_jacobian1;
    Eigen::MatrixXd marginal_covariance(6,6);
    Eigen::Matrix3d covariance_idfrom_zero_offdiagonal, covariance_idto_zero_offdiagonal, identity_block, zero_block;
    covariance_idfrom_zero_offdiagonal <<   0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0;
    covariance_idto_zero_offdiagonal <<   0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0;
    zero_block <<   0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0;

    Eigen::MatrixXd covariance_idfrom(3,3), covariance_idto(3,3), covariance_idtofrom(3,3), covariance_idfromto(3,3);
    solver.r_MarginalCovariance().save_Diagonal(covariance_idfrom, vertex_from, vertex_from);
    solver.r_MarginalCovariance().save_Diagonal(covariance_idto, vertex_to, vertex_to);
    if (vertex_from > vertex_to)
    {
        solver.r_MarginalCovariance().save_Diagonal(covariance_idfromto, vertex_from, vertex_to);
        marginal_covariance << covariance_idfrom, covariance_idfromto, covariance_idfromto.transpose(), covariance_idto;
    } else{
        solver.r_MarginalCovariance().save_Diagonal(covariance_idfromto, vertex_to, vertex_from);
        marginal_covariance << covariance_idfrom, covariance_idfromto.transpose(), covariance_idfromto, covariance_idto;

    }
    innovation_cov = joined_matrix* marginal_covariance * joined_matrix.transpose() + information.inverse();
    cov_inv = innovation_cov.inverse();

    //std::cout << "transform norm:  " << r_v_error.norm() << std::endl;
    //std::cout << "information norm: " << cov_inv.norm() << std::endl;

    del_obj_function = r_v_error.dot(cov_inv * r_v_error);
    mi_gain = log(innovation_cov.determinant() / information.inverse().determinant());
    fprintf(full_analysis_file, "%d %d %f %f %f %f\n", vertex_from, vertex_to, r_v_error.norm(), cov_inv.norm(), del_obj_function, mi_gain);




}


/**
 * Split a string into multiple string around the specified separator.
 * Example:
 * @code
 * 		std::list<std::string> v = split("Hello the world!", ' ');
 * @endcode
 * The list v will contain {"Hello", "the", "world!"}
 * @param str the string
 * @param separator the separator character
 * @return the list of strings
 */
std::list<std::string> uSplit(const std::string & str, char separator)
{
    std::list<std::string> v;
    std::string buf;
    for(unsigned int i=0; i<str.size(); ++i)
    {
        if(str[i] != separator)
        {
            buf += str[i];
        }
        else if(buf.size())
        {
            v.push_back(buf);
            buf = "";
        }
    }
    if(buf.size())
    {
        v.push_back(buf);
    }
    return v;
}

/**
 * Convert a std::list to a std::vector.
 * @param list the list
 * @return the vector
 */
template<class V>
std::vector<V> uListToVector(const std::list<V> & list)
{
    return std::vector<V>(list.begin(), list.end());
}

std::string uReplaceChar(const std::string & str, char before, char after)
{
    std::string result = str;
    for(unsigned int i=0; i<result.size(); ++i)
    {
        if(result[i] == before)
        {
            result[i] = after;
        }
    }
    return result;
}


double uStr2Double(const std::string & str)
{
    double value = 0.0;
    std::istringstream istr(uReplaceChar(str, ',', '.').c_str());
    istr.imbue(std::locale("C"));
    istr >> value;
    return value;
}

template<class CSystemType, class CSolverType>
bool analyze_edge_set(FILE * file_pointer, CSystemType &system, CSolverType & solver, int edge_nature, FILE * save_file, FILE * real_ofc_file, FILE * full_analysis_file, bool verbose)
{
    CTimer t;
    double f_time_before, f_time_after;

    char line[400];
    while ( fgets (line , 400 , file_pointer) != NULL )
    {
        f_time_before= t.f_Time();
        std::vector<std::string> strList = uListToVector(uSplit(uReplaceChar(line, '\n', ' '), ' '));
        if(strList.size() == 30)
        {
            //EDGE_SE3:QUAT
            /*int idFrom = atoi(strList[1].c_str());
            int idTo = atoi(strList[2].c_str());
            float x = uStr2Float(strList[3]);
            float y = uStr2Float(strList[4]);
            float z = uStr2Float(strList[5]);
            float roll = uStr2Float(strList[6]);
            float pitch = uStr2Float(strList[7]);
            float yaw = uStr2Float(strList[8]);
            Eigen::Matrix<double, 6, 1> edge;
            edge << x, y, z, roll, pitch, yaw;

            system.r_Add_Edge(CEdgePose3D(idFrom, idTo, edge, information, system));
             */
        }
        else if(strList.size() == 12)
        {
            //EDGE_SE2
            int vertex_from = atoi(strList[1].c_str());
            int vertex_to = atoi(strList[2].c_str());
            double x = uStr2Double(strList[3]);
            double y = uStr2Double(strList[4]);
            double rot = uStr2Double(strList[5]);

            Eigen::MatrixXd information(3,3);
            information << uStr2Double(strList[6]), uStr2Double(strList[7]), uStr2Double(strList[8]),
                uStr2Double(strList[7]), uStr2Double(strList[9]), uStr2Double(strList[10]),
                uStr2Double(strList[8]), uStr2Double(strList[10]), uStr2Double(strList[11]);

            CEdgePose2D new_edge;
            new_edge = CEdgePose2D(vertex_from, vertex_to, Eigen::Vector3d(x, y, rot), information, system);

            if ((vertex_to - vertex_from) != 1) // if reached loop closure edge
            {
                if (verbose == true)
                {
                    std::cout << "number of edges: " << system.n_Edge_Num() << std::endl;
                    fprintf(stderr, "Solve again: \n"); //only solve when a loop closure edge is loaded
                }

                f_time_before= t.f_Time();
                solver.Optimize(5, 1e-5);
                if (verbose == true)
                {
                    f_time_after= t.f_Time();
                    printf("\nthis iteration of solving took %f sec\n", f_time_after-f_time_before);

                }

                double before = solver.get_residual_chi2_error();

                double delta_obj, mi;
                calculate_ofc(new_edge, information, solver, vertex_from, vertex_to, full_analysis_file, delta_obj, mi);
                fprintf(save_file, "%d %d %f\n", vertex_from, vertex_to, delta_obj);
                if (edge_nature == 1)
                {
                    //std::cout << "OFC due to outlier: " << delta_obj << std::endl;
                    //std::cout << "number of edges: " << system.n_Edge_Num() << std::endl;
                }
                else if (edge_nature == 0)
                {
                    //std::cout << "OFC due to inlier: " << delta_obj << std::endl;
                    //std::cout << "number of edges: " << system.n_Edge_Num() << std::endl;
                }

                system.r_Add_Edge(new_edge);
                //solver.Optimize(5, 1e-5);
                double after = solver.get_residual_chi2_error();
                //solver.Optimize(5, 1e-5);
                //std::cout << "difference: " << after-before << std::endl;
                //std::cout << "difference: " << solver.get_residual_error()- before << std::endl;



            }
            else{
                system.r_Add_Edge(new_edge); // adding all odometry edges
                if (verbose == true)
                {
                    f_time_after= t.f_Time();
                    printf("\nthis iteration took %f sec\n", f_time_after-f_time_before);

                }
            }

        }
        else if(strList.size())
        {
            fprintf(stderr, "Error parsing graph file");
        }
    }


}





bool TCommandLineArgs::Parse(int n_arg_num, const char **p_arg_list)
{
    for(int i = 1; i < n_arg_num; ++ i) {
        if(!strcmp(p_arg_list[i], "--help") || !strcmp(p_arg_list[i], "-h")) {
            PrintHelp();
            //fprintf(stderr, "no help for you! mwuhahaha! (please read Main.cpp)\n"); // t_odo
            return false; // quit
        } else if(!strcmp(p_arg_list[i], "--verbose") || !strcmp(p_arg_list[i], "-v"))
            b_verbose = true;
        else if(!strcmp(p_arg_list[i], "--silent") || !strcmp(p_arg_list[i], "-s"))
            b_verbose = false;
        else if(!strcmp(p_arg_list[i], "--use-schur") || !strcmp(p_arg_list[i], "-us"))
            b_use_schur = true;
        else if(!strcmp(p_arg_list[i], "--no-show") || !strcmp(p_arg_list[i], "-ns"))
            b_no_show = true;
        else if(!strcmp(p_arg_list[i], "--no-commandline") || !strcmp(p_arg_list[i], "-nc"))
            b_show_commandline = false;
        else if(!strcmp(p_arg_list[i], "--do-marginals") || !strcmp(p_arg_list[i], "-dm"))
            b_do_marginals = true;
        else if(!strcmp(p_arg_list[i], "--lambda") || !strcmp(p_arg_list[i], "-,\\"))
            n_solver_choice = nlsolver_Lambda;
        else if(!strcmp(p_arg_list[i], "--lambda-lm") || !strcmp(p_arg_list[i], "-,\\lm"))
            n_solver_choice = nlsolver_LambdaLM;
        else if(!strcmp(p_arg_list[i], "--lambda-dl") || !strcmp(p_arg_list[i], "-,\\dl"))
            n_solver_choice = nlsolver_LambdaDL;
        else if(!strcmp(p_arg_list[i], "--no-flags") || !strcmp(p_arg_list[i], "-nf"))
            b_show_flags = false;
        else if(!strcmp(p_arg_list[i], "--run-matrix-unit-tests") || !strcmp(p_arg_list[i], "-rmut"))
            b_run_matrix_unit_tests = true;
        else if(!strcmp(p_arg_list[i], "--no-detailed-timing") || !strcmp(p_arg_list[i], "-ndt"))
            b_show_detailed_timing = false;
        else if(!strcmp(p_arg_list[i], "--use-old-code") || !strcmp(p_arg_list[i], "-uogc"))
            b_use_old_system = true;
        else if(!strcmp(p_arg_list[i], "--dump-system-matrix") || !strcmp(p_arg_list[i], "-dsm"))
            b_write_system_matrix = true;
        else if(!strcmp(p_arg_list[i], "--no-bitmaps") || !strcmp(p_arg_list[i], "-nb")) {
            b_write_bitmaps = false;
            b_no_show = true; // no bitmaps ... what can it show?
        } else if(!strcmp(p_arg_list[i], "--no-solution") || !strcmp(p_arg_list[i], "-ns"))
            b_write_solution = false;
        else if(!strcmp(p_arg_list[i], "--xz-plots") || !strcmp(p_arg_list[i], "-xz"))
            b_xz_plots = true;
        else if(!strcmp(p_arg_list[i], "--pose-only") || !strcmp(p_arg_list[i], "-po"))
            b_pose_only = true;
        else if(!strcmp(p_arg_list[i], "--a-solver") || !strcmp(p_arg_list[i], "-A"))
            n_solver_choice = nlsolver_A;
        else if(!strcmp(p_arg_list[i], "--l-solver") || !strcmp(p_arg_list[i], "-L"))
            n_solver_choice = nlsolver_L;
        else if(!strcmp(p_arg_list[i], "--fast-l") || !strcmp(p_arg_list[i], "-fL"))
            n_solver_choice = nlsolver_FastL;
        else if(i + 1 == n_arg_num) {
            fprintf(stderr, "error: argument \'%s\': missing value or an unknown argument\n", p_arg_list[i]);
            return false;
        } else if(!strcmp(p_arg_list[i], "--infile") || !strcmp(p_arg_list[i], "-i"))
            p_s_input_file = p_arg_list[++ i];
        else if(!strcmp(p_arg_list[i], "-in"))
            p_s_inlier_file = p_arg_list[++ i];
        else if(!strcmp(p_arg_list[i], "-ou"))
            p_s_outlier_file = p_arg_list[++ i];
        else if(!strcmp(p_arg_list[i], "--parse-lines-limit") || !strcmp(p_arg_list[i], "-pll"))
            n_max_lines_to_process = atol(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--linear-solve-period") || !strcmp(p_arg_list[i], "-lsp"))
            n_linear_solve_each_n_steps = atol(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--nonlinear-solve-period") || !strcmp(p_arg_list[i], "-nsp"))
            n_nonlinear_solve_each_n_steps = atol(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--max-nonlinear-solve-iters") || !strcmp(p_arg_list[i], "-mnsi"))
            n_max_nonlinear_solve_iteration_num = atol(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--nonlinear-solve-error-thresh") || !strcmp(p_arg_list[i], "-nset"))
            f_nonlinear_solve_error_threshold = atof(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--max-final-nonlinear-solve-iters") || !strcmp(p_arg_list[i], "-mfnsi"))
            n_max_final_optimization_iteration_num = atol(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--final-nonlinear-solve-error-thresh") || !strcmp(p_arg_list[i], "-fnset"))
            f_final_optimization_threshold = atof(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--omp-set-num-threads"))
            n_omp_threads = atol(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--omp-set-dynamic"))
            b_omp_dynamic = (atol(p_arg_list[++ i]) != 0);
        else if(!strcmp(p_arg_list[i], "--run-matrix-benchmarks") || !strcmp(p_arg_list[i], "-rmb")) {
            if(i + 2 >= n_arg_num) {
                fprintf(stderr, "error: argument \'%s\': missing the second value\n", p_arg_list[i]);
                return false;
            }
            b_run_matrix_benchmarks = true;
            p_s_bench_name = p_arg_list[++ i];
            p_s_bench_type = p_arg_list[++ i];
            if(strcmp(p_s_bench_type, "alloc") &&
               strcmp(p_s_bench_type, "factor") &&
               strcmp(p_s_bench_type, "all")) {
                fprintf(stderr, "error: argument \'%s\': unknown benchmark type\n", p_arg_list[i]);
                return false;
            }
        } else if(!strcmp(p_arg_list[i], "--dummy-param") || !strcmp(p_arg_list[i], "-dp"))
            n_dummy_param = atol(p_arg_list[++ i]);
        else {
            fprintf(stderr, "error: argument \'%s\': an unknown argument\n", p_arg_list[i]);
            return false;
        }
    }
    // "parse" cmdline

    return true;
}
