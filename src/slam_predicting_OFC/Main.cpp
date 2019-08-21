//
// Created by amber on 2019-08-21.
//
int n_dummy_param = 0;
/**< @brief a dummy parameter, used as a convenient commandline input, intended for debugging / testing */
#include "slam_predicting_OFC/Main.h"

#include <stdio.h> // printf
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
    TCommandLineArgs t_cmd_args;
    t_cmd_args.Defaults(); // set defaults
    if(!t_cmd_args.Parse(n_arg_num, p_arg_list))
        return -1;
    // parse commandline

    if(!t_cmd_args.p_s_input_file) {
        fprintf(stderr, "error: no input file specified;\n");

        return -1;
    }

    if(t_cmd_args.b_show_commandline) {
        printf("> ./SLAM_plus_plus");
        for(int i = 1; i < n_arg_num; ++ i)
            printf(" %s", p_arg_list[i]);
        printf("\n");
    }
    // display commandline



    {
        FILE *p_fr;
        if((p_fr = fopen(t_cmd_args.p_s_input_file, "rb")))
            fclose(p_fr);
        else {
            fprintf(stderr, "error: can't open input file \'%s\'\n", t_cmd_args.p_s_input_file);
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
    TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy( true, frequency::Never(), mpart_Diagonal, mpart_Diagonal);

    CNonlinearSolver_Lambda<CSystemType, CLinearSolverType> solver(system, t_incremental_config, t_marginals_config);

    std::string file_path = "/home/amber/pose_dataset/vertigo/manhattan/originalDataset/Olson/manhattanOlson3500.g2o";
    load_graph(file_path, system);
    system.Plot2D("resultUnoptim.tga", plot_quality::plot_Printing); // plot in print quality

    solver.Optimize();
    // optimize the system

    solver.r_MarginalCovariance().Dump_Diagonal();
    system.Plot2D("result.tga", plot_quality::plot_Printing); // plot in print quality
    solver.Dump(); // show some stats

    return 0;
}

template <class CSystemType>
void Add_More_ManhattanEdges(CSystemType &system, Eigen::Matrix3d information)
{
    system.r_Add_Edge(CEdgePose2D(4, 5, Eigen::Vector3d(1.01603, 0.0145648, -0.0163041), information, system));
    system.r_Add_Edge(CEdgePose2D(5, 6, Eigen::Vector3d(1.02389, 0.00680757, 0.0109813), information, system));
    system.r_Add_Edge(CEdgePose2D(6, 7, Eigen::Vector3d(0.957734, 0.00315932, 0.0109005), information, system));
    system.r_Add_Edge(CEdgePose2D(7, 8, Eigen::Vector3d(-1.02382, -0.0136683, -3.09324), information, system));
    system.r_Add_Edge(CEdgePose2D(5, 9, Eigen::Vector3d(0.0339432, 0.0324387, -3.1274), information, system));
    system.r_Add_Edge(CEdgePose2D(8, 9, Eigen::Vector3d(1.02344, 0.0139844, -0.00780158), information, system));
    system.r_Add_Edge(CEdgePose2D(3, 10, Eigen::Vector3d(0.0440195, 0.988477, -1.56353), information, system));
    system.r_Add_Edge(CEdgePose2D(9, 10, Eigen::Vector3d(1.00335, 0.0222496, 0.0234909), information, system));
    system.r_Add_Edge(CEdgePose2D(10, 11, Eigen::Vector3d(0.977245, 0.0190423, -0.0286232), information, system));
    system.r_Add_Edge(CEdgePose2D(11, 12, Eigen::Vector3d(-0.99688, -0.0255117, 3.15627), information, system));
    system.r_Add_Edge(CEdgePose2D(12, 13, Eigen::Vector3d(0.990646, 0.0183964, -0.0165195), information, system));
    system.r_Add_Edge(CEdgePose2D(8, 14, Eigen::Vector3d(0.0158085, 0.0210588, 3.12831), information, system));
    system.r_Add_Edge(CEdgePose2D(13, 14, Eigen::Vector3d(0.945873, 0.00889308, -0.00260169), information, system));
    system.r_Add_Edge(CEdgePose2D(7, 15, Eigen::Vector3d(-0.0147277, -0.00159495, -0.0195786), information, system));
    system.r_Add_Edge(CEdgePose2D(14, 15, Eigen::Vector3d(1.00001, 0.00642824, 0.0282342), information, system));
    system.r_Add_Edge(CEdgePose2D(15, 16, Eigen::Vector3d(0.0378719, -1.02609, -1.5353), information, system));
    system.r_Add_Edge(CEdgePose2D(16, 17, Eigen::Vector3d(0.98379, 0.019891, 0.0240848), information, system));
    system.r_Add_Edge(CEdgePose2D(17, 18, Eigen::Vector3d(0.957199, 0.0295867, -0.0115004), information, system));
    system.r_Add_Edge(CEdgePose2D(18, 19, Eigen::Vector3d(0.99214, 0.0192015, -0.00729783), information, system));
    system.r_Add_Edge(CEdgePose2D(19, 20, Eigen::Vector3d(-0.0459215, -1.01632, -1.53912), information, system));
    system.r_Add_Edge(CEdgePose2D(20, 21, Eigen::Vector3d(0.99845, -0.00523202, -0.0340973), information, system));
    system.r_Add_Edge(CEdgePose2D(21, 22, Eigen::Vector3d(0.988728, 0.00903381, -0.0129141), information, system));
    system.r_Add_Edge(CEdgePose2D(22, 23, Eigen::Vector3d(0.989422, 0.00698231, -0.0242835), information, system));
    system.r_Add_Edge(CEdgePose2D(23, 24, Eigen::Vector3d(-1.00201, -0.00626341, 3.13974), information, system));
    system.r_Add_Edge(CEdgePose2D(24, 25, Eigen::Vector3d(1.01535, 0.00491314, 3.02279e-05), information, system));
    system.r_Add_Edge(CEdgePose2D(21, 26, Eigen::Vector3d(-0.95214, -0.0418463, 3.13475), information, system));
    system.r_Add_Edge(CEdgePose2D(25, 26, Eigen::Vector3d(1.03299, -0.00172652, 0.0224073), information, system));
    system.r_Add_Edge(CEdgePose2D(19, 27, Eigen::Vector3d(-0.0176158, -0.0052181, 1.56791), information, system));
    system.r_Add_Edge(CEdgePose2D(26, 27, Eigen::Vector3d(0.989137, -0.00857052, -0.0209045), information, system));
    system.r_Add_Edge(CEdgePose2D(27, 28, Eigen::Vector3d(-0.0483998, 0.981715, 1.56408), information, system));
    system.r_Add_Edge(CEdgePose2D(28, 29, Eigen::Vector3d(1.03082, -0.021271, -0.06069), information, system));
    system.r_Add_Edge(CEdgePose2D(29, 30, Eigen::Vector3d(1.01192, 0.0164477, -0.0352014), information, system));
    system.r_Add_Edge(CEdgePose2D(30, 31, Eigen::Vector3d(0.991338, 0.00781231, 0.0305919), information, system));
    system.r_Add_Edge(CEdgePose2D(31, 32, Eigen::Vector3d(0.00861057, -0.974025, -1.56961), information, system));
    system.r_Add_Edge(CEdgePose2D(32, 33, Eigen::Vector3d(1.04256, 0.0106692, 0.0220136), information, system));
    system.r_Add_Edge(CEdgePose2D(33, 34, Eigen::Vector3d(0.990826, 0.0166949, -0.0427845), information, system));
    system.r_Add_Edge(CEdgePose2D(34, 35, Eigen::Vector3d(0.995988, 0.029526, -0.0144112), information, system));
    system.r_Add_Edge(CEdgePose2D(35, 36, Eigen::Vector3d(-0.0107743, 0.996051, 1.59416), information, system));
    system.r_Add_Edge(CEdgePose2D(36, 37, Eigen::Vector3d(1.00499, 0.0110863, -0.00316511), information, system));
    system.r_Add_Edge(CEdgePose2D(37, 38, Eigen::Vector3d(1.03843, 0.0146778, -0.0323211), information, system));
    system.r_Add_Edge(CEdgePose2D(38, 39, Eigen::Vector3d(1.00625, 0.00674436, -0.0280641), information, system));
    system.r_Add_Edge(CEdgePose2D(39, 40, Eigen::Vector3d(0.0561635, 0.984988, -4.70377), information, system));
    system.r_Add_Edge(CEdgePose2D(40, 41, Eigen::Vector3d(0.984656, -0.0319246, 0.0110837), information, system));
    system.r_Add_Edge(CEdgePose2D(41, 42, Eigen::Vector3d(1.00266, 0.030635, 0.0300476), information, system));
    system.r_Add_Edge(CEdgePose2D(42, 43, Eigen::Vector3d(0.986417, -0.0130982, -0.0241183), information, system));
    system.r_Add_Edge(CEdgePose2D(43, 44, Eigen::Vector3d(0.97872, 0.0120778, -0.0117429), information, system));
    system.r_Add_Edge(CEdgePose2D(44, 45, Eigen::Vector3d(0.996113, -0.0407306, -0.0152182), information, system));
    system.r_Add_Edge(CEdgePose2D(45, 46, Eigen::Vector3d(1.00255, -0.00216301, -0.0105021), information, system));
    system.r_Add_Edge(CEdgePose2D(46, 47, Eigen::Vector3d(0.999641, -0.0336501, 0.0188071), information, system));
    system.r_Add_Edge(CEdgePose2D(47, 48, Eigen::Vector3d(-0.949748, 0.0117583, 3.11376), information, system));
    system.r_Add_Edge(CEdgePose2D(48, 49, Eigen::Vector3d(1.01739, 0.0123797, -0.00893411), information, system));
    system.r_Add_Edge(CEdgePose2D(49, 50, Eigen::Vector3d(1.01548, 0.0274024, -0.019191), information, system));
    system.r_Add_Edge(CEdgePose2D(40, 51, Eigen::Vector3d(2.97743, 0.0326539, 3.1211), information, system));
    system.r_Add_Edge(CEdgePose2D(50, 51, Eigen::Vector3d(1.05227, 0.0147383, -0.00136236), information, system));
    system.r_Add_Edge(CEdgePose2D(51, 52, Eigen::Vector3d(-0.0108141, -0.98436, -1.56099), information, system));
    system.r_Add_Edge(CEdgePose2D(52, 53, Eigen::Vector3d(1.03071, 0.00895876, -0.00840075), information, system));
    system.r_Add_Edge(CEdgePose2D(53, 54, Eigen::Vector3d(0.98342, 0.00979391, -0.0306844), information, system));
    system.r_Add_Edge(CEdgePose2D(7, 55, Eigen::Vector3d(-0.0335046, -0.00680906, -1.58407), information, system));
    system.r_Add_Edge(CEdgePose2D(54, 55, Eigen::Vector3d(1.01204, -0.015331, 0.00584842), information, system));
    system.r_Add_Edge(CEdgePose2D(14, 56, Eigen::Vector3d(0.00385417, 0.000186059, -3.14717), information, system));
    system.r_Add_Edge(CEdgePose2D(8, 56, Eigen::Vector3d(-0.0196555, 0.00673762, 0.0127182), information, system));
    system.r_Add_Edge(CEdgePose2D(55, 56, Eigen::Vector3d(-0.00365754, -0.984986, -1.57285), information, system));
    system.r_Add_Edge(CEdgePose2D(5, 57, Eigen::Vector3d(-0.048046, -0.00753482, -3.16285), information, system));
    system.r_Add_Edge(CEdgePose2D(56, 57, Eigen::Vector3d(1.031, -0.0163252, -0.0169613), information, system));
    system.r_Add_Edge(CEdgePose2D(57, 58, Eigen::Vector3d(0.983393, -0.0113447, -0.0148402), information, system));
    system.r_Add_Edge(CEdgePose2D(58, 59, Eigen::Vector3d(1.01024, 0.011576, 0.00432891), information, system));
    system.r_Add_Edge(CEdgePose2D(59, 60, Eigen::Vector3d(0.0201084, -1.00859, 4.73677), information, system));
    system.r_Add_Edge(CEdgePose2D(60, 61, Eigen::Vector3d(0.992544, -0.00406336, 0.00906878), information, system));
    system.r_Add_Edge(CEdgePose2D(61, 62, Eigen::Vector3d(0.980911, -0.0126781, 0.0247609), information, system));
    system.r_Add_Edge(CEdgePose2D(62, 63, Eigen::Vector3d(1.00765, -0.0370944, -0.00745089), information, system));
    system.r_Add_Edge(CEdgePose2D(47, 64, Eigen::Vector3d(-0.992098, -0.0164591, 3.12281), information, system));
    system.r_Add_Edge(CEdgePose2D(63, 64, Eigen::Vector3d(-0.0145417, -0.998609, -1.54739), information, system));
    system.r_Add_Edge(CEdgePose2D(64, 65, Eigen::Vector3d(1.03794, -0.0168313, -0.0130817), information, system));
    system.r_Add_Edge(CEdgePose2D(65, 66, Eigen::Vector3d(0.9912, 0.0115711, -0.0249519), information, system));
    system.r_Add_Edge(CEdgePose2D(66, 67, Eigen::Vector3d(0.949443, -0.0154924, -0.0091255), information, system));
    system.r_Add_Edge(CEdgePose2D(43, 68, Eigen::Vector3d(0.993623, 0.0391936, -0.00106149), information, system));
    system.r_Add_Edge(CEdgePose2D(67, 68, Eigen::Vector3d(-0.978361, -0.00927414, -3.13791), information, system));
    system.r_Add_Edge(CEdgePose2D(45, 69, Eigen::Vector3d(-0.006758, -0.00679624, -0.00213649), information, system));
    system.r_Add_Edge(CEdgePose2D(68, 69, Eigen::Vector3d(1.00367, -0.0352973, 0.0340684), information, system));
    system.r_Add_Edge(CEdgePose2D(69, 70, Eigen::Vector3d(1.02981, 0.00255454, 0.0150012), information, system));
    system.r_Add_Edge(CEdgePose2D(70, 71, Eigen::Vector3d(1.03652, 0.0118072, -0.00163612), information, system));
    system.r_Add_Edge(CEdgePose2D(71, 72, Eigen::Vector3d(0.00398168, -0.993979, 4.69836), information, system));
    system.r_Add_Edge(CEdgePose2D(72, 73, Eigen::Vector3d(0.969371, -0.0306017, -0.0326511), information, system));
    system.r_Add_Edge(CEdgePose2D(73, 74, Eigen::Vector3d(0.985691, 0.0111442, -0.00166414), information, system));
    system.r_Add_Edge(CEdgePose2D(74, 75, Eigen::Vector3d(0.981205, -0.00596464, 0.0226695), information, system));
    system.r_Add_Edge(CEdgePose2D(75, 76, Eigen::Vector3d(-0.00825988, 0.981841, -4.71863), information, system));
    system.r_Add_Edge(CEdgePose2D(76, 77, Eigen::Vector3d(1.01399, 0.0332094, -0.0649213), information, system));
    system.r_Add_Edge(CEdgePose2D(77, 78, Eigen::Vector3d(1.02795, 0.00984078, 0.0340066), information, system));
    system.r_Add_Edge(CEdgePose2D(78, 79, Eigen::Vector3d(1.00265, -0.00774271, 0.00950595), information, system));
    system.r_Add_Edge(CEdgePose2D(79, 80, Eigen::Vector3d(-0.0102099, -0.978673, 4.74423), information, system));
    system.r_Add_Edge(CEdgePose2D(80, 81, Eigen::Vector3d(1.01265, 0.0192011, -0.00199527), information, system));
    system.r_Add_Edge(CEdgePose2D(81, 82, Eigen::Vector3d(0.994241, -0.0319085, -0.0197558), information, system));
    system.r_Add_Edge(CEdgePose2D(82, 83, Eigen::Vector3d(1.00925, 0.00590969, -0.0214812), information, system));
    system.r_Add_Edge(CEdgePose2D(83, 84, Eigen::Vector3d(-0.0184826, 1.03307, -4.72819), information, system));
    system.r_Add_Edge(CEdgePose2D(84, 85, Eigen::Vector3d(0.984696, 0.0196236, 0.00877518), information, system));
    system.r_Add_Edge(CEdgePose2D(85, 86, Eigen::Vector3d(0.993027, 0.010799, 0.0107298), information, system));
    system.r_Add_Edge(CEdgePose2D(86, 87, Eigen::Vector3d(0.992905, 0.0213607, 0.0110665), information, system));
    system.r_Add_Edge(CEdgePose2D(87, 88, Eigen::Vector3d(0.00121839, 1.04031, 1.53711), information, system));
    system.r_Add_Edge(CEdgePose2D(88, 89, Eigen::Vector3d(1.00767, -0.0150986, 0.0147958), information, system));
    system.r_Add_Edge(CEdgePose2D(89, 90, Eigen::Vector3d(1.01226, -0.00539061, 0.030011), information, system));
    system.r_Add_Edge(CEdgePose2D(90, 91, Eigen::Vector3d(1.03457, 0.00297329, -0.00901519), information, system));
    system.r_Add_Edge(CEdgePose2D(91, 92, Eigen::Vector3d(-0.0159521, 0.972423, 1.55259), information, system));
    system.r_Add_Edge(CEdgePose2D(92, 93, Eigen::Vector3d(0.990753, 0.0620248, -0.0146912), information, system));
    system.r_Add_Edge(CEdgePose2D(93, 94, Eigen::Vector3d(0.971423, 0.0142496, 0.000217408), information, system));
    system.r_Add_Edge(CEdgePose2D(94, 95, Eigen::Vector3d(1.02272, -0.0278824, 0.000365479), information, system));
    system.r_Add_Edge(CEdgePose2D(95, 96, Eigen::Vector3d(-0.0193242, 1.04934, 1.57253), information, system));
    system.r_Add_Edge(CEdgePose2D(96, 97, Eigen::Vector3d(1.03931, -0.0130887, 0.0113687), information, system));
    system.r_Add_Edge(CEdgePose2D(97, 98, Eigen::Vector3d(0.993004, 0.0393656, -0.0105709), information, system));
    system.r_Add_Edge(CEdgePose2D(98, 99, Eigen::Vector3d(1.03897, -0.0238964, -0.0173253), information, system));
    system.r_Add_Edge(CEdgePose2D(99, 100, Eigen::Vector3d(-0.985853, -0.00979836, -3.15198), information, system));
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
    b_verbose = true;
    // verbosity

    b_use_schur = false;

    b_run_matrix_benchmarks = false;
    b_run_matrix_unit_tests = false;
    b_use_old_system = false; // t_odo - make this commandline
    b_pose_only = false;
    b_use_SE3 = false; // note this is not overriden in commandline but detected in peek-parsing

    p_s_input_file = 0; /** <@brief path to the data file */
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
bool load_graph(const std::string &fileName, CSystemType &system){

    FILE * file = 0;
    file = fopen(fileName.c_str(), "r");

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
                fprintf(stderr, "Error parsing graph file %s on line \"%s\" (strList.size()=%d)", fileName.c_str(), line, (int)strList.size());
            }
        }

        fprintf(stdin, "Graph loaded from %s", fileName.c_str());
        fclose(file);
    }
    else
    {
        fprintf(stderr, "Cannot open file %s", fileName.c_str());
        return false;
    }
    return true;

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
