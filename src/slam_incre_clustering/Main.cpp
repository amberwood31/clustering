//
// Created by amber on 2019-08-21.
//
int n_dummy_param = 0;
/**< @brief a dummy parameter, used as a convenient commandline input, intended for debugging / testing */
#include "slam_incre_clustering/Main.h"
#include "slam_incre_clustering/incre_solver.hpp"
#include "rrr/cluster.hpp"

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
    DisplaySwitches();
    CTimer t;
    double start, end;

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
        if((p_fr = fopen(t_cmd_args.p_s_input_file, "rb")))
            fclose(p_fr);
        else {
            fprintf(stderr, "error: can't open input file \'%s\'\n", t_cmd_args.p_s_input_file);
            return -1;
        }
    }
    // see if the input is there;

    // spatial clustering
    IntPairSet loops;
    bool graph_2d = LoadLoopClosures(t_cmd_args.p_s_input_file, loops);

    Clusterizer clusterizer;
    clusterizer.clusterize(loops, t_cmd_args.n_spatial_clustering_threshold);

    std::cout<<"Number of Clusters found : "<<clusterizer.clusterCount()<<std::endl;
    std::cout<<"print all clusters: " << std::endl;
    for(size_t i=0; i< clusterizer.clusterCount(); i++)
    {
        IntPairSet tempset = clusterizer.getClusterByID(i);
        for (IntPairSet::iterator temppair = tempset.begin(); temppair != tempset.end(); temppair++)
        {
            std::cout<<i<<": " << temppair->first << " " << temppair->second << std::endl;
        }
    }

    IntPairSet rejected_loops;
    const char *clustering_outputs = "clustering_results.txt";
    FILE * clustering_output_file = fopen(clustering_outputs, "w");


    start = t.f_Time();
    if (graph_2d)
    {
        Incre_Solver<true> incremental_solver;
        incremental_solver.load_dataset_into_object(t_cmd_args.p_s_input_file);

        for (size_t i=0; i< clusterizer.clusterCount();i++) // TODO_LOCAL: is this the most efficient way?
        {
            std::cout << "Analyzing cluster " << i << std::endl;
            IntPairSet cluster_i = clusterizer.getClusterByID(i);
            if (cluster_i.size() >1) // only analyze clusters that have multiple edges
            {
                IntPairSet outlier_set;
                IntPairDoubleMap temp_cluster_score_i, consistant_cluster_score_i;

                incremental_solver.initialize_solver();

                if (incremental_solver.analyze_edge_set(cluster_i, rejected_loops, outlier_set, t_cmd_args.f_chi2_dif_test_threshold, temp_cluster_score_i))
                {
                    consistant_cluster_score_i = temp_cluster_score_i;
                } else{

                    incremental_solver.initialize_solver();
                    consistant_cluster_score_i = incremental_solver.analyze_outlier_set(outlier_set, rejected_loops, t_cmd_args.f_chi2_dif_test_threshold);
                }

                std::cout << " " << std::endl; //add empty line to indicate clustering

                for (IntPairDoubleMap::const_iterator ite = consistant_cluster_score_i.begin(); ite != consistant_cluster_score_i.end(); ite++)
                {
                    fprintf(clustering_output_file, "CLUSTER %d %d %lf\n", ite->first.first, ite->first.second, ite->second);
                }
                fprintf(clustering_output_file, "\n"); // add one new line to separate clusters in text output file

                //std::cout << "leaving if loop" << std::endl;

            } else{
                fprintf(clustering_output_file, "CLUSTER %d %d %lf\n", cluster_i.begin()->first, cluster_i.begin()->second, 0.0);
                fprintf(clustering_output_file, "\n");
            }
            //std::cout << "leaving for loop" << std::endl;

        }
        // analyze the clusters

    }
    else
    {
        Incre_Solver<false> incremental_solver;
        incremental_solver.load_dataset_into_object(t_cmd_args.p_s_input_file);


        for (size_t i=0; i< clusterizer.clusterCount();i++) // TODO_LOCAL: is this the most efficient way?
        {
            std::cout << "Analyzing cluster " << i << std::endl;
            IntPairSet cluster_i = clusterizer.getClusterByID(i);
            if (cluster_i.size() >1) // only analyze clusters that have multiple edges
            {
                IntPairSet outlier_set;
                IntPairDoubleMap temp_cluster_score_i, consistant_cluster_score_i;

                incremental_solver.initialize_solver();

                if (incremental_solver.analyze_edge_set(cluster_i, rejected_loops, outlier_set, t_cmd_args.f_chi2_dif_test_threshold, temp_cluster_score_i))
                {
                    consistant_cluster_score_i = temp_cluster_score_i;
                } else{

                    incremental_solver.initialize_solver();
                    consistant_cluster_score_i = incremental_solver.analyze_outlier_set(outlier_set, rejected_loops, t_cmd_args.f_chi2_dif_test_threshold);
                }

                std::cout << " " << std::endl; //add empty line to indicate clustering

                for (IntPairDoubleMap::const_iterator ite = consistant_cluster_score_i.begin(); ite != consistant_cluster_score_i.end(); ite++)
                {
                    fprintf(clustering_output_file, "CLUSTER %d %d %lf\n", ite->first.first, ite->first.second, ite->second);
                }
                fprintf(clustering_output_file, "\n"); // add one new line to separate clusters in text output file

                //std::cout << "leaving if loop" << std::endl;

            } else{
                fprintf(clustering_output_file, "CLUSTER %d %d %lf\n", cluster_i.begin()->first, cluster_i.begin()->second, 0.0);
                fprintf(clustering_output_file, "\n");
            }
            //std::cout << "leaving for loop" << std::endl;

        }
        // analyze the clusters
    }


    for (IntPairSet::const_iterator ii = rejected_loops.begin(); ii != rejected_loops.end(); ii ++)
    {
        fprintf(clustering_output_file, "CLUSTER_R %d %d\n", ii->first, ii->second);
    }
    fclose(clustering_output_file);

    end = t.f_Time();
    double f_time = end - start;
    printf("\nwhole solving took " PRItimeprecise " (%f sec)\n", PRItimeparams(f_time), f_time);

    return 0;
}

bool LoadLoopClosures(const char* file_name, IntPairSet& loops)
{
    FILE * file = fopen(file_name, "r");
    char line[400];
    bool graph_2d;
    std::string edge_2d_signature("EDGE_SE2");
    std::string edge_3d_signature("EDGE_SE3:QUAT");
    while ( fgets (line , 400 , file) != NULL )
    {
        std::vector<std::string> strList = uListToVector(uSplit(uReplaceChar(line, '\n', ' '), ' '));
        if(strList.size() == 31 && strList[0] == edge_3d_signature)
        {
            //EDGE_SE3:QUAT
            graph_2d = false;
            int vertex_from = atoi(strList[1].c_str());
            int vertex_to = atoi(strList[2].c_str());

            //if (strList[0] == 3d_ed
            if ((vertex_to - vertex_from) != 1)
            {
                loops.insert(IntPair(vertex_from, vertex_to));
            }

        }
        else if(strList.size() == 12 && strList[0] == edge_2d_signature)
        {
            //EDGE_SE2
            graph_2d = true;
            int vertex_from = atoi(strList[1].c_str());
            int vertex_to = atoi(strList[2].c_str());

            if ((vertex_to - vertex_from) != 1) // if reached loop closure edge
            {
                loops.insert(IntPair(vertex_from, vertex_to));
            }

        }
        else if(strList.size())
        {
            fprintf(stderr, "Load_loop_closures: Error parsing graph file");
        }
    }

    return graph_2d;



}


void TCommandLineArgs::Defaults()
{
    b_show_commandline = true;
    b_show_flags = true;
    b_show_detailed_timing = true;
    b_verbose = false;
    // verbosity

    b_use_schur = false;

    b_use_old_system = false; // t_odo - make this commandline
    b_pose_only = false;
    b_use_SE3 = false; // note this is not overriden in commandline but detected in peek-parsing

    p_s_input_file = 0; /** <@brief path to the data file */
    n_max_lines_to_process = 0; /** <@brief maximal number of lines to process */
    n_spatial_clustering_threshold = 50;
    f_chi2_dif_test_threshold = 0.95;


    n_linear_solve_each_n_steps = 0; /**< @brief linear solve period, in steps (0 means disabled) */
    n_nonlinear_solve_each_n_steps = 0; /**< @brief nonlinear solve period, in steps (0 means disabled) */
    n_max_nonlinear_solve_iteration_num = 10; /**< @brief maximal number of iterations in nonlinear solve step */
    f_nonlinear_solve_error_threshold = 20; /**< @brief error threshold for nonlinear solve */
    n_max_final_optimization_iteration_num = 5; // as many other solvers
    f_final_optimization_threshold = 0.01;
    // optimization mode for slam


    n_omp_threads = size_t(-1);
    b_omp_dynamic = false;

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
        else if(!strcmp(p_arg_list[i], "--no-commandline") || !strcmp(p_arg_list[i], "-nc"))
            b_show_commandline = false;
        else if(!strcmp(p_arg_list[i], "--no-flags") || !strcmp(p_arg_list[i], "-nf"))
            b_show_flags = false;
        else if(!strcmp(p_arg_list[i], "--no-detailed-timing") || !strcmp(p_arg_list[i], "-ndt"))
            b_show_detailed_timing = false;
        else if(!strcmp(p_arg_list[i], "--use-old-code") || !strcmp(p_arg_list[i], "-uogc"))
            b_use_old_system = true;
        else if(!strcmp(p_arg_list[i], "--pose-only") || !strcmp(p_arg_list[i], "-po"))
            b_pose_only = true;
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
        else if(!strcmp(p_arg_list[i], "--n_spatial_clustering_threshold") || !strcmp(p_arg_list[i], "-cs"))
            n_spatial_clustering_threshold = atol(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--nonlinear-solve-error-thresh") || !strcmp(p_arg_list[i], "-nset"))
            f_nonlinear_solve_error_threshold = atof(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--chi2_difference_test_threshold") || !strcmp(p_arg_list[i], "-cdtt"))
            f_chi2_dif_test_threshold = atof(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--max-final-nonlinear-solve-iters") || !strcmp(p_arg_list[i], "-mfnsi"))
            n_max_final_optimization_iteration_num = atol(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--final-nonlinear-solve-error-thresh") || !strcmp(p_arg_list[i], "-fnset"))
            f_final_optimization_threshold = atof(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--omp-set-num-threads"))
            n_omp_threads = atol(p_arg_list[++ i]);
        else if(!strcmp(p_arg_list[i], "--omp-set-dynamic"))
            b_omp_dynamic = (atol(p_arg_list[++ i]) != 0);
        else if(!strcmp(p_arg_list[i], "--dummy-param") || !strcmp(p_arg_list[i], "-dp"))
            n_dummy_param = atol(p_arg_list[++ i]);
        else {
            fprintf(stderr, "error: argument \'%s\': an unknown argument\n", p_arg_list[i]);
            return false;
        }
    }
    // "parse" cmdline

    return true;
}


/**
 *	@brief prints all the important compiler / optimization switches this app was built with
 */
void DisplaySwitches()
{
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
    printf("SLAM++ version x64 (compiled at %s)\nbuilt with the following flags:\n", __DATE__);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
    printf("SLAM++ version x86 (compiled at %s)\nbuilt with the following flags:\n", __DATE__);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64

#ifdef _DEBUG
    printf("%s\n", "_DEBUG");
#endif // _DEBUG
#ifdef __FAST_MATH__
    printf("%s\n", "__FAST_MATH__");
#endif // __FAST_MATH__
#ifdef _OPENMP
#pragma omp parallel
#pragma omp master
    {
        printf("%s (%d threads)\n", "_OPENMP", omp_get_num_threads());
    }
#endif // _OPENMP

#ifdef __USE_NATIVE_CHOLESKY
    printf("%s\n", "__USE_NATIVE_CHOLESKY");
#elif defined(__USE_CHOLMOD)
    printf("%s\n", "__USE_CHOLMOD");
#ifdef __CHOLMOD_SHORT
	printf("%s\n", "__CHOLMOD_SHORT");
#endif // __CHOLMOD_SHORT
#else // __USE_CHOLMOD
#ifdef __USE_CXSPARSE
    printf("%s\n", "__USE_CXSPARSE");
#ifdef __CXSPARSE_SHORT
	printf("%s\n", "__CXSPARSE_SHORT");
#endif // __CXSPARSE_SHORT
#else // __USE_CXSPARSE
    printf("%s\n", "__USE_CSPARSE");
#endif // __USE_CXSPARSE
#endif // __USE_CHOLMOD

    const char *p_stacked_fmt[] = {"%s", ", %s", ",\n%s"},
        *p_stacked_fmtB[] = {"%s=%d", ", %s=%d", ",\n%s=%d"};
    const int n_stacking_cols = 3;
    int n_stacked_fmt = 0;
#define STACKED_FMT (p_stacked_fmt[(++ n_stacked_fmt - 1)? ((n_stacked_fmt - 1) % n_stacking_cols)? 1 : 2 : 0])
#define STACKED_FMTB (p_stacked_fmtB[(++ n_stacked_fmt - 1)? ((n_stacked_fmt - 1) % n_stacking_cols)? 1 : 2 : 0])
    // utility for stacking tokens into simple paragraphs

#ifdef GPU_BLAS
    printf("%s\n", "GPU_BLAS");
#endif // GPU_BLAS

#define MKSTRING(a) EXPANDSTRING(a)
#define EXPANDSTRING(a) #a
    printf(STACKED_FMT, "EIGEN_" MKSTRING(EIGEN_WORLD_VERSION) "." MKSTRING(EIGEN_MAJOR_VERSION)
                        "." MKSTRING(EIGEN_MINOR_VERSION));
    // always print Eigen version, now there are two to choose from

#ifdef EIGEN_VECTORIZE
    printf(STACKED_FMT, "EIGEN_VECTORIZE");
#endif // EIGEN_VECTORIZE
#ifdef EIGEN_UNALIGNED_VECTORIZE
    printf(STACKED_FMTB, "EIGEN_UNALIGNED_VECTORIZE", int(EIGEN_UNALIGNED_VECTORIZE));
#endif // EIGEN_UNALIGNED_VECTORIZE
#ifdef EIGEN_VECTORIZE_SSE
    printf(STACKED_FMT, "EIGEN_VECTORIZE_SSE");
#endif // EIGEN_VECTORIZE_SSE
#ifdef EIGEN_VECTORIZE_SSE2
    printf(STACKED_FMT, "EIGEN_VECTORIZE_SSE2");
#endif // EIGEN_VECTORIZE_SSE2
#ifdef EIGEN_VECTORIZE_SSE3
    printf(STACKED_FMT, "EIGEN_VECTORIZE_SSE3");
#endif // EIGEN_VECTORIZE_SSE3
#ifdef EIGEN_VECTORIZE_SSSE3
    printf(STACKED_FMT, "EIGEN_VECTORIZE_SSSE3");
#endif // EIGEN_VECTORIZE_SSSE3
#ifdef EIGEN_VECTORIZE_SSE4_1
    printf(STACKED_FMT, "EIGEN_VECTORIZE_SSE4_1");
#endif // EIGEN_VECTORIZE_SSE4_1
#ifdef EIGEN_VECTORIZE_SSE4_2
    printf(STACKED_FMT, "EIGEN_VECTORIZE_SSE4_2");
#endif // EIGEN_VECTORIZE_SSE4_2
#ifdef EIGEN_VECTORIZE_FMA
    printf(STACKED_FMT, "EIGEN_VECTORIZE_FMA");
#endif // EIGEN_VECTORIZE_FMA
#ifdef EIGEN_VECTORIZE_AVX
    printf(STACKED_FMT, "EIGEN_VECTORIZE_AVX");
#endif // EIGEN_VECTORIZE_AVX
#ifdef EIGEN_VECTORIZE_AVX2
    printf(STACKED_FMT, "EIGEN_VECTORIZE_AVX2");
#endif // EIGEN_VECTORIZE_AVX2
#ifdef EIGEN_VECTORIZE_AVX512
    printf(STACKED_FMT, "EIGEN_VECTORIZE_AVX512");
#endif // EIGEN_VECTORIZE_AVX512
#ifdef EIGEN_VECTORIZE_AVX512DQ
    printf(STACKED_FMT, "EIGEN_VECTORIZE_AVX512DQ");
#endif // EIGEN_VECTORIZE_AVX512DQ
#ifdef EIGEN_VECTORIZE_NEON
    printf(STACKED_FMT, "EIGEN_VECTORIZE_NEON");
#endif // EIGEN_VECTORIZE_NEON
    if(n_stacked_fmt)
        printf("\n");
    // print Eigen flags stacked

#ifdef __DISABLE_GPU
    printf("%s\n", "__DISABLE_GPU");
#endif // __DISABLE_GPU
#ifdef __SCHUR_USE_DENSE_SOLVER
    printf("%s\n", "__SCHUR_USE_DENSE_SOLVER");
#endif // __SCHUR_USE_DENSE_SOLVER
#ifdef __SCHUR_DENSE_SOLVER_USE_GPU
    printf("%s\n", "__SCHUR_DENSE_SOLVER_USE_GPU");
#endif // __SCHUR_DENSE_SOLVER_USE_GPU

#ifdef __BLOCK_BENCH_DUMP_MATRIX_IMAGES
    printf("%s\n", "__BLOCK_BENCH_DUMP_MATRIX_IMAGES");
#endif // __BLOCK_BENCH_DUMP_MATRIX_IMAGES
#ifdef __BLOCK_BENCH_BLOCK_TYPE_A
    printf("%s\n", "__BLOCK_BENCH_BLOCK_TYPE_A");
#else // __BLOCK_BENCH_BLOCK_TYPE_A
    printf("%s\n", "__BLOCK_BENCH_BLOCK_TYPE_B"); // nonexistent, but want to know
#endif // __BLOCK_BENCH_BLOCK_TYPE_A
#ifdef __BLOCK_BENCH_CHOLESKY_USE_AMD
    printf("%s\n", "__BLOCK_BENCH_CHOLESKY_USE_AMD");
#endif // __BLOCK_BENCH_CHOLESKY_USE_AMD
#ifdef __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT
    printf("%s\n", "__BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT");
#endif // __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT
#ifdef __BLOCK_ADD_UNIT_TEST_DUMP_MATRIX_IMAGES
    printf("%s\n", "__BLOCK_ADD_UNIT_TEST_DUMP_MATRIX_IMAGES");
#endif // __BLOCK_ADD_UNIT_TEST_DUMP_MATRIX_IMAGES
#ifdef __BLOCK_MUL_UNIT_TEST_DUMP_MATRIX_IMAGES
    printf("%s\n", "__BLOCK_MUL_UNIT_TEST_DUMP_MATRIX_IMAGES");
#endif // __BLOCK_MUL_UNIT_TEST_DUMP_MATRIX_IMAGES

#if 0 // this is not so interesting at this point (tuning the solvers, the matrix is stable)
    #ifdef __UBER_BLOCK_MATRIX_SUPRESS_FBS
	printf("%s\n", "__UBER_BLOCK_MATRIX_SUPRESS_FBS");
#endif // __UBER_BLOCK_MATRIX_SUPRESS_FBS
#ifdef __UBER_BLOCK_MATRIX_LAZY_PRODUCT
	printf("%s\n", "__UBER_BLOCK_MATRIX_LAZY_PRODUCT");
#endif // __UBER_BLOCK_MATRIX_LAZY_PRODUCT
#ifdef __UBER_BLOCK_MATRIX_FBS_LAZY_PRODUCT
	printf("%s\n", "__UBER_BLOCK_MATRIX_FBS_LAZY_PRODUCT");
#endif // __UBER_BLOCK_MATRIX_FBS_LAZY_PRODUCT
#ifdef __UBER_BLOCK_MATRIX_PERFCOUNTERS
	printf("%s\n", "__UBER_BLOCK_MATRIX_PERFCOUNTERS");
#endif // __UBER_BLOCK_MATRIX_PERFCOUNTERS
#ifdef __UBER_BLOCK_MATRIX_MULTIPLICATION_MEMORY_DEBUGGING
	printf("%s\n", "__UBER_BLOCK_MATRIX_MULTIPLICATION_MEMORY_DEBUGGING");
#endif // __UBER_BLOCK_MATRIX_MULTIPLICATION_MEMORY_DEBUGGING
#ifdef __UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR
	printf("%s\n", "__UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR");
#endif // __UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR
#ifdef __UBER_BLOCK_MATRIX_HYBRID_AT_A
	printf("%s\n", "__UBER_BLOCK_MATRIX_HYBRID_AT_A");
#endif // __UBER_BLOCK_MATRIX_HYBRID_AT_A
#ifdef __UBER_BLOCK_MATRIX_MULTIPLICATION_PREALLOCATES_BLOCK_LISTS
	printf("%s\n", "__UBER_BLOCK_MATRIX_MULTIPLICATION_PREALLOCATES_BLOCK_LISTS");
#endif // __UBER_BLOCK_MATRIX_MULTIPLICATION_PREALLOCATES_BLOCK_LISTS
	printf("CUberBlockMatrix::map_Alignment = %d (%d, %d)\n", CUberBlockMatrix::map_Alignment,
		CUberBlockMatrix::pool_MemoryAlignment, CUberBlockMatrix::_TyDenseAllocator::n_memory_align);
#endif // 0
#ifdef __UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON
    printf("%s\n", "__UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON"); // this one is new (Q2 2017)
#endif // __UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON
#ifdef __UBER_BLOCK_MATRIX_LEGACY_FBS_GEMM
    printf("%s\n", "__UBER_BLOCK_MATRIX_LEGACY_FBS_GEMM"); // this one is new (Q2 2017)
#endif // __UBER_BLOCK_MATRIX_LEGACY_FBS_GEMM
#ifdef __UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY
    printf("%s\n", "__UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY");
#endif // __UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY

#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
    printf("%s\n", "__SLAM_COUNT_ITERATIONS_AS_VERTICES");
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
#ifdef __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
    printf("%s\n", "__AUTO_UNARY_FACTOR_ON_VERTEX_ZERO");
#endif // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
#ifdef __MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT
    printf("%s\n", "__MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT");
#endif // __MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT
#ifdef __MATRIX_ORDERING_CACHE_ALIGN
    printf("%s\n", "__MATRIX_ORDERING_CACHE_ALIGN");
#endif // __MATRIX_ORDERING_CACHE_ALIGN
#ifdef __MATRIX_ORDERING_USE_AMD1
    printf("%s\n", "__MATRIX_ORDERING_USE_AMD1");
#endif // __MATRIX_ORDERING_USE_AMD1
#ifdef __MATRIX_ORDERING_USE_AMD_AAT
    printf("%s\n", "__MATRIX_ORDERING_USE_AMD_AAT");
#endif // __MATRIX_ORDERING_USE_AMD_AAT
#ifdef __MATRIX_ORDERING_USE_MMD
    printf("%s\n", "__MATRIX_ORDERING_USE_MMD");
#endif // __MATRIX_ORDERING_USE_MMD

#ifdef __SEGREGATED_MAKE_CHECKED_ITERATORS
    printf("%s\n", "__SEGREGATED_MAKE_CHECKED_ITERATORS");
#endif // __SEGREGATED_MAKE_CHECKED_ITERATORS
#ifdef __SEGREGATED_COMPILE_FAP_TESTS
    printf("%s\n", "__SEGREGATED_COMPILE_FAP_TESTS");
#endif // __SEGREGATED_COMPILE_FAP_TESTS

#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
    printf("%s\n", "__FLAT_SYSTEM_USE_THUNK_TABLE");
#ifdef __FLAT_SYSTEM_STATIC_THUNK_TABLE
    printf("%s\n", "__FLAT_SYSTEM_STATIC_THUNK_TABLE");
#endif // __FLAT_SYSTEM_STATIC_THUNK_TABLE
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
#ifdef __FLAT_SYSTEM_ALIGNED_MEMORY
    printf("%s\n", "__FLAT_SYSTEM_ALIGNED_MEMORY");
#endif // __FLAT_SYSTEM_ALIGNED_MEMORY

#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
    printf("%s\n", "__SE_TYPES_SUPPORT_A_SOLVERS");
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
    printf("%s\n", "__SE_TYPES_SUPPORT_LAMBDA_SOLVERS");
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
#ifdef __SE_TYPES_SUPPORT_L_SOLVERS
    printf("%s\n", "__SE_TYPES_SUPPORT_L_SOLVERS");
#endif // __SE_TYPES_SUPPORT_L_SOLVERS

#ifdef __BASE_TYPES_ALLOW_CONST_VERTICES
    printf("%s\n", "__BASE_TYPES_ALLOW_CONST_VERTICES");
#endif // __BASE_TYPES_ALLOW_CONST_VERTICES
#ifdef __SLAM_APP_USE_CONSTANT_VERTICES
    printf("%s\n", "__SLAM_APP_USE_CONSTANT_VERTICES");
#endif // __SLAM_APP_USE_CONSTANT_VERTICES
#ifdef __GRAPH_TYPES_ALIGN_OPERATOR_NEW
#if defined(_MSC_VER) && !defined(__MWERKS__)
#define MAKESTRING2(x) #x // msvc fails with a missing argument error when using double stringification
#else // _MSC_VER && !__MWERKS__
#define MAKESTRING(x) #x
#define MAKESTRING2(x) MAKESTRING(x)
#endif // _MSC_VER && !__MWERKS__
    if(*MAKESTRING2(__GRAPH_TYPES_ALIGN_OPERATOR_NEW)) // if not an empty macro
        printf("%s\n", "__GRAPH_TYPES_ALIGN_OPERATOR_NEW");
#endif // __GRAPH_TYPES_ALIGN_OPERATOR_NEW
#ifdef __BASE_TYPES_USE_ALIGNED_MATRICES
    printf("%s\n", "__BASE_TYPES_USE_ALIGNED_MATRICES");
#endif // __BASE_TYPES_USE_ALIGNED_MATRICES

#ifdef __NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2
    printf("%s\n", "__NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2");
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN
    printf("%s\n", "__LAMBDA_USE_V2_REDUCTION_PLAN");
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN

#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
    printf("%s\n", "__NONLINEAR_SOLVER_L_DUMP_TIMESTEPS");
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
#ifdef __NONLINEAR_SOLVER_L_DUMP_CHI2
    printf("%s\n", "__NONLINEAR_SOLVER_L_DUMP_CHI2");
#ifdef __NONLINEAR_SOLVER_L_DUMP_CHI2_AT_LAST_EDGE
	printf("%s\n", "__NONLINEAR_SOLVER_L_DUMP_CHI2_AT_LAST_EDGE");
#endif // __NONLINEAR_SOLVER_L_DUMP_CHI2_AT_LAST_EDGE
#endif // __NONLINEAR_SOLVER_L_DUMP_CHI2
#ifdef __NONLINEAR_SOLVER_L_DUMP_DENSITY
    printf("%s\n", "__NONLINEAR_SOLVER_L_DUMP_DENSITY");
#endif // __NONLINEAR_SOLVER_L_DUMP_DENSITY
#ifdef __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
    printf("%s\n", "__NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES");
#endif // __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
#ifdef __NONLINEAR_SOLVER_L_DETAILED_TIMING
    printf("%s\n", "__NONLINEAR_SOLVER_L_DETAILED_TIMING");
#endif // __NONLINEAR_SOLVER_L_DETAILED_TIMING
#ifdef __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
    printf("%s\n", "__NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST");
#endif // __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
#ifdef __NONLINEAR_SOLVER_L_USE_RESUMED_BACKSUBST
    printf("%s\n", "__NONLINEAR_SOLVER_L_USE_RESUMED_BACKSUBST");
#endif // __NONLINEAR_SOLVER_L_USE_RESUMED_BACKSUBST
#ifdef __NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY
    printf("%s\n", "__NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY");
#endif // __NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY

#ifdef __NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1
    printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1");
#endif // __NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1
#ifdef __NONLINEAR_SOLVER_FAST_L_ALWAYS_L_UPDATE
    printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_ALWAYS_L_UPDATE");
#endif // __NONLINEAR_SOLVER_FAST_L_ALWAYS_L_UPDATE
#ifdef __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING
    printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING");
#endif // __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING
#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS
    printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS");
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS
#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2
    printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_DUMP_CHI2");
#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2_AT_LAST_EDGE
	printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_DUMP_CHI2_AT_LAST_EDGE");
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2_AT_LAST_EDGE
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2
#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY
    printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY");
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY
#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
    printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES");
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
#ifdef __NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING
    printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING");
#endif // __NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING
#ifdef __NONLINEAR_SOLVER_FAST_L_ENABLE_DENSE_CHOLESKY
    printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_ENABLE_DENSE_CHOLESKY");
#endif // __NONLINEAR_SOLVER_FAST_L_ENABLE_DENSE_CHOLESKY
    printf("\n");
}

