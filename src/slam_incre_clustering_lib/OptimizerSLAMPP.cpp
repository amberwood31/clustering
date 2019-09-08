//
// Created by amber on 2019-09-06.
//


#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP
#include "slam/ConfigSolvers.h" // nonlinear graph solvers
#include "slam/LinearSolver_UberBlock.h"
#include "slam/LinearSolver_CholMod.h"
//#include "slam/LinearSolver_Schur.h"
//#include "slam/LinearSolver_CSparse.h"
//#include "slam/LinearSolver_CXSparse.h" // linear solvers (only one is required)
//#include "slam/SE2_Types.h" // SE(2) types, will be included in OptimizerSLAMPP.h
#include "slam/SE3_Types.h"
#include "slam_app/ParsePrimitives.h"
#include "slam_incre_clustering_lib/OptimizerSLAMPP.h"

#include <list>

/*
 *								=== CDelayedOptimizationCaller ===
 */

/**
 *	@brief utility class for enabling or disabling optimization in nonlinear solvers
 *
 *	@tparam CSolverType is nonlinear solver type
 *	@tparam b_can_delay is delayed optimization support flag
 */
template <class CSolverType, const bool b_can_delay>
class CDelayedOptimizationCaller {
public:
    /**
     *	@brief delays optimization upon Incremental_Step()
     *	@param[in] r_solver is solver to delay the optimization at (unused)
     */
    static void Delay(CSolverType &UNUSED(r_solver))
    {}

    /**
     *	@brief enables optimization upon Incremental_Step()
     *
     *	This is default behavior. In case it was disabled (by Delay_Optimization()),
     *	and optimization is required, this will also run the optimization.
     *
     *	@param[in] r_solver is solver to enable the optimization at (unused)
     */
    static void Enable(CSolverType &UNUSED(r_solver))
    {}
};

/**
 *	@brief utility class for enabling or disabling optimization in nonlinear solvers
 *		(specialization for solvers which support enabling or disabling optimization)
 *
 *	@tparam CSolverType is nonlinear solver type
 */
template <class CSolverType>
class CDelayedOptimizationCaller<CSolverType, true> {
public:
    /**
     *	@brief delays optimization upon Incremental_Step()
     *	@param[in] r_solver is solver to delay the optimization at
     */
    static void Delay(CSolverType &r_solver)
    {
        r_solver.Delay_Optimization();
    }

    /**
     *	@brief enables optimization upon Incremental_Step()
     *
     *	This is default behavior. In case it was disabled (by Delay_Optimization()),
     *	and optimization is required, this will also run the optimization.
     *
     *	@param[in] r_solver is solver to enable the optimization at
     */
    static void Enable(CSolverType &r_solver)
    {
        r_solver.Enable_Optimization();
    }
};

/*
 *								=== ~CDelayedOptimizationCaller ===
 */


/*
 *								=== CSLAMOptimizer ===
 */

class CSLAMOptimizer::CSLAMOptimizerCore {
public:
    typedef MakeTypelist(CVertexPose2D) TVertexTypelist;
    typedef MakeTypelist(CEdgePose2D) TEdgeTypelist;
    typedef CFlatSystem<CVertexPose2D, TVertexTypelist, CEdgePose2D, TEdgeTypelist> CSystemType;

    typedef CLinearSolver_UberBlock<CSystemType::_TyHessianMatrixBlockList> CLinearSolverType;
    typedef CNonlinearSolver_FastL<CSystemType, CLinearSolverType> CNonlinearSolverType;
    CEdgePose2D new_edge;
    int vertex_from;
    int vertex_to;
    Eigen::Matrix3d new_edge_information;

protected:
    CSystemType m_system;
    CNonlinearSolverType m_solver;


public:
    inline CSLAMOptimizerCore(bool b_verbose = false, bool b_use_schur = false, int max_iteration = 5, double error_tolerance = 1e-5)
        :m_solver(m_system, TIncrementalSolveSetting(solve::nonlinear, frequency::Every(1), max_iteration, error_tolerance),
            TMarginalsComputationPolicy(true, frequency::Every(1), EBlockMatrixPart(mpart_LastColumn | mpart_Diagonal), EBlockMatrixPart(mpart_LastColumn | mpart_Diagonal), mpart_Nothing),
            b_verbose, CLinearSolverType(), b_use_schur)
    {}

    CSystemType::_TyConstVertexRef r_Vertex(size_t n_index) const
    {
        return m_system.r_Vertex_Pool()[n_index];
    }

    CSystemType::_TyVertexRef r_Vertex(size_t n_index)
    {
        return m_system.r_Vertex_Pool()[n_index];
    }

    inline CSystemType &r_System()
    {
        return m_system;
    }

    inline CNonlinearSolverType &r_Solver()
    {
        return m_solver;
    }

    inline const CSystemType &r_System() const
    {
        return m_system;
    }

    inline const CNonlinearSolverType &r_Solver() const
    {
        return m_solver;
    }

private:
    CSLAMOptimizerCore(const CSLAMOptimizerCore &r_other); // no copy
    CSLAMOptimizerCore &operator =(const CSLAMOptimizerCore &r_other); // no copy
};

CSLAMOptimizer::CSLAMOptimizer(bool b_verbose, bool b_use_schur, int max_iteration, double error_tolerance) // throw(srd::bad_alloc)
    :m_p_optimizer(new CSLAMOptimizerCore(b_verbose, b_use_schur, max_iteration, error_tolerance))
{}

CSLAMOptimizer::~CSLAMOptimizer()
{
    delete m_p_optimizer;
}

size_t CSLAMOptimizer::n_Vertex_Num() const
{
    return m_p_optimizer->r_System().r_Vertex_Pool().n_Size();
}


Eigen::Map<Eigen::VectorXd> CSLAMOptimizer::r_Vertex_State(size_t n_index)
{
    return m_p_optimizer->r_Vertex(n_index).v_State();
}

void CSLAMOptimizer::Delay_Optimization()
{
    CDelayedOptimizationCaller<CSLAMOptimizerCore::CNonlinearSolverType,
        CSLAMOptimizerCore::CNonlinearSolverType::solver_HasDelayedOptimization>::Delay(m_p_optimizer->r_Solver());
    // some solvers do not implement this
}

void CSLAMOptimizer::Enable_Optimization() // throw(srd::bad_alloc, std::runtime_error)
{
    CDelayedOptimizationCaller<CSLAMOptimizerCore::CNonlinearSolverType,
        CSLAMOptimizerCore::CNonlinearSolverType::solver_HasDelayedOptimization>::Enable(m_p_optimizer->r_Solver());
    // some solvers do not implement this
}

void CSLAMOptimizer::Optimize(size_t n_max_iteration_num /*= 5*/, double f_min_dx_norm /*= .01*/) // throw(srd::bad_alloc, std::runtime_error)
{
    m_p_optimizer->r_Solver().Optimize(n_max_iteration_num, f_min_dx_norm);
}

void CSLAMOptimizer::Show_Stats() const
{
    m_p_optimizer->r_Solver().Dump();
}

bool CSLAMOptimizer::Dump_State(const char *p_s_filename) const
{
    return m_p_optimizer->r_System().Dump(p_s_filename);
}

void CSLAMOptimizer::Generate_plot() const
{
    m_p_optimizer->r_System().Plot2D("result.tga", plot_quality::plot_Printing); // plot in print quality
}

bool CSLAMOptimizer::Load_New_Edge(int id_from, int id_to, double x, double y, double rot, Eigen::Matrix3d information)
{
    m_p_optimizer->new_edge = CEdgePose2D(id_from, id_to, Eigen::Vector3d(x, y, rot), information, m_p_optimizer->r_System());
    m_p_optimizer->vertex_from = id_from;
    m_p_optimizer->vertex_to = id_to;
    m_p_optimizer->new_edge_information = information;
}

double CSLAMOptimizer::Calculate_Ofc(FILE *full_analysis_file) {
    Eigen::Matrix3d r_t_jacobian0, r_t_jacobian1, cov_inv, innovation_cov;
    Eigen::Vector3d r_v_expectation, r_v_error;
    m_p_optimizer->new_edge.Calculate_Jacobians_Expectation_Error(r_t_jacobian0, r_t_jacobian1, r_v_expectation,r_v_error);
    // optimize the system

    Eigen::MatrixXd joined_matrix(3,6);
    joined_matrix << - r_t_jacobian0, - r_t_jacobian1;
    Eigen::MatrixXd marginal_covariance(6,6);

    Eigen::MatrixXd covariance_idfrom(3,3), covariance_idto(3,3), covariance_idtofrom(3,3), covariance_idfromto(3,3);
    m_p_optimizer->r_Solver().r_MarginalCovariance().save_Diagonal(covariance_idfrom, m_p_optimizer->vertex_from, m_p_optimizer->vertex_from);
    m_p_optimizer->r_Solver().r_MarginalCovariance().save_Diagonal(covariance_idto, m_p_optimizer->vertex_to, m_p_optimizer->vertex_to);
    if (m_p_optimizer->vertex_from > m_p_optimizer->vertex_to)
    {
        m_p_optimizer->r_Solver().r_MarginalCovariance().save_Diagonal(covariance_idfromto, m_p_optimizer->vertex_from, m_p_optimizer->vertex_to);
        marginal_covariance << covariance_idfrom, covariance_idfromto, covariance_idfromto.transpose(), covariance_idto;
    } else{
        m_p_optimizer->r_Solver().r_MarginalCovariance().save_Diagonal(covariance_idfromto, m_p_optimizer->vertex_to, m_p_optimizer->vertex_from);
        marginal_covariance << covariance_idfrom, covariance_idfromto.transpose(), covariance_idfromto, covariance_idto;

    }

    innovation_cov = joined_matrix* marginal_covariance * joined_matrix.transpose() + m_p_optimizer->new_edge_information.inverse();
    cov_inv = innovation_cov.inverse();

    //std::cout << "transform norm:  " << r_v_error.norm() << std::endl;
    //std::cout << "information norm: " << cov_inv.norm() << std::endl;

    double del_obj_function = r_v_error.dot(cov_inv * r_v_error);
    //double mi_gain = log(innovation_cov.determinant() / information.inverse().determinant());// TODO_LOCAL: maybe this increases the processing time?
    fprintf(full_analysis_file, "%d %d %lf\n", m_p_optimizer->vertex_from, m_p_optimizer->vertex_to, del_obj_function);


    return del_obj_function;
}

void CSLAMOptimizer::Increment_Once(){

    m_p_optimizer->r_Solver().Incremental_Step(m_p_optimizer->r_System().r_Add_Edge(m_p_optimizer->new_edge));

}


/**
 *	@brief prints all the important compiler / optimization switches this app was built with
 */
void CSLAMOptimizer::DisplaySwitches(){

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


