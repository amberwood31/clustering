//
// Created by amber on 2019-10-23.
//

#ifndef SLAM_PLUS_PLUS_INCRE_SOLVER_HPP
#define SLAM_PLUS_PLUS_INCRE_SOLVER_HPP
#include "slam/LinearSolver_UberBlock.h"
#include "slam/LinearSolver_CholMod.h"
#include "slam/ConfigSolvers.h" // nonlinear graph solvers
#include "slam/SE2_Types.h" // SE(2) types
#include "slam/SE3_Types.h"
#include "slam_app/ParsePrimitives.h"
#include "rrr/types.hpp"
#include "slam_incre_clustering/utils.hpp"
#include "slam_incre_clustering/dataset_loader.hpp"
#include "rrr/cluster.hpp"



template <const bool b_2D_SLAM>
class Incre_Solver{

    Dataset_Loader dataset_loader;

public:
    /**
     *	@brief parameters stored as enum
     */
    enum {
        n_pose_dimension = (b_2D_SLAM)? 3 : 6,                 /**< @brief vertex dimension */
        n_pose_dimension_doubled = (b_2D_SLAM)? 6 : 12
    };

    typedef TEdgeData<n_pose_dimension> _TyEdgeData; /**< @brief parsed edge type */
    typedef Eigen::Matrix<double, n_pose_dimension, n_pose_dimension> _TyMatrix; /**< @copydoc _TyEdgeData::_TyMatrix */
    typedef Eigen::Matrix<double, n_pose_dimension, 1> _TyVector; /**< @copydoc _TyEdgeData::_TyVector */
    typedef Eigen::Matrix<double, n_pose_dimension, n_pose_dimension_doubled> _TyJoinMatrix;
    typedef Eigen::Matrix<double, n_pose_dimension_doubled, n_pose_dimension_doubled> _TyCovMatrix;

    typedef typename CChooseType<CVertexPose2D, CVertexPose3D, b_2D_SLAM>::_TyResult _TyVertexPose; /**< @brief pose vertex type */
    typedef typename CChooseType<CEdgePose2D, CEdgePose3D, b_2D_SLAM>::_TyResult _TyEdgePose; /**< @brief pose-pose edge type */


    typedef typename MakeTypelist(_TyVertexPose) TVertexTypelist; /**< @brief list of vertex types */
    typedef typename MakeTypelist(_TyEdgePose) TEdgeTypelist; /**< @brief list of edge types */
    typedef CFlatSystem<_TyVertexPose, TVertexTypelist, _TyEdgePose, TEdgeTypelist> CSystemType; /**< @brief optimized system type */
    typedef CLinearSolver_UberBlock<typename CSystemType::_TyHessianMatrixBlockList> CLinearSolverType; /**< @brief linear solver type */
    typedef CNonlinearSolver_Lambda<CSystemType, CLinearSolverType> CSolverType; /**< @brief nonlinear solver type */



protected:
    CSystemType* system = NULL;
    CSolverType* solver = NULL;
    std::vector<_TyEdgeData> edges;
    //std::vector<_TyEdgeData> loop_closures;

public:
    Incre_Solver()
    {}

    ~Incre_Solver()
    {
        if (system != NULL)
        {
            std::cout << "deleted final system" << std::endl;
            delete system;
        }

        if (solver != NULL)
        {
            std::cout << "deleted final solver" << std::endl;
            delete solver;
        }
    }

    void analyze_cluster(const char * input_file, double threshold, const char * clustering_output_file)
    {
        load_dataset_into_object(input_file);
        FILE * clustering_outputs = fopen(clustering_output_file, "w");

        initialize_solver();

        analyze_edge_set(threshold, clustering_outputs);
        // analyze the clusters

        fclose(clustering_outputs);

    }

    void initialize_solver()
    {
        if (system != NULL)
            delete system;
        if (solver != NULL)
            delete solver;

        TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy( true, frequency::Every(1), EBlockMatrixPart(mpart_LastColumn | mpart_Diagonal), EBlockMatrixPart(mpart_LastColumn | mpart_Diagonal));
        //t_marginals_config.OnCalculate_marginals(false);

        system = new CSystemType();
        solver = new CSolverType(*system, solve::Nonlinear(frequency::Every(1), 5, 1e-5), t_marginals_config);

    }

    bool load_dataset_into_object(const char *p_s_input_file)
    {
        if(!dataset_loader.Load_Dataset(p_s_input_file, edges))
            return false;
        // put all dataset edges into edges

//        for(size_t i = 0, n = edges.size(); i < n; ++ i) {
//            const _TyEdgeData &r_edge = edges[i];
//            if((r_edge.p_vertex[1] - r_edge.p_vertex[0]) != 1 ) {
//                loop_closures.push_back(r_edge);
//                // put it to the list of loop closures
//
//            }
//            // check loop closures
//        }

        return true;
    }



    double calculate_ofc(_TyEdgePose &new_edge, const _TyMatrix &information, int vertex_from, int vertex_to)
    {
        _TyMatrix r_t_jacobian0, r_t_jacobian1, cov_inv, innovation_cov;
        _TyVector r_v_expectation, r_v_error;
        new_edge.Calculate_Jacobians_Expectation_Error(r_t_jacobian0, r_t_jacobian1, r_v_expectation,r_v_error);

        _TyJoinMatrix joined_matrix;
        joined_matrix << - r_t_jacobian0, - r_t_jacobian1;
        _TyCovMatrix marginal_covariance;

        Eigen::MatrixXd covariance_idfrom, covariance_idto, covariance_idtofrom, covariance_idfromto;
        // TODO_LOCAL: why using _TyMatrix instead of Eigen::MatrixXd here cause compilation error about rvalue in next line//
        solver->r_MarginalCovariance().save_Diagonal(covariance_idfrom, vertex_from, vertex_from);
        solver->r_MarginalCovariance().save_Diagonal(covariance_idto, vertex_to, vertex_to);
        if (vertex_from > vertex_to)
        {
            solver->r_MarginalCovariance().save_Diagonal(covariance_idfromto, vertex_from, vertex_to);
            marginal_covariance << covariance_idfrom, covariance_idfromto, covariance_idfromto.transpose(), covariance_idto;
        } else{
            solver->r_MarginalCovariance().save_Diagonal(covariance_idfromto, vertex_to, vertex_from);
            marginal_covariance << covariance_idfrom, covariance_idfromto.transpose(), covariance_idfromto, covariance_idto;
        }

        const CUberBlockMatrix &r_marginals = solver->r_MarginalCovariance().r_SparseMatrix();
        // get the marginals

        //std::cout<< "in Main: matrix n_rows: " << r_marginals.n_Row_Num() << ", n_columns: " << r_marginals.n_Column_Num() << std::endl;
        _ASSERTE(r_marginals.n_BlockColumn_Num() == system->r_Vertex_Pool().n_Size());


        innovation_cov = joined_matrix* marginal_covariance * joined_matrix.transpose() + information.inverse();
        cov_inv = innovation_cov.inverse();
//        std::cout << "marginal_covariance: " << marginal_covariance << std::endl;
//        std::cout << "edge_information: " << information << std::endl;

        double del_obj_function = r_v_error.dot(cov_inv * r_v_error);
        //double mi_gain = log(innovation_cov.determinant() / information.inverse().determinant());// TODO_LOCAL: maybe this increases the processing time?
//        std::cout << "error vector: " << r_v_error << std::endl;
//        std::cout << "innovation_cov: " << innovation_cov << std::endl;
//        std::cout << "cov_inv: " << cov_inv << std::endl;
//        std::cout << "del_obj_func: "<< del_obj_function << std::endl;
        return del_obj_function;
    }

    bool analyze_edge_set(float chi2_threshold, FILE* file)
    {

        std::cout <<"number of edges(should be 0): " << system->n_Edge_Num() << std::endl;
        std::cout << "number of nodes(should be 0): " << system->n_Vertex_Num() << std::endl;

        for (typename std::vector<_TyEdgeData>::const_iterator iter = edges.begin(); iter != edges.end(); iter ++)
        {

            _TyEdgePose new_edge = _TyEdgePose(iter->p_vertex[0], iter->p_vertex[1], iter->v_measurement, iter->t_information, *system);

            if ((iter->p_vertex[1] - iter->p_vertex[0]) != 1) // if reached loop closure edge
            {

                //solver.Optimize(5, 1e-5);

                //double before = solver.get_residual_chi2_error();
                IntPair edge_pair(iter->p_vertex[0], iter->p_vertex[1]);

                //std::cout <<"number of edges: " << system->n_Edge_Num() << std::endl;
                //std::cout << "number of nodes: " << system->n_Vertex_Num() << std::endl;
                //solver->Enable_Optimization(); //enable optimization when there is a LC edge
                solver->Optimize(30, 1e-8);
                double delta_obj = calculate_ofc(new_edge, iter->t_information, iter->p_vertex[0], iter->p_vertex[1]);


                int dof = n_pose_dimension; // difference between previous iteration, instead of dof of the whole graph
                // multiplied by 3 in 2D cases
                double evil_scale = utils::p(fabs(delta_obj), dof);  // handle negative value

                fprintf(file, "%ld %ld %lf %lf\n", iter->p_vertex[0], iter->p_vertex[1], delta_obj, evil_scale);


                {   // use chi2 difference test

                    std::cout << "edge: " << iter->p_vertex[0] << " "  << iter->p_vertex[1] << " " << evil_scale << std::endl;
                    if (fabs(delta_obj) < utils::chi2(dof, chi2_threshold))
                    {

                        //solver->Incremental_Step(system->r_Add_Edge(new_edge)); // incrementally solve
                        system->r_Add_Edge(new_edge);

                    }
                }


            }
            else{ // odometry edge
                //solver->Delay_Optimization(); //don't start optimizing right away
                //solver->Incremental_Step(system->r_Add_Edge(new_edge));
                system->r_Add_Edge(new_edge);

            }

        }


        std::cout <<"number of edges: " << system->n_Edge_Num() << std::endl;
        std::cout << "number of nodes: " << system->n_Vertex_Num() << std::endl;

        system->Plot2D("result.tga", plot_quality::plot_Printing);
        system->Dump("output.txt");

    }



};

#endif //SLAM_PLUS_PLUS_INCRE_SOLVER_HPP
