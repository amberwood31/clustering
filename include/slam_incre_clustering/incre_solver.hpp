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

    void analyze_cluster(const char * input_file, Clusterizer& clusterizer, double threshold, const char * clustering_output_file)
    {
        load_dataset_into_object(input_file);
        FILE * clustering_outputs = fopen(clustering_output_file, "w");
        IntPairSet rejected_loops;

        for (size_t i=0; i< clusterizer.clusterCount();i++) // TODO_LOCAL: is this the most efficient way?
        {
            std::cout << "Analyzing cluster " << i << std::endl;
            IntPairSet cluster_i = clusterizer.getClusterByID(i);
            if (cluster_i.size() >1) // only analyze clusters that have multiple edges
            {
                IntPairSet outlier_set;
                IntPairDoubleMap temp_cluster_score_i, consistant_cluster_score_i;

                initialize_solver();

                if (analyze_edge_set(cluster_i, rejected_loops, outlier_set, threshold, temp_cluster_score_i))
                {
                    consistant_cluster_score_i = temp_cluster_score_i;
                } else{

                    initialize_solver();
                    consistant_cluster_score_i = analyze_outlier_set(outlier_set, rejected_loops, threshold);
                }

                std::cout << " " << std::endl; //add empty line to indicate clustering

                for (IntPairDoubleMap::const_iterator ite = consistant_cluster_score_i.begin(); ite != consistant_cluster_score_i.end(); ite++)
                {
                    fprintf(clustering_outputs, "CLUSTER %d %d %lf\n", ite->first.first, ite->first.second, ite->second);
                }
                fprintf(clustering_outputs, "\n"); // add one new line to separate clusters in text output file

                //std::cout << "leaving if loop" << std::endl;

            } else{
                fprintf(clustering_outputs, "CLUSTER %d %d %lf\n", cluster_i.begin()->first, cluster_i.begin()->second, 0.0);
                fprintf(clustering_outputs, "\n");
            }
            //std::cout << "leaving for loop" << std::endl;

        }
        // analyze the clusters


        for (IntPairSet::const_iterator ii = rejected_loops.begin(); ii != rejected_loops.end(); ii ++)
        {
            fprintf(clustering_outputs, "CLUSTER_R %d %d\n", ii->first, ii->second);
        }
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

        double del_obj_function = r_v_error.dot(cov_inv * r_v_error);
        //double mi_gain = log(innovation_cov.determinant() / information.inverse().determinant());// TODO_LOCAL: maybe this increases the processing time?

        return del_obj_function;
    }

    bool analyze_edge_set(IntPairSet& cluster, IntPairSet& rejected_loops, IntPairSet& Outlier_Set, float chi2_threshold, IntPairDoubleMap& loops_score)
    {
        int counter = 0;

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
                const bool is_in = cluster.find(edge_pair) != cluster.end();

                if (is_in == true)
                {
                    //std::cout <<"number of edges: " << system->n_Edge_Num() << std::endl;
                    //std::cout << "number of nodes: " << system->n_Vertex_Num() << std::endl;
                    counter +=1;
                    //solver->Enable_Optimization(); //enable optimization when there is a LC edge
                    solver->Optimize();
                    double delta_obj = calculate_ofc(new_edge, iter->t_information, iter->p_vertex[0], iter->p_vertex[1]);
                    // TODO_LOCAL: why iter->t_information is rvalue?

                    int dof = n_pose_dimension; // difference between previous iteration, instead of dof of the whole graph
                    // multiplied by 3 in 2D cases
                    double evil_scale = utils::p(fabs(delta_obj), dof);  // handle negative value
                    loops_score[edge_pair] = evil_scale;

                    {   // use chi2 difference test

                        std::cout << "edge: " << iter->p_vertex[0] << " "  << iter->p_vertex[1] << " " << evil_scale << std::endl;
                        if (fabs(delta_obj) < utils::chi2(dof, chi2_threshold))
                        {

                            //solver->Incremental_Step(system->r_Add_Edge(new_edge)); // incrementally solve
                            system->r_Add_Edge(new_edge);

                        }
                    }

                }
            }
            else{ // odometry edge
                //solver->Delay_Optimization(); //don't start optimizing right away
                //solver->Incremental_Step(system->r_Add_Edge(new_edge));
                system->r_Add_Edge(new_edge);

            }

            if (counter >= cluster.size())
                break;

        }


        std::cout <<"number of edges: " << system->n_Edge_Num() << std::endl;
        std::cout << "number of nodes: " << system->n_Vertex_Num() << std::endl;


        IntPairSet Inlier_Set;
        if (examine_loops_score(loops_score, Inlier_Set, Outlier_Set, chi2_threshold) == true)
        {
            return true;
        }
        else{
            if (Inlier_Set.size() > Outlier_Set.size()) // TODO_LOCAL: whether to include equal case here?
            {
                if (Inlier_Set.size() == 1 && Outlier_Set.size() == 1) //in this case, there is not enough evidence to believe this inlier is actually an inlier
                {
                    loops_score[ *Inlier_Set.begin()] = 0.0;
                }
                for(IntPairSet::const_iterator ip = Outlier_Set.begin(); ip != Outlier_Set.end(); ip++)
                {
                    cluster.erase(*ip); // delete outliers from the input cluster
                    loops_score.erase(*ip);
                    rejected_loops.insert(*ip);
                }
                return true;
            }
            else
            {
                for(IntPairSet::const_iterator ip = Inlier_Set.begin(); ip != Inlier_Set.end(); ip++)
                {
                    rejected_loops.insert(*ip);
                }
                std::cout << "Need second test" << std::endl;
                return false;

            }


        }

    }

    bool examine_loops_score(const IntPairDoubleMap& loops_score, IntPairSet& Inlier_Set, IntPairSet& Outlier_Set, double chi2_threshold)
    {
        bool allInlier = true;
        for(IntPairDoubleMap::const_iterator it = loops_score.begin(); it!=loops_score.end(); it++)
        {
            if (it->second > chi2_threshold)
            {
                Outlier_Set.insert(it->first);
                allInlier = false;
            }else
            {
                Inlier_Set.insert(it->first);
            }
        }
        return allInlier;

    }

    IntPairDoubleMap analyze_outlier_set(IntPairSet& cluster, IntPairSet & rejected_loops, float chi2_threshold)
    {
        std::cout <<"number of edges(should be 0): " << system->n_Edge_Num() << std::endl;
        std::cout << "number of nodes(should be 0: " << system->n_Vertex_Num() << std::endl;

        int counter = 0;

        IntPairDoubleMap loops_score;
        bool first_outlier_added = false;

        for (typename std::vector<_TyEdgeData>::const_iterator iter = edges.begin(); iter != edges.end(); iter ++)
        {

            _TyEdgePose new_edge = _TyEdgePose(iter->p_vertex[0], iter->p_vertex[1], iter->v_measurement, iter->t_information, * system);

            if ((iter->p_vertex[1] - iter->p_vertex[0]) != 1) // if reached loop closure edge
            {

                //solver.Optimize(5, 1e-5);

                IntPair edge_pair(iter->p_vertex[0], iter->p_vertex[1]);
                const bool is_in = cluster.find(edge_pair) != cluster.end();
                if (is_in == true)
                {
                    counter +=1;

                    //solver->Enable_Optimization(); //enable optimization when there is a LC edge
                    solver->Optimize(5, 1e-5);
                    double delta_obj = calculate_ofc(new_edge, iter->t_information, iter->p_vertex[0], iter->p_vertex[1]);

                    int dof = n_pose_dimension * 1; // difference between previous iteration, instead of the current dof
                    // multiplied by 3 in 2D cases
                    double evil_scale = utils::p(fabs(delta_obj), dof);  // handle negative value
                    std::cout << "edge: " << iter->p_vertex[0] << " "  << iter->p_vertex[1] << " " << evil_scale << std::endl;

                    // all edges entered this function should eventually get score 0.0, but not here.
                    // because their actual score will be needed later to determine their status within this cluster
                    if (first_outlier_added == true)
                    {
                        loops_score[edge_pair] = evil_scale;

                        // use chi2 difference test
                        if (fabs(delta_obj) < utils::chi2(dof, chi2_threshold))
                        {
                            //solver->Incremental_Step(system->r_Add_Edge(new_edge)); // incrementally solve
                            system->r_Add_Edge(new_edge);

                        }

                    } else{
                        first_outlier_added = true;
                        system->r_Add_Edge(new_edge);
                        loops_score[edge_pair] = 0.0 ; //1st edge is  accepted anyway
                    }

                }


            }
            else{ // odometry edge
                //solver->Delay_Optimization(); //don't start optimizing right away
                //solver->Incremental_Step(system->r_Add_Edge(new_edge));
                system->r_Add_Edge(new_edge);
            }

            if (counter >= cluster.size())
                break;

        }

        std::cout <<"number of edges: " << system->n_Edge_Num() << std::endl;
        std::cout << "number of nodes: " << system->n_Vertex_Num() << std::endl;

        IntPairSet Inlier_Set, Outlier_Set;
        if (examine_loops_score(loops_score, Inlier_Set, Outlier_Set, chi2_threshold))
        {
            for(IntPairDoubleMap::iterator id = loops_score.begin(); id != loops_score.end(); id++)
            {
                id->second = 0.0; // zero all the score here
            }

            return loops_score;
        }
        else{
            if (Inlier_Set.size() >= Outlier_Set.size()) // TODO_LOCAL: need to include equal case here.
                // Otherwise could get stuck in this function when there are only two inconsistant edges in the cluster
            {

                if (Inlier_Set.size() == Outlier_Set.size())
                {
                    std::cout << "TIE" << std::endl;
                }
                for(IntPairSet::const_iterator ip = Outlier_Set.begin(); ip != Outlier_Set.end(); ip++)
                {
                    cluster.erase(*ip); // delete outliers from the input cluster
                    loops_score.erase((*ip));
                    rejected_loops.insert(*ip);
                }

                for(IntPairDoubleMap::iterator id = loops_score.begin(); id != loops_score.end(); id++)
                {
                    id->second = 0.0; // zero all the score here
                }
                return loops_score;
            }
            else // TODO_LOCAL: what to do if cluster is still mixed
            {
                for(IntPairSet::const_iterator ip = Inlier_Set.begin(); ip != Inlier_Set.end(); ip++)
                {
                    rejected_loops.insert(*ip);
                }

                std::cout << "Need third test" << std::endl;
                initialize_solver(); // clean the system and solver
                return analyze_outlier_set(Outlier_Set, rejected_loops, chi2_threshold);

            }


        }

    }

};

#endif //SLAM_PLUS_PLUS_INCRE_SOLVER_HPP
