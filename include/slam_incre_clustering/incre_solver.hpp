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
#include "slam_incre_clustering/utilities.h"



class solver_2d{
public:
    typedef MakeTypelist(CVertexPose2D) TVertexTypelist;
    typedef MakeTypelist(CEdgePose2D) TEdgeTypelist;
    typedef CFlatSystem<CVertexPose2D, TVertexTypelist, CEdgePose2D, TEdgeTypelist> CSystemType;
    typedef CLinearSolver_UberBlock<CSystemType::_TyHessianMatrixBlockList> CLinearSolverType;
    typedef CNonlinearSolver_Lambda<CSystemType, CLinearSolverType> CSolverType;


protected:
    CSystemType* system = NULL;
    CSolverType* solver = NULL;

public:
    solver_2d()
    {}

    ~solver_2d()
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


    double calculate_ofc( CEdgePose2D &new_edge, Eigen::MatrixXd &information, int vertex_from, int vertex_to)
    {
        Eigen::Matrix3d r_t_jacobian0, r_t_jacobian1, cov_inv, innovation_cov;
        Eigen::Vector3d r_v_expectation, r_v_error;
        new_edge.Calculate_Jacobians_Expectation_Error(r_t_jacobian0, r_t_jacobian1, r_v_expectation,r_v_error);

        Eigen::MatrixXd joined_matrix(3,6);
        joined_matrix << - r_t_jacobian0, - r_t_jacobian1;
        Eigen::MatrixXd marginal_covariance(6,6);

        Eigen::MatrixXd covariance_idfrom(3,3), covariance_idto(3,3), covariance_idtofrom(3,3), covariance_idfromto(3,3);
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
        double mi_gain = log(innovation_cov.determinant() / information.inverse().determinant());// TODO_LOCAL: maybe this increases the processing time?

        return del_obj_function;
    }

    bool analyze_edge_set(FILE * file_pointer, IntPairSet& cluster, IntPairSet& rejected_loops, IntPairSet& Outlier_Set, float chi2_threshold, IntPairDoubleMap& loops_score)
    {
        char line[400];
        int counter = 0;

        std::cout <<"number of edges(should be 0): " << system->n_Edge_Num() << std::endl;
        std::cout << "number of nodes(should be 0): " << system->n_Vertex_Num() << std::endl;

        while ( fgets (line , 400 , file_pointer) != NULL and counter < cluster.size())
        {
            //solver->Delay_Optimization(); //don't start optimizing right away
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
                new_edge = CEdgePose2D(vertex_from, vertex_to, Eigen::Vector3d(x, y, rot), information, *system);

                if ((vertex_to - vertex_from) != 1) // if reached loop closure edge
                {

                    //solver.Optimize(5, 1e-5);

                    //double before = solver.get_residual_chi2_error();
                    IntPair edge_pair(vertex_from, vertex_to);
                    const bool is_in = cluster.find(edge_pair) != cluster.end();

                    if (is_in == true)
                    {
                        //std::cout <<"number of edges: " << system->n_Edge_Num() << std::endl;
                        //std::cout << "number of nodes: " << system->n_Vertex_Num() << std::endl;
                        counter +=1;
                        //solver->Enable_Optimization(); //enable optimization when there is a LC edge
                        solver->Optimize();
                        double delta_obj = calculate_ofc(new_edge, information, vertex_from, vertex_to);

                        int dof = 3 * 1; // difference between previous iteration, instead of dof of the whole graph
                        // multiplied by 3 in 2D cases
                        double evil_scale = utils::p(fabs(delta_obj), dof);  // handle negative value
                        loops_score[edge_pair] = evil_scale;

                        {   // use chi2 difference test

                            std::cout << "edge: " << vertex_from << " "  << vertex_to << " " << evil_scale << std::endl;
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

            }
            else if(strList.size())
            {
                fprintf(stderr, "analyze_edge_set: Error parsing graph file");
            }
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

    IntPairDoubleMap analyze_outlier_set(FILE * file_pointer, IntPairSet& cluster, IntPairSet & rejected_loops, float chi2_threshold)
    {
        rewind(file_pointer); // reset the file pointer to the beginning
        std::cout <<"number of edges(should be 0): " << system->n_Edge_Num() << std::endl;
        std::cout << "number of nodes(should be 0: " << system->n_Vertex_Num() << std::endl;

        char line[400];
        int counter = 0;

        IntPairDoubleMap loops_score;
        bool first_outlier_added = false;

        while ( fgets (line , 400 , file_pointer) != NULL and counter < cluster.size())
        {
            //solver->Delay_Optimization(); //don't start optimizing right away
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
                new_edge = CEdgePose2D(vertex_from, vertex_to, Eigen::Vector3d(x, y, rot), information, * system);

                if ((vertex_to - vertex_from) != 1) // if reached loop closure edge
                {

                    //solver.Optimize(5, 1e-5);

                    IntPair edge_pair(vertex_from, vertex_to);
                    const bool is_in = cluster.find(edge_pair) != cluster.end();
                    if (is_in == true)
                    {
                        counter +=1;

                        //solver->Enable_Optimization(); //enable optimization when there is a LC edge
                        solver->Optimize(5, 1e-5);
                        double delta_obj = calculate_ofc(new_edge, information,  vertex_from, vertex_to);

                        int dof = 3 * 1; // difference between previous iteration, instead of the current dof
                        // multiplied by 3 in 2D cases
                        double evil_scale = utils::p(fabs(delta_obj), dof);  // handle negative value
                        std::cout << "edge: " << vertex_from << " "  << vertex_to << " " << evil_scale << std::endl;

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

            }
            else if(strList.size())
            {
                fprintf(stderr, "analyze_outlier_set: Error parsing graph file");
            }
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
                return analyze_outlier_set(file_pointer, Outlier_Set, rejected_loops, chi2_threshold);

            }


        }

    }

};

#endif //SLAM_PLUS_PLUS_INCRE_SOLVER_HPP
