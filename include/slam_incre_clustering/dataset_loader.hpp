//
// Created by amber on 2019-10-25.
// The struct TEdgeData is extracted from the slam_compact_pose_ijrr example in SLAM_PP source code
//

#ifndef SLAM_PLUS_PLUS_DATASET_LOADER_H
#define SLAM_PLUS_PLUS_DATASET_LOADER_H


#include "slam/SE2_Types.h" // SE(2) types
#include "slam/SE3_Types.h" // SE(3) types
#include "slam_incre_clustering/utilities.h"
#include <stdio.h> // fget



/**
*	@brief parsed edge data
*	@tparam n_dimension is measurement dimension (either 3 for 2D or 6 for 3D)
*/
template <const int n_dimension = 3>
struct TEdgeData {
    typedef Eigen::Matrix<double, n_dimension, n_dimension, Eigen::DontAlign> _TyMatrix; /**< @brief matrix type */
    typedef Eigen::Matrix<double, n_dimension, 1, Eigen::DontAlign> _TyVector; /**< @brief vector type */
    size_t p_vertex[2]; /**< @brief indices of the vertices */
    _TyVector v_measurement; /**< @brief measurement vector */
    _TyMatrix t_information; /**< @brief information matrix */

    /**
     *	@brief default constructor; has no effect
     */
    TEdgeData()
    {}

    /**
     *	@brief constructor; initializes the edge data
     *
     *	@param[in] n_vertex_0 is index of the first vertex
     *	@param[in] n_vertex_1 is index of the second vertex
     *	@param[in] r_v_measurement is measurement vector
     *	@param[in] r_t_information is information matrix
     */
    TEdgeData(size_t n_vertex_0, size_t n_vertex_1,
              const _TyVector &r_v_measurement, const _TyMatrix &r_t_information)
        :v_measurement(r_v_measurement), t_information(r_t_information)
    {
        p_vertex[0] = n_vertex_0;
        p_vertex[1] = n_vertex_1;
    }

    /**
     *	@brief comparison operator
     *	@param[in] r_edge is an edge
     *	@return Returns true if this edge connects the same two vertices
     *		as r_edge does (in any order), otherwise returns false.
     */
    bool operator ==(const std::pair<size_t, size_t> &r_edge) const
    {
        return (p_vertex[0] == r_edge.first && p_vertex[1] == r_edge.second) ||
               (p_vertex[1] == r_edge.first && p_vertex[0] == r_edge.second);
    }
};

typedef TEdgeData<3> TEdgeData2D; /**< @brief 2D edge data type */
typedef TEdgeData<6> TEdgeData3D; /**< @brief 3D edge data type */



class Dataset_Loader
{
public:


    typedef TEdgeData<3> TEdgeData2D; /**< @brief 2D edge data type */
    typedef TEdgeData<6> TEdgeData3D; /**< @brief 3D edge data type */



    bool Load_Dataset(const char *p_s_filename, std::vector<TEdgeData2D> &r_edges)
    {
        r_edges.clear(); // !!

        char line[400];
        FILE * file = fopen(p_s_filename, "r");
        while ( fgets (line , 400 , file) != NULL ) {
            std::vector<std::string> strList = uListToVector(uSplit(uReplaceChar(line, '\n', ' '), ' '));
            if (strList.size() == 12) {
                //EDGE_SE2
                int vertex_from = atoi(strList[1].c_str());
                int vertex_to = atoi(strList[2].c_str());
                double x = uStr2Double(strList[3]);
                double y = uStr2Double(strList[4]);
                double rot = uStr2Double(strList[5]);

                TEdgeData2D::_TyVector measurement;
                measurement << x, y, rot;

                TEdgeData2D::_TyMatrix information;
                information << uStr2Double(strList[6]), uStr2Double(strList[7]), uStr2Double(strList[8]),
                    uStr2Double(strList[7]), uStr2Double(strList[9]), uStr2Double(strList[10]),
                    uStr2Double(strList[8]), uStr2Double(strList[10]), uStr2Double(strList[11]);

                r_edges.push_back(TEdgeData2D(vertex_from, vertex_to, measurement, information));
            }
        }
        fclose(file);

        printf("loaded \'%s\' : " PRIsize " edges\n", p_s_filename, r_edges.size());

        return true;
    }

    bool Load_Dataset(const char *p_s_filename, std::vector<TEdgeData3D> &r_edges)
    {
        r_edges.clear(); // !!

        char line[400];
        FILE * file = fopen(p_s_filename, "r");
        while ( fgets (line , 400 , file) != NULL ) {
            std::vector<std::string> strList = uListToVector(uSplit(uReplaceChar(line, '\n', ' '), ' '));
            if (strList.size() == 31) {
                //EDGE_SE3::QUAT
                int vertex_from = atoi(strList[1].c_str());
                int vertex_to = atoi(strList[2].c_str());
                double x = uStr2Double(strList[3]);
                double y = uStr2Double(strList[4]);
                double z = uStr2Double(strList[5]);
                double qx = uStr2Double(strList[6]);
                double qy = uStr2Double(strList[7]);
                double qz = uStr2Double(strList[8]);
                double qw = uStr2Double(strList[9]);

                Eigen::Quaternion<double> quat(qw, qx, qy, qz);
                Eigen::Matrix3d rot_matrix = quat.toRotationMatrix();
                Eigen::Vector3d axis = C3DJacobians::v_RotMatrix_to_AxisAngle(rot_matrix);

                TEdgeData3D::_TyVector measurement;
                measurement << x, y, z, axis(0), axis(1), axis(2);

                TEdgeData3D::_TyMatrix information;
                information << uStr2Double(strList[10]), uStr2Double(strList[11]), uStr2Double(strList[12]), uStr2Double(strList[13]), uStr2Double(strList[14]), uStr2Double(strList[15]),
                                uStr2Double(strList[11]), uStr2Double(strList[16]), uStr2Double(strList[17]), uStr2Double(strList[18]), uStr2Double(strList[19]), uStr2Double(strList[20]),
                                uStr2Double(strList[12]), uStr2Double(strList[17]), uStr2Double(strList[21]), uStr2Double(strList[22]), uStr2Double(strList[23]), uStr2Double(strList[24]),
                                uStr2Double(strList[13]), uStr2Double(strList[18]), uStr2Double(strList[22]), uStr2Double(strList[25]), uStr2Double(strList[26]), uStr2Double(strList[27]),
                                uStr2Double(strList[14]), uStr2Double(strList[19]), uStr2Double(strList[23]), uStr2Double(strList[26]), uStr2Double(strList[28]), uStr2Double(strList[29]),
                                uStr2Double(strList[15]), uStr2Double(strList[20]), uStr2Double(strList[24]), uStr2Double(strList[27]), uStr2Double(strList[29]), uStr2Double(strList[30]);


                r_edges.push_back(TEdgeData3D(vertex_from, vertex_to, measurement, information));
            }
        }
        fclose(file);
        printf("loaded \'%s\' : " PRIsize " edges\n", p_s_filename, r_edges.size());

        return true;
    }


};

#endif //SLAM_PLUS_PLUS_DATASET_LOADER_H
