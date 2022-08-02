//
// Created by sanek on 22.06.2020.
//
#include <iostream>
#include <string>
#include <fstream>
#include <cmath>

// #include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <chrono>
#include <random> // for std::random_device и std::mt19937
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT; // type of point in points_cloud
typedef pcl::PointCloud<PointT> PointCloudT; //type of Points_Cloud


void PC_tetrahedron_generation(PointCloudT::Ptr cloud, const std::vector<float> & tetraedron_params, int density) // density is points to unit of edge
{   
    std::vector<int> num_points_faces = {floor(tetraedron_params[0]*density), floor(tetraedron_params[1]*density), floor(tetraedron_params[2]*density)};
    // coordinates generation
    std::vector<std::vector<float>> coordinates_of_mesh_axis(3);
    for (int ind_face = 0; ind_face < num_points_faces.size(); ind_face++)
    {
        for (int ind_step = 0; ind_step < num_points_faces[ind_face]; ind_step++)
        {
            coordinates_of_mesh_axis[ind_face].push_back(float(ind_step)/float(density));
        }
    }
    for (int ind_axis_first = 0; ind_axis_first < coordinates_of_mesh_axis.size(); ind_axis_first++)
    {
        for (int ind_step_first = 0; ind_step_first < num_points_faces[ind_axis_first]; ind_step_first++)
        {
            for (int ind_axis_second = ind_axis_first; ind_axis_second < coordinates_of_mesh_axis.size(); ind_axis_second++)
            {
                for (int ind_step_second = 0; ind_step_second < num_points_faces[ind_axis_second]; ind_step_second++)
                {
                    std::vector<float> vector_point = {0.0, 0.0, 0.0};
                    vector_point[ind_axis_first] = coordinates_of_mesh_axis[ind_axis_first][ind_step_first];
                    vector_point[ind_axis_second] = coordinates_of_mesh_axis[ind_axis_second][ind_step_second];
                    PointT point(vector_point[0], vector_point[1], vector_point[2]);
                    cloud->push_back(point);
                }
            }        
        }
    }
}


void Visualization(const PointCloudT::ConstPtr cloud_pcl)
{
    // To create window for demonstration of Points Cloud
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color (cloud_pcl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud_pcl, cloud_color);
    
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
    float zoom;
    zoom = 50.0;
    // Set camera position and orientation
    viewer.setCameraPosition (-zoom-3.68332, zoom+2.94092, zoom+5.71266, zoom+0.289847, zoom+0.921947, -zoom-0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size

    // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce();
    }
}

int main(int argc, char* argv[])
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen (seed);
    std::uniform_real_distribution<double> dis (-1.0,1.0);
    std::cout << "Hello from Sanya)\n";
    PointCloudT::Ptr cloud(new PointCloudT);  // Original point cloud

    std::vector<float> tetrahedron_params = {3, 6, 9};
    int density = 5; // density is points to unit of edge(1 m) 

    PC_tetrahedron_generation(cloud, tetrahedron_params, density);

    Visualization(cloud);

    // std::string file_PC;
    // std::ofstream fout;
    // std::string output_dir;
    // std::string out_path_for_PC;
    // std::string out_path_for_Tr_M;

    // if(pcl::console::parse_argument(argc, argv, "-o", output_dir) == -1)
    // {
    //     PCL_ERROR ("Need an output directory! Please use <input cloud> -o <output dir>\n");
    //     return(-1);
    // }

    // // Defining a rotation matrix and translation vector
    // Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
    // double theta = M_PI / 2;  // The angle of rotation in radians
    // double theta_r;

    // out_path_for_PC = output_dir + "/tetrahedron_base.pcd";
    // std::cout << out_path_for_PC << std::endl;

    // pcl::io::savePCDFile (out_path_for_PC, *cloud);
    // std::cerr << "Saved " << cloud->points.size() << " data points to " << out_path_for_PC << std::endl;
    // // PointCloudT::Ptr cloud_trans(new PointCloudT); 
    // for( int i = 0; i < 10; i++)
    // {
    //     PointCloudT::Ptr cloud_trans(new PointCloudT); 
    //     theta_r = theta * dis(gen); //(-pi/2, pi/2 )
    //     std::cout << "theta_r: " << theta_r << std::endl;
    //     // // rotation around X axis
    //     // transformation_matrix (1, 1) = std::cos (theta_r);
    //     // transformation_matrix (1, 2) = -std::sin (theta_r);
    //     // transformation_matrix (2, 1) = std::sin (theta_r);
    //     // transformation_matrix (2, 2) = std::cos (theta_r);

    //     // // rotation around Y axis
    //     // transformation_matrix (0, 0) = std::cos (theta_r);
    //     // transformation_matrix (2, 0) = -std::sin (theta_r);
    //     // transformation_matrix (0, 2) = std::sin (theta_r);
    //     // transformation_matrix (2, 2) = std::cos (theta_r);
               
    //     // rotation around Z axis
    //     transformation_matrix (0, 0) = std::cos (theta_r);
    //     transformation_matrix (0, 1) = -std::sin (theta_r);
    //     transformation_matrix (1, 0) = std::sin (theta_r);
    //     transformation_matrix (1, 1) = std::cos (theta_r);
                    
    //     // A translation on X axis (0.5 meters)
    //     transformation_matrix (0, 3) = dis(gen)*0.5; // (-0.5, 0.5)
    //     // A translation on Y axis (0.4 meters)
    //     transformation_matrix (1, 3) =  dis(gen)*0.5; // (-0.5, 0.5)
    //     // A translation on Z axis (0.4 meters)
    //     transformation_matrix (2, 3) =  dis(gen)*0.5; // (-0.5, 0.5)
    //     // Executing the transformation
    //     pcl::transformPointCloud (*cloud, *cloud_trans, transformation_matrix);

    //     //  Visualization(cloud_trans);

    //     out_path_for_PC = output_dir + "tetrahedron_trans_" + std::to_string(i+1) + ".pcd";
    //     std::cout << out_path_for_PC << std::endl;
    //     pcl::io::savePCDFile (out_path_for_PC, *cloud_trans);
    //     std::cerr << "Saved " << cloud->points.size() << " data points to " << out_path_for_PC << std::endl;

    //     out_path_for_Tr_M = output_dir + "/" + "tetrahedron_trans_" + std::to_string(i+1) + ".txt";
    //     fout.open(out_path_for_Tr_M);
    //     fout << "Transformation_matrix:" << std::endl;
    //     for(int i = 0; i < transformation_matrix.rows(); ++i)
    //     {
    //         fout << transformation_matrix(i, 0) << " " <<transformation_matrix(i, 1) << " "
    //              << transformation_matrix(i, 2) << " " << transformation_matrix(i, 3)
    //              << std::endl;
    //     }
    //     fout.close();
    //     std::cerr << "Saved " << out_path_for_Tr_M << std::endl;
    // }

    return 0;
}
