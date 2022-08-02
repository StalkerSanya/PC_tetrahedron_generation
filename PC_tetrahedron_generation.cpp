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
