/*****************************************************************
 *
 * Copyright (c) 2022, Nanyang Technological University, Singapore
 *
 * Authors: Pengyu Yin
 * Contact: pengyu001@e.ntu.edu.sg
 *
 ****************************************************************/

#include <locale>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>

#include <eigen_conversions/eigen_msg.h>

#include <visualization_msgs/Marker.h>

// #include "dlo/dlo.h"

// Our modules
// #include "fpfh_manager.hpp"
#include "filtering.hpp"
// #include "imageProjection.hpp"
// #include "patchwork.hpp"
#include "cluster_manager.hpp"
#include "semantic_teaser.hpp"
// #include "conversion.hpp"
// #include "utility.h"
#include "dataio.hpp"
// #include "eval.hpp"

pcl::PointCloud<PointType>::ConstPtr getCloud(std::string filename);
void setParams (int semantic_class, double cluster_distance_threshold, int minNum, int maxNum, clusterManager::ClusterParams& params, clusterManager::DCVCParam &seg_param);
void merge_label(const string label_file_path, pcl::PointCloud<PointType>::Ptr raw_pc, pcl::PointCloud<PointL>::Ptr semantic_pc, double label_deter_rate);
void apply_color_mapping_spvnas(int label, int &r, int &g, int &b);
void color_pc(const pcl::PointCloud<PointL>::Ptr semantic_cloud, pcl::PointCloud<PointRGB>::Ptr colored_cloud);
void setCovMatsMarkers(visualization_msgs::MarkerArray &markerArray, const pcl::PointCloud<PointType>::Ptr cloud, const std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> &covariances, const std::vector<float> rgb_color, int id);
pcl::PointCloud<PointL>::Ptr random_downsample_pl(pcl::PointCloud<PointL>::Ptr cloud_ori, int ratio);

const char separator    = ' ';
const int nameWidth     = 22;
const int numWidth      = 8;

int main(int argc, char **argv) {
    ros::init(argc, argv, "semantic_clustering");
    ros::NodeHandle nh;

    // building cluster params
    int building_class_num;
    double building_min_cluster_dist;
    int building_min_point_num, building_max_point_num;
    bool use_building;
    bool use_DCVC_building;
    int building_minSeg;

    nh.param<int>("/building_param/class_num", building_class_num, 0);
    nh.param<double>("/building_param/min_dist", building_min_cluster_dist, 0.5);
    nh.param<int>("/building_param/min_num",  building_min_point_num, 5);
    nh.param<int>("/building_param/max_num",  building_max_point_num, 200);
    nh.param<bool>("/building_param/use_building",  use_building, false);
    nh.param<bool>("/building_param/use_DCVD", use_DCVC_building, false);
    nh.param<int>("/building_param/DCVC_min_num", building_minSeg, 0);

    // car cluster params
    int car_class_num;
    double car_min_cluster_dist;
    int car_min_point_num, car_max_point_num;
    bool use_car;
    bool use_DCVC_car;
    int car_minSeg;

    nh.param<int>("/car_param/class_num", car_class_num, 0);
    nh.param<double>("/car_param/min_dist", car_min_cluster_dist, 0.5);
    nh.param<int>("/car_param/min_num",  car_min_point_num, 5);
    nh.param<int>("/car_param/max_num",  car_max_point_num, 200);
    nh.param<bool>("/car_param/use_car",  use_car, false);
    nh.param<bool>("/car_param/use_DCVC",  use_DCVC_car, false);
    nh.param<int>("/car_param/DCVC_min_num", car_minSeg, 0);
    
    // vegetation cluster params
    int vegetation_class_num;
    double vegetation_min_cluster_dist;
    int vegetation_min_point_num, vegetation_max_point_num;
    bool use_veg;
    bool use_DCVC_veg;
    int veg_minSeg;

    nh.param<int>("/vegetation_param/class_num", vegetation_class_num, 0);
    nh.param<double>("/vegetation_param/min_dist", vegetation_min_cluster_dist, 0.5);
    nh.param<int>("/vegetation_param/min_num",  vegetation_min_point_num, 5);
    nh.param<int>("/vegetation_param/max_num",  vegetation_max_point_num, 200);
    nh.param<bool>("/vegetation_param/use_veg",  use_veg, false);
    nh.param<bool>("/vegetation_param/use_DCVD", use_DCVC_veg, false);
    nh.param<int>("/vegetation_param/DCVC_min_num", veg_minSeg, 0);

    // trunk cluster params
    int trunk_class_num;
    double trunk_min_cluster_dist;
    int trunk_min_point_num, trunk_max_point_num;
    bool use_trunk;
    bool use_DCVC_trunk;
    int trunk_minSeg;

    nh.param<int>("/trunk_param/class_num", trunk_class_num, 0);
    nh.param<double>("/trunk_param/min_dist", trunk_min_cluster_dist, 0.5);
    nh.param<int>("/trunk_param/min_num",  trunk_min_point_num, 5);
    nh.param<int>("/trunk_param/max_num",  trunk_max_point_num, 200);
    nh.param<bool>("/trunk_param/use_trunk",  use_trunk, false);
    nh.param<bool>("/trunk_param/use_DCVD", use_DCVC_trunk, false);
    nh.param<int>("/trunk_param/DCVC_min_num", trunk_minSeg, 0);

    // DCVC segmentation params
    double startR, deltaR, deltaP, deltaA;
    int minSeg;
    nh.param<double>("/DCVC_param/startR", startR, 0.0);
    nh.param<double>("/DCVC_param/deltaR", deltaR, 0.0);
    nh.param<double>("/DCVC_param/deltaP", deltaP, 0.0);
    nh.param<double>("/DCVC_param/deltaA", deltaA, 0.0);
    nh.param<int>("/DCVC_param/minSeg", minSeg, 0);

    // registration parmas
    double noise_level;
    nh.param<double>("/noise_level", noise_level, 0.06);
    double distribution_noise_level;
    nh.param<double>("/distribution_noise_level", distribution_noise_level, 0.06);

    // std::string gt_file_path;
    // nh.param<std::string>("/gt_file_path", gt_file_path, "");
    // std::string calib_file_path;
    // nh.param<std::string>("/calib_file_path", calib_file_path, "");


    // Eigen::Matrix4d calib_mat;
    // if (!load_calib_mat(calib_file_path, calib_mat)) {
    //     ROS_ERROR("Could not read calib_mat!");
    //     return 0;
    // }

    // std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> GT_list_camera;
    // GT_list_camera = load_poses_from_transform_matrix(gt_file_path);

    // scan and label locations
    // std::string src_scan_location, tgt_scan_location, src_label_loacation, tgt_label_loacation;
    // nh.param<std::string>("/src_scan_location", src_scan_location, "");
    // nh.param<std::string>("/tgt_scan_location", tgt_scan_location, "");
    // nh.param<std::string>("/src_label_loacation", src_label_loacation, "");
    // nh.param<std::string>("/tgt_label_loacation", tgt_label_loacation, "");

    int src_indx, tgt_indx;
    nh.param<int>("/src_indx",  src_indx, 0);
    nh.param<int>("/tgt_indx",  tgt_indx, 0);

    // std::ostringstream ss;
    // ss << std::setw(6) << std::setfill('0') << src_indx;
    // std::string src_file_name(ss.str());
    // ss.str(""); // clear ss
    // ss << std::setw(6) << std::setfill('0') << tgt_indx;
    // std::string tgt_file_name(ss.str());

    bool solving_w_cov;
    nh.param<bool>("/solving_w_cov", solving_w_cov, true);

    // comparative results
    bool conduct_other_methods;
    nh.param<bool>("/conduct_other_methods", conduct_other_methods, false);

    double inital_yaw_rate;
    nh.param<double>("/inital_yaw_rate", inital_yaw_rate, 0.0);

    double label_deter_rate;
    nh.param<double>("/label_deter_rate", label_deter_rate, 0.0);

    // publishers
    ros::Publisher SrcPublisher                = nh.advertise<sensor_msgs::PointCloud2>("/source", 100);
    ros::Publisher SrcColoredPublisher         = nh.advertise<sensor_msgs::PointCloud2>("/srccolored", 100);
    ros::Publisher SrcCarCenterPublisher       = nh.advertise<sensor_msgs::PointCloud2>("/src_car_nodes", 100);
    ros::Publisher SrcBuildingCenterPublisher  = nh.advertise<sensor_msgs::PointCloud2>("/src_building_nodes", 100);
    ros::Publisher SrcVegCenterPublisher       = nh.advertise<sensor_msgs::PointCloud2>("/src_veg_nodes", 100);
    ros::Publisher SrcTrunkPublisher           = nh.advertise<sensor_msgs::PointCloud2>("/src_trunk_nodes", 100);
    ros::Publisher SrcCovPublisher             = nh.advertise<visualization_msgs::MarkerArray>("/src_cov", 100);

    ros::Publisher TgtPublisher                = nh.advertise<sensor_msgs::PointCloud2>("/target", 100);
    ros::Publisher TgtColoredPublisher         = nh.advertise<sensor_msgs::PointCloud2>("/tgtcolored", 100);
    ros::Publisher TgtCarCenterPublisher       = nh.advertise<sensor_msgs::PointCloud2>("/tgt_car_nodes", 100);
    ros::Publisher TgtBuildingCenterPublisher  = nh.advertise<sensor_msgs::PointCloud2>("/tgt_building_nodes", 100);
    ros::Publisher TgtVegCenterPublisher       = nh.advertise<sensor_msgs::PointCloud2>("/tgt_veg_nodes", 100);
    ros::Publisher TgtTrunkPublisher           = nh.advertise<sensor_msgs::PointCloud2>("/tgt_trunk_nodes", 100);
    ros::Publisher TgtCovPublisher             = nh.advertise<visualization_msgs::MarkerArray>("/tgt_cov", 100);

    ros::Publisher InlierCorrPublisher         = nh.advertise<visualization_msgs::Marker>("/inlierCorres", 100);
    ros::Publisher InitalCorrPublisher         = nh.advertise<visualization_msgs::Marker>("/initCorres", 100);

    ros::Publisher TransformedPublisher        = nh.advertise<sensor_msgs::PointCloud2>("/transformed_pc", 100);

    std::string project_path = ros::package::getPath("segregator");
    string src_path = project_path + "/materials/000000.bin";
    string tgt_path = project_path + "/materials/004413.bin";

    // load cloud
    pcl::PointCloud<PointType>::Ptr srcRaw(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr tgtRaw(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr srcInt(new pcl::PointCloud<PointType>);

    // string src_path            = src_scan_location + src_file_name + ".bin";
    // string tgt_path            = tgt_scan_location + tgt_file_name + ".bin";
   
    *srcInt                    = *getCloud(src_path);
    *tgtRaw                    = *getCloud(tgt_path);

    // robustness test
    Eigen::Matrix4d vis_mat = Eigen::MatrixXd::Identity(4, 4);
    vis_mat(2, 3) = 20;

    // inital rotation
    Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(inital_yaw_rate * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();

    vis_mat.topLeftCorner(3, 3) = rotationMatrix;

    pcl::transformPointCloud(*srcInt, *srcRaw, vis_mat);

    // load label
    std::string src_label_path = project_path + "/materials/000000.label";
    std::string tgt_label_path = project_path + "/materials/004413.label";

    // concatenate labels with point clouds
    pcl::PointCloud<PointL>::Ptr srcSemanticPc(new pcl::PointCloud<PointL>);
    // pcl::PointCloud<PointL>::Ptr srcSemanticPc_ori(new pcl::PointCloud<PointL>);
    pcl::PointCloud<PointL>::Ptr tgtSemanticPc(new pcl::PointCloud<PointL>);
    merge_label(src_label_path, srcRaw, srcSemanticPc, -1);
    merge_label(tgt_label_path, tgtRaw, tgtSemanticPc, label_deter_rate);

    // srcSemanticPc = random_downsample_pl(srcSemanticPc_ori, 2);

    clusterManager::DCVCParam building_DCVC_param;
    building_DCVC_param.startR = startR;
    building_DCVC_param.deltaR = deltaR;
    building_DCVC_param.deltaP = deltaP;
    building_DCVC_param.deltaA = deltaA;
    building_DCVC_param.minSeg = building_minSeg;
    clusterManager::DCVCParam car_DCVC_param;
    car_DCVC_param.startR = startR;
    car_DCVC_param.deltaR = deltaR;
    car_DCVC_param.deltaP = deltaP;
    car_DCVC_param.deltaA = deltaA;
    car_DCVC_param.minSeg = car_minSeg;
    clusterManager::DCVCParam veg_DCVC_param;
    veg_DCVC_param.startR = startR;
    veg_DCVC_param.deltaR = deltaR;
    veg_DCVC_param.deltaP = deltaP;
    veg_DCVC_param.deltaA = deltaA;
    veg_DCVC_param.minSeg = veg_minSeg;
    clusterManager::DCVCParam trunk_DCVC_param;
    trunk_DCVC_param.startR = startR;
    trunk_DCVC_param.deltaR = deltaR;
    trunk_DCVC_param.deltaP = deltaP;
    trunk_DCVC_param.deltaA = deltaA;
    trunk_DCVC_param.minSeg = trunk_minSeg;

    clusterManager::ClusterParams car_params;
    setParams(car_class_num, car_min_cluster_dist, car_min_point_num, car_max_point_num, car_params, car_DCVC_param);
    clusterManager::ClusterParams building_params;
    setParams(building_class_num, building_min_cluster_dist, building_min_point_num, building_max_point_num, building_params, building_DCVC_param);
    clusterManager::ClusterParams veg_params;
    setParams(vegetation_class_num, vegetation_min_cluster_dist, vegetation_min_point_num, vegetation_max_point_num, veg_params, veg_DCVC_param);
    clusterManager::ClusterParams trunk_params;
    setParams(trunk_class_num, trunk_min_cluster_dist, trunk_min_point_num, trunk_max_point_num, trunk_params, trunk_DCVC_param);

    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

    std::vector<pcl::PointCloud<PointType>> src_sem_vec;
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> src_covariances;
    std::vector<pcl::PointCloud<PointType>> tgt_sem_vec;
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> tgt_covariances;

    // src cloud nodes
    pcl::PointCloud<PointType>::Ptr srcCarCloud(new pcl::PointCloud<PointType>);
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> src_car_covariances;
    if (use_car) {
        clusterManager src_car_node;
        src_car_node.reset(car_params);
        if (!use_DCVC_car) {
            src_car_node.dbscanSeg(srcSemanticPc);
        }
        else {
            src_car_node.segmentPointCloud(srcSemanticPc);
        }
        src_car_node.computeCentroidAndCov(srcCarCloud, src_car_covariances);
        src_sem_vec.emplace_back(*srcCarCloud); 
        std::copy(std::begin(src_car_covariances), std::end(src_car_covariances), std::back_inserter(src_covariances));
    }

    pcl::PointCloud<PointType>::Ptr srcTrunkCloud(new pcl::PointCloud<PointType>);
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> src_trunk_covariances;
    if (use_trunk) {
        clusterManager src_trunk_node;
        src_trunk_node.reset(trunk_params);
        if (!use_DCVC_trunk) {
            src_trunk_node.dbscanSeg(srcSemanticPc);
        }
        else {
            src_trunk_node.segmentPointCloud(srcSemanticPc);
        }
        src_trunk_node.computeCentroidAndCov(srcTrunkCloud, src_trunk_covariances);
        src_sem_vec.emplace_back(*srcTrunkCloud);
        std::copy(std::begin(src_trunk_covariances), std::end(src_trunk_covariances), std::back_inserter(src_covariances));
    }
    
    // tgt clouds nodes
    pcl::PointCloud<PointType>::Ptr tgtCarCloud(new pcl::PointCloud<PointType>);
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> tgt_car_covariances;
    if (use_car) {
        clusterManager tgt_car_node;
        tgt_car_node.reset(car_params);
        if (!use_DCVC_car) {
            tgt_car_node.dbscanSeg(tgtSemanticPc);
        }
        else {
            tgt_car_node.segmentPointCloud(tgtSemanticPc);
        }
        tgt_car_node.computeCentroidAndCov(tgtCarCloud, tgt_car_covariances);
        tgt_sem_vec.emplace_back(*tgtCarCloud);
        std::copy(std::begin(tgt_car_covariances), std::end(tgt_car_covariances), std::back_inserter(tgt_covariances));
    }

    pcl::PointCloud<PointType>::Ptr tgtTrunkCloud(new pcl::PointCloud<PointType>);
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> tgt_trunk_covariances;
    if (use_trunk) {
        clusterManager tgt_trunk_node;
        tgt_trunk_node.reset(trunk_params);
        if (!use_DCVC_trunk) {
            tgt_trunk_node.dbscanSeg(tgtSemanticPc);
        }
        else {
            tgt_trunk_node.segmentPointCloud(tgtSemanticPc);
        }
        tgt_trunk_node.computeCentroidAndCov(tgtTrunkCloud, tgt_trunk_covariances);
        tgt_sem_vec.emplace_back(*tgtTrunkCloud);
        std::copy(std::begin(tgt_trunk_covariances), std::end(tgt_trunk_covariances), std::back_inserter(tgt_covariances));
    }
    
    // backgroud points feature extraction
    pcl::PointCloud<PointType>::Ptr src_matched_bg(new pcl::PointCloud<PointType>);
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> src_bg_covariances;
    pcl::PointCloud<PointType>::Ptr tgt_matched_bg(new pcl::PointCloud<PointType>);
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> tgt_bg_covariances;

    pcl::PointCloud<PointType>::Ptr srcVegetationCloud(new pcl::PointCloud<PointType>);
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> src_veg_covariances;
    pcl::PointCloud<PointType>::Ptr tgtVegetationCloud(new pcl::PointCloud<PointType>);
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> tgt_veg_covariances;

    pcl::PointCloud<PointType>::Ptr srcBuildingCloud(new pcl::PointCloud<PointType>);
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> src_building_covariances;
    pcl::PointCloud<PointType>::Ptr tgtBuildingCloud(new pcl::PointCloud<PointType>);
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> tgt_building_covariances;
    
    if (use_building) {
        clusterManager building_node;
        building_node.reset(building_params);

        if (building_node.fpfh_feature_extraction(srcSemanticPc, tgtSemanticPc)) {
            srcBuildingCloud = building_node.getSrcMatchedPointCloud();
            tgtBuildingCloud = building_node.getTgtMatchedPointCloud();

            src_building_covariances = building_node.getSrcCovMat();
            tgt_building_covariances = building_node.getTgtCovMat();
            
            std::copy(std::begin(src_building_covariances), std::end(src_building_covariances), std::back_inserter(src_bg_covariances));
            std::copy(std::begin(tgt_building_covariances), std::end(tgt_building_covariances), std::back_inserter(tgt_bg_covariances));
        }
    }

    if (use_veg) {
        clusterManager veg_node;
        veg_node.reset(veg_params);

        if (veg_node.fpfh_feature_extraction(srcSemanticPc, tgtSemanticPc)) {
            srcVegetationCloud = veg_node.getSrcMatchedPointCloud();
            tgtVegetationCloud = veg_node.getTgtMatchedPointCloud();

            src_veg_covariances = veg_node.getSrcCovMat();
            tgt_veg_covariances = veg_node.getTgtCovMat();

            std::copy(std::begin(src_veg_covariances), std::end(src_veg_covariances), std::back_inserter(src_bg_covariances));
            std::copy(std::begin(tgt_veg_covariances), std::end(tgt_veg_covariances), std::back_inserter(tgt_bg_covariances));
        }
    }

    if (use_building || use_veg) {
        *src_matched_bg = (*srcBuildingCloud) + (*srcVegetationCloud);
        *tgt_matched_bg = (*tgtBuildingCloud) + (*tgtVegetationCloud);
    }
    
    std::cout << "\033[1;32m";

    std::cout << left  << setw(nameWidth) << setfill(separator) << "# of Car Nodes" << " | ";
    std::cout << right << setw(numWidth)  << setfill(separator) << srcCarCloud->points.size() << " | ";
    std::cout << right << setw(numWidth)  << setfill(separator) << tgtCarCloud->points.size() << std::endl;


    std::cout << left  << setw(nameWidth) << setfill(separator) << "# of Building Nodes" << " | ";
    std::cout << right << setw(numWidth)  << setfill(separator) << srcBuildingCloud->points.size() << " | ";
    std::cout << right << setw(numWidth)  << setfill(separator) << tgtBuildingCloud->points.size() << std::endl;


    std::cout << left  << setw(nameWidth) << setfill(separator) << "# of Vegetation Nodes" << " | ";
    std::cout << right << setw(numWidth)  << setfill(separator) << srcVegetationCloud->points.size() << " | ";
    std::cout << right << setw(numWidth)  << setfill(separator) << tgtVegetationCloud->points.size() << std::endl;

    std::cout << left  << setw(nameWidth) << setfill(separator) << "# of Trunk Nodes" << " | ";
    std::cout << right << setw(numWidth)  << setfill(separator) << srcTrunkCloud->points.size() << " | ";
    std::cout << right << setw(numWidth)  << setfill(separator) << tgtTrunkCloud->points.size() << "\033[0m" << std::endl;
    
    std::cout << "\033[0m";

    std::chrono::system_clock::time_point before_optim = std::chrono::system_clock::now();
    
    semanticTeaser::Params params;
    params.teaser_params.noise_bound = noise_level;
    params.teaser_params.distribution_noise_bound = distribution_noise_level;

    semanticTeaser semSolver(params);

    // semSolver.solve(src, tgt);
    if (solving_w_cov) {
        semSolver.solve_for_multiclass_with_cov(src_sem_vec, tgt_sem_vec, 
                                                src_covariances, tgt_covariances, 
                                                *src_matched_bg, *tgt_matched_bg,
                                                src_bg_covariances, src_bg_covariances);
        // semSolver.solve_for_multiclass_with_cov(src_sem_vec, tgt_sem_vec, 
        //                                         src_covariances, tgt_covariances);
    }
    else {
        semSolver.solve_for_multiclass(src_sem_vec, tgt_sem_vec);
    }
    
    auto solution = semSolver.get_solution();

    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*srcRaw, *transformed_cloud, solution);

    std::chrono::duration<double> sec = std::chrono::system_clock::now() - start;
    std::chrono::duration<double> optim_sec = std::chrono::system_clock::now() - before_optim;
    std::cout << setprecision(4) << "\033[1;32mTotal takes: " << sec.count() << " sec. ";
    std::cout << "(Semantic Clustering: " << sec.count() - optim_sec.count() << " sec. + Semantic Teaser: " << optim_sec.count() << " sec.)\033[0m" << std::endl;

    // GICP
    // pcl::PointCloud<PointType>::Ptr gicp_aligned(new pcl::PointCloud<PointType>);
    // Eigen::Matrix4d solution_gicp;

    // std::chrono::system_clock::time_point gicp_begin = std::chrono::system_clock::now();

    // if (conduct_other_methods) {
    //     pcl::PointCloud<PointType>::Ptr src_filtered(new pcl::PointCloud<PointType>);
    //     pcl::PointCloud<PointType>::Ptr tgt_filtered(new pcl::PointCloud<PointType>);

    //     static pcl::VoxelGrid<PointType> scan_filter;
    //     scan_filter.setLeafSize(0.25, 0.25, 0.25);

    //     scan_filter.setInputCloud(srcRaw);
    //     scan_filter.filter(*src_filtered);

    //     scan_filter.setInputCloud(tgtRaw);
    //     scan_filter.filter(*tgt_filtered);

    //     // pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp_solver;
    //     nano_gicp::NanoGICP<PointType, PointType> gicp_solver;
    
    //     gicp_solver.setInputSource(src_filtered);
    //     gicp_solver.setInputTarget(tgt_filtered);
    //     gicp_solver.setCorrespondenceRandomness(10);
    //     gicp_solver.setMaximumIterations(15);
        
    //     std::chrono::duration<double> pre_time = std::chrono::system_clock::now() - gicp_begin;

    //     gicp_solver.setNumThreads(4);
    //     gicp_solver.align(*gicp_aligned);

    //     solution_gicp = gicp_solver.getFinalTransformation().cast<double>();

    // }
    // std::chrono::duration<double> gicp_time = std::chrono::system_clock::now() - gicp_begin;
    // std::cout << setprecision(4) << "\033[1;32mVoxelGICP takes: " << gicp_time.count() << " sec. \033[0m" << std::endl;

    //////////////////////////////////////////
    // evaluation
    // Eigen::Matrix4d gt_body = GT_list_camera[tgt_indx].inverse() * GT_list_camera[src_indx];
    // std::cout << gt_body << std::endl;

    // Eigen::Matrix4d gt_lidar = calib_mat.inverse() * gt_body * calib_mat * vis_mat.inverse();
    //////////////////////////////////////////

    // vgicp
    // Eigen::Matrix4d lo_body_vgicp = solution_gicp;

    // Segregator result
    // Eigen::Matrix4d lo_body = solution;

    // Eval eval;
    // double translation_error;
    // double rotation_error;
    // eval.compute_adj_rpe(gt_lidar, solution, translation_error, rotation_error);

    // double translation_error_vgicp;
    // double rotation_error_vgicp;
    // eval.compute_adj_rpe(gt_lidar, lo_body_vgicp, translation_error_vgicp, rotation_error_vgicp);

    // std::cout << setprecision(4) << "\033[1;32m";
    // std::cout << "Segregator Translation Error = " << translation_error << " meter." << std::endl;
    // std::cout << "           Rotational  Error = " << rotation_error << " deg.\033[0m" << std::endl;

    // for visualization
    pcl::PointCloud<PointRGB>::Ptr srcColoredRaw(new pcl::PointCloud<PointRGB>);
    pcl::PointCloud<PointRGB>::Ptr tgtColoredRaw(new pcl::PointCloud<PointRGB>);

    color_pc(srcSemanticPc, srcColoredRaw);
    color_pc(tgtSemanticPc, tgtColoredRaw);

    sensor_msgs::PointCloud2 SrcMsg           = cloud2msg(*srcRaw);
    sensor_msgs::PointCloud2 SrcColoredMsg    = cloud2msg(*srcColoredRaw);
    sensor_msgs::PointCloud2 SrcCarCenterMsg  = cloud2msg(*srcCarCloud);
    sensor_msgs::PointCloud2 SrcBuildingCenterMsg = cloud2msg(*srcBuildingCloud);
    sensor_msgs::PointCloud2 SrcVegCenterMsg  = cloud2msg(*srcVegetationCloud);
    sensor_msgs::PointCloud2 SrcTrunkMsg      = cloud2msg(*srcTrunkCloud);

    sensor_msgs::PointCloud2 TgtMsg           = cloud2msg(*tgtRaw);
    sensor_msgs::PointCloud2 TgtColoredMsg    = cloud2msg(*tgtColoredRaw);
    sensor_msgs::PointCloud2 TgtCarCenterMsg  = cloud2msg(*tgtCarCloud);
    sensor_msgs::PointCloud2 TgtBuildingCenterMsg = cloud2msg(*tgtBuildingCloud);
    sensor_msgs::PointCloud2 TgtVegCenterMsg  = cloud2msg(*tgtVegetationCloud);
    sensor_msgs::PointCloud2 TgtTrunkMsg      = cloud2msg(*tgtTrunkCloud);

    sensor_msgs::PointCloud2 TransformedMsg   = cloud2msg(*transformed_cloud);

    // other methodes
    // sensor_msgs::PointCloud2 GicpAlignedMsg   = cloud2msg(*gicp_aligned);

    // correspondence visualization
    pcl::PointCloud<PointType> srcMaxClique;
    pcl::PointCloud<PointType> tgtMaxClique;
    semSolver.getMaxCliques(srcMaxClique, tgtMaxClique);
    visualization_msgs::Marker inlierCorrMarker;
    setCorrespondenceMarker(srcMaxClique, tgtMaxClique, inlierCorrMarker, 0.5, {0.0, 1.0, 0.0}, 0);

    pcl::PointCloud<PointType> srcMatched;
    pcl::PointCloud<PointType> tgtMatched;
    semSolver.getInitCorr(srcMatched, tgtMatched);
    visualization_msgs::Marker initalCorrMarker;
    setCorrespondenceMarker(srcMatched, tgtMatched, initalCorrMarker, 0.08, {1.0, 0.0, 0.0}, 0);

    visualization_msgs::MarkerArray srcCovMarker, tgtCovMarker;
    setCovMatsMarkers(srcCovMarker, srcBuildingCloud, src_building_covariances, {0.0, 0.0, 1.0}, 5);
    setCovMatsMarkers(srcCovMarker, tgtBuildingCloud, tgt_building_covariances, {1.0, 0.0, 0.0}, 1000);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        SrcPublisher.publish(SrcMsg);
        SrcColoredPublisher.publish(SrcColoredMsg);
        SrcCarCenterPublisher.publish(SrcCarCenterMsg);
        SrcBuildingCenterPublisher.publish(SrcBuildingCenterMsg);
        SrcVegCenterPublisher.publish(SrcVegCenterMsg);
        SrcTrunkPublisher.publish(SrcTrunkMsg);
        SrcCovPublisher.publish(srcCovMarker);

        TgtPublisher.publish(TgtMsg);
        TgtColoredPublisher.publish(TgtColoredMsg);
        TgtCarCenterPublisher.publish(TgtCarCenterMsg);
        TgtBuildingCenterPublisher.publish(TgtBuildingCenterMsg);
        TgtVegCenterPublisher.publish(TgtVegCenterMsg);
        TgtTrunkPublisher.publish(TgtTrunkMsg);
        TgtCovPublisher.publish(tgtCovMarker);

        InlierCorrPublisher.publish(inlierCorrMarker);
        InitalCorrPublisher.publish(initalCorrMarker);

        TransformedPublisher.publish(TransformedMsg);
        // GICPAlignedPublisher.publish(GicpAlignedMsg);

        loop_rate.sleep();
    }

    return 0;
}

void setParams(int semantic_class, double cluster_distance_threshold, int minNum, int maxNum, clusterManager::ClusterParams& params, clusterManager::DCVCParam &seg_param) {
    params.semanticLabel    = semantic_class;
    params.clusterTolerance = cluster_distance_threshold;
    params.minClusterSize   = minNum;
    params.maxClusterSize   = maxNum;

    params.startR = seg_param.startR;
    params.deltaR = seg_param.deltaR;
    params.deltaP = seg_param.deltaP;
    params.deltaA = seg_param.deltaA;
    params.minSeg = seg_param.minSeg;
}

pcl::PointCloud<PointType>::ConstPtr getCloud(std::string filename) {
    FILE *file                    = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "error: failed to load point cloud " << filename << std::endl;
        return nullptr;
    }

    std::vector<float> buffer(1000000);
    size_t             num_points =
                               fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    cloud->resize(num_points);

    for (int i = 0; i < num_points; i++) {
        auto &pt = cloud->at(i);
        pt.x = buffer[i * 4];
        pt.y = buffer[i * 4 + 1];
        pt.z = buffer[i * 4 + 2];
        // Intensity is not in use
//         pt.intensity = buffer[i * 4 + 3];
    }

    return cloud;
}

/**
* @brief      Merge cloud and label to semantic_pc.
* @param[in]  label_file_path; raw_point_cloud; out_semantic_pc
* @return     None.
*/
void merge_label(const string label_file_path, 
                 pcl::PointCloud<PointType>::Ptr raw_pc, 
                 pcl::PointCloud<PointL>::Ptr semantic_pc, 
                 double label_deter_rate) {
    
    // read label file
    std::ifstream in_stream(label_file_path.c_str(), std::ios::in | std::ios::binary);
    vector<uint16_t> cloud_label_vec;
    cloud_label_vec.reserve(1000000);
    
    if (in_stream.is_open()) {
        // with 16 lower bit semantic label, 16 higher bit instance label
        uint32_t cur_whole_label;
        uint16_t cur_sem_label;

        while(in_stream.read((char * )&cur_whole_label, sizeof(cur_whole_label))) {
            cur_sem_label = cur_whole_label & 0xFFFF;
            cloud_label_vec.emplace_back(cur_sem_label);
        }
    }
    else {
        std::cerr << "error: failed to load label " << label_file_path << std::endl;
        return;
    }

    //check size equal
    if (raw_pc->points.size() != cloud_label_vec.size()) {
        std::cerr << "error: Point cloud size != label size" << std::endl;
        std::cout << "Point cloud size: " << raw_pc->points.size() << std::endl;
        std::cout << "Label size      : " << cloud_label_vec.size()      << std::endl;
        return;
    }

    for (int i = 0; i < cloud_label_vec.size(); i++) {
        double cur_rand = (double) rand() / (RAND_MAX);
        if (cur_rand <= label_deter_rate) {
            cloud_label_vec[i] = 20;
        }
    }

    // semantic_pc.reset(new pcl::PointCloud<PointL>);
    for (int i = 0; i < raw_pc->points.size(); i++) {
        PointL tmpL;
        tmpL.x = raw_pc->points[i].x;
        tmpL.y = raw_pc->points[i].y;
        tmpL.z = raw_pc->points[i].z;
        tmpL.label = cloud_label_vec[i];
        semantic_pc->points.push_back(tmpL);
    }

    semantic_pc->width = semantic_pc->points.size();
    semantic_pc->height = 1;
}


void apply_color_mapping_spvnas(int label, int &r, int &g, int &b) {
    switch (label)
    {
    case 0: //car
    {
        r = 100;
        g = 150;
        b = 245;
        break;
    }
    case 1: //bicycle
    {
        r = 100;
        g = 230;
        b = 245;
        break;
    }
    case 2: //motorcycle
    {
        r = 30;
        g = 60;
        b = 150;
        break;
    }
    case 3: //truck
    {
        r = 80;
        g = 30;
        b = 180;
        break;
    }
    case 4: //other-vehicle
    {
        r = 0;
        g = 0;
        b = 255;
        break;
    }
    case 5: //person
    {
        r = 255;
        g = 30;
        b = 30;
        break;
    }
    case 6: //bicyclist
    {
        r = 255;
        g = 40;
        b = 200;
        break;
    }
    case 7: //motorcyclist
    {
        r = 150;
        g = 30;
        b = 90;
        break;
    }
    case 8: //road
    {
        r = 255;
        g = 0;
        b = 255;
        break;
    }
    case 9: //parking
    {
        r = 255;
        g = 150;
        b = 255;
        break;
    }
    case 10: //sidewalk
    {
        r = 75;
        g = 0;
        b = 75;
        break;
    }
    case 11: //other-ground
    {
        r = 175;
        g = 0;
        b = 75;
        break;
    }
    case 12: //building
    {
        r = 255;
        g = 200;
        b = 0;
        break;
    }
    case 13: //fence
    {
        r = 255;
        g = 120;
        b = 50;
        break;
    }
    case 14: //vegetation
    {
        r = 0;
        g = 175;
        b = 0;
        break;
    }
    case 15: //trunk
    {
        r = 135;
        g = 60;
        b = 0;
        break;
    }
    case 16: //terrain
    {
        r = 150;
        g = 240;
        b = 80;
        break;
    }
    case 17: //pole
    {
        r = 255;
        g = 240;
        b = 150;
        break;
    }
    case 18: //traffic-sign
    {
        r = 255;
        g = 0;
        b = 0;
        break;
    }
    default: //moving objects
    {
        r = 0;
        g = 0;
        b = 0;
    }
    }
}

/**
* @brief      Color point cloud according to per point semantic labels.
* @param[in]  semantic_cloud: input semantic cloud ptr (with label)
* @param[in]  colored_cloud:  colored cloud ptr
*/
void color_pc(const pcl::PointCloud<PointL>::Ptr semantic_cloud, pcl::PointCloud<PointRGB>::Ptr colored_cloud) {
    int r, g, b;
    uint16_t temp_label;
    PointRGB temp_pt;
    for (int i = 0; i < semantic_cloud->points.size(); ++i) {
        temp_pt.x  = semantic_cloud->points[i].x;
        temp_pt.y  = semantic_cloud->points[i].y;
        temp_pt.z  = semantic_cloud->points[i].z;
        temp_label = semantic_cloud->points[i].label;
        apply_color_mapping_spvnas((int)temp_label, r, g, b);
        temp_pt.r  = r;
        temp_pt.g  = g;
        temp_pt.b  = b;
        colored_cloud->points.push_back(temp_pt);
    }
}

void setCovMatsMarkers(visualization_msgs::MarkerArray &markerArray,
                       const pcl::PointCloud<PointType>::Ptr cloud, 
                       const std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> &covariances,
                       const std::vector<float> rgb_color = {0.0, 0.0, 0.0},
                       int id = 0) {
    // int id = 1;
    Eigen::EigenSolver<Eigen::Matrix3d> es;
    for (int i = 0; i < covariances.size(); ++i) {
        visualization_msgs::Marker covMarker;

        covMarker.header.frame_id = "map";
        covMarker.header.stamp = ros::Time();
        covMarker.ns = "my_namespace";
        covMarker.id = id; // To avoid overlap
        covMarker.type = visualization_msgs::Marker::CYLINDER;
        covMarker.action = visualization_msgs::Marker::ADD;

        PointType tempP = cloud->points[i];
        covMarker.pose.position.x = tempP.x;
        covMarker.pose.position.y = tempP.y;
        covMarker.pose.position.z = tempP.z;

        es.compute(covariances[i], true);
        covMarker.scale.x = sqrt(es.eigenvalues()(0).real());
        covMarker.scale.y = sqrt(es.eigenvalues()(1).real());
        covMarker.scale.z = sqrt(es.eigenvalues()(2).real());

        covMarker.color.r = rgb_color[0];
        covMarker.color.g = rgb_color[1];
        covMarker.color.b = rgb_color[2];
        covMarker.color.a = 1.0; // Don't forget to set the alpha!

        Eigen::Matrix3d eigen_mat = es.eigenvectors().real();
        Eigen::Matrix3d rot_mat = eigen_mat.transpose();
        // eigen_mat.normalize();
        Eigen::Quaterniond quat(rot_mat);
        quat.normalize();

        geometry_msgs::Quaternion geo_quat;
        tf::quaternionEigenToMsg(quat, geo_quat);

        covMarker.pose.orientation.x = geo_quat.x;
        covMarker.pose.orientation.y = geo_quat.y;
        covMarker.pose.orientation.z = geo_quat.z;
        covMarker.pose.orientation.w = geo_quat.w;

        markerArray.markers.push_back(covMarker);
        id++;
    }
}

pcl::PointCloud<PointL>::Ptr random_downsample_pl(pcl::PointCloud<PointL>::Ptr cloud_ori, int ratio){
    pcl::PointCloud<PointL>::Ptr sampled_pc(new pcl::PointCloud<PointL>);

    for (int i = 0; i < cloud_ori->points.size(); ++i) {
        if ( i % ratio == 0) {
            sampled_pc->points.push_back(cloud_ori->points[i]);
        }
    }

    return sampled_pc;
}