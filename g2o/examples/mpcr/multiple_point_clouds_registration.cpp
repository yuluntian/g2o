//
// Created by yulun on 12/24/22.
//
#include <iostream>
#include <map>
#include <Eigen/Geometry>
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam3d/types_slam3d.h"

#if defined G2O_HAVE_CHOLMOD
G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
#else
G2O_USE_OPTIMIZATION_LIBRARY(eigen);
#endif

G2O_USE_OPTIMIZATION_LIBRARY(dense);

int main(int argc, const char *argv[]) {
  if (argc < 5) {
    std::cout << "Multiple point clouds registration." << std::endl;
    std::cout << "Usage: " << argv[0] << "[measurements.txt] [T_world_origin.txt] [points_world.txt] [output.txt]"
              << std::endl;
    return -1;
  }

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(true);
  std::string solverName = "lm_var_cholmod";
  g2o::OptimizationAlgorithmProperty solverProperty;
  optimizer.setAlgorithm(
      g2o::OptimizationAlgorithmFactory::instance()->construct(solverName, solverProperty));

  int vertex_id = 0;
  int num_poses = 0;
  int num_points = 0;
  int num_edges = 0;
  int origin_id, point_id;
  std::string line;
  double qx, qy, qz, qw, tx, ty, tz;

  // Sensor offset
  auto sensorOffset = new g2o::ParameterSE3Offset();
  sensorOffset->setId(0);
  optimizer.addParameter(sensorOffset);

  // Load poses
  std::map<int, g2o::VertexSE3 *> poses;
  std::ifstream poses_file(argv[2]);
  std::getline(poses_file, line);  // The first line is header
  while (std::getline(poses_file, line)) {
    std::stringstream strstrm(line);
    strstrm >> origin_id >> qx >> qy >> qz >> qw >> tx >> ty >> tz;
    Eigen::Quaterniond quat(qw, qx, qy, qz);
    g2o::Vector3 position(tx, ty, tz);
    g2o::Isometry3 T_world_origin;
    T_world_origin.linear() = quat.toRotationMatrix();
    T_world_origin.translation() = position;
    // g2o::Isometry3 T_origin_world = T_world_origin.inverse();
    auto vertex = new g2o::VertexSE3();
    vertex->setId(vertex_id);
    vertex->setEstimate(T_world_origin);
    if (vertex_id == 0) {
      vertex->setFixed(true);
    }
    optimizer.addVertex(vertex);
    poses[origin_id] = vertex;
    vertex_id++;
    num_poses++;
  }
  std::cout << "Loaded " << num_poses << " pose variables." << std::endl;
  poses_file.close();

  // Load points
  std::map<int, g2o::VertexPointXYZ *> points;
  std::ifstream points_file(argv[3]);
  std::getline(points_file, line);  // The first line is header
  while (std::getline(points_file, line)) {
    std::stringstream strstrm(line);
    strstrm >> point_id >> tx >> ty >> tz;
    g2o::Vector3 point_world(tx, ty, tz);
    auto vertex = new g2o::VertexPointXYZ();
    vertex->setId(vertex_id);
    vertex->setEstimate(point_world);
    optimizer.addVertex(vertex);
    points[point_id] = vertex;
    vertex_id++;
    num_points++;
  }
  std::cout << "Loaded " << num_points << " point variables." << std::endl;
  points_file.close();

  // Load measurements
  std::ifstream measurements_file(argv[1]);
  std::getline(measurements_file, line);  // The first line is header
  while (std::getline(measurements_file, line)) {
    std::stringstream strstrm(line);
    strstrm >> origin_id >> point_id >> tx >> ty >> tz;
    printf("Origin %i -> point %i: %f, %f, %f\n", origin_id, point_id, tx, ty, tz);
    g2o::Vector3 obs(tx, ty, tz);
    auto edge = new g2o::EdgeSE3PointXYZ();
    auto vertex_pose = poses[origin_id];
    auto vertex_point = points[point_id];
    edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vertex_pose));
    edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vertex_point));
    edge->setMeasurement(obs);
    edge->information() = 1e6 * Eigen::Matrix3d::Identity();
    edge->setParameterId(0, 0);
    optimizer.addEdge(edge);
    num_edges++;
  }
  measurements_file.close();
  std::cout << "Loaded " << num_edges << " edges." << std::endl;

  optimizer.initializeOptimization();
  double average_error = 0;
  for (const auto e : optimizer.edges()) {
    auto edge = dynamic_cast<g2o::EdgeSE3PointXYZ *> (e);
    edge->computeError();
    g2o::Vector3 error = edge->error();
    average_error += error.norm();
  }
  average_error = average_error / (double) optimizer.edges().size();
  std::cout << "Average error before optimization: " << average_error << " m." << std::endl;

  optimizer.optimize(50);

  // Compute average error
  average_error = 0;
  for (const auto e : optimizer.edges()) {
    auto edge = dynamic_cast<g2o::EdgeSE3PointXYZ *> (e);
    edge->computeError();
    g2o::Vector3 error = edge->error();
    average_error += error.norm();
  }
  average_error = average_error / (double) optimizer.edges().size();
  std::cout << "Average error after optimization: " << average_error << " m." << std::endl;

  // Extract covariances over the point variables
  g2o::SparseBlockMatrix<Eigen::MatrixXd> C;
  std::vector<std::pair<int, int>> index_pairs;
  for (const auto it : points) {
    auto vertex_point = it.second;
    index_pairs.emplace_back(vertex_point->hessianIndex(), vertex_point->hessianIndex());
  }
  if (!optimizer.computeMarginals(C, index_pairs)) {
    std::cout << "Fail to compute marginals!" << std::endl;
    return -1;
  }

  // Save to file
  std::ofstream points_output_file(argv[4]);
  if (!points_output_file.is_open()) {
    std::cout << "Cannot open output file " << argv[4];
    return -1;
  }
  points_output_file << std::fixed << std::setprecision(15);
  points_output_file << "point_id x y z sigma_x sigma_y sigma_z\n";
  for (const auto it : points) {
    int point_id_output = it.first;
    auto vertex = it.second;
    auto position = vertex->estimate();
    double x = position(0);
    double y = position(1);
    double z = position(2);
    Eigen::Matrix3d Cp = *C.block(vertex->hessianIndex(), vertex->hessianIndex());
    double sigma_x = std::sqrt(Cp(0, 0));
    double sigma_y = std::sqrt(Cp(1, 1));
    double sigma_z = std::sqrt(Cp(2, 2));
    printf("Point %i: (%.2f, %.2f, %.2f), sigma (%.1e, %.1e, %.1e)\n",
           point_id_output, x, y, z, sigma_x, sigma_y, sigma_z);
    points_output_file << point_id_output << " ";
    points_output_file << x << " ";
    points_output_file << y << " ";
    points_output_file << z << " ";
    points_output_file << sigma_x << " ";
    points_output_file << sigma_y << " ";
    points_output_file << sigma_z << "\n";
  }
  points_output_file.close();
  std::cout << "Saved estimated points to " << argv[4] << std::endl;

  return 0;

}