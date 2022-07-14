/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#include "calibrate_lidars/calibrate_lidars.h"

#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>

G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

Eigen::Isometry3d
toIsometry(Eigen::Matrix4f mat)
{
  Eigen::Matrix4d mat_double = mat.cast<double>();
  Eigen::Isometry3d iso;
  iso = mat_double;
  return iso;
}

Eigen::Matrix4f
toMatrix(Eigen::Isometry3d iso)
{
  Eigen::Matrix4d mat_double = iso.matrix();
  Eigen::Matrix4f mat = mat_double.cast<float>();
  return mat;
}

calibrate_lidars::CalibrateLidars::CalibrateLidars()
{
  graph_.reset(new g2o::SparseOptimizer());
  g2o::OptimizationAlgorithmProperty solver_property;
  graph_->setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct("lm_var", solver_property));
}

calibrate_lidars::CalibrateLidars::~CalibrateLidars()
{
  graph_.reset();
}

bool
calibrate_lidars::CalibrateLidars::addNode(Eigen::Matrix4f estimate, int id, bool fixed)
{
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(id);
  vertex->setEstimate(toIsometry(estimate));
  vertex->setFixed(fixed);
  graph_->addVertex(vertex);
  node_lookup_[id] = vertex;
  return true;
}

bool
calibrate_lidars::CalibrateLidars::addEdge(int id_to,
                                           int id_from,
                                           Eigen::Matrix4f connection,
                                           Eigen::MatrixXd information)
{
  g2o::EdgeSE3* edge(new g2o::EdgeSE3());
  edge->setMeasurement(toIsometry(connection));
  edge->setInformation(information);
  edge->vertices()[0] = node_lookup_[id_from];
  edge->vertices()[1] = node_lookup_[id_to];
  graph_->addEdge(edge);
  return true;
}

bool
calibrate_lidars::CalibrateLidars::optimize()
{
  graph_->initializeOptimization();
  graph_->setVerbose(true);
  double before = graph_->chi2();
  graph_->optimize(50);
  std::cout << "Before " << before << " after " << graph_->chi2() << std::endl;
  return true;
}

std::map<int, Eigen::Matrix4f>
calibrate_lidars::CalibrateLidars::retrieveCorrected()
{
  std::map<int, Eigen::Matrix4f> data;
  for (const auto node : graph_->vertices())
  {
    Eigen::Isometry3d pose = dynamic_cast<g2o::VertexSE3*>(node.second)->estimate();
    data[node.first] = toMatrix(pose);
  }
  return data;
}
