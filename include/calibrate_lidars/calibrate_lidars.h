/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#ifndef CALIBRATE_LIDARS_CALIBRATE_LIDARS_H_
#define CALIBRATE_LIDARS_CALIBRATE_LIDARS_H_

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <memory>

namespace calibrate_lidars
{
class CalibrateLidars
{
public:
  CalibrateLidars();
  ~CalibrateLidars();
  bool addNode(Eigen::Matrix4f estimate, int id, bool fixed);
  bool addEdge(int id_to, int id_from, Eigen::Matrix4f connection, Eigen::MatrixXd information);
  bool optimize();
  std::map<int, Eigen::Matrix4f> retrieveCorrected();

private:
  std::unique_ptr<g2o::SparseOptimizer> graph_;
  std::map<int, g2o::VertexSE3*> node_lookup_;
};

} // namespace calibrate_lidars

#endif
