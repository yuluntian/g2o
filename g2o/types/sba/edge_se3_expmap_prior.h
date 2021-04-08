//
// Created by yulun on 4/1/21.
//

#ifndef G2O_G2O_TYPES_SBA_EDGE_SE3_EXPMAP_PRIOR_H_
#define G2O_G2O_TYPES_SBA_EDGE_SE3_EXPMAP_PRIOR_H_

#include "g2o/core/base_unary_edge.h"
#include "g2o_types_sba_api.h"
#include "vertex_se3_expmap.h"

namespace g2o {

/**
 * \brief 6D edge between a fixed SE3Quat and VertexSE3Expmap
 */
class G2O_TYPES_SBA_API EdgeSE3ExpmapPrior
    : public BaseUnaryEdge<6, SE3Quat, VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeSE3ExpmapPrior();

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;
  void setMeasurement(const SE3Quat& m) override;
  void computeError() override;
  void linearizeOplus() override;
 private:
  // Store inverse of prior to avoid redundant calculations
  SE3Quat measurement_inv_;
};

}  // namespace g2o

#endif //G2O_G2O_TYPES_SBA_EDGE_SE3_EXPMAP_PRIOR_H_
