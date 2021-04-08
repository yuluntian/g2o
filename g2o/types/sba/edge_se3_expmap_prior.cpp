//
// Created by yulun on 4/1/21.
//

#include "edge_se3_expmap_prior.h"

namespace g2o {

EdgeSE3ExpmapPrior::EdgeSE3ExpmapPrior() : BaseUnaryEdge<6, SE3Quat, VertexSE3Expmap>() {}

bool EdgeSE3ExpmapPrior::read(std::istream &is) {
  Vector7 meas;
  internal::readVector(is, meas);
  setMeasurement(SE3Quat(meas));
  return readInformationMatrix(is);
}

bool EdgeSE3ExpmapPrior::write(std::ostream &os) const {
  internal::writeVector(os, measurement().toVector());
  return writeInformationMatrix(os);
}

void EdgeSE3ExpmapPrior::setMeasurement(const SE3Quat &m) {
  _measurement = m;
  measurement_inv_ = m.inverse();
}

void EdgeSE3ExpmapPrior::computeError() {
  const auto *v = dynamic_cast<const VertexSE3Expmap *>(_vertices[0]);
  SE3Quat prior_inv_T = measurement_inv_ * v->estimate();
  _error = prior_inv_T.log();
}

void EdgeSE3ExpmapPrior::linearizeOplus() {
  _jacobianOplusXi = measurement_inv_.adj();
}

}

