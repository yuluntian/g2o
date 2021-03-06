// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "edge_line2d.h"

#ifdef G2O_USE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

using namespace g2o;
using namespace Eigen;

EdgeLine2D::EdgeLine2D() : BaseBinaryEdge<2, Line2D, VertexLine2D, VertexLine2D>() {
  _information.setIdentity();
  _error.setZero();
}

bool EdgeLine2D::read(std::istream& is) {
  internal::readVector(is, _measurement);
  return readInformationMatrix(is);
}

bool EdgeLine2D::write(std::ostream& os) const {
  internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
void EdgeLine2D::linearizeOplus() {
  _jacobianOplusXi = -Matrix2::Identity();
  _jacobianOplusXj = Matrix2::Identity();
}
#endif
