// Project-maintained FAST-LIO contract tests under the repository BSD license.
#include "effective_transition_test_support.hpp"

namespace {

using effective_transition_test::makeVect2;
using effective_transition_test::makeVect3;

Eigen::Matrix<double, 3, 2> s2Mx(const S2& base, const vect2& delta) {
  S2 mutable_base = base;
  Eigen::Matrix<double, 3, 2> result;
  mutable_base.S2_Mx(result, delta);
  return result;
}

Eigen::Matrix<double, 3, 2> zeroScaleS2Mx(
    const S2& base, const vect2& delta) {
  S2 mutable_base = base;
  Eigen::Matrix<double, 3, 2> basis;
  mutable_base.S2_Bx(basis);
  if (delta.norm() < MTK::tolerance<double>()) {
    return -MTK::hat(base.get_vect()) * basis;
  }
  const vect3 rotation_vector = basis * delta;
  SO3 rotation;
  rotation.w() = MTK::exp<double, 3>(
      rotation.vec(), rotation_vector, double(1 / 2));
  return -rotation.toRotationMatrix() * MTK::hat(base.get_vect()) *
      MTK::A_matrix(rotation_vector).transpose() * basis;
}

Eigen::Matrix<double, 3, 2> finiteDifferenceS2Ambient(
    const S2& base, const vect2& delta, const double epsilon) {
  Eigen::Matrix<double, 3, 2> result;
  for (int column = 0; column < 2; ++column) {
    vect2 positive_delta = delta;
    vect2 negative_delta = delta;
    positive_delta[column] += epsilon;
    negative_delta[column] -= epsilon;
    S2 positive = base;
    S2 negative = base;
    positive.boxplus(positive_delta);
    negative.boxplus(negative_delta);
    result.col(column) =
        (positive.get_vect() - negative.get_vect()) / (2.0 * epsilon);
  }
  return result;
}

Eigen::Matrix2d finiteDifferenceS2Transport(
    const S2& source, const vect2& center_delta, const S2& target,
    const double epsilon) {
  Eigen::Matrix2d result;
  for (int column = 0; column < 2; ++column) {
    vect2 positive_delta = center_delta;
    vect2 negative_delta = center_delta;
    positive_delta[column] += epsilon;
    negative_delta[column] -= epsilon;
    S2 positive = source;
    S2 negative = source;
    positive.boxplus(positive_delta);
    negative.boxplus(negative_delta);
    vect2 positive_error;
    vect2 negative_error;
    positive.boxminus(positive_error, target);
    negative.boxminus(negative_error, target);
    result.col(column) =
        (positive_error - negative_error) / (2.0 * epsilon);
  }
  return result;
}

Eigen::Matrix3d finiteDifferenceSO3Transport(
    const SO3& source, const vect3& center_delta, const SO3& target,
    const double epsilon) {
  Eigen::Matrix3d result;
  for (int column = 0; column < 3; ++column) {
    vect3 positive_delta = center_delta;
    vect3 negative_delta = center_delta;
    positive_delta[column] += epsilon;
    negative_delta[column] -= epsilon;
    SO3 positive = source;
    SO3 negative = source;
    positive.boxplus(positive_delta);
    negative.boxplus(negative_delta);
    vect3 positive_error;
    vect3 negative_error;
    positive.boxminus(positive_error, target);
    negative.boxminus(negative_error, target);
    result.col(column) =
        (positive_error - negative_error) / (2.0 * epsilon);
  }
  return result;
}

TEST(S2HalfScaleRegression, IntegerDivisionIsZero) {
  using scalar = double;
  EXPECT_EQ(static_cast<double>(scalar(1 / 2)), 0.0);
  EXPECT_EQ(static_cast<double>(scalar(0.5)), 0.5);
}

TEST(S2MxRegression, MatchesBoxplusAmbientFiniteDifference) {
  S2 base;
  base.boxplus(makeVect2(0.22, -0.17));
  const vect2 delta = makeVect2(0.09, -0.06);
  const Eigen::Matrix<double, 3, 2> numerical =
      finiteDifferenceS2Ambient(base, delta, 1.0e-7);
  const double corrected_error =
      (s2Mx(base, delta) - numerical).cwiseAbs().maxCoeff();
  const double zero_scale_error =
      (zeroScaleS2Mx(base, delta) - numerical).cwiseAbs().maxCoeff();
  // Validated finite differences converge below 1e-8 for this fixture.
  EXPECT_LT(corrected_error, 2.0e-8);
  EXPECT_GT(zero_scale_error, 1.0e-3);
}

TEST(S2TransportRegression, FinalIterationCMatchesFiniteDifference) {
  S2 propagated;
  propagated.boxplus(makeVect2(0.17, -0.12));
  const vect2 displacement = makeVect2(0.09, -0.06);
  S2 iteration = propagated;
  iteration.boxplus(displacement);
  Eigen::Matrix<double, 2, 3> nx;
  iteration.S2_Nx_yy(nx);
  const Eigen::Matrix2d analytic = nx * s2Mx(propagated, displacement);
  const Eigen::Matrix2d numerical = finiteDifferenceS2Transport(
      propagated, displacement, iteration, 1.0e-7);
  EXPECT_LT((analytic - numerical).cwiseAbs().maxCoeff(), 2.0e-8);
}

TEST(S2TransportRegression, FinalResetUsesBeforeStateBase) {
  S2 propagated;
  propagated.boxplus(makeVect2(0.13, -0.08));
  S2 before = propagated;
  before.boxplus(makeVect2(0.07, 0.05));
  const vect2 correction = makeVect2(-0.06, 0.1);
  S2 post = before;
  post.boxplus(correction);
  Eigen::Matrix<double, 2, 3> nx;
  post.S2_Nx_yy(nx);
  const Eigen::Matrix2d required = nx * s2Mx(before, correction);
  const Eigen::Matrix2d propagated_base = nx * s2Mx(propagated, correction);
  const Eigen::Matrix2d numerical = finiteDifferenceS2Transport(
      before, correction, post, 1.0e-7);
  EXPECT_LT((required - numerical).cwiseAbs().maxCoeff(), 2.0e-8);
  EXPECT_GT((propagated_base - numerical).cwiseAbs().maxCoeff(), 1.0e-4);
}

TEST(SO3TransportRegression, FinalIterationCMatchesFiniteDifference) {
  SO3 propagated;
  propagated.boxplus(makeVect3(0.11, -0.07, 0.09));
  const vect3 displacement = makeVect3(0.13, -0.08, 0.06);
  SO3 iteration = propagated;
  iteration.boxplus(displacement);
  const Eigen::Matrix3d analytic =
      MTK::A_matrix(displacement).transpose();
  const Eigen::Matrix3d numerical = finiteDifferenceSO3Transport(
      propagated, displacement, iteration, 1.0e-7);
  EXPECT_LT((analytic - numerical).cwiseAbs().maxCoeff(), 2.0e-8);
}

TEST(SO3TransportRegression, FinalResetGMatchesFiniteDifference) {
  SO3 before;
  before.boxplus(makeVect3(-0.09, 0.14, 0.05));
  const vect3 correction = makeVect3(0.07, 0.04, -0.1);
  SO3 post = before;
  post.boxplus(correction);
  const Eigen::Matrix3d analytic = MTK::A_matrix(correction).transpose();
  const Eigen::Matrix3d numerical = finiteDifferenceSO3Transport(
      before, correction, post, 1.0e-7);
  EXPECT_LT((analytic - numerical).cwiseAbs().maxCoeff(), 2.0e-8);
}

}  // namespace
