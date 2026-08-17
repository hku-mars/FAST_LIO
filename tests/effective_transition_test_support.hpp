// Project-maintained FAST-LIO contract tests under the repository BSD license.
#ifndef FAST_LIO_EFFECTIVE_TRANSITION_TEST_SUPPORT_HPP_
#define FAST_LIO_EFFECTIVE_TRANSITION_TEST_SUPPORT_HPP_

#include <omp.h>

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include "use-ikfom.hpp"

namespace effective_transition_test {

using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;
using Covariance = Filter::cov;
using StateVector = Eigen::Matrix<double, state_ikfom::DOF, 1>;

static_assert(state_ikfom::DOF == 23, "Tests require FAST-LIO IKFoM23");

struct MeasurementScenario {
  int dimension = 8;
  int callback_count = 0;
  int valid_callback_count = 0;
  bool always_invalid = false;
  bool invalidate_after_first = false;
  Eigen::Matrix<double, Eigen::Dynamic, 12> final_h_x;
};

inline MeasurementScenario*& currentScenario() {
  static MeasurementScenario* scenario = nullptr;
  return scenario;
}

inline vect3 makeVect3(const double x, const double y, const double z) {
  vect3 result;
  result << x, y, z;
  return result;
}

inline vect2 makeVect2(const double x, const double y) {
  vect2 result;
  result << x, y;
  return result;
}

inline void syntheticMeasurement(
    state_ikfom& state, esekfom::dyn_share_datastruct<double>& data) {
  MeasurementScenario* const scenario = currentScenario();
  ASSERT_NE(scenario, nullptr);
  ++scenario->callback_count;
  if (scenario->always_invalid ||
      (scenario->invalidate_after_first && scenario->callback_count > 1)) {
    data.valid = false;
    return;
  }

  data.valid = true;
  ++scenario->valid_callback_count;
  data.h_x.resize(scenario->dimension, 12);
  data.h.resize(scenario->dimension);
  for (int row = 0; row < scenario->dimension; ++row) {
    for (int column = 0; column < 12; ++column) {
      const double phase = static_cast<double>((row + 1) * (column + 2));
      data.h_x(row, column) =
          0.035 * std::sin(0.19 * phase) +
          (row == column ? 0.24 : 0.0);
    }
    data.h(row) =
        0.015 * std::cos(0.31 * static_cast<double>(row + 1)) +
        0.0002 * state.pos[0];
  }
  scenario->final_h_x = data.h_x;
}

inline StateVector deterministicStateDelta() {
  StateVector delta;
  for (int index = 0; index < delta.rows(); ++index) {
    delta[index] = 0.004 * std::sin(0.37 * static_cast<double>(index + 1));
  }
  delta.template segment<3>(3) << 0.12, -0.08, 0.05;
  delta.template segment<3>(6) << -0.04, 0.03, 0.06;
  delta.template segment<2>(21) << 0.09, -0.07;
  return delta;
}

inline state_ikfom deterministicState() {
  state_ikfom state;
  state.boxplus(deterministicStateDelta());
  return state;
}

inline Covariance deterministicCovariance() {
  Covariance factor = Covariance::Identity();
  for (int row = 0; row < factor.rows(); ++row) {
    factor(row, row) = 0.35 + 0.01 * static_cast<double>(row);
    for (int column = 0; column < row; ++column) {
      factor(row, column) = 0.025 * std::sin(
          0.23 * static_cast<double>((row + 1) * (column + 1)));
    }
  }
  Covariance covariance = factor * factor.transpose();
  covariance.template block<2, 3>(21, 0).array() += 0.012;
  covariance.template block<3, 2>(0, 21) =
      covariance.template block<2, 3>(21, 0).transpose();
  return covariance;
}

inline input_ikfom deterministicInput(const double scale) {
  input_ikfom input;
  input.acc = makeVect3(
      0.18 * scale, -0.11 * scale, 9.72 + 0.07 * scale);
  input.gyro = makeVect3(
      0.03 * scale, -0.02 * scale, 0.015 * scale);
  return input;
}

inline Filter::processnoisecovariance deterministicProcessNoise(
    const double scale) {
  Filter::processnoisecovariance noise =
      Filter::processnoisecovariance::Zero();
  for (int index = 0; index < noise.rows(); ++index) {
    noise(index, index) = scale *
        (2.0e-5 + 1.3e-6 * static_cast<double>(index + 1));
  }
  return noise;
}

inline void initializeFilter(
    Filter* filter, MeasurementScenario* scenario, const int iterations = 1,
    const bool export_enabled = true) {
  ASSERT_NE(filter, nullptr);
  ASSERT_NE(scenario, nullptr);
  currentScenario() = scenario;
  double limits[state_ikfom::DOF];
  std::fill(limits, limits + state_ikfom::DOF, 1.0e-12);
  filter->init_dyn_share(
      get_f, df_dx, df_dw, syntheticMeasurement, iterations, limits);
  state_ikfom initial_state = deterministicState();
  Covariance initial_covariance = deterministicCovariance();
  filter->change_x(initial_state);
  filter->change_P(initial_covariance);
  filter->setEffectiveTransitionExportEnabled(export_enabled);
}

inline void applyTwoPredictions(Filter* filter) {
  double first_dt = 0.013;
  double second_dt = 0.021;
  Filter::processnoisecovariance first_noise = deterministicProcessNoise(1.0);
  Filter::processnoisecovariance second_noise = deterministicProcessNoise(1.7);
  filter->predict(first_dt, first_noise, deterministicInput(1.0));
  filter->predict(second_dt, second_noise, deterministicInput(1.4));
}

inline Covariance rowMajorMatrix(
    const std::array<double, esekfom::kEffectiveTransitionMatrixSize>& data) {
  Covariance matrix;
  for (int row = 0; row < matrix.rows(); ++row) {
    for (int column = 0; column < matrix.cols(); ++column) {
      matrix(row, column) = data[static_cast<std::size_t>(
          row * matrix.cols() + column)];
    }
  }
  return matrix;
}

inline double maxAbs(const Covariance& matrix) {
  return matrix.cwiseAbs().maxCoeff();
}

struct CorrectedRun {
  Filter filter;
  MeasurementScenario scenario;
  Covariance previous_covariance;
  Covariance predicted_covariance;
  state_ikfom final_state;
  Covariance final_covariance;
  esekfom::EffectiveTransitionRecord record;
};

inline CorrectedRun runCorrected(const int measurement_dimension) {
  CorrectedRun run;
  run.scenario.dimension = measurement_dimension;
  initializeFilter(&run.filter, &run.scenario);
  run.previous_covariance = run.filter.get_P();
  run.filter.establishEffectiveTransitionBaseline(10.0);
  run.filter.beginEffectiveTransitionCycle();
  applyTwoPredictions(&run.filter);
  run.predicted_covariance = run.filter.get_P();
  double solve_time = 0.0;
  run.filter.update_iterated_dyn_share_modified(0.001, solve_time);
  run.filter.finalizeEffectiveTransitionEpoch(
      10.1, esekfom::kEffectiveTransitionAuditedReferenceFrame,
      esekfom::kEffectiveTransitionAuditedStatePoseFrame);
  EXPECT_TRUE(run.filter.hasEffectiveTransitionRecord());
  run.final_state = run.filter.get_x();
  run.final_covariance = run.filter.get_P();
  run.record = run.filter.latestEffectiveTransitionRecord();
  return run;
}

inline void expectReconstruction(
    const CorrectedRun& run, const double tolerance = 2.0e-10) {
  ASSERT_TRUE(run.record.valid);
  ASSERT_EQ(run.record.update_kind,
            esekfom::EffectiveTransitionKind::LIDAR_CORRECTED);
  const Covariance transition = rowMajorMatrix(run.record.M_row_major);
  const Covariance noise = rowMajorMatrix(run.record.W_row_major);
  const Covariance recorded = rowMajorMatrix(run.record.P_current_row_major);
  EXPECT_EQ(maxAbs(recorded - run.final_covariance), 0.0);
  const Covariance reconstructed =
      transition * run.previous_covariance * transition.transpose() + noise;
  EXPECT_LT(maxAbs(run.final_covariance - reconstructed), tolerance);
  EXPECT_LT(run.record.diagnostics.max_abs_reconstruction_error, tolerance);
  EXPECT_LT(run.record.diagnostics.W_symmetry_error, tolerance);
  EXPECT_GT(run.record.diagnostics.min_W_sym_eigenvalue, -tolerance);
}

}  // namespace effective_transition_test

#endif  // FAST_LIO_EFFECTIVE_TRANSITION_TEST_SUPPORT_HPP_
