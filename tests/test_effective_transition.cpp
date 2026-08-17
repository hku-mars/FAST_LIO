// Project-maintained FAST-LIO contract tests under the repository BSD license.
#include "effective_transition_test_support.hpp"

#include <iostream>

namespace {

using namespace effective_transition_test;

void expectExplicitMeasurementNoise(
    const CorrectedRun& run, const double tolerance = 2.0e-10) {
  Eigen::Matrix<double, Eigen::Dynamic, state_ikfom::DOF> full_h =
      Eigen::Matrix<double, Eigen::Dynamic, state_ikfom::DOF>::Zero(
          run.scenario.dimension, state_ikfom::DOF);
  full_h.leftCols(12) = run.scenario.final_h_x;
  const Covariance transported_prediction =
      run.filter.effectiveFinalIterationTransport() *
      run.predicted_covariance *
      run.filter.effectiveFinalIterationTransport().transpose();
  const Eigen::MatrixXd gain =
      transported_prediction * full_h.transpose() *
      (full_h * transported_prediction * full_h.transpose() +
       0.001 * Eigen::MatrixXd::Identity(
                   run.scenario.dimension, run.scenario.dimension)).inverse();
  EXPECT_LT(maxAbs(run.filter.effectiveKxPreReset() - gain * full_h),
            tolerance);
  EXPECT_LT(maxAbs(run.filter.effectiveMeasurementNoise() -
                   0.001 * gain * gain.transpose()),
            tolerance);
}

TEST(EffectivePrediction, MultipleStepsReconstructCovariance) {
  Filter filter;
  MeasurementScenario scenario;
  initializeFilter(&filter, &scenario);
  const Covariance previous = filter.get_P();
  filter.establishEffectiveTransitionBaseline(1.0);
  filter.beginEffectiveTransitionCycle();
  applyTwoPredictions(&filter);
  const Covariance reconstructed =
      filter.effectivePredictionTransition() * previous *
          filter.effectivePredictionTransition().transpose() +
      filter.effectivePredictionNoise();
  EXPECT_LT(maxAbs(filter.get_P() - reconstructed), 1.0e-12);
  EXPECT_GT(filter.effectivePredictionNoise().norm(), 0.0);
}

TEST(EffectivePrediction, ZeroDurationDoesNotChangeAccumulation) {
  Filter filter;
  MeasurementScenario scenario;
  initializeFilter(&filter, &scenario);
  filter.establishEffectiveTransitionBaseline(2.0);
  filter.beginEffectiveTransitionCycle();
  applyTwoPredictions(&filter);
  const Covariance covariance = filter.get_P();
  const Covariance transition = filter.effectivePredictionTransition();
  const Covariance noise = filter.effectivePredictionNoise();
  double zero_dt = 0.0;
  Filter::processnoisecovariance zero_noise = deterministicProcessNoise(2.3);
  filter.predict(zero_dt, zero_noise, deterministicInput(0.8));
  EXPECT_LT(maxAbs(filter.get_P() - covariance), 3.0e-16);
  EXPECT_LT(maxAbs(filter.effectivePredictionTransition() - transition),
            3.0e-16);
  EXPECT_EQ(maxAbs(filter.effectivePredictionNoise() - noise), 0.0);
}

TEST(EffectiveMeasurement, SmallDimensionReconstructsWithExplicitGain) {
  const CorrectedRun run = runCorrected(8);
  expectReconstruction(run);
  expectExplicitMeasurementNoise(run);
}

TEST(EffectiveMeasurement, BoundaryDimensionUsesCompactNoiseIdentity) {
  const CorrectedRun run = runCorrected(23);
  expectReconstruction(run);
  expectExplicitMeasurementNoise(run);
  EXPECT_EQ(run.record.measurement_dimension, 23U);
}

TEST(EffectiveMeasurement, LargeDimensionUsesCompactNoiseIdentity) {
  const CorrectedRun run = runCorrected(40);
  expectReconstruction(run);
  expectExplicitMeasurementNoise(run);
  EXPECT_EQ(run.record.measurement_dimension, 40U);
}

TEST(EffectiveMeasurement, FullCrossCovarianceReconstructsAllEntries) {
  const CorrectedRun run = runCorrected(8);
  expectReconstruction(run);
  EXPECT_GT((run.previous_covariance.template block<3, 2>(0, 21).norm()),
            0.0);
  EXPECT_GT((run.previous_covariance.template block<3, 2>(12, 21).norm()),
            0.0);
  const Covariance transition = rowMajorMatrix(run.record.M_row_major);
  const Covariance noise = rowMajorMatrix(run.record.W_row_major);
  const Covariance reconstructed = transition * run.previous_covariance *
      transition.transpose() + noise;
  EXPECT_LT(maxAbs(run.final_covariance - reconstructed), 2.0e-10);
}

TEST(EffectiveTransitionRecord, PredictionOnlyIsACompletedSequenceEpoch) {
  Filter filter;
  MeasurementScenario scenario;
  initializeFilter(&filter, &scenario);
  const Covariance previous = filter.get_P();
  filter.establishEffectiveTransitionBaseline(20.0, 7U, 11U);
  filter.beginEffectiveTransitionCycle();
  applyTwoPredictions(&filter);
  filter.finalizeEffectiveTransitionEpoch(20.2, "camera_init", "body");
  ASSERT_TRUE(filter.hasEffectiveTransitionRecord());
  const esekfom::EffectiveTransitionRecord& record =
      filter.latestEffectiveTransitionRecord();
  EXPECT_TRUE(record.valid);
  EXPECT_EQ(record.update_kind,
            esekfom::EffectiveTransitionKind::PREDICTION_ONLY);
  EXPECT_EQ(record.from_update_sequence, 11U);
  EXPECT_EQ(record.to_update_sequence, 12U);
  const Covariance transition = rowMajorMatrix(record.M_row_major);
  const Covariance noise = rowMajorMatrix(record.W_row_major);
  EXPECT_LT(maxAbs(filter.get_P() -
                   (transition * previous * transition.transpose() + noise)),
            1.0e-12);
}

TEST(EffectiveTransitionRecord, AllInvalidMeasurementRemainsPredictionOnly) {
  Filter filter;
  MeasurementScenario scenario;
  scenario.always_invalid = true;
  initializeFilter(&filter, &scenario);
  filter.establishEffectiveTransitionBaseline(30.0);
  filter.beginEffectiveTransitionCycle();
  applyTwoPredictions(&filter);
  double solve_time = 0.0;
  filter.update_iterated_dyn_share_modified(0.001, solve_time);
  filter.finalizeEffectiveTransitionEpoch(30.1, "camera_init", "body");
  ASSERT_TRUE(filter.hasEffectiveTransitionRecord());
  EXPECT_TRUE(filter.latestEffectiveTransitionRecord().valid);
  EXPECT_EQ(filter.latestEffectiveTransitionRecord().update_kind,
            esekfom::EffectiveTransitionKind::PREDICTION_ONLY);
  EXPECT_EQ(filter.latestEffectiveTransitionRecord().measurement_dimension,
            0U);
}

TEST(EffectiveTransitionRecord, PartialInvalidUpdateBreaksAndRebaselinesRun) {
  Filter filter;
  MeasurementScenario scenario;
  scenario.invalidate_after_first = true;
  initializeFilter(&filter, &scenario, 2);
  filter.establishEffectiveTransitionBaseline(40.0, 3U, 5U);
  filter.beginEffectiveTransitionCycle();
  applyTwoPredictions(&filter);
  double solve_time = 0.0;
  filter.update_iterated_dyn_share_modified(0.001, solve_time);
  filter.finalizeEffectiveTransitionEpoch(40.1, "camera_init", "body");
  ASSERT_TRUE(filter.hasEffectiveTransitionRecord());
  EXPECT_FALSE(filter.latestEffectiveTransitionRecord().valid);
  EXPECT_EQ(filter.latestEffectiveTransitionRecord().status,
            esekfom::EffectiveTransitionStatus::PARTIAL_INVALID_UPDATE);

  filter.clearEffectiveTransitionRecord();
  filter.beginEffectiveTransitionCycle();
  filter.finalizeEffectiveTransitionEpoch(40.2, "camera_init", "body");
  EXPECT_FALSE(filter.hasEffectiveTransitionRecord());
  filter.beginEffectiveTransitionCycle();
  filter.finalizeEffectiveTransitionEpoch(40.3, "camera_init", "body");
  ASSERT_TRUE(filter.hasEffectiveTransitionRecord());
  EXPECT_EQ(filter.latestEffectiveTransitionRecord().estimator_run_id, 4U);
  EXPECT_EQ(filter.latestEffectiveTransitionRecord().from_update_sequence, 0U);
}

TEST(EffectiveTransitionRecord, TimestampRollbackBreaksContinuity) {
  Filter filter;
  MeasurementScenario scenario;
  initializeFilter(&filter, &scenario);
  filter.establishEffectiveTransitionBaseline(50.0, 2U, 8U);
  filter.beginEffectiveTransitionCycle();
  filter.finalizeEffectiveTransitionEpoch(49.9, "camera_init", "body");
  ASSERT_TRUE(filter.hasEffectiveTransitionRecord());
  EXPECT_FALSE(filter.latestEffectiveTransitionRecord().valid);
  EXPECT_EQ(filter.latestEffectiveTransitionRecord().status,
            esekfom::EffectiveTransitionStatus::CONTINUITY_BROKEN);
}

TEST(EffectiveTransitionRecord, ContractIsRowMajorCurrentAndValueOwned) {
  Filter filter;
  MeasurementScenario scenario;
  initializeFilter(&filter, &scenario);
  filter.establishEffectiveTransitionBaseline(60.0, 5U, 9U);
  filter.beginEffectiveTransitionCycle();
  applyTwoPredictions(&filter);
  filter.finalizeEffectiveTransitionEpoch(60.1, "camera_init", "body");
  ASSERT_TRUE(filter.hasEffectiveTransitionRecord());
  esekfom::EffectiveTransitionRecord copy =
      filter.latestEffectiveTransitionRecord();
  EXPECT_EQ(copy.contract_id, esekfom::kEffectiveTransitionContractId);
  EXPECT_EQ(copy.base_source_revision,
            esekfom::kEffectiveTransitionBaseSourceRevision);
  EXPECT_EQ(copy.state_dof, 23U);
  EXPECT_TRUE(copy.complete_and_current);
  EXPECT_DOUBLE_EQ(copy.M_row_major[1U],
                   filter.effectivePredictionTransition()(0, 1));
  for (int index = 0; index < 3; ++index) {
    EXPECT_DOUBLE_EQ(copy.current_position[static_cast<std::size_t>(index)],
                     filter.get_x().pos(index));
  }
  for (int index = 0; index < 4; ++index) {
    EXPECT_DOUBLE_EQ(
        copy.current_orientation_xyzw[static_cast<std::size_t>(index)],
        filter.get_x().rot.coeffs()[index]);
  }
  filter.clearEffectiveTransitionRecord();
  EXPECT_FALSE(filter.hasEffectiveTransitionRecord());
  EXPECT_FALSE(filter.latestEffectiveTransitionRecord().complete_and_current);
  EXPECT_TRUE(copy.complete_and_current);
  std::cout << "production_record_size_bytes="
            << sizeof(esekfom::EffectiveTransitionRecord) << '\n';
}

TEST(EffectiveFilterInvariance, EnabledExporterDoesNotAlterStateOrCovariance) {
  Filter disabled;
  Filter enabled;
  MeasurementScenario disabled_scenario;
  MeasurementScenario enabled_scenario;
  disabled_scenario.dimension = 40;
  enabled_scenario.dimension = 40;
  initializeFilter(&disabled, &disabled_scenario, 1, false);
  initializeFilter(&enabled, &enabled_scenario, 1, true);
  enabled.establishEffectiveTransitionBaseline(70.0);
  enabled.beginEffectiveTransitionCycle();

  applyTwoPredictions(&disabled);
  currentScenario() = &disabled_scenario;
  double disabled_solve_time = 0.0;
  disabled.update_iterated_dyn_share_modified(0.001, disabled_solve_time);

  applyTwoPredictions(&enabled);
  currentScenario() = &enabled_scenario;
  double enabled_solve_time = 0.0;
  enabled.update_iterated_dyn_share_modified(0.001, enabled_solve_time);

  StateVector difference;
  enabled.get_x().boxminus(difference, disabled.get_x());
  EXPECT_EQ(difference.cwiseAbs().maxCoeff(), 0.0);
  EXPECT_EQ(maxAbs(enabled.get_P() - disabled.get_P()), 0.0);
}

TEST(EffectiveTransitionRecord, ExportIsDisabledByDefault) {
  Filter filter;
  MeasurementScenario scenario;
  initializeFilter(&filter, &scenario, 1, false);
  EXPECT_FALSE(filter.effectiveTransitionExportEnabled());
  filter.beginEffectiveTransitionCycle();
  applyTwoPredictions(&filter);
  filter.finalizeEffectiveTransitionEpoch(80.0, "camera_init", "body");
  EXPECT_FALSE(filter.hasEffectiveTransitionRecord());
}

}  // namespace
