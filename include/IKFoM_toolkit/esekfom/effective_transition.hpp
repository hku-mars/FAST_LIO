// Project-maintained extension to hku-mars/FAST_LIO under the repository's
// BSD license. The estimator remains attributable to its upstream authors.
#ifndef ESEKFOM_EFFECTIVE_TRANSITION_HPP_
#define ESEKFOM_EFFECTIVE_TRANSITION_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace esekfom {

constexpr std::size_t kEffectiveTransitionStateDof = 23U;
constexpr std::size_t kEffectiveTransitionMatrixSize =
    kEffectiveTransitionStateDof * kEffectiveTransitionStateDof;
constexpr std::size_t kEffectiveTransitionPositionIndex = 0U;
constexpr std::size_t kEffectiveTransitionRotationIndex = 3U;
constexpr std::size_t kEffectiveTransitionExtrinsicRotationIndex = 6U;
constexpr std::size_t kEffectiveTransitionExtrinsicTranslationIndex = 9U;
constexpr std::size_t kEffectiveTransitionVelocityIndex = 12U;
constexpr std::size_t kEffectiveTransitionGyroBiasIndex = 15U;
constexpr std::size_t kEffectiveTransitionAccelerometerBiasIndex = 18U;
constexpr std::size_t kEffectiveTransitionGravityIndex = 21U;

constexpr const char* kEffectiveTransitionContractId =
    "FAST_LIO_IKFOM23_EFFECTIVE_TRANSITION_S2CORR_V2";
constexpr const char* kEffectiveTransitionBaseSourceRevision =
    "7cc4175de6f8ba2edf34bab02a42195b141027e9";
constexpr const char* kEffectiveTransitionAuditedReferenceFrame =
    "camera_init";
constexpr const char* kEffectiveTransitionAuditedStatePoseFrame = "body";

enum class EffectiveTransitionKind {
  INVALID = 0,
  LIDAR_CORRECTED = 1,
  PREDICTION_ONLY = 2
};

enum class EffectiveTransitionStatus {
  INVALID = 0,
  VALID = 1,
  PARTIAL_INVALID_UPDATE = 2,
  CONTINUITY_BROKEN = 3,
  NONFINITE = 4
};

struct EffectiveTransitionDiagnostics {
  double max_abs_reconstruction_error =
      std::numeric_limits<double>::quiet_NaN();
  double frobenius_reconstruction_error =
      std::numeric_limits<double>::quiet_NaN();
  double relative_frobenius_error =
      std::numeric_limits<double>::quiet_NaN();
  double W_symmetry_error = std::numeric_limits<double>::quiet_NaN();
  double min_W_sym_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double P_symmetry_error = std::numeric_limits<double>::quiet_NaN();
  double min_P_sym_eigenvalue = std::numeric_limits<double>::quiet_NaN();
};

struct EffectiveTransitionRecord {
  EffectiveTransitionRecord() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    M_row_major.fill(nan);
    W_row_major.fill(nan);
    P_current_row_major.fill(nan);
    current_position.fill(nan);
    current_orientation_xyzw.fill(nan);
  }

  std::string contract_id = kEffectiveTransitionContractId;
  std::string base_source_revision =
      kEffectiveTransitionBaseSourceRevision;
  uint64_t estimator_run_id = 0U;
  uint64_t from_update_sequence = 0U;
  uint64_t to_update_sequence = 0U;
  double from_timestamp = std::numeric_limits<double>::quiet_NaN();
  double to_timestamp = std::numeric_limits<double>::quiet_NaN();
  std::string reference_frame_id;
  std::string state_pose_frame_id;
  EffectiveTransitionKind update_kind = EffectiveTransitionKind::INVALID;
  EffectiveTransitionStatus status = EffectiveTransitionStatus::INVALID;
  uint32_t state_dof =
      static_cast<uint32_t>(kEffectiveTransitionStateDof);
  uint32_t measurement_dimension = 0U;
  std::array<double, kEffectiveTransitionMatrixSize> M_row_major;
  std::array<double, kEffectiveTransitionMatrixSize> W_row_major;
  std::array<double, kEffectiveTransitionMatrixSize> P_current_row_major;
  std::array<double, 3U> current_position;
  std::array<double, 4U> current_orientation_xyzw;
  bool valid = false;
  bool complete_and_current = false;
  EffectiveTransitionDiagnostics diagnostics;
};

}  // namespace esekfom

#endif  // ESEKFOM_EFFECTIVE_TRANSITION_HPP_
