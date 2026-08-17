# PB-CASLAM FAST-LIO effective transition contract

This tree contains a project-maintained corrected S2 covariance transport for
the audited FAST-LIO revision. FAST-LIO remains the work of the upstream
`hku-mars/FAST_LIO` authors and retains its BSD license and attribution.

## Provenance and scope

- Upstream repository: `hku-mars/FAST_LIO`
- Base source revision: `7cc4175de6f8ba2edf34bab02a42195b141027e9`
- Contract: `FAST_LIO_IKFOM23_EFFECTIVE_TRANSITION_S2CORR_V2`
- Locally validated ikd-Tree dependency revision: `e2e3f4e9d3`

The S2 covariance path uses an explicit floating half in `S2_Mx`, avoiding
integer division before scalar conversion while retaining
`MTK::A_matrix(Bu).transpose()`. In the active FAST-LIO runtime method
`update_iterated_dyn_share_modified()`, the final S2 covariance reset uses
`x_before`, the same state on which the correction is applied. Final-reset
base semantics in other IKFoM update variants were not audited and are not
covered by this contract. The common `S2_Mx` correction necessarily applies
to all callers.

## Effective transition

For one completed scan-posterior epoch, the exported first-order model is

```
delta_x_k = M_k delta_x_(k-1) + eta_k
Cov(eta_k) = W_k

M = G (I - K_x) C A_pred
W = G [(I - K_x) C W_pred C^T (I - K_x)^T + N_measurement] G^T
```

`A_pred` and `W_pred` are accumulated from the exact matrices used by each
prediction covariance update. For measurement dimension below 23,
`N_measurement = R K K^T`. At and above 23 dimensions, the implementation uses
the compact identity `R U (H^T H) U^T`, where `U = P_inv(:, 0:12)`, without
retaining a 23-by-measurement-dimension gain. `W` is the modeled process plus
measurement noise contribution; it is not defined from a posterior residual.

## State and matrix layout

The contract is fixed to the 23-dimensional FAST-LIO `state_ikfom` tangent:

| Indices | State component |
| --- | --- |
| 0-2 | Position |
| 3-5 | Rotation |
| 6-8 | Extrinsic rotation |
| 9-11 | Extrinsic translation |
| 12-14 | Velocity |
| 15-17 | Gyroscope bias |
| 18-20 | Accelerometer bias |
| 21-22 | Gravity S2 tangent |

Position perturbations are additive in the reference/world coordinates and
SO3 uses right perturbations. The 23-by-23 `M`, `W`, and current posterior
covariance arrays are value-owned and row-major (`row * 23 + column`). The
audited runtime labels the reference frame `camera_init` and the physical
state pose frame `body`; the contract performs no TF or LiDAR-frame
conversion. FAST-LIO does not own collaborative robot identity, so no robot ID
is assigned.

## Lifecycle

The exporter is disabled by default, transport-neutral, observational, and
stores only one current record. No ROS topic, service, action, parameter,
thread, socket, shared-memory channel, or file transport is added. Clearing a
record makes it unavailable and prevents stale reuse.

Sequence increments once per completed scan-posterior epoch, not per IMU
prediction. A clean epoch without a valid measurement correction is exported
as `PREDICTION_ONLY` and increments sequence. If a valid iteration mutates the
state but no normal final covariance commit occurs, the result is
`PARTIAL_INVALID_UPDATE`: no valid transition is emitted, continuity is
broken, and the next run is rebaselined. Reinitialization, external state or
covariance replacement, timestamp rollback, and other detected discontinuity
also prevent a transition from crossing the run boundary.

## Validation evidence

The clean production candidate is intended to reproduce the Task025B-5
validation performed with the existing local UGV Mid360 replay dataset. That
replay produced 917 transitions: 916 LiDAR-corrected epochs and one natural
prediction-only epoch. The maximum full 23-by-23 covariance reconstruction
error was approximately `5.44e-14`; exporter-enabled and exporter-disabled
state and covariance outputs were identical, with no NaN/Inf and no sequence
gaps. This dataset is validation evidence, not a production runtime
dependency.

The exported `M/W` is a first-order, filter-consistent effective transition
for the corrected FAST-LIO ESEKF covariance model. It is not an exact full-SLAM
posterior covariance: the historical map and map-induced correlations are not
represented as a joint stochastic state.
