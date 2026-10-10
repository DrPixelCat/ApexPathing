# AI-assisted class review queue

Snapshot: 2026-10-07; HEAD `b594eff`; current uncommitted source included.

The best identifier is provenance: a recorded AI edit, followed by a human review status. Code style and the Git author are not reliable authorship tests. A Codex edit proves AI involvement in a file, not that AI originally wrote the entire class. **Creation call recorded** means an Add File patch exists; the file may have been recreated or moved from older code. **Modification call recorded** means edits were recorded without creation evidence. Package/import-only edits are separated into the coverage queue.

Scanned 271 available local Codex sessions for this exact repository, 143 production Java types/files and 134 test Java files. FTC SDK/sample files and package-info.java are excluded. Nested types are reviewed with their enclosing file.

Evidence labels:

- **Codex edit recorded**: a patch or literal Python write appears in a tool call, and its returned output has a success marker. A batched command success marker may belong to another command; inspect the cited call when exact provenance matters.
- **Write call recorded**: a concrete patch/write call exists, but success could not be established from the saved output.
- **Recent Git change only**: changed since 2026-08-01; a coverage safety net, not proof of AI involvement.
- **Unknown**: no captured write evidence or recent Git change. This does not mean human-written.

Ranking is approximate reading effort, using comment-free nonblank lines (L), branch/operator occurrences (B), and project imports (I): score = L + 3B + 5I. Thresholds: <100, <250, <550, <1100, then 1100+. B is a lexical indicator, not cyclomatic complexity. Compact numerical/state-estimation classes have explicit minimum tiers for the mathematical background they require, noted on their entries. Ranks are ascending by score within each tier; no dependency ordering is implied.

For each file: explain its purpose in your own words; comment units, coordinate frames, invariants, and non-obvious decisions; remove duplication/dead compatibility code only after understanding callers; read the relevant tests and run them when behavior changes. Tick the box once you have reviewed, commented, and cleaned it to your satisfaction. A checked box records human review, never authorship.

## Primary queue: recorded Codex involvement

128 production files have recorded writes beyond package/import-only patches. Start here. An asterisk after the evidence label means the file has uncommitted changes in this snapshot.

### 1. Small types and helpers

- [ ] **1. [tuning/localizer/calibration/CalibrationAxis.java](../../TeamCode/src/main/java/tuning/localizer/calibration/CalibrationAxis.java)** — Creation call recorded; Write call recorded; score 2 (L=2, B=0, I=0).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 782; historical package mapped by unique filename.
- [ ] **2. [localizers/util/DataFilter.java](../../TeamCode/src/main/java/localizers/util/DataFilter.java)** — Modification call recorded; Write call recorded; score 6 (L=6, B=0, I=0).
  Evidence: session `01a101f0-fed2-7fe2-9c8f-70cb84584776`, 2026-10-03, patch-update, JSONL line 204.
- [ ] **3. [paths/constraint/PathConstraint.java](../../TeamCode/src/main/java/paths/constraint/PathConstraint.java)** — Modification call recorded; Write call recorded; score 11 (L=11, B=0, I=0).
  Evidence: session `01a0ff49-4135-7d91-be01-a690dad9fdd5`, 2026-10-03, patch-update, JSONL line 401.
- [ ] **4. [tuning/localizer/calibration/DirectionTrial.java](../../TeamCode/src/main/java/tuning/localizer/calibration/DirectionTrial.java)** — Creation call recorded; Write call recorded; score 12 (L=12, B=0, I=0).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-11, patch-add, JSONL line 2921; historical package mapped by unique filename.
- [ ] **5. [geometry/ArcPose.java](../../TeamCode/src/main/java/geometry/ArcPose.java)** — Modification call recorded; Write call recorded; score 19 (L=16, B=1, I=0).
  Evidence: session `01a10ce3-3642-7063-b347-f2f9753b9684`, 2026-10-05, patch-update, JSONL line 466.
- [ ] **6. [follower/FollowerDiagnostics.java](../../TeamCode/src/main/java/follower/FollowerDiagnostics.java)** — Modification call recorded; Write call recorded; score 20 (L=10, B=0, I=2).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, patch-update, JSONL line 317; historical package mapped by unique filename.
- [ ] **7. [geometry/ParametricSegment.java](../../TeamCode/src/main/java/geometry/ParametricSegment.java)** — Modification call recorded; Codex edit recorded; score 20 (L=17, B=1, I=0).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 1017.
- [ ] **8. [localizers/BaseLocalizerConstants.java](../../TeamCode/src/main/java/localizers/BaseLocalizerConstants.java)** — Modification call recorded; Write call recorded; score 20 (L=12, B=1, I=1).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-update, JSONL line 701.
- [ ] **9. [tuning/localizer/calibration/CalibrationSnapshot.java](../../TeamCode/src/main/java/tuning/localizer/calibration/CalibrationSnapshot.java)** — Creation call recorded; Write call recorded; score 21 (L=13, B=1, I=1).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 782; historical package mapped by unique filename.
- [ ] **10. [paths/heading/HeadingInterpolator.java](../../TeamCode/src/main/java/paths/heading/HeadingInterpolator.java)** — Modification call recorded; Write call recorded; score 22 (L=12, B=0, I=2).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, patch-update, JSONL line 1296.
- [ ] **11. [tuning/localizer/calibration/SpinTrial.java](../../TeamCode/src/main/java/tuning/localizer/calibration/SpinTrial.java)** — Creation call recorded; Write call recorded; score 22 (L=22, B=0, I=0).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 782; historical package mapped by unique filename.
- [ ] **12. [tuning/follower/TuningMeasurements.java](../../TeamCode/src/main/java/tuning/follower/TuningMeasurements.java)** — Modification call recorded; Write call recorded; score 26 (L=15, B=2, I=1).
  Evidence: session `01a101d8-787a-76d3-b28e-f89bf5542219`, 2026-10-03, patch-update, JSONL line 182.
- [ ] **13. [tuning/localizer/calibration/CalibrationCandidate.java](../../TeamCode/src/main/java/tuning/localizer/calibration/CalibrationCandidate.java)** — Creation call recorded; Write call recorded; score 27 (L=21, B=2, I=0).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 782; historical package mapped by unique filename.
- [ ] **14. [follower/FollowerStuckWatchdog.java](../../TeamCode/src/main/java/follower/FollowerStuckWatchdog.java)** — Creation call recorded; Write call recorded; score 28 (L=19, B=3, I=0).
  Evidence: session `01a09203-c009-7773-873d-5f5a7aa11352`, 2026-09-11, patch-add, JSONL line 180; historical package mapped by unique filename.
- [ ] **15. [paths/movements/FollowerMovement.java](../../TeamCode/src/main/java/paths/movements/FollowerMovement.java)** — Modification call recorded; Write call recorded; score 31 (L=21, B=0, I=2).
  Evidence: session `01a0843c-610a-7d30-90b3-92a7ba8a36fc`, 2026-09-09, patch-update, JSONL line 247.
- [ ] **16. [config/ApexConstants.java](../../TeamCode/src/main/java/config/ApexConstants.java)** — Modification call recorded; Write call recorded; score 36 (L=12, B=3, I=3).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-update, JSONL line 757; historical package mapped by unique filename.
- [ ] **17. [geometry/Scalar.java](../../TeamCode/src/main/java/geometry/Scalar.java)** — Modification call recorded; Write call recorded; score 37 (L=31, B=2, I=0).
  Evidence: session `01a0a04a-101c-7fe2-800d-1a6cea465755`, 2026-09-14, patch-update, JSONL line 52.
- [ ] **18. [paths/constraint/AngularConstraint.java](../../TeamCode/src/main/java/paths/constraint/AngularConstraint.java)** — Modification call recorded; Write call recorded; score 39 (L=28, B=2, I=1).
  Evidence: session `01a0ff49-4135-7d91-be01-a690dad9fdd5`, 2026-10-03, patch-update, JSONL line 401.
- [ ] **19. [paths/constraint/TranslationalConstraint.java](../../TeamCode/src/main/java/paths/constraint/TranslationalConstraint.java)** — Modification call recorded; Write call recorded; score 39 (L=28, B=2, I=1).
  Evidence: session `01a0ff49-4135-7d91-be01-a690dad9fdd5`, 2026-10-03, patch-update, JSONL line 401.
- [ ] **20. [follower/MotionFeedforward.java](../../TeamCode/src/main/java/follower/MotionFeedforward.java)** — Creation call recorded; Write call recorded; score 46 (L=31, B=5, I=0).
  Evidence: session `01a09203-c009-7773-873d-5f5a7aa11352`, 2026-09-11, patch-add, JSONL line 180; historical package mapped by unique filename.
- [ ] **21. [follower/PathProgressEstimator.java](../../TeamCode/src/main/java/follower/PathProgressEstimator.java)** — Creation call recorded; Write call recorded; score 47 (L=32, B=5, I=0).
  Evidence: session `01a09203-c009-7773-873d-5f5a7aa11352`, 2026-09-11, patch-add, JSONL line 180; historical package mapped by unique filename.
- [ ] **22. [geometry/Dist.java](../../TeamCode/src/main/java/geometry/Dist.java)** — Modification call recorded; Write call recorded; score 49 (L=43, B=2, I=0).
  Evidence: session `01a0a04a-101c-7fe2-800d-1a6cea465755`, 2026-09-14, patch-update, JSONL line 52.
- [ ] **23. [localizers/OdometryPod.java](../../TeamCode/src/main/java/localizers/OdometryPod.java)** — Modification call recorded; Write call recorded; score 50 (L=41, B=3, I=0).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-update, JSONL line 715.
- [ ] **24. [paths/heading/TankInterpolator.java](../../TeamCode/src/main/java/paths/heading/TankInterpolator.java)** — Modification call recorded; Write call recorded; score 51 (L=32, B=3, I=2).
  Evidence: session `01a0ff49-4135-7d91-be01-a690dad9fdd5`, 2026-10-03, patch-update, JSONL line 418.
- [ ] **25. [trajectory/profile/MotionParameters.java](../../TeamCode/src/main/java/trajectory/profile/MotionParameters.java)** — Modification call recorded; Codex edit recorded; score 51 (L=51, B=0, I=0).
  Evidence: session `019f3e0b-3507-73f2-8381-aca2db393328`, 2026-07-07, patch-update, JSONL line 75; historical package mapped by unique filename.
- [ ] **26. [paths/heading/ReversedHeadingInterpolator.java](../../TeamCode/src/main/java/paths/heading/ReversedHeadingInterpolator.java)** — Creation call recorded; Write call recorded; score 52 (L=39, B=1, I=2).
  Evidence: session `01a0843c-610a-7d30-90b3-92a7ba8a36fc`, 2026-09-09, patch-add, JSONL line 247.
- [ ] **27. [org/firstinspires/ftc/teamcode/apexpathing/TeleOpTest.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/TeleOpTest.java)** — Modification call recorded; Write call recorded; score 56 (L=40, B=2, I=2).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-14, patch-update, JSONL line 1577.
- [ ] **28. [feedforward/HolonomicFeedforward.java](../../TeamCode/src/main/java/feedforward/HolonomicFeedforward.java)** — Creation call recorded; Write call recorded; score 58 (L=37, B=2, I=3).
  Evidence: session `01a0fde7-ad4c-7f92-b52b-41d65f9bf889`, 2026-10-02, patch-add, JSONL line 260; historical package mapped by unique filename.
- [ ] **29. [tuning/follower/acceleration/AccelerationSearch.java](../../TeamCode/src/main/java/tuning/follower/acceleration/AccelerationSearch.java)** — Creation call recorded; Write call recorded; score 65 (L=38, B=9, I=0).
  Evidence: session `01a093e1-ae3d-7191-95bd-6d48841fa253`, 2026-09-12, patch-add, JSONL line 278; historical package mapped by unique filename.
- [ ] **30. [follower/PathCompletionPolicy.java](../../TeamCode/src/main/java/follower/PathCompletionPolicy.java)** — Creation call recorded; Write call recorded; score 66 (L=39, B=9, I=0).
  Evidence: session `01a09203-c009-7773-873d-5f5a7aa11352`, 2026-09-11, patch-add, JSONL line 180; historical package mapped by unique filename.
- [ ] **31. [tuning/localizer/LocalizerAdapter.java](../../TeamCode/src/main/java/tuning/localizer/LocalizerAdapter.java)** — Creation call recorded; Write call recorded; score 68 (L=21, B=4, I=7).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 782.
- [ ] **32. [tuning/follower/TunerContext.java](../../TeamCode/src/main/java/tuning/follower/TunerContext.java)** — Modification call recorded; Write call recorded; score 75 (L=40, B=5, I=4).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-update, JSONL line 694.
- [ ] **33. [localizers/CalibrationJson.java](../../TeamCode/src/main/java/localizers/CalibrationJson.java)** — Creation call recorded; Write call recorded; score 81 (L=43, B=6, I=4).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 708.
- [ ] **34. [tuning/localizer/phases/FilterMetrics.java](../../TeamCode/src/main/java/tuning/localizer/phases/FilterMetrics.java)** — Creation call recorded; Write call recorded; score 81 (L=49, B=4, I=4).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 928; historical package mapped by unique filename.
- [ ] **35. [tuning/TunerContext.java](../../TeamCode/src/main/java/tuning/TunerContext.java)** — Creation call recorded; Codex edit recorded; score 86 (L=56, B=10, I=0).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-09, write-command, JSONL line 345.
- [ ] **36. [drivetrains/BaseDrivetrainConstants.java](../../TeamCode/src/main/java/drivetrains/BaseDrivetrainConstants.java)** — Modification call recorded; Write call recorded; score 88 (L=48, B=10, I=2).
  Evidence: session `01a10ce3-3642-7063-b347-f2f9753b9684`, 2026-10-05, patch-update, JSONL line 466.
- [ ] **37. [feedforward/MotorPowerModel.java](../../TeamCode/src/main/java/feedforward/MotorPowerModel.java)** — Creation call recorded; Write call recorded *; score 88 (L=43, B=15, I=0).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-02, patch-add, JSONL line 2251; historical package mapped by unique filename.
- [ ] **38. [geometry/Vector.java](../../TeamCode/src/main/java/geometry/Vector.java)** — Modification call recorded; Write call recorded; score 96 (L=78, B=6, I=0).
  Evidence: session `01a10ce3-3642-7063-b347-f2f9753b9684`, 2026-10-05, patch-update, JSONL line 474.

### 2. Focused logic

- [ ] **39. [trajectory/VelocityPursuit.java](../../TeamCode/src/main/java/trajectory/VelocityPursuit.java)** — Creation call recorded; Write call recorded; score 100 (L=46, B=18, I=0).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-05, patch-add, JSONL line 2902.
- [ ] **40. [storage/ApexStorage.java](../../TeamCode/src/main/java/storage/ApexStorage.java)** — Modification call recorded; Codex edit recorded; score 101 (L=62, B=13, I=0).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-09, write-command, JSONL line 345; historical package mapped by unique filename.
- [ ] **41. [localizers/util/LowPassFilter.java](../../TeamCode/src/main/java/localizers/util/LowPassFilter.java)** — Modification call recorded; Write call recorded; score 103 (L=79, B=8, I=0).
  Evidence: session `01a101f0-fed2-7fe2-9c8f-70cb84584776`, 2026-10-03, patch-update, JSONL line 204.
- [ ] **42. [controllers/PDSController.java](../../TeamCode/src/main/java/controllers/PDSController.java)** — Creation call recorded; Write call recorded; score 108 (L=79, B=8, I=1).
  Evidence: session `01a05fb2-639d-7312-95ef-44b7ab13aca0`, 2026-09-02, patch-update, JSONL line 863.
- [ ] **43. [tuning/localizer/TuningPhase.java](../../TeamCode/src/main/java/tuning/localizer/TuningPhase.java)** — Creation call recorded; Write call recorded; score 112 (L=79, B=11, I=0).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 900.
- [ ] **44. [follower/MovementCallbackRunner.java](../../TeamCode/src/main/java/follower/MovementCallbackRunner.java)** — Creation call recorded; Write call recorded; score 113 (L=52, B=12, I=5).
  Evidence: session `01a09203-c009-7773-873d-5f5a7aa11352`, 2026-09-11, patch-add, JSONL line 180; historical package mapped by unique filename.
- [ ] **45. [trajectory/TrajectorySample.java](../../TeamCode/src/main/java/trajectory/TrajectorySample.java)** — Creation call recorded; Codex edit recorded; score 113 (L=78, B=0, I=7).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 790.
- [ ] **46. [trajectory/generators/SwerveProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/SwerveProfileGenerator.java)** — Creation call recorded; Codex edit recorded; score 113 (L=63, B=5, I=7).
  Evidence: session `019f2428-2020-7842-902c-743f4bd95173`, 2026-07-02, patch-update, JSONL line 148; historical package mapped by unique filename.
- [ ] **47. [trajectory/generators/TankProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/TankProfileGenerator.java)** — Creation call recorded; Codex edit recorded; score 114 (L=65, B=8, I=5).
  Evidence: session `019f2428-2020-7842-902c-743f4bd95173`, 2026-07-02, patch-update, JSONL line 544; historical package mapped by unique filename.
- [ ] **48. [trajectory/OnlineTrajectoryPlanner.java](../../TeamCode/src/main/java/trajectory/OnlineTrajectoryPlanner.java)** — Creation call recorded; Codex edit recorded; score 117 (L=59, B=11, I=5).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 720.
- [ ] **49. [geometry/GeometryFactory.java](../../TeamCode/src/main/java/geometry/GeometryFactory.java)** — Modification call recorded; Write call recorded; score 118 (L=67, B=7, I=6).
  Evidence: session `01a10ce3-3642-7063-b347-f2f9753b9684`, 2026-10-05, patch-update, JSONL line 466.
- [ ] **50. [org/firstinspires/ftc/teamcode/apexpathing/SimpleDriveTest.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/SimpleDriveTest.java)** — Modification call recorded; Write call recorded; score 118 (L=62, B=7, I=7).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-14, patch-update, JSONL line 1577.
- [ ] **51. [tuning/follower/sysid/ManualFeedforwardProfile.java](../../TeamCode/src/main/java/tuning/follower/sysid/ManualFeedforwardProfile.java)** — Creation call recorded; Write call recorded; score 123 (L=63, B=20, I=0).
  Evidence: session `01a10db1-3700-7c11-9ea1-92a260fb773a`, 2026-10-05, patch-add, JSONL line 71.
- [ ] **52. [trajectory/generators/ConstraintSchedule.java](../../TeamCode/src/main/java/trajectory/generators/ConstraintSchedule.java)** — Creation call recorded; Write call recorded; score 124 (L=64, B=10, I=6).
  Evidence: session `01a0ff49-4135-7d91-be01-a690dad9fdd5`, 2026-10-03, patch-add, JSONL line 401; historical package mapped by unique filename.
- [ ] **53. [trajectory/StoppingHorizon.java](../../TeamCode/src/main/java/trajectory/StoppingHorizon.java)** — Creation call recorded; Codex edit recorded; score 125 (L=59, B=12, I=6).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 1552.
- [ ] **54. [drivetrains/Mecanum.java](../../TeamCode/src/main/java/drivetrains/Mecanum.java)** — Modification call recorded; Codex edit recorded; score 127 (L=96, B=7, I=2).
  Evidence: session `019f2428-2020-7842-902c-743f4bd95173`, 2026-07-09, patch-update, JSONL line 1026.
- [ ] **55. [tuning/localizer/phases/RelativeImuAccumulator.java](../../TeamCode/src/main/java/tuning/localizer/phases/RelativeImuAccumulator.java)** — Creation call recorded; Write call recorded; score 130 (L=82, B=11, I=3).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 900; historical package mapped by unique filename.
- [ ] **56. [org/firstinspires/ftc/teamcode/apexpathing/ExampleAutoPath.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/ExampleAutoPath.java)** — Modification call recorded; Write call recorded; score 132 (L=79, B=1, I=10).
  Evidence: session `01a05fb2-639d-7312-95ef-44b7ab13aca0`, 2026-09-02, patch-update, JSONL line 5316.
- [ ] **57. [geometry/CubicSpline1D.java](../../TeamCode/src/main/java/geometry/CubicSpline1D.java)** — Modification call recorded; Write call recorded; score 133 (L=82, B=17, I=0).
  Evidence: session `01a0ff49-4135-7d91-be01-a690dad9fdd5`, 2026-10-03, patch-update, JSONL line 393.
- [ ] **58. [follower/FollowerPathState.java](../../TeamCode/src/main/java/follower/FollowerPathState.java)** — Creation call recorded; Codex edit recorded; score 139 (L=104, B=10, I=1).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 720; historical package mapped by unique filename.
- [ ] **59. [paths/movements/Turn.java](../../TeamCode/src/main/java/paths/movements/Turn.java)** — Modification call recorded; Codex edit recorded; score 141 (L=78, B=11, I=6).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-09, write-command, JSONL line 276.
- [ ] **60. [controllers/DriveController.java](../../TeamCode/src/main/java/controllers/DriveController.java)** — Creation call recorded; Write call recorded; score 146 (L=107, B=8, I=3).
  Evidence: session `019f4cc6-d8be-7e11-9ccf-1a61a18d009a`, 2026-07-10, patch-add, JSONL line 874; historical package mapped by unique filename.
- [ ] **61. [controllers/TurnController.java](../../TeamCode/src/main/java/controllers/TurnController.java)** — Creation call recorded; Codex edit recorded; score 151 (L=108, B=11, I=2).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 351.
- [ ] **62. [feedforward/CenteredBrakingFeedforward.java](../../TeamCode/src/main/java/feedforward/CenteredBrakingFeedforward.java)** — Creation call recorded; Write call recorded; score 156 (L=74, B=19, I=5).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-04, patch-add, JSONL line 644.
- [ ] **63. [follower/VelocityReference.java](../../TeamCode/src/main/java/follower/VelocityReference.java)** — Creation call recorded; Write call recorded; score 156 (L=87, B=23, I=0).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-05, patch-add, JSONL line 2179.
- [ ] **64. [trajectory/generators/TurnProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/TurnProfileGenerator.java)** — Modification call recorded; Codex edit recorded; score 161 (L=91, B=15, I=5).
  Evidence: session `019f3e0b-3507-73f2-8381-aca2db393328`, 2026-07-07, patch-update, JSONL line 75; historical package mapped by unique filename.
- [ ] **65. [drivetrains/DualActuated.java](../../TeamCode/src/main/java/drivetrains/DualActuated.java)** — Modification call recorded; Codex edit recorded; score 166 (L=118, B=16, I=0).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-09, write-command, JSONL line 276.
- [ ] **66. [tuning/follower/feedback/ManualResponseMetrics.java](../../TeamCode/src/main/java/tuning/follower/feedback/ManualResponseMetrics.java)** — Creation call recorded; Write call recorded; score 168 (L=112, B=17, I=1).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-14, patch-add, JSONL line 2712; historical package mapped by unique filename.
- [ ] **67. [drivetrains/BaseDrivetrain.java](../../TeamCode/src/main/java/drivetrains/BaseDrivetrain.java)** — Modification call recorded; Write call recorded; score 171 (L=102, B=23, I=0).
  Evidence: session `01a10ce3-3642-7063-b347-f2f9753b9684`, 2026-10-05, patch-update, JSONL line 466.
- [ ] **68. [geometry/BSpline.java](../../TeamCode/src/main/java/geometry/BSpline.java)** — Modification call recorded; Codex edit recorded; score 171 (L=129, B=14, I=0).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 1017.
- [ ] **69. [trajectory/TrajectoryField2D.java](../../TeamCode/src/main/java/trajectory/TrajectoryField2D.java)** — Creation call recorded; Codex edit recorded; score 178 (L=99, B=18, I=5).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 838.
- [ ] **70. [localizers/ThreeWheel.java](../../TeamCode/src/main/java/localizers/ThreeWheel.java)** — Modification call recorded; Write call recorded; score 179 (L=129, B=10, I=4).
  Evidence: session `01a0875b-07a5-7f93-a637-04e8fd233e1a`, 2026-09-09, patch-update, JSONL line 66.
- [ ] **71. [localizers/LocalizerSet.java](../../TeamCode/src/main/java/localizers/LocalizerSet.java)** — Creation call recorded; Write call recorded; score 183 (L=98, B=20, I=5).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 757; historical package mapped by unique filename.
- [ ] **72. [paths/builders/PathBuilder.java](../../TeamCode/src/main/java/paths/builders/PathBuilder.java)** — Modification call recorded; Codex edit recorded; score 184 (L=99, B=15, I=8).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 769.
- [ ] **73. [paths/builders/TankPathBuilder.java](../../TeamCode/src/main/java/paths/builders/TankPathBuilder.java)** — Modification call recorded; Write call recorded; score 185 (L=107, B=11, I=9).
  Evidence: session `01a06a9a-abdb-7a11-9981-8ae4299ff7b2`, 2026-09-04, patch-update, JSONL line 677.
- [ ] **74. [localizers/TwoWheel.java](../../TeamCode/src/main/java/localizers/TwoWheel.java)** — Modification call recorded; Write call recorded; score 194 (L=138, B=12, I=4).
  Evidence: session `01a0875b-07a5-7f93-a637-04e8fd233e1a`, 2026-09-09, patch-update, JSONL line 66.
- [ ] **75. [paths/builders/TurnBuilder.java](../../TeamCode/src/main/java/paths/builders/TurnBuilder.java)** — Modification call recorded; Codex edit recorded; score 197 (L=108, B=23, I=4).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-09, write-command, JSONL line 276.
- [ ] **76. [tuning/TuningCsvWriter.java](../../TeamCode/src/main/java/tuning/TuningCsvWriter.java)** — Modification call recorded; Write call recorded; score 203 (L=111, B=29, I=1).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-update, JSONL line 631.
- [ ] **77. [tuning/localizer/LocalizationTunerContext.java](../../TeamCode/src/main/java/tuning/localizer/LocalizationTunerContext.java)** — Creation call recorded; Write call recorded; score 231 (L=122, B=23, I=8).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 900.
- [ ] **78. [follower/TurnExecutor.java](../../TeamCode/src/main/java/follower/TurnExecutor.java)** — Creation call recorded; Codex edit recorded; score 238 (L=112, B=22, I=12).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 351; historical package mapped by unique filename.
- [ ] **79. [feedforward/MotionCharacterization.java](../../TeamCode/src/main/java/feedforward/MotionCharacterization.java)** — Creation call recorded; Write call recorded; score 240 (L=111, B=43, I=0).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-02, patch-add, JSONL line 2251; historical package mapped by unique filename.
- [ ] **80. [tuning/follower/TuningPhase.java](../../TeamCode/src/main/java/tuning/follower/TuningPhase.java)** — Modification call recorded; Write call recorded; score 240 (L=145, B=30, I=1).
  Evidence: session `01a08d5f-9ae8-7360-8783-b9f88d2bbd70`, 2026-09-10, patch-update, JSONL line 128.
- [ ] **81. [localizers/MecanumDriveEncoders.java](../../TeamCode/src/main/java/localizers/MecanumDriveEncoders.java)** — Modification call recorded; Write call recorded; score 241 (L=174, B=19, I=2).
  Evidence: session `01a0875b-07a5-7f93-a637-04e8fd233e1a`, 2026-09-09, patch-update, JSONL line 66.

### 3. Several interacting responsibilities

- [ ] **82. [tuning/follower/characterization/MotorAuthorityRegression.java](../../TeamCode/src/main/java/tuning/follower/characterization/MotorAuthorityRegression.java)** — Creation call recorded; Codex edit recorded *; score 128 (L=60, B=21, I=1).
  Evidence: session `01a116fe-1282-7610-906a-7aac27ed0ad6`, 2026-10-07, write-command, JSONL line 140.
  Review focus: parameter identification and physical interpretation (domain prerequisite influences tier).
- [ ] **83. [tuning/follower/sysid/SysIdRegression.java](../../TeamCode/src/main/java/tuning/follower/sysid/SysIdRegression.java)** — Creation call recorded; Codex edit recorded *; score 236 (L=110, B=42, I=0).
  Evidence: session `01a11718-7f12-7033-aa18-d1a236b77ee5`, 2026-10-07, write-command, JSONL line 137.
  Review focus: regression conditioning, excitation and fit quality (domain prerequisite influences tier).
- [ ] **84. [follower/HolonomicCommandAllocator.java](../../TeamCode/src/main/java/follower/HolonomicCommandAllocator.java)** — Creation call recorded; Write call recorded; score 254 (L=177, B=14, I=7).
  Evidence: session `01a09203-c009-7773-873d-5f5a7aa11352`, 2026-09-11, patch-add, JSONL line 187; historical package mapped by unique filename.
- [ ] **85. [drivetrains/CoaxialSwerve.java](../../TeamCode/src/main/java/drivetrains/CoaxialSwerve.java)** — Modification call recorded; Write call recorded; score 270 (L=160, B=30, I=4).
  Evidence: session `01a0a04a-101c-7fe2-800d-1a6cea465755`, 2026-09-14, patch-update, JSONL line 511.
- [ ] **86. [tuning/localizer/phases/DistancePhase.java](../../TeamCode/src/main/java/tuning/localizer/phases/DistancePhase.java)** — Creation call recorded; Write call recorded; score 270 (L=150, B=30, I=6).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 907; historical package mapped by unique filename.
- [ ] **87. [org/firstinspires/ftc/teamcode/apexpathing/LocalizationTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/LocalizationTuner.java)** — Creation call recorded; Write call recorded *; score 271 (L=133, B=31, I=9).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 935.
- [ ] **88. [follower/TankPathFollower.java](../../TeamCode/src/main/java/follower/TankPathFollower.java)** — Creation call recorded; Codex edit recorded; score 280 (L=168, B=19, I=11).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-02, write-command, JSONL line 2288; historical package mapped by unique filename.
- [ ] **89. [localizers/OTOS.java](../../TeamCode/src/main/java/localizers/OTOS.java)** — Modification call recorded; Write call recorded; score 283 (L=209, B=18, I=4).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-update, JSONL line 708.
- [ ] **90. [localizers/TankDriveEncoders.java](../../TeamCode/src/main/java/localizers/TankDriveEncoders.java)** — Creation call recorded; Write call recorded; score 290 (L=198, B=29, I=1).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-09, patch-add, JSONL line 106.
- [ ] **91. [trajectory/generators/MecanumProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/MecanumProfileGenerator.java)** — Creation call recorded; Codex edit recorded; score 295 (L=164, B=27, I=10).
  Evidence: session `019f2428-2020-7842-902c-743f4bd95173`, 2026-07-02, patch-update, JSONL line 551; historical package mapped by unique filename.
- [ ] **92. [paths/heading/HolonomicInterpolator.java](../../TeamCode/src/main/java/paths/heading/HolonomicInterpolator.java)** — Modification call recorded; Codex edit recorded; score 297 (L=153, B=43, I=3).
  Evidence: session `019f48d7-b199-75c2-9422-3cfb88a0dac3`, 2026-07-09, patch-update, JSONL line 93.
- [ ] **93. [controllers/FeedbackController.java](../../TeamCode/src/main/java/controllers/FeedbackController.java)** — Creation call recorded; Write call recorded; score 303 (L=221, B=24, I=2).
  Evidence: session `01a0a04a-101c-7fe2-800d-1a6cea465755`, 2026-09-14, patch-add, JSONL line 191.
- [ ] **94. [tuning/follower/feedback/HeadingPhase.java](../../TeamCode/src/main/java/tuning/follower/feedback/HeadingPhase.java)** — Modification call recorded; Write call recorded; score 317 (L=204, B=26, I=7).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-13, patch-update, JSONL line 246; historical package mapped by unique filename.
- [ ] **95. [tuning/localizer/phases/SpinPhase.java](../../TeamCode/src/main/java/tuning/localizer/phases/SpinPhase.java)** — Creation call recorded; Write call recorded; score 321 (L=191, B=30, I=8).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 914; historical package mapped by unique filename.
- [ ] **96. [tuning/follower/feedback/DrivePhase.java](../../TeamCode/src/main/java/tuning/follower/feedback/DrivePhase.java)** — Modification call recorded; Write call recorded; score 332 (L=211, B=27, I=8).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-13, patch-update, JSONL line 246; historical package mapped by unique filename.
- [ ] **97. [tuning/localizer/phases/DirectionsPhase.java](../../TeamCode/src/main/java/tuning/localizer/phases/DirectionsPhase.java)** — Creation call recorded; Write call recorded; score 335 (L=189, B=32, I=10).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 2334; historical package mapped by unique filename.
- [ ] **98. [geometry/PathSegment.java](../../TeamCode/src/main/java/geometry/PathSegment.java)** — Modification call recorded; Codex edit recorded; score 339 (L=222, B=39, I=0).
  Evidence: session `019f2428-2020-7842-902c-743f4bd95173`, 2026-07-02, patch-update, JSONL line 268.
- [ ] **99. [tuning/localizer/phases/FilterPhase.java](../../TeamCode/src/main/java/tuning/localizer/phases/FilterPhase.java)** — Creation call recorded; Write call recorded; score 352 (L=213, B=33, I=8).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 928; historical package mapped by unique filename.
- [ ] **100. [trajectory/profile/FFLut.java](../../TeamCode/src/main/java/trajectory/profile/FFLut.java)** — Modification call recorded; Write call recorded; score 377 (L=191, B=62, I=0).
  Evidence: session `01a06760-70c9-7dd2-8d18-aa3d9074aa29`, 2026-09-03, patch-update, JSONL line 350; historical package mapped by unique filename.
- [ ] **101. [localizers/Pinpoint.java](../../TeamCode/src/main/java/localizers/Pinpoint.java)** — Modification call recorded; Write call recorded; score 399 (L=273, B=32, I=6).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-update, JSONL line 708.
- [ ] **102. [tuning/follower/characterization/MotionCharacterizationPhase.java](../../TeamCode/src/main/java/tuning/follower/characterization/MotionCharacterizationPhase.java)** — Creation call recorded; Codex edit recorded *; score 400 (L=184, B=52, I=12).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-02, write-command, JSONL line 2298; historical package mapped by unique filename.
- [ ] **103. [paths/movements/Path.java](../../TeamCode/src/main/java/paths/movements/Path.java)** — Modification call recorded; Codex edit recorded; score 422 (L=222, B=30, I=22).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-09, write-command, JSONL line 276.
- [ ] **104. [localizers/LocalizationConstants.java](../../TeamCode/src/main/java/localizers/LocalizationConstants.java)** — Creation call recorded; Write call recorded; score 431 (L=235, B=57, I=5).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 701; historical package mapped by unique filename.
- [ ] **105. [org/firstinspires/ftc/teamcode/apexpathing/FollowerTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/FollowerTuner.java)** — Modification call recorded; Codex edit recorded; score 496 (L=247, B=63, I=12).
  Evidence: session `019faf85-ccdc-73b2-b8fc-d9dc4bf603e1`, 2026-07-29, patch-update, JSONL line 108.
- [ ] **106. [tuning/follower/centripetal/CentripetalPhase.java](../../TeamCode/src/main/java/tuning/follower/centripetal/CentripetalPhase.java)** — Modification call recorded; Codex edit recorded; score 523 (L=292, B=52, I=15).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 1853; historical package mapped by unique filename.

### 4. Stateful algorithms and workflows

- [ ] **107. [trajectory/profile/CoupledMotionProfile.java](../../TeamCode/src/main/java/trajectory/profile/CoupledMotionProfile.java)** — Creation call recorded; Codex edit recorded; score 279 (L=126, B=51, I=0).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 824; historical package mapped by unique filename.
  Review focus: backward controllability and forward reachability assumptions (domain prerequisite influences tier).
- [ ] **108. [feedforward/MecanumWheelModel.java](../../TeamCode/src/main/java/feedforward/MecanumWheelModel.java)** — Creation call recorded; Write call recorded; score 311 (L=134, B=59, I=0).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-05, patch-add, JSONL line 1143.
  Review focus: coupled wheel authority and body/wheel coordinate conversion (domain prerequisite influences tier).
- [ ] **109. [tuning/follower/tracking/ProfiledTrackingOptimizer.java](../../TeamCode/src/main/java/tuning/follower/tracking/ProfiledTrackingOptimizer.java)** — Creation call recorded; Codex edit recorded *; score 316 (L=203, B=36, I=1).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-02, write-command, JSONL line 2437; historical package mapped by unique filename.
  Review focus: search objective, trial validity and confirmation ownership (domain prerequisite influences tier).
- [ ] **110. [localizers/util/KalmanFilter.java](../../TeamCode/src/main/java/localizers/util/KalmanFilter.java)** — Creation call recorded; Write call recorded; score 334 (L=187, B=49, I=0).
  Evidence: session `01a101f0-fed2-7fe2-9c8f-70cb84584776`, 2026-10-03, patch-add, JSONL line 733.
  Review focus: matrix prediction, covariance invariants, Joseph correction (domain prerequisite influences tier).
- [ ] **111. [localizers/util/PoseEstimator.java](../../TeamCode/src/main/java/localizers/util/PoseEstimator.java)** — Modification call recorded; Write call recorded; score 396 (L=220, B=52, I=4).
  Evidence: session `01a101f0-fed2-7fe2-9c8f-70cb84584776`, 2026-10-03, patch-update, JSONL line 874.
  Review focus: pose fusion, coordinate frames and covariance propagation (domain prerequisite influences tier).
- [ ] **112. [trajectory/profile/DynamicMotionProfile.java](../../TeamCode/src/main/java/trajectory/profile/DynamicMotionProfile.java)** — Creation call recorded; Write call recorded; score 407 (L=254, B=51, I=0).
  Evidence: session `01a09d6d-7659-77f0-83bb-f2cd823e4eff`, 2026-09-14, patch-add, JSONL line 583; historical package mapped by unique filename.
  Review focus: acceleration/braking envelopes and time integration (domain prerequisite influences tier).
- [ ] **113. [localizers/util/AdaptiveKalmanFilter.java](../../TeamCode/src/main/java/localizers/util/AdaptiveKalmanFilter.java)** — Modification call recorded; Write call recorded; score 426 (L=300, B=42, I=0).
  Evidence: session `019ffe9f-c203-7f92-a5e2-43fa3e7c2de6`, 2026-08-14, patch-update, JSONL line 730.
  Review focus: adaptive noise estimation and robust sensor weighting (domain prerequisite influences tier).
- [ ] **114. [localizers/BaseLocalizer.java](../../TeamCode/src/main/java/localizers/BaseLocalizer.java)** — Modification call recorded; Write call recorded; score 511 (L=299, B=54, I=10).
  Evidence: session `019ffe9f-c203-7f92-a5e2-43fa3e7c2de6`, 2026-08-14, patch-update, JSONL line 730.
  Review focus: sensor/filter lifecycle, timing and calibration ownership (domain prerequisite influences tier).
- [ ] **115. [follower/HolonomicPathFollower.java](../../TeamCode/src/main/java/follower/HolonomicPathFollower.java)** — Creation call recorded; Codex edit recorded; score 551 (L=255, B=77, I=13).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-02, write-command, JSONL line 2288; historical package mapped by unique filename.
- [ ] **116. [paths/builders/HolonomicPathBuilder.java](../../TeamCode/src/main/java/paths/builders/HolonomicPathBuilder.java)** — Modification call recorded; Codex edit recorded; score 564 (L=298, B=67, I=13).
  Evidence: session `019f2428-2020-7842-902c-743f4bd95173`, 2026-07-02, patch-update, JSONL line 734.
- [ ] **117. [tuning/follower/limits/LimitsPhase.java](../../TeamCode/src/main/java/tuning/follower/limits/LimitsPhase.java)** — Modification call recorded; Write call recorded; score 570 (L=356, B=58, I=8).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-14, patch-update, JSONL line 3413; historical package mapped by unique filename.
- [ ] **118. [tuning/follower/feedback/PDSRoutine.java](../../TeamCode/src/main/java/tuning/follower/feedback/PDSRoutine.java)** — Creation call recorded; Write call recorded; score 572 (L=328, B=73, I=5).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-13, patch-add, JSONL line 132; historical package mapped by unique filename.
- [ ] **119. [follower/FollowerConstants.java](../../TeamCode/src/main/java/follower/FollowerConstants.java)** — Creation call recorded; Codex edit recorded; score 608 (L=378, B=70, I=4).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-09, write-command, JSONL line 267; historical package mapped by unique filename.
- [ ] **120. [tuning/follower/acceleration/RampRegression.java](../../TeamCode/src/main/java/tuning/follower/acceleration/RampRegression.java)** — Creation call recorded; Write call recorded; score 615 (L=276, B=113, I=0).
  Evidence: session `01a09390-cef5-7bc3-a70f-b0b175237ec0`, 2026-09-12, patch-add, JSONL line 146; historical package mapped by unique filename.
- [ ] **121. [org/firstinspires/ftc/teamcode/apexpathing/AutoTest.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/AutoTest.java)** — Creation call recorded; Codex edit recorded; score 730 (L=475, B=65, I=12).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 393.
- [ ] **122. [tuning/follower/sysid/FeedforwardTuner.java](../../TeamCode/src/main/java/tuning/follower/sysid/FeedforwardTuner.java)** — Creation call recorded; Codex edit recorded *; score 808 (L=399, B=123, I=8).
  Evidence: session `01a11718-7f12-7033-aa18-d1a236b77ee5`, 2026-10-07, write-command, JSONL line 137.
- [ ] **123. [tuning/follower/acceleration/AccelerationFeedforwardPhase.java](../../TeamCode/src/main/java/tuning/follower/acceleration/AccelerationFeedforwardPhase.java)** — Creation call recorded; Write call recorded *; score 852 (L=503, B=98, I=11).
  Evidence: session `01a093e1-ae3d-7191-95bd-6d48841fa253`, 2026-09-12, patch-add, JSONL line 287; historical package mapped by unique filename.
- [ ] **124. [tuning/follower/tracking/ProfiledTrackingPhase.java](../../TeamCode/src/main/java/tuning/follower/tracking/ProfiledTrackingPhase.java)** — Creation call recorded; Codex edit recorded *; score 864 (L=478, B=107, I=13).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-02, write-command, JSONL line 2437; historical package mapped by unique filename.
- [ ] **125. [tuning/localizer/LocalizerAdapters.java](../../TeamCode/src/main/java/tuning/localizer/LocalizerAdapters.java)** — Creation call recorded; Write call recorded; score 914 (L=515, B=103, I=18).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 789.
- [ ] **126. [follower/Follower.java](../../TeamCode/src/main/java/follower/Follower.java)** — Creation call recorded; Codex edit recorded; score 936 (L=599, B=79, I=20).
  Evidence: session `019faf85-ccdc-73b2-b8fc-d9dc4bf603e1`, 2026-07-29, patch-update, JSONL line 89; historical package mapped by unique filename.
- [ ] **127. [localizers/Octoquad.java](../../TeamCode/src/main/java/localizers/Octoquad.java)** — Modification call recorded; Write call recorded; score 990 (L=801, B=53, I=6).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-update, JSONL line 708.

### 5. Largest review effort

- [ ] **128. [trajectory/generators/BaseProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/BaseProfileGenerator.java)** — Creation call recorded; Codex edit recorded; score 1272 (L=602, B=205, I=11).
  Evidence: session `019f2428-2020-7842-902c-743f4bd95173`, 2026-07-02, patch-update, JSONL line 125; historical package mapped by unique filename.

## Coverage queue: provenance not established

Review these too if you want full ownership of the library. Recent changes deserve attention, but are not classified as AI-generated. Sorted with the same simple-to-complex metric.

### 1. Small types and helpers

- [ ] **1. [geometry/AngleUnit.java](../../TeamCode/src/main/java/geometry/AngleUnit.java)** — Not established; Unknown; score 9 (L=9, B=0, I=0).
  Evidence: no captured provenance.
- [ ] **2. [geometry/DistUnit.java](../../TeamCode/src/main/java/geometry/DistUnit.java)** — Not established; Unknown; score 12 (L=12, B=0, I=0).
  Evidence: no captured provenance.
- [ ] **3. [localizers/util/FilterState.java](../../TeamCode/src/main/java/localizers/util/FilterState.java)** — Not established; Recent Git change only; score 15 (L=15, B=0, I=0).
  Evidence: Git `4defc9f 2026-08-07 - feedforward tuner - Self-tuning kalman filter - Low pass filter in case kalman filter doesn't work - Updated matrix class to support kalman filter`.
- [ ] **4. [paths/heading/InterpolationStyle.java](../../TeamCode/src/main/java/paths/heading/InterpolationStyle.java)** — Not established; Unknown; score 15 (L=15, B=0, I=0).
  Evidence: no captured provenance.
- [ ] **5. [paths/heading/HeadingNode.java](../../TeamCode/src/main/java/paths/heading/HeadingNode.java)** — Not established; Unknown; score 18 (L=13, B=0, I=1).
  Evidence: no captured provenance.
- [ ] **6. [geometry/PathPoint.java](../../TeamCode/src/main/java/geometry/PathPoint.java)** — Not established; Unknown; score 24 (L=24, B=0, I=0).
  Evidence: no captured provenance.
- [ ] **7. [drivetrains/Motor.java](../../TeamCode/src/main/java/drivetrains/Motor.java)** — Not established; Unknown; score 43 (L=37, B=2, I=0).
  Evidence: no captured provenance.
- [ ] **8. [drivetrains/Tank.java](../../TeamCode/src/main/java/drivetrains/Tank.java)** — Not established; Unknown; score 43 (L=37, B=2, I=0).
  Evidence: no captured provenance.
- [ ] **9. [drivetrains/Kiwi.java](../../TeamCode/src/main/java/drivetrains/Kiwi.java)** — Not established; Unknown; score 48 (L=39, B=3, I=0).
  Evidence: no captured provenance.
- [ ] **10. [tuning/follower/centripetal/BinarySearch.java](../../TeamCode/src/main/java/tuning/follower/centripetal/BinarySearch.java)** — Not established; Recent Git change only; score 48 (L=30, B=6, I=0).
  Evidence: Git `c54c4c0 2026-10-05 Integrate coupled mecanum feedforward, unified localization, and review repairs`.
- [ ] **11. [geometry/Angle.java](../../TeamCode/src/main/java/geometry/Angle.java)** — Not established; Recent Git change only; score 56 (L=47, B=3, I=0).
  Evidence: Git `c54c4c0 2026-10-05 Integrate coupled mecanum feedforward, unified localization, and review repairs`.
- [ ] **12. [org/firstinspires/ftc/teamcode/apexpathing/Constants.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/Constants.java)** — Not established; Recent Git change only; score 65 (L=29, B=2, I=6).
  Evidence: Git `5232d34 2026-10-03 refactor: organize pathing classes by responsibility`.
- [ ] **13. [geometry/Pose.java](../../TeamCode/src/main/java/geometry/Pose.java)** — Not established; Recent Git change only; score 71 (L=62, B=3, I=0).
  Evidence: Git `c54c4c0 2026-10-05 Integrate coupled mecanum feedforward, unified localization, and review repairs`.
- [ ] **14. [paths/Callback.java](../../TeamCode/src/main/java/paths/Callback.java)** — Not established; Recent Git change only; score 71 (L=45, B=7, I=1).
  Evidence: Git `c54c4c0 2026-10-05 Integrate coupled mecanum feedforward, unified localization, and review repairs`.

### 2. Focused logic

- [ ] **15. [geometry/Matrix.java](../../TeamCode/src/main/java/geometry/Matrix.java)** — Not established; Recent Git change only; score 236 (L=128, B=36, I=0).
  Evidence: Git `c54c4c0 2026-10-05 Integrate coupled mecanum feedforward, unified localization, and review repairs`.

## Supporting tests and simulator code

Review the tests alongside the production class they exercise. This separate queue prevents small test classes from crowding out production code; simulation infrastructure can itself be complex.

### 1. Small types and helpers

- [ ] **1. [org/firstinspires/ftc/teamcode/sim/CenteredBrakingAutoTestSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/CenteredBrakingAutoTestSimulationTest.java)** — Creation call recorded; Write call recorded; score 7 (L=7, B=0, I=0).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-04, patch-add, JSONL line 660.
- [ ] **2. [tuning/follower/feedback/HeadingPhaseTest.java](../../TeamCode/src/test/java/tuning/follower/feedback/HeadingPhaseTest.java)** — Creation call recorded; Write call recorded; score 10 (L=10, B=0, I=0).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-14, patch-add, JSONL line 2840; historical package mapped by unique filename.
- [ ] **3. [tuning/localizer/phases/SpinPhaseTest.java](../../TeamCode/src/test/java/tuning/localizer/phases/SpinPhaseTest.java)** — Creation call recorded; Write call recorded; score 12 (L=12, B=0, I=0).
  Evidence: session `01a0912a-07e0-7b32-b299-a7bbf2ce6d39`, 2026-09-11, patch-add, JSONL line 116; historical package mapped by unique filename.
- [ ] **4. [org/firstinspires/ftc/teamcode/sim/SimulateApexPathing.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimulateApexPathing.java)** — Creation call recorded; Write call recorded; score 13 (L=13, B=0, I=0).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-14, patch-update, JSONL line 586.
- [ ] **5. [org/firstinspires/ftc/teamcode/sim/TankSimulationEnvironment.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/TankSimulationEnvironment.java)** — Creation call recorded; Write call recorded; score 15 (L=12, B=1, I=0).
  Evidence: session `01a0922c-312f-7d10-b6e9-7ad68173803e`, 2026-09-11, patch-add, JSONL line 670.
- [ ] **6. [drivetrains/PowerLimitRegressionTest.java](../../TeamCode/src/test/java/drivetrains/PowerLimitRegressionTest.java)** — Creation call recorded; Write call recorded; score 17 (L=17, B=0, I=0).
  Evidence: session `01a10ce3-3642-7063-b347-f2f9753b9684`, 2026-10-05, patch-add, JSONL line 969.
- [ ] **7. [org/firstinspires/ftc/teamcode/apexpathing/AutoTestSequenceTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/apexpathing/AutoTestSequenceTest.java)** — Modification call recorded; Write call recorded; score 17 (L=17, B=0, I=0).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-14, patch-update, JSONL line 1577.
- [ ] **8. [org/firstinspires/ftc/teamcode/sim/TankFollowerTuner.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/TankFollowerTuner.java)** — Creation call recorded; Write call recorded; score 17 (L=14, B=1, I=0).
  Evidence: session `01a0922c-312f-7d10-b6e9-7ad68173803e`, 2026-09-11, patch-add, JSONL line 171.
- [ ] **9. [org/firstinspires/ftc/teamcode/sim/TankLocalizationTuner.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/TankLocalizationTuner.java)** — Creation call recorded; Write call recorded; score 17 (L=14, B=1, I=0).
  Evidence: session `01a0922c-312f-7d10-b6e9-7ad68173803e`, 2026-09-11, patch-add, JSONL line 171.
- [ ] **10. [tuning/follower/sysid/FeedforwardTunerTest.java](../../TeamCode/src/test/java/tuning/follower/sysid/FeedforwardTunerTest.java)** — Creation call recorded; Write call recorded; score 17 (L=17, B=0, I=0).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-14, patch-update, JSONL line 2840; historical package mapped by unique filename.
- [ ] **11. [org/firstinspires/ftc/teamcode/sim/TankAutoTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/TankAutoTest.java)** — Creation call recorded; Write call recorded; score 18 (L=15, B=1, I=0).
  Evidence: session `01a0922c-312f-7d10-b6e9-7ad68173803e`, 2026-09-11, patch-add, JSONL line 171.
- [ ] **12. [trajectory/profile/ReviewRegressionTest.java](../../TeamCode/src/test/java/trajectory/profile/ReviewRegressionTest.java)** — Creation call recorded; Write call recorded; score 18 (L=18, B=0, I=0).
  Evidence: session `01a10ce3-3642-7063-b347-f2f9753b9684`, 2026-10-05, patch-add, JSONL line 882.
- [ ] **13. [org/firstinspires/ftc/teamcode/sim/StraightTwistFeedforwardSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/StraightTwistFeedforwardSimulationTest.java)** — Creation call recorded; Write call recorded; score 19 (L=19, B=0, I=0).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-05, patch-add, JSONL line 789.
- [ ] **14. [org/firstinspires/ftc/teamcode/sim/SimulateTankApexPathing.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimulateTankApexPathing.java)** — Creation call recorded; Write call recorded; score 21 (L=16, B=0, I=1).
  Evidence: session `01a0922c-312f-7d10-b6e9-7ad68173803e`, 2026-09-11, patch-add, JSONL line 171.
- [ ] **15. [org/firstinspires/ftc/teamcode/sim/StockSimTestSupport.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/StockSimTestSupport.java)** — Creation call recorded; Write call recorded; score 23 (L=20, B=1, I=0).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-02, patch-add, JSONL line 1484.
- [ ] **16. [geometry/ReviewRegressionTest.java](../../TeamCode/src/test/java/geometry/ReviewRegressionTest.java)** — Creation call recorded; Write call recorded; score 28 (L=28, B=0, I=0).
  Evidence: session `01a10ce3-3642-7063-b347-f2f9753b9684`, 2026-10-05, patch-add, JSONL line 882.
- [ ] **17. [trajectory/profile/FFLutDistanceTest.java](../../TeamCode/src/test/java/trajectory/profile/FFLutDistanceTest.java)** — Modification call recorded; Write call recorded; score 31 (L=31, B=0, I=0).
  Evidence: session `01a0967a-ea7a-7191-9d97-cd2a114a58df`, 2026-09-12, patch-update, JSONL line 529; historical package mapped by unique filename.
- [ ] **18. [tuning/follower/feedback/ManualResponseMetricsTest.java](../../TeamCode/src/test/java/tuning/follower/feedback/ManualResponseMetricsTest.java)** — Creation call recorded; Write call recorded *; score 31 (L=28, B=1, I=0).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-14, patch-add, JSONL line 2840; historical package mapped by unique filename.
- [ ] **19. [controllers/PDSControllerTest.java](../../TeamCode/src/test/java/controllers/PDSControllerTest.java)** — Creation call recorded; Write call recorded; score 33 (L=28, B=0, I=1).
  Evidence: session `01a05fb2-639d-7312-95ef-44b7ab13aca0`, 2026-09-02, patch-add, JSONL line 1595.
- [ ] **20. [follower/CompletionRegressionTest.java](../../TeamCode/src/test/java/follower/CompletionRegressionTest.java)** — Creation call recorded; Write call recorded; score 33 (L=28, B=0, I=1).
  Evidence: session `01a10ce3-3642-7063-b347-f2f9753b9684`, 2026-10-05, patch-add, JSONL line 882.
- [ ] **21. [geometry/ScalarTest.java](../../TeamCode/src/test/java/geometry/ScalarTest.java)** — Creation call recorded; Write call recorded; score 35 (L=35, B=0, I=0).
  Evidence: session `01a0a04a-101c-7fe2-800d-1a6cea465755`, 2026-09-14, patch-add, JSONL line 52.
- [ ] **22. [follower/ProfiledMotionFeedforwardTest.java](../../TeamCode/src/test/java/follower/ProfiledMotionFeedforwardTest.java)** — Creation call recorded; Write call recorded; score 36 (L=27, B=3, I=0).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-01, patch-add, JSONL line 1120; historical package mapped by unique filename.
- [ ] **23. [org/codeblooded/ftcodesim/simulator/FTCodeSimTelemetryInstaller.java](../../TeamCode/src/test/java/org/codeblooded/ftcodesim/simulator/FTCodeSimTelemetryInstaller.java)** — Not established; Recent Git change only; score 37 (L=25, B=4, I=0).
  Evidence: Git `ce3bb9c 2026-08-13 Remove tests and simulator support from main`.
- [ ] **24. [follower/FollowerConstantsVelocityFeedbackTest.java](../../TeamCode/src/test/java/follower/FollowerConstantsVelocityFeedbackTest.java)** — Creation call recorded; Write call recorded; score 38 (L=38, B=0, I=0).
  Evidence: session `01a0b5dc-c69e-7bc1-832a-ab3f654000a3`, 2026-09-18, patch-add, JSONL line 145; historical package mapped by unique filename.
- [ ] **25. [tuning/follower/acceleration/AccelerationValidationTest.java](../../TeamCode/src/test/java/tuning/follower/acceleration/AccelerationValidationTest.java)** — Not established; Unknown *; score 39 (L=21, B=6, I=0).
  Evidence: no captured provenance.
- [ ] **26. [org/firstinspires/ftc/teamcode/sim/SimulatorSysIdMotorOverrideTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimulatorSysIdMotorOverrideTest.java)** — Creation call recorded; Write call recorded; score 41 (L=26, B=5, I=0).
  Evidence: session `01a10db1-3700-7c11-9ea1-92a260fb773a`, 2026-10-05, patch-add, JSONL line 2021.
- [ ] **27. [trajectory/generators/TankTractionLimitTest.java](../../TeamCode/src/test/java/trajectory/generators/TankTractionLimitTest.java)** — Creation call recorded; Write call recorded; score 41 (L=31, B=0, I=2).
  Evidence: session `01a0967a-ea7a-7191-9d97-cd2a114a58df`, 2026-09-12, patch-add, JSONL line 674; historical package mapped by unique filename.
- [ ] **28. [tuning/follower/acceleration/AccelerationSearchTest.java](../../TeamCode/src/test/java/tuning/follower/acceleration/AccelerationSearchTest.java)** — Creation call recorded; Write call recorded *; score 41 (L=41, B=0, I=0).
  Evidence: session `01a093e1-ae3d-7191-95bd-6d48841fa253`, 2026-09-12, patch-add, JSONL line 278; historical package mapped by unique filename.
- [ ] **29. [tuning/localizer/RelativeImuAccumulatorTest.java](../../TeamCode/src/test/java/tuning/localizer/RelativeImuAccumulatorTest.java)** — Creation call recorded; Write call recorded; score 41 (L=30, B=2, I=1).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 1041.
- [ ] **30. [trajectory/VelocityPursuitTest.java](../../TeamCode/src/test/java/trajectory/VelocityPursuitTest.java)** — Creation call recorded; Write call recorded; score 45 (L=39, B=2, I=0).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-05, patch-add, JSONL line 2916.
- [ ] **31. [storage/ApexStorageTest.java](../../TeamCode/src/test/java/storage/ApexStorageTest.java)** — Creation call recorded; Write call recorded; score 46 (L=40, B=2, I=0).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-09, patch-add, JSONL line 397; historical package mapped by unique filename.
- [ ] **32. [org/firstinspires/ftc/teamcode/sim/SimulatorSysIdMotorOverride.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimulatorSysIdMotorOverride.java)** — Creation call recorded; Write call recorded; score 47 (L=29, B=6, I=0).
  Evidence: session `01a10db1-3700-7c11-9ea1-92a260fb773a`, 2026-10-05, patch-add, JSONL line 2013.
- [ ] **33. [paths/heading/HolonomicInterpolatorTest.java](../../TeamCode/src/test/java/paths/heading/HolonomicInterpolatorTest.java)** — Not established; Recent Git change only; score 47 (L=32, B=0, I=3).
  Evidence: Git `ce3bb9c 2026-08-13 Remove tests and simulator support from main`.
- [ ] **34. [follower/VelocityReferenceTest.java](../../TeamCode/src/test/java/follower/VelocityReferenceTest.java)** — Creation call recorded; Write call recorded; score 48 (L=45, B=1, I=0).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-05, patch-add, JSONL line 2197.
- [ ] **35. [org/firstinspires/ftc/teamcode/sim/FeedforwardModelDiagnosticTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/FeedforwardModelDiagnosticTest.java)** — Creation call recorded; Write call recorded; score 49 (L=43, B=2, I=0).
  Evidence: session `01a06760-70c9-7dd2-8d18-aa3d9074aa29`, 2026-09-08, patch-add, JSONL line 1421.
- [ ] **36. [org/firstinspires/ftc/teamcode/sim/StockSimulatorPlantTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/StockSimulatorPlantTest.java)** — Creation call recorded; Write call recorded; score 50 (L=41, B=3, I=0).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-02, patch-add, JSONL line 1492.
- [ ] **37. [org/firstinspires/ftc/teamcode/sim/ApexSimTelemetryTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/ApexSimTelemetryTest.java)** — Not established; Recent Git change only; score 52 (L=52, B=0, I=0).
  Evidence: Git `ce3bb9c 2026-08-13 Remove tests and simulator support from main`.
- [ ] **38. [tuning/follower/limits/LimitsPhaseTest.java](../../TeamCode/src/test/java/tuning/follower/limits/LimitsPhaseTest.java)** — Modification call recorded; Write call recorded; score 52 (L=41, B=2, I=1).
  Evidence: session `01a09485-143c-7121-bb56-4f884db94c91`, 2026-09-12, patch-update, JSONL line 64; historical package mapped by unique filename.
- [ ] **39. [trajectory/profile/FFLutTimeTest.java](../../TeamCode/src/test/java/trajectory/profile/FFLutTimeTest.java)** — Creation call recorded; Write call recorded; score 54 (L=54, B=0, I=0).
  Evidence: session `01a06760-70c9-7dd2-8d18-aa3d9074aa29`, 2026-09-03, patch-add, JSONL line 386; historical package mapped by unique filename.
- [ ] **40. [controllers/TurnControllerTest.java](../../TeamCode/src/test/java/controllers/TurnControllerTest.java)** — Creation call recorded; Write call recorded; score 55 (L=50, B=0, I=1).
  Evidence: session `019f4cc6-d8be-7e11-9ccf-1a61a18d009a`, 2026-07-10, patch-add, JSONL line 502; historical package mapped by unique filename.
- [ ] **41. [org/firstinspires/ftc/teamcode/sim/StableSimHardwareMapTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/StableSimHardwareMapTest.java)** — Modification call recorded; Write call recorded; score 60 (L=54, B=2, I=0).
  Evidence: session `01a0e5c4-64c7-7f60-aead-04c6224ce1e3`, 2026-09-28, patch-update, JSONL line 587.
- [ ] **42. [org/firstinspires/ftc/teamcode/sim/TankSimulationConstants.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/TankSimulationConstants.java)** — Creation call recorded; Write call recorded; score 62 (L=26, B=2, I=6).
  Evidence: session `01a0922c-312f-7d10-b6e9-7ad68173803e`, 2026-09-11, patch-add, JSONL line 171.
- [ ] **43. [follower/IndependentPathReferenceTest.java](../../TeamCode/src/test/java/follower/IndependentPathReferenceTest.java)** — Creation call recorded; Write call recorded; score 63 (L=38, B=0, I=5).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-05, patch-add, JSONL line 1459.
- [ ] **44. [localizers/util/KalmanFilterTest.java](../../TeamCode/src/test/java/localizers/util/KalmanFilterTest.java)** — Creation call recorded; Write call recorded; score 65 (L=65, B=0, I=0).
  Evidence: session `01a101f0-fed2-7fe2-9c8f-70cb84584776`, 2026-10-03, patch-add, JSONL line 760.
- [ ] **45. [org/firstinspires/ftc/teamcode/sim/SimulatorTuning.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimulatorTuning.java)** — Creation call recorded; Write call recorded; score 65 (L=51, B=3, I=1).
  Evidence: session `01a0cbc7-4a94-74e0-9dad-57f92f82a3fc`, 2026-09-23, patch-add, JSONL line 722.
- [ ] **46. [tuning/follower/sysid/ManualFeedforwardProfileTest.java](../../TeamCode/src/test/java/tuning/follower/sysid/ManualFeedforwardProfileTest.java)** — Creation call recorded; Write call recorded; score 66 (L=60, B=2, I=0).
  Evidence: session `01a10db1-3700-7c11-9ea1-92a260fb773a`, 2026-10-05, patch-add, JSONL line 104.
- [ ] **47. [org/firstinspires/ftc/teamcode/sim/MecanumFrictionModelTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/MecanumFrictionModelTest.java)** — Creation call recorded; Write call recorded; score 69 (L=60, B=3, I=0).
  Evidence: session `01a0b524-5a00-73e1-9983-94a0ebd9eff7`, 2026-09-18, patch-add, JSONL line 292.
- [ ] **48. [tuning/follower/sysid/SysIdValidationTest.java](../../TeamCode/src/test/java/tuning/follower/sysid/SysIdValidationTest.java)** — Modification call recorded; Codex edit recorded *; score 69 (L=42, B=9, I=0).
  Evidence: session `01a11718-7f12-7033-aa18-d1a236b77ee5`, 2026-10-07, write-command, JSONL line 137.
- [ ] **49. [trajectory/profile/CoupledMotionProfileTest.java](../../TeamCode/src/test/java/trajectory/profile/CoupledMotionProfileTest.java)** — Creation call recorded; Write call recorded; score 70 (L=40, B=10, I=0).
  Evidence: session `01a0fde7-ad4c-7f92-b52b-41d65f9bf889`, 2026-10-02, patch-add, JSONL line 409; historical package mapped by unique filename.
- [ ] **50. [tuning/TuningCsvWriterTest.java](../../TeamCode/src/test/java/tuning/TuningCsvWriterTest.java)** — Modification call recorded; Write call recorded; score 74 (L=63, B=2, I=1).
  Evidence: session `01a0a0fc-fd18-7e91-8097-713445349d79`, 2026-09-14, patch-update, JSONL line 1717.
- [ ] **51. [trajectory/profile/DynamicMotionProfileTest.java](../../TeamCode/src/test/java/trajectory/profile/DynamicMotionProfileTest.java)** — Creation call recorded; Write call recorded; score 76 (L=61, B=5, I=0).
  Evidence: session `01a09d6d-7659-77f0-83bb-f2cd823e4eff`, 2026-09-14, patch-add, JSONL line 611; historical package mapped by unique filename.
- [ ] **52. [geometry/CubicSplineLookupTest.java](../../TeamCode/src/test/java/geometry/CubicSplineLookupTest.java)** — Creation call recorded; Write call recorded; score 80 (L=47, B=11, I=0).
  Evidence: session `01a0ff49-4135-7d91-be01-a690dad9fdd5`, 2026-10-03, patch-add, JSONL line 481.
- [ ] **53. [org/firstinspires/ftc/teamcode/apexpathing/FollowerTunerSequenceTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/apexpathing/FollowerTunerSequenceTest.java)** — Modification call recorded; Codex edit recorded; score 80 (L=71, B=3, I=0).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-02, write-command, JSONL line 2298.
- [ ] **54. [tuning/follower/tracking/ProfiledTrackingOptimizerTest.java](../../TeamCode/src/test/java/tuning/follower/tracking/ProfiledTrackingOptimizerTest.java)** — Creation call recorded; Write call recorded *; score 84 (L=61, B=6, I=1).
  Evidence: session `01a0e42c-e6bc-7340-ae3b-2aa4be2362a7`, 2026-09-27, patch-add, JSONL line 965; historical package mapped by unique filename.
- [ ] **55. [localizers/PinpointPacketTest.java](../../TeamCode/src/test/java/localizers/PinpointPacketTest.java)** — Creation call recorded; Write call recorded; score 86 (L=55, B=7, I=2).
  Evidence: session `01a10ce3-3642-7063-b347-f2f9753b9684`, 2026-10-05, patch-add, JSONL line 1103.
- [ ] **56. [localizers/util/LocalizationNoise.java](../../TeamCode/src/test/java/localizers/util/LocalizationNoise.java)** — Creation call recorded; Write call recorded; score 87 (L=40, B=9, I=4).
  Evidence: session `01a101f0-fed2-7fe2-9c8f-70cb84584776`, 2026-10-03, patch-add, JSONL line 760.
- [ ] **57. [org/firstinspires/ftc/teamcode/sim/ProfiledTrackingValidationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/ProfiledTrackingValidationTest.java)** — Modification call recorded; Codex edit recorded *; score 92 (L=53, B=13, I=0).
  Evidence: session `01a11718-7f12-7033-aa18-d1a236b77ee5`, 2026-10-07, write-command, JSONL line 151.
- [ ] **58. [org/firstinspires/ftc/teamcode/sim/StableSimHardwareMap.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/StableSimHardwareMap.java)** — Modification call recorded; Write call recorded; score 93 (L=66, B=9, I=0).
  Evidence: session `01a0e5c4-64c7-7f60-aead-04c6224ce1e3`, 2026-09-28, patch-update, JSONL line 582.
- [ ] **59. [org/firstinspires/ftc/teamcode/sim/SimApexImu.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimApexImu.java)** — Creation call recorded; Write call recorded; score 94 (L=43, B=17, I=0).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-11, patch-add, JSONL line 2709.

### 2. Focused logic

- [ ] **60. [org/codeblooded/ftcodesim/simulator/ApexAdvantageScopeLoggerTest.java](../../TeamCode/src/test/java/org/codeblooded/ftcodesim/simulator/ApexAdvantageScopeLoggerTest.java)** — Not established; Recent Git change only; score 100 (L=65, B=0, I=7).
  Evidence: Git `ce3bb9c 2026-08-13 Remove tests and simulator support from main`.
- [ ] **61. [localizers/LocalizerCalibrationValuesTest.java](../../TeamCode/src/test/java/localizers/LocalizerCalibrationValuesTest.java)** — Creation call recorded; Write call recorded; score 101 (L=58, B=1, I=8).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 1092.
- [ ] **62. [org/firstinspires/ftc/teamcode/sim/ReleaseDrivetrainPhysicsTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/ReleaseDrivetrainPhysicsTest.java)** — Creation call recorded; Write call recorded; score 101 (L=42, B=18, I=1).
  Evidence: session `01a0cbc7-4a94-74e0-9dad-57f92f82a3fc`, 2026-09-23, patch-add, JSONL line 799.
- [ ] **63. [follower/HolonomicCommandAllocatorTest.java](../../TeamCode/src/test/java/follower/HolonomicCommandAllocatorTest.java)** — Creation call recorded; Write call recorded; score 105 (L=80, B=0, I=5).
  Evidence: session `01a0cbc7-4a94-74e0-9dad-57f92f82a3fc`, 2026-09-23, patch-add, JSONL line 72; historical package mapped by unique filename.
- [ ] **64. [org/firstinspires/ftc/teamcode/sim/FollowerReviewRegressionTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/FollowerReviewRegressionTest.java)** — Creation call recorded; Write call recorded; score 108 (L=71, B=4, I=5).
  Evidence: session `01a10ce3-3642-7063-b347-f2f9753b9684`, 2026-10-05, patch-add, JSONL line 1217.
- [ ] **65. [follower/FollowerProfilesTest.java](../../TeamCode/src/test/java/follower/FollowerProfilesTest.java)** — Creation call recorded; Write call recorded; score 114 (L=104, B=0, I=2).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-09, patch-add, JSONL line 352; historical package mapped by unique filename.
- [ ] **66. [org/firstinspires/ftc/teamcode/sim/ApexSimTelemetry.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/ApexSimTelemetry.java)** — Not established; Recent Git change only; score 115 (L=97, B=6, I=0).
  Evidence: Git `ce3bb9c 2026-08-13 Remove tests and simulator support from main`.
- [ ] **67. [org/firstinspires/ftc/teamcode/sim/AutoTestPowerGraphs.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/AutoTestPowerGraphs.java)** — Creation call recorded; Write call recorded; score 117 (L=78, B=13, I=0).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-04, patch-add, JSONL line 75.
- [ ] **68. [tuning/follower/centripetal/CentripetalPhaseTest.java](../../TeamCode/src/test/java/tuning/follower/centripetal/CentripetalPhaseTest.java)** — Creation call recorded; Write call recorded; score 118 (L=77, B=7, I=4).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-14, patch-add, JSONL line 555; historical package mapped by unique filename.
- [ ] **69. [localizers/TankMecanumDriveEncodersTest.java](../../TeamCode/src/test/java/localizers/TankMecanumDriveEncodersTest.java)** — Modification call recorded; Write call recorded; score 119 (L=96, B=1, I=4).
  Evidence: session `01a101f0-fed2-7fe2-9c8f-70cb84584776`, 2026-10-03, patch-update, JSONL line 433.
- [ ] **70. [tuning/follower/characterization/MotorAuthorityRegressionTest.java](../../TeamCode/src/test/java/tuning/follower/characterization/MotorAuthorityRegressionTest.java)** — Creation call recorded; Codex edit recorded *; score 123 (L=82, B=12, I=1).
  Evidence: session `01a116fe-1282-7610-906a-7aac27ed0ad6`, 2026-10-07, write-command, JSONL line 161.
- [ ] **71. [tuning/follower/feedback/PDSRoutineTest.java](../../TeamCode/src/test/java/tuning/follower/feedback/PDSRoutineTest.java)** — Creation call recorded; Codex edit recorded *; score 128 (L=119, B=3, I=0).
  Evidence: session `01a11718-7f12-7033-aa18-d1a236b77ee5`, 2026-10-07, write-command, JSONL line 219.
- [ ] **72. [org/codeblooded/ftcodesim/simulator/ApexAdvantageScopeLogger.java](../../TeamCode/src/test/java/org/codeblooded/ftcodesim/simulator/ApexAdvantageScopeLogger.java)** — Not established; Recent Git change only; score 130 (L=78, B=4, I=8).
  Evidence: Git `5232d34 2026-10-03 refactor: organize pathing classes by responsibility`.
- [ ] **73. [org/firstinspires/ftc/teamcode/sim/FeedforwardCalibrationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/FeedforwardCalibrationTest.java)** — Creation call recorded; Write call recorded; score 131 (L=83, B=16, I=0).
  Evidence: session `01a06760-70c9-7dd2-8d18-aa3d9074aa29`, 2026-09-08, patch-add, JSONL line 1520.
- [ ] **74. [ExampleAutoProfileRegressionTest.java](../../TeamCode/src/test/java/ExampleAutoProfileRegressionTest.java)** — Creation call recorded; Write call recorded; score 133 (L=66, B=4, I=11).
  Evidence: session `019ffe9f-c203-7f92-a5e2-43fa3e7c2de6`, 2026-08-14, patch-add, JSONL line 364.
- [ ] **75. [org/firstinspires/ftc/teamcode/sim/CornerTrackingSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/CornerTrackingSimulationTest.java)** — Creation call recorded; Codex edit recorded; score 136 (L=93, B=11, I=2).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 956.
- [ ] **76. [feedforward/MecanumWheelModelTest.java](../../TeamCode/src/test/java/feedforward/MecanumWheelModelTest.java)** — Creation call recorded; Write call recorded; score 140 (L=83, B=19, I=0).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-05, patch-add, JSONL line 1302.
- [ ] **77. [geometry/PathSegmentTest.java](../../TeamCode/src/test/java/geometry/PathSegmentTest.java)** — Modification call recorded; Write call recorded; score 140 (L=102, B=11, I=1).
  Evidence: session `01a083fd-1d13-7ff0-b4a9-8428a24960ae`, 2026-09-09, patch-update, JSONL line 152.
- [ ] **78. [controllers/FeedbackControllerTest.java](../../TeamCode/src/test/java/controllers/FeedbackControllerTest.java)** — Creation call recorded; Write call recorded; score 145 (L=140, B=0, I=1).
  Evidence: session `01a0a04a-101c-7fe2-800d-1a6cea465755`, 2026-09-14, patch-add, JSONL line 191.
- [ ] **79. [org/codeblooded/ftcodesim/simulator/FTCodeSimLinearOpModeRunner.java](../../TeamCode/src/test/java/org/codeblooded/ftcodesim/simulator/FTCodeSimLinearOpModeRunner.java)** — Modification call recorded; Write call recorded; score 146 (L=99, B=14, I=1).
  Evidence: session `01a0cede-4c8f-7ac0-b087-28f0474ebdcb`, 2026-09-23, patch-update, JSONL line 384.
- [ ] **80. [paths/movements/MovementReversalTest.java](../../TeamCode/src/test/java/paths/movements/MovementReversalTest.java)** — Creation call recorded; Write call recorded; score 148 (L=97, B=2, I=9).
  Evidence: session `01a0843c-610a-7d30-90b3-92a7ba8a36fc`, 2026-09-09, patch-add, JSONL line 333.
- [ ] **81. [tuning/follower/acceleration/RampRegressionTest.java](../../TeamCode/src/test/java/tuning/follower/acceleration/RampRegressionTest.java)** — Modification call recorded; Write call recorded; score 152 (L=110, B=14, I=0).
  Evidence: session `01a101d8-787a-76d3-b28e-f89bf5542219`, 2026-10-03, write-command, JSONL line 182.
- [ ] **82. [org/firstinspires/ftc/teamcode/sim/TankTuningWorkflowSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/TankTuningWorkflowSimulationTest.java)** — Modification call recorded; Write call recorded; score 153 (L=92, B=12, I=5).
  Evidence: session `01a0962a-cbf8-7f80-98c4-6147fbce62bc`, 2026-09-12, patch-update, JSONL line 686.
- [ ] **83. [localizers/util/PoseEstimatorTuningTest.java](../../TeamCode/src/test/java/localizers/util/PoseEstimatorTuningTest.java)** — Creation call recorded; Write call recorded; score 154 (L=107, B=9, I=4).
  Evidence: session `01a101f0-fed2-7fe2-9c8f-70cb84584776`, 2026-10-03, patch-add, JSONL line 787.
- [ ] **84. [tuning/follower/characterization/ValidatedMotorAuthorityRegression.java](../../TeamCode/src/test/java/tuning/follower/characterization/ValidatedMotorAuthorityRegression.java)** — Modification call recorded; Codex edit recorded *; score 154 (L=71, B=26, I=1).
  Evidence: session `01a116fe-1282-7610-906a-7aac27ed0ad6`, 2026-10-07, write-command, JSONL line 140.
- [ ] **85. [org/firstinspires/ftc/teamcode/sim/DualActuatedProfilesTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/DualActuatedProfilesTest.java)** — Creation call recorded; Write call recorded; score 158 (L=99, B=3, I=10).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-09, patch-add, JSONL line 361.
- [ ] **86. [trajectory/generators/AnalyticPowerBoundsTest.java](../../TeamCode/src/test/java/trajectory/generators/AnalyticPowerBoundsTest.java)** — Creation call recorded; Write call recorded; score 158 (L=63, B=15, I=10).
  Evidence: session `01a0ff49-4135-7d91-be01-a690dad9fdd5`, 2026-10-03, patch-add, JSONL line 481; historical package mapped by unique filename.
- [ ] **87. [com/qualcomm/robotcore/eventloop/opmode/SimLinearOpModeBridge.java](../../TeamCode/src/test/java/com/qualcomm/robotcore/eventloop/opmode/SimLinearOpModeBridge.java)** — Not established; Recent Git change only; score 159 (L=114, B=15, I=0).
  Evidence: Git `ce3bb9c 2026-08-13 Remove tests and simulator support from main`.
- [ ] **88. [org/firstinspires/ftc/teamcode/sim/CentripetalVectorSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/CentripetalVectorSimulationTest.java)** — Modification call recorded; Codex edit recorded; score 163 (L=99, B=8, I=8).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 1770.
- [ ] **89. [org/firstinspires/ftc/teamcode/sim/TunedAutoTestSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/TunedAutoTestSimulationTest.java)** — Creation call recorded; Write call recorded; score 166 (L=107, B=18, I=1).
  Evidence: session `01a081ea-fc91-7aa0-a44e-ecd4c621967f`, 2026-09-08, patch-add, JSONL line 340.
- [ ] **90. [org/firstinspires/ftc/teamcode/sim/TankProfiledTrackingPhaseSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/TankProfiledTrackingPhaseSimulationTest.java)** — Creation call recorded; Codex edit recorded *; score 168 (L=98, B=15, I=5).
  Evidence: session `01a11718-7f12-7033-aa18-d1a236b77ee5`, 2026-10-07, write-command, JSONL line 151.
- [ ] **91. [localizers/LocalizerSetTest.java](../../TeamCode/src/test/java/localizers/LocalizerSetTest.java)** — Creation call recorded; Write call recorded; score 170 (L=95, B=10, I=9).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 1092; historical package mapped by unique filename.
- [ ] **92. [org/codeblooded/ftcodesim/ascope/ApexAdvantageScopeLayoutTest.java](../../TeamCode/src/test/java/org/codeblooded/ftcodesim/ascope/ApexAdvantageScopeLayoutTest.java)** — Modification call recorded; Write call recorded; score 175 (L=166, B=3, I=0).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-14, patch-update, JSONL line 472.
- [ ] **93. [localizers/util/AdaptiveKalmanFilterTest.java](../../TeamCode/src/test/java/localizers/util/AdaptiveKalmanFilterTest.java)** — Creation call recorded; Write call recorded; score 178 (L=130, B=16, I=0).
  Evidence: session `019ffe9f-c203-7f92-a5e2-43fa3e7c2de6`, 2026-08-14, patch-add, JSONL line 758.
- [ ] **94. [tuning/follower/sysid/SysIdRegressionTest.java](../../TeamCode/src/test/java/tuning/follower/sysid/SysIdRegressionTest.java)** — Modification call recorded; Codex edit recorded *; score 179 (L=131, B=16, I=0).
  Evidence: session `01a11718-7f12-7033-aa18-d1a236b77ee5`, 2026-10-07, write-command, JSONL line 137.
- [ ] **95. [trajectory/OnlinePlanningBenchmarkTest.java](../../TeamCode/src/test/java/trajectory/OnlinePlanningBenchmarkTest.java)** — Creation call recorded; Write call recorded; score 182 (L=88, B=18, I=8).
  Evidence: session `01a0ff49-4135-7d91-be01-a690dad9fdd5`, 2026-10-03, patch-add, JSONL line 496.
- [ ] **96. [org/firstinspires/ftc/teamcode/sim/StockAccelerationDiagnosticTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/StockAccelerationDiagnosticTest.java)** — Creation call recorded; Write call recorded; score 185 (L=123, B=19, I=1).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-02, patch-add, JSONL line 1699.
- [ ] **97. [localizers/LocalizationConstantsTest.java](../../TeamCode/src/test/java/localizers/LocalizationConstantsTest.java)** — Creation call recorded; Write call recorded; score 188 (L=162, B=2, I=4).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 1041; historical package mapped by unique filename.
- [ ] **98. [org/firstinspires/ftc/teamcode/sim/ApexSimulation.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/ApexSimulation.java)** — Modification call recorded; Write call recorded; score 193 (L=158, B=10, I=1).
  Evidence: session `01a081ea-fc91-7aa0-a44e-ecd4c621967f`, 2026-09-08, patch-update, JSONL line 398.
- [ ] **99. [org/firstinspires/ftc/teamcode/sim/TrajectoryLifecycleSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/TrajectoryLifecycleSimulationTest.java)** — Modification call recorded; Codex edit recorded; score 203 (L=132, B=7, I=10).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 937.
- [ ] **100. [org/firstinspires/ftc/teamcode/sim/VelocityFeedbackExperimentTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/VelocityFeedbackExperimentTest.java)** — Creation call recorded; Write call recorded; score 227 (L=144, B=26, I=1).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-05, patch-add, JSONL line 2141.
- [ ] **101. [org/firstinspires/ftc/teamcode/sim/HeadingFeedforwardSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/HeadingFeedforwardSimulationTest.java)** — Creation call recorded; Write call recorded; score 232 (L=124, B=31, I=3).
  Evidence: session `01a0cbc7-4a94-74e0-9dad-57f92f82a3fc`, 2026-09-23, patch-add, JSONL line 340.
- [ ] **102. [org/firstinspires/ftc/teamcode/sim/SimApexPinpoint.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimApexPinpoint.java)** — Modification call recorded; Write call recorded; score 241 (L=155, B=17, I=7).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-update, JSONL line 1529.

### 3. Several interacting responsibilities

- [ ] **103. [feedforward/MotionCharacterizationTest.java](../../TeamCode/src/test/java/feedforward/MotionCharacterizationTest.java)** — Creation call recorded; Write call recorded; score 252 (L=164, B=16, I=8).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-02, patch-add, JSONL line 2503; historical package mapped by unique filename.
- [ ] **104. [follower/FollowerVectorTest.java](../../TeamCode/src/test/java/follower/FollowerVectorTest.java)** — Modification call recorded; Write call recorded; score 254 (L=211, B=1, I=8).
  Evidence: session `01a05fb2-639d-7312-95ef-44b7ab13aca0`, 2026-09-02, patch-update, JSONL line 1990; historical package mapped by unique filename.
- [ ] **105. [org/firstinspires/ftc/teamcode/sim/StraightTwistFeedforwardDiagnosticTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/StraightTwistFeedforwardDiagnosticTest.java)** — Creation call recorded; Write call recorded; score 256 (L=174, B=19, I=5).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-05, patch-add, JSONL line 942.
- [ ] **106. [org/firstinspires/ftc/teamcode/sim/AccelerationFeedforwardPhaseSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/AccelerationFeedforwardPhaseSimulationTest.java)** — Creation call recorded; Write call recorded *; score 266 (L=177, B=23, I=4).
  Evidence: session `01a093e1-ae3d-7191-95bd-6d48841fa253`, 2026-09-12, patch-add, JSONL line 363.
- [ ] **107. [org/firstinspires/ftc/teamcode/sim/TankPathFollowingSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/TankPathFollowingSimulationTest.java)** — Creation call recorded; Codex edit recorded; score 269 (L=176, B=11, I=12).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 897.
- [ ] **108. [localizers/BaseLocalizerTest.java](../../TeamCode/src/test/java/localizers/BaseLocalizerTest.java)** — Modification call recorded; Write call recorded; score 274 (L=242, B=4, I=4).
  Evidence: session `019ffe9f-c203-7f92-a5e2-43fa3e7c2de6`, 2026-08-14, patch-update, JSONL line 758.
- [ ] **109. [org/firstinspires/ftc/teamcode/sim/FeedforwardPhaseSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/FeedforwardPhaseSimulationTest.java)** — Creation call recorded; Write call recorded *; score 275 (L=191, B=18, I=6).
  Evidence: session `01a06760-70c9-7dd2-8d18-aa3d9074aa29`, 2026-09-08, patch-add, JSONL line 1743.
- [ ] **110. [org/firstinspires/ftc/teamcode/sim/AngularCharacterizationMeasurementTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/AngularCharacterizationMeasurementTest.java)** — Creation call recorded; Write call recorded *; score 289 (L=176, B=26, I=7).
  Evidence: session `01a10e9c-43cc-78a0-9910-0cf631a76067`, 2026-10-06, patch-add, JSONL line 417.
- [ ] **111. [trajectory/generators/MecanumCoupledProfileTest.java](../../TeamCode/src/test/java/trajectory/generators/MecanumCoupledProfileTest.java)** — Creation call recorded; Write call recorded; score 291 (L=165, B=22, I=12).
  Evidence: session `01a0fde7-ad4c-7f92-b52b-41d65f9bf889`, 2026-10-02, patch-add, JSONL line 336; historical package mapped by unique filename.
- [ ] **112. [tuning/localizer/LocalizerAdaptersTest.java](../../TeamCode/src/test/java/tuning/localizer/LocalizerAdaptersTest.java)** — Creation call recorded; Write call recorded; score 297 (L=196, B=2, I=19).
  Evidence: session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, 2026-09-10, patch-add, JSONL line 1041.
- [ ] **113. [org/firstinspires/ftc/teamcode/sim/TankAutoAccuracySimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/TankAutoAccuracySimulationTest.java)** — Modification call recorded; Codex edit recorded; score 304 (L=223, B=22, I=3).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 824.
- [ ] **114. [tuning/follower/sysid/SysIdGraphs.java](../../TeamCode/src/test/java/tuning/follower/sysid/SysIdGraphs.java)** — Modification call recorded; Write call recorded; score 311 (L=191, B=40, I=0).
  Evidence: session `01a10db1-3700-7c11-9ea1-92a260fb773a`, 2026-10-05, patch-update, JSONL line 563.
- [ ] **115. [org/firstinspires/ftc/teamcode/sim/MotionCharacterizationSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/MotionCharacterizationSimulationTest.java)** — Creation call recorded; Codex edit recorded *; score 312 (L=167, B=35, I=8).
  Evidence: session `01a116fe-1282-7610-906a-7aac27ed0ad6`, 2026-10-07, write-command, JSONL line 140.
- [ ] **116. [org/firstinspires/ftc/teamcode/sim/DualActuatedPathFollowingSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/DualActuatedPathFollowingSimulationTest.java)** — Creation call recorded; Codex edit recorded; score 315 (L=191, B=18, I=14).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 824.
- [ ] **117. [org/firstinspires/ftc/teamcode/sim/ProfiledTrackingPhaseSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/ProfiledTrackingPhaseSimulationTest.java)** — Creation call recorded; Codex edit recorded *; score 328 (L=235, B=21, I=6).
  Evidence: session `01a11718-7f12-7033-aa18-d1a236b77ee5`, 2026-10-07, write-command, JSONL line 151.
- [ ] **118. [org/codeblooded/ftcodesim/ascope/ApexAdvantageScopeLayout.java](../../TeamCode/src/test/java/org/codeblooded/ftcodesim/ascope/ApexAdvantageScopeLayout.java)** — Modification call recorded; Write call recorded; score 329 (L=206, B=41, I=0).
  Evidence: session `019ffd65-bddf-7582-94ef-e71ff4926458`, 2026-08-14, patch-update, JSONL line 472.
- [ ] **119. [trajectory/generators/ProfilePowerLimitTest.java](../../TeamCode/src/test/java/trajectory/generators/ProfilePowerLimitTest.java)** — Creation call recorded; Codex edit recorded; score 344 (L=214, B=10, I=20).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 393; historical package mapped by unique filename.
- [ ] **120. [localizers/util/PoseEstimatorTest.java](../../TeamCode/src/test/java/localizers/util/PoseEstimatorTest.java)** — Not established; Recent Git change only; score 361 (L=251, B=30, I=4).
  Evidence: Git `c54c4c0 2026-10-05 Integrate coupled mecanum feedforward, unified localization, and review repairs`.
- [ ] **121. [org/firstinspires/ftc/teamcode/sim/MeasuredMecanumExperimentTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/MeasuredMecanumExperimentTest.java)** — Creation call recorded; Write call recorded; score 384 (L=199, B=50, I=7).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-05, patch-add, JSONL line 1099.
- [ ] **122. [org/firstinspires/ftc/teamcode/sim/VelocityFeedbackSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/VelocityFeedbackSimulationTest.java)** — Creation call recorded; Write call recorded; score 403 (L=234, B=48, I=5).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-05, patch-add, JSONL line 2247.
- [ ] **123. [org/firstinspires/ftc/teamcode/sim/LinePowerSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/LinePowerSimulationTest.java)** — Creation call recorded; Write call recorded; score 441 (L=292, B=33, I=10).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-04, patch-add, JSONL line 225.
- [ ] **124. [PathProfileConvergenceTest.java](../../TeamCode/src/test/java/PathProfileConvergenceTest.java)** — Creation call recorded; Codex edit recorded; score 444 (L=307, B=9, I=22).
  Evidence: session `019f2428-2020-7842-902c-743f4bd95173`, 2026-07-02, patch-update, JSONL line 406.
- [ ] **125. [org/firstinspires/ftc/teamcode/sim/SysIdMethodComparisonTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SysIdMethodComparisonTest.java)** — Creation call recorded; Write call recorded; score 460 (L=196, B=78, I=6).
  Evidence: session `01a10db1-3700-7c11-9ea1-92a260fb773a`, 2026-10-05, patch-add, JSONL line 1561.
- [ ] **126. [org/firstinspires/ftc/teamcode/sim/BrakingFeedforwardDiagnosticTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/BrakingFeedforwardDiagnosticTest.java)** — Creation call recorded; Write call recorded; score 477 (L=271, B=52, I=10).
  Evidence: session `01a108da-d5af-7623-bf0b-04fca99bec74`, 2026-10-04, patch-add, JSONL line 400.
- [ ] **127. [trajectory/TrajectoryTest.java](../../TeamCode/src/test/java/trajectory/TrajectoryTest.java)** — Creation call recorded; Codex edit recorded; score 527 (L=396, B=17, I=16).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 824.

### 4. Stateful algorithms and workflows

- [ ] **128. [org/firstinspires/ftc/teamcode/sim/AutoTestSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/AutoTestSimulationTest.java)** — Creation call recorded; Codex edit recorded; score 585 (L=375, B=65, I=3).
  Evidence: session `01a0cbc7-4a94-74e0-9dad-57f92f82a3fc`, 2026-09-23, write-command, JSONL line 194.
- [ ] **129. [org/firstinspires/ftc/teamcode/sim/FollowerTunerTelemetryTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/FollowerTunerTelemetryTest.java)** — Creation call recorded; Codex edit recorded *; score 633 (L=458, B=50, I=5).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-02, write-command, JSONL line 2437.
- [ ] **130. [org/firstinspires/ftc/teamcode/sim/FreshSysIdManualValidationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/FreshSysIdManualValidationTest.java)** — Creation call recorded; Write call recorded *; score 700 (L=351, B=103, I=8).
  Evidence: session `01a10db1-3700-7c11-9ea1-92a260fb773a`, 2026-10-05, patch-add, JSONL line 276.
- [ ] **131. [org/firstinspires/ftc/teamcode/sim/ApexSimulationTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/ApexSimulationTest.java)** — Modification call recorded; Codex edit recorded; score 875 (L=693, B=44, I=10).
  Evidence: session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, 2026-10-02, write-command, JSONL line 824.
- [ ] **132. [org/firstinspires/ftc/teamcode/sim/StraightDecelerationComparisonTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/StraightDecelerationComparisonTest.java)** — Creation call recorded; Write call recorded; score 960 (L=550, B=105, I=19).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-01, patch-add, JSONL line 287.
- [ ] **133. [org/firstinspires/ftc/teamcode/sim/BrakingSlopeSweepTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/BrakingSlopeSweepTest.java)** — Creation call recorded; Write call recorded; score 1055 (L=568, B=129, I=20).
  Evidence: session `01a0f827-dfbd-7310-a314-27dee99100b8`, 2026-10-01, patch-add, JSONL line 732.

### 5. Largest review effort

- [ ] **134. [org/firstinspires/ftc/teamcode/sim/LocalizationTunerWorkflowTest.java](../../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/LocalizationTunerWorkflowTest.java)** — Creation call recorded; Codex edit recorded *; score 1256 (L=957, B=73, I=16).
  Evidence: session `01a11718-7f12-7033-aa18-d1a236b77ee5`, 2026-10-07, write-command, JSONL line 137.

## Keeping the list useful

For future AI changes, record the affected file paths and the chat/commit reference in a review ledger. Reopen the review checkbox for substantive edits after human review. Prefer file-level tracking over an `AI-generated` comment in every class, because most files have mixed authorship.

Regenerate from the repository root with `python tools/build_ai_review_inventory.py`. Existing checked boxes are preserved by file path; substantive new edits require you to clear them. The scanner recognizes patches and common literal Python writes, not every possible shell/editor operation. Missing, cloud-only, other-machine, or differently located sessions and non-Codex AI edits can be missed. Basename mapping bridges old package moves only when unambiguous. Evidence contains references only; the underlying private chat logs are not copied into this repository.
