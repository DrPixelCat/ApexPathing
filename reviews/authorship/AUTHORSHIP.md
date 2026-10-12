# Main-source authorship audit

Reviewed 2026-10-10; HEAD `b594eff9`; includes the pre-existing working-tree changes.

Covers 144 Apex Java source files and 68 bundled FTC SDK/sample files. The inventory includes 356 named types. Author-tag changes are limited to the 69 classes relevant to Joel H credit. Added SDK, unrelated-class, and nested-type provenance notes from the initial audit were withdrawn.

## How to read the credits

`@author` credits describe a documented contribution, including an earlier implementation or a specifically stated design requirement. They do not assert that a person typed every line. Joel H is credited for the scoped guidance documented in this audit. Class JavaDoc contains the decided author tags without audit commentary. AI implementation details are not assigned to Joel H merely because a commit used his account.

Git commits are the primary evidence for contributions by people other than Joel. A tag introduced in Joel's commit does not establish another person's contribution. The revised review separates substantive code changes from comment, whitespace, package, and import changes. A human account does not prove human-written implementation. Recorded Codex changes prove involvement in a file, not wholesale generation. A creation call can be a split, copy, or recreation. No percentages of human versus AI authorship can be established from these records.

No new non-Joel author tags are added. Other people's original tag wording is preserved, except the mistaken Dylan tag on Scalar. The SparkFun OTOS and Ethan Doak/goBILDA driver credits are retained on their actual nested drivers. Copyright holders in SDK notices are not automatically converted into individual authors.

## Evidence and method

1. Inventoried every Java file under both main source trees and parsed named types with the JDK compiler API.
2. Collected the original history/tag inventory, then separately reviewed normal 50% rename history without merges and compared substantive code changes. The latter is recorded in git-contributions.json.gz.
3. Checked discontinuities and original non-Joel tags. Predecessor/tag lists are retained as historical leads in the inventory; they are not automatically promoted to author tags on rewritten or extracted classes. An earliest followed commit is not necessarily class creation.
4. Cross-referenced the existing AI-edit inventory with direct user messages in repository-specific local sessions. Approval-review transcripts and assistant-authored suggestions were not treated as independent user design statements. Approved plans establish requested behavior, not sole invention of their algorithms.
5. Applied Joel-specific author tags, removed authorship commentary from JavaDoc as requested, restored unrelated documentation, and checked source fingerprints and author tags with the Java parser.

Aliases DrPixelCat, Dr Pixel Cat, DrPixelCat24, JoelyBoi, Joel H / 7842a, and the user-confirmed 7842 variants, and the draft author Joel Holder resolve to **Joel H**. Other aliases are resolved by matching historical credit and Git identity evidence: Dylan B./DylDev; Sohum Arora/sohum-arora/paradoxical-duck; Xander Haemel/Xhaemel/Xander-r343; Topher F./MooseCoding; Atharv G./agurnani.

The existing AI inventory is dated 2026-10-07 and contains references to 271 local sessions. Relevant later sessions were also inspected, including the creation of TranslationAxis and DeltaTime. This is not a complete record of every AI tool, machine, private conversation, or earlier prompt. Successful batched calls may have ambiguous per-operation success; the JSON retains that distinction. Session IDs and line references permit local verification; private chat transcripts are not exported.

## Correction to the initial audit

The user clarified that the review should establish Joel H's contribution and leave other people's authorship alone. Initial non-Joel additions in 35 files were withdrawn, along with team/name normalization and generic predecessor credits. Existing non-Joel tags retain their original spelling and ordering. A shared predecessor does not automatically confer authorship of a new helper class.

Scalar is the one existing tag corrected under the user's explicit rule. It was added in commit `812e284c` by JoelyBoi and subsequently changed only through that account. Its Dylan tag did not have independent class-contribution evidence. That tag was removed; Joel H's explicit unitless-Scalar API guidance remains credited.

Four preserved non-Joel tags do not have a direct account match in normal followed class history: Sohum Arora in ApexConstants, Xander Haemel in CoaxialSwerve and Follower, and Atharv G. in Motor. They were introduced/carried by Dylan's commits, rather than Joel-only new-class commits, and may describe earlier source lineage. They remain unchanged because the user asked to leave unrelated credits alone; this audit does not newly certify their scope.

Wolfpack Machina remains acknowledged as PDS-controller inspiration. Road Runner contributors are acknowledged for the feedforward/braking and ramp-tuning inspiration recorded in [the braking implementation note](../../doc/ProfiledBrakingImplementation.md) and [the SysId source comparison](../../doc/SysIdRoadRunnerComparison.md). WPILib SysId is credited as workflow inspiration, not as the author of Apex regression code.

## Profile-generation draft as design evidence

The locally available draft **Controlling Mobile Robots at Full Normalized Utilization on Parametric Trajectories** names Joel Holder as author (page 1). Its declaration on page 16 reports Gemini assistance with explanations, wording, and research, and attributes its core concepts and algorithmic designs to the author. This is affirmative evidence for scoped design credit; it does not prove that Joel H hand-wrote the Java implementation or establish mathematical novelty.

The file was supplied as design guidance in session `019f2428-2020-7842-902c-743f4bd95173`, user-message lines 6 and 16. The local copy was inspected for this audit; its exact historical revision at the time of the original chat is not established. The source filename, hash, page references, and attribution are recorded in `design-sources.json`. Formatting/editing reviewers listed in the paper are not automatically treated as authors of the Java algorithms.

| Class/file | Design credited to Joel H | Draft pages |
| --- | --- | --- |
| [controllers/DriveController.java](../../TeamCode/src/main/java/controllers/DriveController.java) | saturation-aware allocation principles | 11, 12 |
| [follower/HolonomicCommandAllocator.java](../../TeamCode/src/main/java/follower/HolonomicCommandAllocator.java) | prioritizing heading and path correction ahead of tangential drive during saturation | 11, 12 |
| [geometry/PathSegment.java](../../TeamCode/src/main/java/geometry/PathSegment.java) | finite-difference curvature-derivative estimation for spatial profiling | 15, 16 |
| [trajectory/generators/BaseProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/BaseProfileGenerator.java) | coupled power-constrained profiling | 12, 13, 14, 15 |
| [trajectory/generators/MecanumProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/MecanumProfileGenerator.java) | direction-dependent drivetrain limits and constrained trajectory generation | 4, 5, 14, 15 |
| [trajectory/generators/SwerveProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/SwerveProfileGenerator.java) | combined tangential, centripetal, and heading power constraints | 12, 13, 14, 15 |
| [trajectory/generators/TankProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/TankProfileGenerator.java) | spatial trajectory constraints and heading-aware profile generation | 11, 12, 14, 15 |
| [trajectory/profile/CoupledMotionProfile.java](../../TeamCode/src/main/java/trajectory/profile/CoupledMotionProfile.java) | coupled forward/backward profiling principles | 14, 15 |
| [trajectory/profile/MotionParameters.java](../../TeamCode/src/main/java/trajectory/profile/MotionParameters.java) | the path-relative velocity, acceleration, angular-velocity, and angular-acceleration state | 3, 9, 10 |

## Older Joel H credits requiring personal confirmation

22 files previously credited Joel H, but the available records do not establish a sufficiently specific role. Those credits are preserved here and in credits.json; the JavaDoc does not assert personal authorship until the role is established. Missing evidence does not prove that Joel H did not design or write these classes.

| Class/file | Historical evidence | Unresolved role |
| --- | --- | --- |
| [controllers/PDSController.java](../../TeamCode/src/main/java/controllers/PDSController.java) | [`21dbca6d`](https://github.com/DrPixelCat/ApexPathing/commit/21dbca6d35db4bb631098300a109e18a56a29180) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [drivetrains/DualActuated.java](../../TeamCode/src/main/java/drivetrains/DualActuated.java) | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [drivetrains/Mecanum.java](../../TeamCode/src/main/java/drivetrains/Mecanum.java) | [`5e9d7540`](https://github.com/DrPixelCat/ApexPathing/commit/5e9d75400a1896ae8936189bff63bffa3688d89b) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [follower/FollowerConstants.java](../../TeamCode/src/main/java/follower/FollowerConstants.java) | [`5289a48d`](https://github.com/DrPixelCat/ApexPathing/commit/5289a48ddfbe2b2ace0206b6e06ab2118491930d) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [geometry/ArcPose.java](../../TeamCode/src/main/java/geometry/ArcPose.java) | [`b7419e0c`](https://github.com/DrPixelCat/ApexPathing/commit/b7419e0cd6a63ba86b734e1c13e50e4119645b1a) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [geometry/BSpline.java](../../TeamCode/src/main/java/geometry/BSpline.java) | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [geometry/CubicSpline1D.java](../../TeamCode/src/main/java/geometry/CubicSpline1D.java) | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [geometry/GeometryFactory.java](../../TeamCode/src/main/java/geometry/GeometryFactory.java) | [`283fbb53`](https://github.com/DrPixelCat/ApexPathing/commit/283fbb5378a054ef4d6c6d23da6eb358b4a0399e) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [geometry/Matrix.java](../../TeamCode/src/main/java/geometry/Matrix.java) | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [geometry/ParametricSegment.java](../../TeamCode/src/main/java/geometry/ParametricSegment.java) | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [geometry/PathPoint.java](../../TeamCode/src/main/java/geometry/PathPoint.java) | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [localizers/util/FilterState.java](../../TeamCode/src/main/java/localizers/util/FilterState.java) | [`4defc9f4`](https://github.com/DrPixelCat/ApexPathing/commit/4defc9f42c43559ddb363d112f7883b6207ed48c) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [localizers/util/LowPassFilter.java](../../TeamCode/src/main/java/localizers/util/LowPassFilter.java) | [`4defc9f4`](https://github.com/DrPixelCat/ApexPathing/commit/4defc9f42c43559ddb363d112f7883b6207ed48c) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [paths/Callback.java](../../TeamCode/src/main/java/paths/Callback.java) | [`08aab3eb`](https://github.com/DrPixelCat/ApexPathing/commit/08aab3ebb39f546e5ead3885b2dbbba077f5c27e) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [paths/constraint/AngularConstraint.java](../../TeamCode/src/main/java/paths/constraint/AngularConstraint.java) | [`c814bbe8`](https://github.com/DrPixelCat/ApexPathing/commit/c814bbe863be63fa08d224e23cc7edca806189f8) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [paths/constraint/PathConstraint.java](../../TeamCode/src/main/java/paths/constraint/PathConstraint.java) | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [paths/constraint/TranslationalConstraint.java](../../TeamCode/src/main/java/paths/constraint/TranslationalConstraint.java) | [`c814bbe8`](https://github.com/DrPixelCat/ApexPathing/commit/c814bbe863be63fa08d224e23cc7edca806189f8) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [paths/heading/HeadingInterpolator.java](../../TeamCode/src/main/java/paths/heading/HeadingInterpolator.java) | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [paths/heading/HeadingNode.java](../../TeamCode/src/main/java/paths/heading/HeadingNode.java) | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [paths/heading/InterpolationStyle.java](../../TeamCode/src/main/java/paths/heading/InterpolationStyle.java) | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [paths/heading/TankInterpolator.java](../../TeamCode/src/main/java/paths/heading/TankInterpolator.java) | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |
| [trajectory/profile/FFLut.java](../../TeamCode/src/main/java/trajectory/profile/FFLut.java) | [`a532f4d4`](https://github.com/DrPixelCat/ApexPathing/commit/a532f4d40101edc39f2d94881268d4d1718eda30) | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |

Confirm a concrete contribution, such as the original API design, an algorithm, a specific implementation, or explicit instructions given to an AI. Restore `@author Joel H` with that scope; review or committing code alone should not become implementation credit.

## Apex file-by-file ledger

“AI recorded” means a non-package/import write call was recorded, including potentially documentation edits. “Not established” is not a human-written verdict. The full histories and success flags are in git-inventory.json.gz and credits.json.

| File | Preserved tags and Joel scope | Commit contribution evidence | AI evidence |
| --- | --- | --- | --- |
| [config/ApexConstants.java](../../TeamCode/src/main/java/config/ApexConstants.java) | Original non-Joel tags: Dylan B. 18597 RoboClovers - Delta; Sohum Arora - 22985 Paraducks.  | [`66b7edfb`](https://github.com/DrPixelCat/ApexPathing/commit/66b7edfbcff8e8c288d7eda685263e8fabd363b6) (2026-07-07); substantive accounts: Dylan B., Joel H | Recorded: 1 calls |
| [controllers/DriveController.java](../../TeamCode/src/main/java/controllers/DriveController.java) | Joel H: supplied draft design for saturation-aware allocation principles. | [`9134d259`](https://github.com/DrPixelCat/ApexPathing/commit/9134d259008846970393946e1e69b8249aeceed3) (2026-07-11); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 6 calls |
| [controllers/FeedbackController.java](../../TeamCode/src/main/java/controllers/FeedbackController.java) | Joel H: specified the fluent component API, static-term behavior, and coefficient validation. | [`812e284c`](https://github.com/DrPixelCat/ApexPathing/commit/812e284c8b4cf7f34837eb5fc77f52222b97ce7b) (2026-09-23); substantive accounts: Joel H | Recorded: 10 calls |
| [controllers/PDSController.java](../../TeamCode/src/main/java/controllers/PDSController.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`5c08b02f`](https://github.com/DrPixelCat/ApexPathing/commit/5c08b02f217c5166a71a396656ae68772b03fcb0) (2026-05-25); substantive accounts: Dylan B., Joel H | Recorded: 6 calls |
| [controllers/TurnController.java](../../TeamCode/src/main/java/controllers/TurnController.java) | Joel H: directed displacement-profile turn behavior and the quick-turn versus stored-profile distinction. | [`73b7d155`](https://github.com/DrPixelCat/ApexPathing/commit/73b7d155d56d5564d15c3d5cb5ba04b055c18414) (2026-07-13); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 39 calls |
| [drivetrains/BaseDrivetrain.java](../../TeamCode/src/main/java/drivetrains/BaseDrivetrain.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`5cadf323`](https://github.com/DrPixelCat/ApexPathing/commit/5cadf323a21a43c0ab4b0d6eb3e4fd4751dcaf55) (2026-06-01); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 1 calls |
| [drivetrains/BaseDrivetrainConstants.java](../../TeamCode/src/main/java/drivetrains/BaseDrivetrainConstants.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`5cadf323`](https://github.com/DrPixelCat/ApexPathing/commit/5cadf323a21a43c0ab4b0d6eb3e4fd4751dcaf55) (2026-06-01); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 1 calls |
| [drivetrains/CoaxialSwerve.java](../../TeamCode/src/main/java/drivetrains/CoaxialSwerve.java) | Original non-Joel tags: Xander Haemel - 31616 404 Not Found; Dylan B. - 18597 RoboClovers - Delta.  | [`5cadf323`](https://github.com/DrPixelCat/ApexPathing/commit/5cadf323a21a43c0ab4b0d6eb3e4fd4751dcaf55) (2026-06-01); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 2 calls |
| [drivetrains/DualActuated.java](../../TeamCode/src/main/java/drivetrains/DualActuated.java) | Individual human implementation author unverified. | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) (2026-06-26); substantive accounts: Dylan B., Joel H | Recorded: 2 calls |
| [drivetrains/Kiwi.java](../../TeamCode/src/main/java/drivetrains/Kiwi.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`ba85ead3`](https://github.com/DrPixelCat/ApexPathing/commit/ba85ead30c61d1e53610780e82754aab99d55188) (2026-05-22); substantive accounts: Dylan B., Joel H, Sohum Arora | Not established |
| [drivetrains/Mecanum.java](../../TeamCode/src/main/java/drivetrains/Mecanum.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`2d7d1eaa`](https://github.com/DrPixelCat/ApexPathing/commit/2d7d1eaa924f7b3fb240ff0e68c87bf9d7b0277d) (2026-03-29); substantive accounts: Dylan B., Joel H, Sohum Arora, Xander Haemel | Recorded: 2 calls |
| [drivetrains/Motor.java](../../TeamCode/src/main/java/drivetrains/Motor.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta; Atharv G. - 13085 Bionic Dutch.  | [`5cadf323`](https://github.com/DrPixelCat/ApexPathing/commit/5cadf323a21a43c0ab4b0d6eb3e4fd4751dcaf55) (2026-06-01); substantive accounts: Dylan B. | Not established |
| [drivetrains/Tank.java](../../TeamCode/src/main/java/drivetrains/Tank.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`7b8f2eb2`](https://github.com/DrPixelCat/ApexPathing/commit/7b8f2eb22b9195162cd6a493d7099e80ea885d73) (2026-03-30); substantive accounts: Atharv G., Dylan B., Joel H, Sohum Arora, Xander Haemel | Not established |
| [drivetrains/TranslationAxis.java](../../TeamCode/src/main/java/drivetrains/TranslationAxis.java) | Joel H: directed calibration-axis ownership and placement of the shared translation axis near the drivetrain. | Uncommitted new file | Recorded: 1 calls |
| [feedforward/CenteredBrakingFeedforward.java](../../TeamCode/src/main/java/feedforward/CenteredBrakingFeedforward.java) | Individual human implementation author unverified. | [`c54c4c01`](https://github.com/DrPixelCat/ApexPathing/commit/c54c4c01cbe5b418cd893f9574d810d5502a7e72) (2026-10-05); substantive accounts: Joel H | Recorded: 3 calls |
| [feedforward/HolonomicFeedforward.java](../../TeamCode/src/main/java/feedforward/HolonomicFeedforward.java) | Individual human implementation author unverified. | [`6a8ea834`](https://github.com/DrPixelCat/ApexPathing/commit/6a8ea8340dc104a724d217433278f017912afba8) (2026-10-02); substantive accounts: Joel H | Recorded: 2 calls |
| [feedforward/MecanumWheelModel.java](../../TeamCode/src/main/java/feedforward/MecanumWheelModel.java) | Individual human implementation author unverified. | [`c54c4c01`](https://github.com/DrPixelCat/ApexPathing/commit/c54c4c01cbe5b418cd893f9574d810d5502a7e72) (2026-10-05); substantive accounts: Joel H | Recorded: 6 calls |
| [feedforward/MotionCharacterization.java](../../TeamCode/src/main/java/feedforward/MotionCharacterization.java) | Individual human implementation author unverified. | [`a1806769`](https://github.com/DrPixelCat/ApexPathing/commit/a1806769bcb710c7c6eb3c6cba239bb795af9439) (2026-10-02); substantive accounts: Joel H | Recorded: 6 calls |
| [feedforward/MotorPowerModel.java](../../TeamCode/src/main/java/feedforward/MotorPowerModel.java) | Individual human implementation author unverified. | [`a1806769`](https://github.com/DrPixelCat/ApexPathing/commit/a1806769bcb710c7c6eb3c6cba239bb795af9439) (2026-10-02); substantive accounts: Joel H | Recorded: 2 calls |
| [follower/Follower.java](../../TeamCode/src/main/java/follower/Follower.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks; Dylan B. - 18597 RoboClovers - Delta; Xander Haemel - 31616 404 Not Found. Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`5289a48d`](https://github.com/DrPixelCat/ApexPathing/commit/5289a48ddfbe2b2ace0206b6e06ab2118491930d) (2026-06-05); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 151 calls |
| [follower/FollowerConstants.java](../../TeamCode/src/main/java/follower/FollowerConstants.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks; Dylan B. - 18597 RoboClovers - Delta.  | [`5289a48d`](https://github.com/DrPixelCat/ApexPathing/commit/5289a48ddfbe2b2ace0206b6e06ab2118491930d) (2026-06-05); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 50 calls |
| [follower/FollowerDiagnostics.java](../../TeamCode/src/main/java/follower/FollowerDiagnostics.java) | Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`3ec0ec15`](https://github.com/DrPixelCat/ApexPathing/commit/3ec0ec15aa7518c4b2d9b7d140a8a42d9aa6fd87) (2026-08-13); substantive accounts: Joel H | Recorded: 1 calls |
| [follower/FollowerPathState.java](../../TeamCode/src/main/java/follower/FollowerPathState.java) | Individual human implementation author unverified. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 27 calls |
| [follower/FollowerStuckWatchdog.java](../../TeamCode/src/main/java/follower/FollowerStuckWatchdog.java) | Joel H: specified public slowed/stuck status, a dynamic stuck timeout, and more tolerant endpoint completion. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 1 calls |
| [follower/HolonomicCommandAllocator.java](../../TeamCode/src/main/java/follower/HolonomicCommandAllocator.java) | Joel H: supplied draft design for prioritizing heading and path correction ahead of tangential drive during saturation. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 7 calls |
| [follower/HolonomicPathFollower.java](../../TeamCode/src/main/java/follower/HolonomicPathFollower.java) | Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 68 calls |
| [follower/MotionFeedforward.java](../../TeamCode/src/main/java/follower/MotionFeedforward.java) | Joel H: directed Road Runner-inspired braking behavior and comparative validation. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 5 calls |
| [follower/MovementCallbackRunner.java](../../TeamCode/src/main/java/follower/MovementCallbackRunner.java) | Individual human implementation author unverified. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 4 calls |
| [follower/PathCompletionPolicy.java](../../TeamCode/src/main/java/follower/PathCompletionPolicy.java) | Joel H: specified public slowed/stuck status, a dynamic stuck timeout, and more tolerant endpoint completion. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 2 calls |
| [follower/PathProgressEstimator.java](../../TeamCode/src/main/java/follower/PathProgressEstimator.java) | Individual human implementation author unverified. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 1 calls |
| [follower/TankPathFollower.java](../../TeamCode/src/main/java/follower/TankPathFollower.java) | Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 33 calls |
| [follower/TurnExecutor.java](../../TeamCode/src/main/java/follower/TurnExecutor.java) | Joel H: directed displacement-profile turn behavior and the quick-turn versus stored-profile distinction. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 12 calls |
| [follower/VelocityReference.java](../../TeamCode/src/main/java/follower/VelocityReference.java) | Individual human implementation author unverified. | [`c54c4c01`](https://github.com/DrPixelCat/ApexPathing/commit/c54c4c01cbe5b418cd893f9574d810d5502a7e72) (2026-10-05); substantive accounts: Joel H | Recorded: 6 calls |
| [geometry/Angle.java](../../TeamCode/src/main/java/geometry/Angle.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`95565800`](https://github.com/DrPixelCat/ApexPathing/commit/95565800fd5f7147dd52b882b1b371e4abd4995a) (2026-05-28); substantive accounts: Dylan B., Joel H | Not established |
| [geometry/AngleUnit.java](../../TeamCode/src/main/java/geometry/AngleUnit.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`95565800`](https://github.com/DrPixelCat/ApexPathing/commit/95565800fd5f7147dd52b882b1b371e4abd4995a) (2026-05-28); substantive accounts: Dylan B. | Not established |
| [geometry/ArcPose.java](../../TeamCode/src/main/java/geometry/ArcPose.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`95565800`](https://github.com/DrPixelCat/ApexPathing/commit/95565800fd5f7147dd52b882b1b371e4abd4995a) (2026-05-28); substantive accounts: Dylan B., Joel H | Recorded: 1 calls |
| [geometry/BSpline.java](../../TeamCode/src/main/java/geometry/BSpline.java) | Original non-Joel tags: Sohum Arora - 22895 Paraducks; Dylan B. - 18597 RoboClovers - Delta.  | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) (2026-05-23); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 2 calls |
| [geometry/CubicSpline1D.java](../../TeamCode/src/main/java/geometry/CubicSpline1D.java) | Individual human implementation author unverified. | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) (2026-06-26); substantive accounts: Dylan B., Joel H | Recorded: 2 calls |
| [geometry/Dist.java](../../TeamCode/src/main/java/geometry/Dist.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta. Joel H: specified a unitless Scalar API based on Dist and shared distance arithmetic. | [`95565800`](https://github.com/DrPixelCat/ApexPathing/commit/95565800fd5f7147dd52b882b1b371e4abd4995a) (2026-05-28); substantive accounts: Dylan B., Joel H | Recorded: 1 calls |
| [geometry/DistUnit.java](../../TeamCode/src/main/java/geometry/DistUnit.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`95565800`](https://github.com/DrPixelCat/ApexPathing/commit/95565800fd5f7147dd52b882b1b371e4abd4995a) (2026-05-28); substantive accounts: Dylan B. | Not established |
| [geometry/GeometryFactory.java](../../TeamCode/src/main/java/geometry/GeometryFactory.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`bc4ef9cf`](https://github.com/DrPixelCat/ApexPathing/commit/bc4ef9cfb215f7e246bf08a1b1e6dfe060ba1f6f) (2026-07-10); substantive accounts: Dylan B., Joel H | Recorded: 1 calls |
| [geometry/Matrix.java](../../TeamCode/src/main/java/geometry/Matrix.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks.  | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) (2026-05-23); substantive accounts: Dylan B., Joel H, Sohum Arora | Not established |
| [geometry/ParametricSegment.java](../../TeamCode/src/main/java/geometry/ParametricSegment.java) | Individual human implementation author unverified. | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) (2026-05-23); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 1 calls |
| [geometry/PathPoint.java](../../TeamCode/src/main/java/geometry/PathPoint.java) | Individual human implementation author unverified. | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) (2026-05-23); substantive accounts: Dylan B., Joel H, Sohum Arora | Not established |
| [geometry/PathSegment.java](../../TeamCode/src/main/java/geometry/PathSegment.java) | Joel H: supplied draft design for finite-difference curvature-derivative estimation for spatial profiling. | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) (2026-05-23); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 8 calls |
| [geometry/Pose.java](../../TeamCode/src/main/java/geometry/Pose.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`95565800`](https://github.com/DrPixelCat/ApexPathing/commit/95565800fd5f7147dd52b882b1b371e4abd4995a) (2026-05-28); substantive accounts: Dylan B., Joel H | Not established |
| [geometry/Scalar.java](../../TeamCode/src/main/java/geometry/Scalar.java) | Joel H: specified a unitless Scalar API based on Dist and shared distance arithmetic. | [`812e284c`](https://github.com/DrPixelCat/ApexPathing/commit/812e284c8b4cf7f34837eb5fc77f52222b97ce7b) (2026-09-23); substantive accounts: Joel H | Recorded: 3 calls |
| [geometry/Vector.java](../../TeamCode/src/main/java/geometry/Vector.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`95565800`](https://github.com/DrPixelCat/ApexPathing/commit/95565800fd5f7147dd52b882b1b371e4abd4995a) (2026-05-28); substantive accounts: Dylan B., Joel H | Recorded: 1 calls |
| [localizers/BaseLocalizer.java](../../TeamCode/src/main/java/localizers/BaseLocalizer.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta. Joel H: directed shared Kalman localization, automatic tuning, and separation of filtering from pose estimation. | [`ced4bd4e`](https://github.com/DrPixelCat/ApexPathing/commit/ced4bd4eab31099949481e33fb593649243571b0) (2026-06-02); substantive accounts: Dylan B., Joel H | Recorded: 16 calls |
| [localizers/BaseLocalizerConstants.java](../../TeamCode/src/main/java/localizers/BaseLocalizerConstants.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`ced4bd4e`](https://github.com/DrPixelCat/ApexPathing/commit/ced4bd4eab31099949481e33fb593649243571b0) (2026-06-02); substantive accounts: Dylan B., Joel H | Recorded: 3 calls |
| [localizers/CalibrationJson.java](../../TeamCode/src/main/java/localizers/CalibrationJson.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 1 calls |
| [localizers/LocalizationConstants.java](../../TeamCode/src/main/java/localizers/LocalizationConstants.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 12 calls |
| [localizers/LocalizerSet.java](../../TeamCode/src/main/java/localizers/LocalizerSet.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 3 calls |
| [localizers/MecanumDriveEncoders.java](../../TeamCode/src/main/java/localizers/MecanumDriveEncoders.java) | Original non-Joel tags: Topher F. - 23571 alum; Dylan B. - 18597 RoboClovers - Delta.  | [`74704e47`](https://github.com/DrPixelCat/ApexPathing/commit/74704e470d285218bbd2453b6dc2683d2253306d) (2026-07-01); substantive accounts: Dylan B., Joel H, Sohum Arora, Topher F. | Recorded: 10 calls |
| [localizers/Octoquad.java](../../TeamCode/src/main/java/localizers/Octoquad.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`cdc3555e`](https://github.com/DrPixelCat/ApexPathing/commit/cdc3555e36c32471a976ba5d195354a2a095622f) (2026-07-25); substantive accounts: Dylan B., Joel H | Recorded: 3 calls |
| [localizers/OdometryPod.java](../../TeamCode/src/main/java/localizers/OdometryPod.java) | Original non-Joel tags: Topher F. - 23571 alum; Dylan B. - 18597 RoboClovers - Delta. Joel H: directed calibration-axis ownership and placement of the shared translation axis near the drivetrain. | [`74704e47`](https://github.com/DrPixelCat/ApexPathing/commit/74704e470d285218bbd2453b6dc2683d2253306d) (2026-07-01); substantive accounts: Dylan B., Joel H, Sohum Arora, Topher F. | Recorded: 3 calls |
| [localizers/OTOS.java](../../TeamCode/src/main/java/localizers/OTOS.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`14633043`](https://github.com/DrPixelCat/ApexPathing/commit/14633043375ab3d88269990c414866e4a27aa49d) (2026-04-17); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 1 calls |
| [localizers/Pinpoint.java](../../TeamCode/src/main/java/localizers/Pinpoint.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`e2fa6545`](https://github.com/DrPixelCat/ApexPathing/commit/e2fa6545a203eb4e5de71e3d6d88ba96fc5be5bd) (2026-03-29); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 3 calls |
| [localizers/TankDriveEncoders.java](../../TeamCode/src/main/java/localizers/TankDriveEncoders.java) | Joel H: requested a separate localization tuner and a multi-motor tank-encoder/IMU localizer. | [`1c3b5a66`](https://github.com/DrPixelCat/ApexPathing/commit/1c3b5a660d87ec96defce35715d7122682397511) (2026-09-10); substantive accounts: Joel H | Recorded: 9 calls |
| [localizers/ThreeWheel.java](../../TeamCode/src/main/java/localizers/ThreeWheel.java) | Original non-Joel tags: Topher F. - 23571 alum; Dylan B. - 18597 RoboClovers - Delta.  | [`74704e47`](https://github.com/DrPixelCat/ApexPathing/commit/74704e470d285218bbd2453b6dc2683d2253306d) (2026-07-01); substantive accounts: Dylan B., Joel H, Sohum Arora, Topher F. | Recorded: 5 calls |
| [localizers/TwoWheel.java](../../TeamCode/src/main/java/localizers/TwoWheel.java) | Original non-Joel tags: Topher F. - 23571 alum; Dylan B. - 18597 RoboClovers - Delta.  | [`74704e47`](https://github.com/DrPixelCat/ApexPathing/commit/74704e470d285218bbd2453b6dc2683d2253306d) (2026-07-01); substantive accounts: Dylan B., Joel H, Sohum Arora, Topher F. | Recorded: 6 calls |
| [localizers/util/AdaptiveKalmanFilter.java](../../TeamCode/src/main/java/localizers/util/AdaptiveKalmanFilter.java) | Joel H: approved default Kalman velocity integration and filter lifecycle requirements. | [`4defc9f4`](https://github.com/DrPixelCat/ApexPathing/commit/4defc9f42c43559ddb363d112f7883b6207ed48c) (2026-08-07); substantive accounts: Joel H | Recorded: 6 calls |
| [localizers/util/DataFilter.java](../../TeamCode/src/main/java/localizers/util/DataFilter.java) | Joel H: directed the timestamp-update API and use of the shared delta-time helper. | [`4defc9f4`](https://github.com/DrPixelCat/ApexPathing/commit/4defc9f42c43559ddb363d112f7883b6207ed48c) (2026-08-07); substantive accounts: Joel H | Recorded: 1 calls |
| [localizers/util/FilterState.java](../../TeamCode/src/main/java/localizers/util/FilterState.java) | Individual human implementation author unverified. | [`4defc9f4`](https://github.com/DrPixelCat/ApexPathing/commit/4defc9f42c43559ddb363d112f7883b6207ed48c) (2026-08-07); substantive accounts: Joel H | Not established |
| [localizers/util/KalmanFilter.java](../../TeamCode/src/main/java/localizers/util/KalmanFilter.java) | Joel H: directed shared Kalman localization, automatic tuning, and separation of filtering from pose estimation. | [`c54c4c01`](https://github.com/DrPixelCat/ApexPathing/commit/c54c4c01cbe5b418cd893f9574d810d5502a7e72) (2026-10-05); substantive accounts: Joel H | Recorded: 1 calls |
| [localizers/util/LowPassFilter.java](../../TeamCode/src/main/java/localizers/util/LowPassFilter.java) | Individual human implementation author unverified. | [`4defc9f4`](https://github.com/DrPixelCat/ApexPathing/commit/4defc9f42c43559ddb363d112f7883b6207ed48c) (2026-08-07); substantive accounts: Joel H | Recorded: 1 calls |
| [localizers/util/PoseEstimator.java](../../TeamCode/src/main/java/localizers/util/PoseEstimator.java) | Joel H: directed shared Kalman localization, automatic tuning, and separation of filtering from pose estimation. | [`c54c4c01`](https://github.com/DrPixelCat/ApexPathing/commit/c54c4c01cbe5b418cd893f9574d810d5502a7e72) (2026-10-05); substantive accounts: Joel H | Recorded: 1 calls |
| [org/firstinspires/ftc/teamcode/apexpathing/AutoTest.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/AutoTest.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks; Dylan B. - 18597 RoboClovers - Delta.  | [`6d8e128a`](https://github.com/DrPixelCat/ApexPathing/commit/6d8e128a8080de3df829015d3c6344996d2d862b) (2026-06-10); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 68 calls |
| [org/firstinspires/ftc/teamcode/apexpathing/Constants.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/Constants.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta; Sohum Arora - 22985 Paraducks.  | [`2d7d1eaa`](https://github.com/DrPixelCat/ApexPathing/commit/2d7d1eaa924f7b3fb240ff0e68c87bf9d7b0277d) (2026-03-29); substantive accounts: Atharv G., Dylan B., Joel H, Sohum Arora, Xander Haemel | Not established |
| [org/firstinspires/ftc/teamcode/apexpathing/ExampleAutoPath.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/ExampleAutoPath.java) | Individual human implementation author unverified. | [`7378b7b7`](https://github.com/DrPixelCat/ApexPathing/commit/7378b7b7f3ff6423287ef74f2100e3ca37a546ca) (2026-05-26); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 10 calls |
| [org/firstinspires/ftc/teamcode/apexpathing/FollowerTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/FollowerTuner.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks; Dylan B. - 18597 RoboClovers - Delta.  | [`be86a40b`](https://github.com/DrPixelCat/ApexPathing/commit/be86a40b099328c1f09d083c488a790dbc314c55) (2026-06-08); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 31 calls |
| [org/firstinspires/ftc/teamcode/apexpathing/LocalizationTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/LocalizationTuner.java) | Joel H: requested a separate localization tuner and a multi-motor tank-encoder/IMU localizer. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 8 calls |
| [org/firstinspires/ftc/teamcode/apexpathing/SimpleDriveTest.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/SimpleDriveTest.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`ab819258`](https://github.com/DrPixelCat/ApexPathing/commit/ab819258145a9586f4e17a02832b414170f646d4) (2026-07-30); substantive accounts: Dylan B., Joel H | Recorded: 1 calls |
| [org/firstinspires/ftc/teamcode/apexpathing/TeleOpTest.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/TeleOpTest.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks; Dylan B. - 18597 RoboClovers - Delta.  | [`db4a8ea0`](https://github.com/DrPixelCat/ApexPathing/commit/db4a8ea077c789b18118ca2ac013ee7263b24911) (2026-06-10); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 1 calls |
| [paths/builders/HolonomicPathBuilder.java](../../TeamCode/src/main/java/paths/builders/HolonomicPathBuilder.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta. Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) (2026-06-26); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 17 calls |
| [paths/builders/PathBuilder.java](../../TeamCode/src/main/java/paths/builders/PathBuilder.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta. Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) (2026-05-23); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 11 calls |
| [paths/builders/TankPathBuilder.java](../../TeamCode/src/main/java/paths/builders/TankPathBuilder.java) | Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) (2026-06-26); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 5 calls |
| [paths/builders/TurnBuilder.java](../../TeamCode/src/main/java/paths/builders/TurnBuilder.java) | Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`08aab3eb`](https://github.com/DrPixelCat/ApexPathing/commit/08aab3ebb39f546e5ead3885b2dbbba077f5c27e) (2026-05-28); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 8 calls |
| [paths/Callback.java](../../TeamCode/src/main/java/paths/Callback.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks.  | [`08aab3eb`](https://github.com/DrPixelCat/ApexPathing/commit/08aab3ebb39f546e5ead3885b2dbbba077f5c27e) (2026-05-28); substantive accounts: Joel H, Sohum Arora | Not established |
| [paths/constraint/AngularConstraint.java](../../TeamCode/src/main/java/paths/constraint/AngularConstraint.java) | Individual human implementation author unverified. | [`c814bbe8`](https://github.com/DrPixelCat/ApexPathing/commit/c814bbe863be63fa08d224e23cc7edca806189f8) (2026-07-02); substantive accounts: Dylan B., Joel H | Recorded: 2 calls |
| [paths/constraint/PathConstraint.java](../../TeamCode/src/main/java/paths/constraint/PathConstraint.java) | Individual human implementation author unverified. | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) (2026-06-26); substantive accounts: Dylan B., Joel H | Recorded: 1 calls |
| [paths/constraint/TranslationalConstraint.java](../../TeamCode/src/main/java/paths/constraint/TranslationalConstraint.java) | Individual human implementation author unverified. | [`c814bbe8`](https://github.com/DrPixelCat/ApexPathing/commit/c814bbe863be63fa08d224e23cc7edca806189f8) (2026-07-02); substantive accounts: Dylan B., Joel H | Recorded: 2 calls |
| [paths/heading/HeadingInterpolator.java](../../TeamCode/src/main/java/paths/heading/HeadingInterpolator.java) | Individual human implementation author unverified. | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) (2026-05-23); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 2 calls |
| [paths/heading/HeadingNode.java](../../TeamCode/src/main/java/paths/heading/HeadingNode.java) | Individual human implementation author unverified. | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) (2026-06-26); substantive accounts: Dylan B., Joel H | Not established |
| [paths/heading/HolonomicInterpolator.java](../../TeamCode/src/main/java/paths/heading/HolonomicInterpolator.java) | Joel H: specified FACING_POINT interpolation and its angular-offset behavior. | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) (2026-06-26); substantive accounts: Dylan B., Joel H | Recorded: 5 calls |
| [paths/heading/InterpolationStyle.java](../../TeamCode/src/main/java/paths/heading/InterpolationStyle.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`be8ba8f9`](https://github.com/DrPixelCat/ApexPathing/commit/be8ba8f974d21aec8f4f3241346d418de61088bb) (2026-05-23); substantive accounts: Dylan B., Joel H, Sohum Arora | Not established |
| [paths/heading/ReversedHeadingInterpolator.java](../../TeamCode/src/main/java/paths/heading/ReversedHeadingInterpolator.java) | Individual human implementation author unverified. | [`b02a556c`](https://github.com/DrPixelCat/ApexPathing/commit/b02a556c8474cca05cea6ef9122511e2c6a3e4c5) (2026-09-09); substantive accounts: Joel H | Recorded: 3 calls |
| [paths/heading/TankInterpolator.java](../../TeamCode/src/main/java/paths/heading/TankInterpolator.java) | Individual human implementation author unverified. | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) (2026-06-26); substantive accounts: Dylan B., Joel H | Recorded: 2 calls |
| [paths/movements/FollowerMovement.java](../../TeamCode/src/main/java/paths/movements/FollowerMovement.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks.  | [`08aab3eb`](https://github.com/DrPixelCat/ApexPathing/commit/08aab3ebb39f546e5ead3885b2dbbba077f5c27e) (2026-05-28); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 1 calls |
| [paths/movements/Path.java](../../TeamCode/src/main/java/paths/movements/Path.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks. Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`08aab3eb`](https://github.com/DrPixelCat/ApexPathing/commit/08aab3ebb39f546e5ead3885b2dbbba077f5c27e) (2026-05-28); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 9 calls |
| [paths/movements/Turn.java](../../TeamCode/src/main/java/paths/movements/Turn.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks. Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`08aab3eb`](https://github.com/DrPixelCat/ApexPathing/commit/08aab3ebb39f546e5ead3885b2dbbba077f5c27e) (2026-05-28); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 6 calls |
| [storage/ApexStorage.java](../../TeamCode/src/main/java/storage/ApexStorage.java) | Individual human implementation author unverified. | [`ac9b134b`](https://github.com/DrPixelCat/ApexPathing/commit/ac9b134ba04d719e18787496ccfda9a7c299b2eb) (2026-08-08); substantive accounts: Joel H | Recorded: 4 calls |
| [trajectory/generators/BaseProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/BaseProfileGenerator.java) | Joel H: supplied draft design for coupled power-constrained profiling. | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) (2026-06-26); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 73 calls |
| [trajectory/generators/ConstraintSchedule.java](../../TeamCode/src/main/java/trajectory/generators/ConstraintSchedule.java) | Individual human implementation author unverified. | [`176e835d`](https://github.com/DrPixelCat/ApexPathing/commit/176e835d71919314cde8b35fc4c2cd8c6904a8c7) (2026-10-02); substantive accounts: Joel H | Recorded: 2 calls |
| [trajectory/generators/MecanumProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/MecanumProfileGenerator.java) | Joel H: supplied draft design for direction-dependent drivetrain limits and constrained trajectory generation. | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) (2026-06-26); substantive accounts: Dylan B., Joel H | Recorded: 24 calls |
| [trajectory/generators/SwerveProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/SwerveProfileGenerator.java) | Joel H: supplied draft design for combined tangential, centripetal, and heading power constraints. | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) (2026-06-26); substantive accounts: Dylan B., Joel H | Recorded: 11 calls |
| [trajectory/generators/TankProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/TankProfileGenerator.java) | Joel H: supplied draft design for spatial trajectory constraints and heading-aware profile generation. | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) (2026-06-26); substantive accounts: Dylan B., Joel H | Recorded: 19 calls |
| [trajectory/generators/TurnProfileGenerator.java](../../TeamCode/src/main/java/trajectory/generators/TurnProfileGenerator.java) | Joel H: directed displacement-profile turn behavior and the quick-turn versus stored-profile distinction. | [`82a22414`](https://github.com/DrPixelCat/ApexPathing/commit/82a2241406f1c72714180925e8dc17ef59b3e7b9) (2026-06-26); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 19 calls |
| [trajectory/OnlineTrajectoryPlanner.java](../../TeamCode/src/main/java/trajectory/OnlineTrajectoryPlanner.java) | Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`cc835930`](https://github.com/DrPixelCat/ApexPathing/commit/cc83593028b980d3247a9cb4dadf92b1468efe77) (2026-10-02); substantive accounts: Joel H | Recorded: 12 calls |
| [trajectory/profile/CoupledMotionProfile.java](../../TeamCode/src/main/java/trajectory/profile/CoupledMotionProfile.java) | Joel H: supplied draft design for coupled forward/backward profiling principles. | [`6a8ea834`](https://github.com/DrPixelCat/ApexPathing/commit/6a8ea8340dc104a724d217433278f017912afba8) (2026-10-02); substantive accounts: Joel H | Recorded: 11 calls |
| [trajectory/profile/DynamicMotionProfile.java](../../TeamCode/src/main/java/trajectory/profile/DynamicMotionProfile.java) | Individual human implementation author unverified. | [`812e284c`](https://github.com/DrPixelCat/ApexPathing/commit/812e284c8b4cf7f34837eb5fc77f52222b97ce7b) (2026-09-23); substantive accounts: Joel H | Recorded: 4 calls |
| [trajectory/profile/FFLut.java](../../TeamCode/src/main/java/trajectory/profile/FFLut.java) | Individual human implementation author unverified. | [`a532f4d4`](https://github.com/DrPixelCat/ApexPathing/commit/a532f4d40101edc39f2d94881268d4d1718eda30) (2026-06-07); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 9 calls |
| [trajectory/profile/MotionParameters.java](../../TeamCode/src/main/java/trajectory/profile/MotionParameters.java) | Joel H: supplied draft design for the path-relative velocity, acceleration, angular-velocity, and angular-acceleration state. | [`a532f4d4`](https://github.com/DrPixelCat/ApexPathing/commit/a532f4d40101edc39f2d94881268d4d1718eda30) (2026-06-07); substantive accounts: Dylan B., Joel H | Recorded: 2 calls |
| [trajectory/StoppingHorizon.java](../../TeamCode/src/main/java/trajectory/StoppingHorizon.java) | Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`cc835930`](https://github.com/DrPixelCat/ApexPathing/commit/cc83593028b980d3247a9cb4dadf92b1468efe77) (2026-10-02); substantive accounts: Joel H | Recorded: 4 calls |
| [trajectory/TrajectoryField2D.java](../../TeamCode/src/main/java/trajectory/TrajectoryField2D.java) | Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`cc835930`](https://github.com/DrPixelCat/ApexPathing/commit/cc83593028b980d3247a9cb4dadf92b1468efe77) (2026-10-02); substantive accounts: Joel H | Recorded: 8 calls |
| [trajectory/TrajectorySample.java](../../TeamCode/src/main/java/trajectory/TrajectorySample.java) | Joel H: specified online lookahead versus stored trajectory behavior and shared vector-reference requirements. | [`cc835930`](https://github.com/DrPixelCat/ApexPathing/commit/cc83593028b980d3247a9cb4dadf92b1468efe77) (2026-10-02); substantive accounts: Joel H | Recorded: 6 calls |
| [trajectory/VelocityPursuit.java](../../TeamCode/src/main/java/trajectory/VelocityPursuit.java) | Individual human implementation author unverified. | [`c54c4c01`](https://github.com/DrPixelCat/ApexPathing/commit/c54c4c01cbe5b418cd893f9574d810d5502a7e72) (2026-10-05); substantive accounts: Joel H | Recorded: 1 calls |
| [tuning/follower/acceleration/AccelerationFeedforwardPhase.java](../../TeamCode/src/main/java/tuning/follower/acceleration/AccelerationFeedforwardPhase.java) | Individual human implementation author unverified. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 30 calls |
| [tuning/follower/acceleration/AccelerationSearch.java](../../TeamCode/src/main/java/tuning/follower/acceleration/AccelerationSearch.java) | Individual human implementation author unverified. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 1 calls |
| [tuning/follower/acceleration/RampRegression.java](../../TeamCode/src/main/java/tuning/follower/acceleration/RampRegression.java) | Individual human implementation author unverified. | [`12ebdeb9`](https://github.com/DrPixelCat/ApexPathing/commit/12ebdeb97cdb61cfb584b83f0c304f604a7f5523) (2026-09-11); substantive accounts: Joel H | Recorded: 8 calls |
| [tuning/follower/centripetal/BinarySearch.java](../../TeamCode/src/main/java/tuning/follower/centripetal/BinarySearch.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta.  | [`0ca92124`](https://github.com/DrPixelCat/ApexPathing/commit/0ca92124106065d6d76c42cf9ba6b29b700c28ca) (2026-07-15); substantive accounts: Dylan B., Joel H, Sohum Arora | Not established |
| [tuning/follower/centripetal/CentripetalPhase.java](../../TeamCode/src/main/java/tuning/follower/centripetal/CentripetalPhase.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks; Dylan B. - 18597 RoboClovers - Delta.  | [`0ca92124`](https://github.com/DrPixelCat/ApexPathing/commit/0ca92124106065d6d76c42cf9ba6b29b700c28ca) (2026-07-15); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 29 calls |
| [tuning/follower/characterization/MotionCharacterizationPhase.java](../../TeamCode/src/main/java/tuning/follower/characterization/MotionCharacterizationPhase.java) | Joel H: specified fewer alternating trials and moving validation into dedicated tests. | [`a1806769`](https://github.com/DrPixelCat/ApexPathing/commit/a1806769bcb710c7c6eb3c6cba239bb795af9439) (2026-10-02); substantive accounts: Joel H | Recorded: 17 calls |
| [tuning/follower/characterization/MotorAuthorityRegression.java](../../TeamCode/src/main/java/tuning/follower/characterization/MotorAuthorityRegression.java) | Individual human implementation author unverified. | [`a1806769`](https://github.com/DrPixelCat/ApexPathing/commit/a1806769bcb710c7c6eb3c6cba239bb795af9439) (2026-10-02); substantive accounts: Joel H | Recorded: 8 calls |
| [tuning/follower/feedback/DrivePhase.java](../../TeamCode/src/main/java/tuning/follower/feedback/DrivePhase.java) | Original non-Joel tags: Dylan B. - 18597 RoboClovers - Delta; Sohum Arora - 22985 Paraducks.  | [`9134d259`](https://github.com/DrPixelCat/ApexPathing/commit/9134d259008846970393946e1e69b8249aeceed3) (2026-07-11); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 41 calls |
| [tuning/follower/feedback/HeadingPhase.java](../../TeamCode/src/main/java/tuning/follower/feedback/HeadingPhase.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks; Dylan B. - 18597 RoboClovers - Delta.  | [`66b7edfb`](https://github.com/DrPixelCat/ApexPathing/commit/66b7edfbcff8e8c288d7eda685263e8fabd363b6) (2026-07-07); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 37 calls |
| [tuning/follower/feedback/ManualResponseMetrics.java](../../TeamCode/src/main/java/tuning/follower/feedback/ManualResponseMetrics.java) | Individual human implementation author unverified. | [`97a4cb79`](https://github.com/DrPixelCat/ApexPathing/commit/97a4cb79e7466cf1e54c5da63a90d14338bbd71e) (2026-08-13); substantive accounts: Joel H | Recorded: 5 calls |
| [tuning/follower/feedback/PDSRoutine.java](../../TeamCode/src/main/java/tuning/follower/feedback/PDSRoutine.java) | Joel H: specified iterative PD-tuning requirements and subsequent workflow refinements. | [`0ca92124`](https://github.com/DrPixelCat/ApexPathing/commit/0ca92124106065d6d76c42cf9ba6b29b700c28ca) (2026-07-15); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 103 calls |
| [tuning/follower/limits/LimitsPhase.java](../../TeamCode/src/main/java/tuning/follower/limits/LimitsPhase.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks; Dylan B. - 18597 RoboClovers - Delta.  | [`0ca92124`](https://github.com/DrPixelCat/ApexPathing/commit/0ca92124106065d6d76c42cf9ba6b29b700c28ca) (2026-07-15); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 11 calls |
| [tuning/follower/sysid/FeedforwardTuner.java](../../TeamCode/src/main/java/tuning/follower/sysid/FeedforwardTuner.java) | Joel H: specified WPILib-style SysId, separate drive/heading phases, and result graphs. | [`12ebdeb9`](https://github.com/DrPixelCat/ApexPathing/commit/12ebdeb97cdb61cfb584b83f0c304f604a7f5523) (2026-09-11); substantive accounts: Joel H | Recorded: 144 calls |
| [tuning/follower/sysid/ManualFeedforwardProfile.java](../../TeamCode/src/main/java/tuning/follower/sysid/ManualFeedforwardProfile.java) | Individual human implementation author unverified. | [`6af3de3d`](https://github.com/DrPixelCat/ApexPathing/commit/6af3de3d808596a675055937a555772cb5445671) (2026-10-05); substantive accounts: Joel H | Recorded: 3 calls |
| [tuning/follower/sysid/SysIdRegression.java](../../TeamCode/src/main/java/tuning/follower/sysid/SysIdRegression.java) | Individual human implementation author unverified. | [`812e284c`](https://github.com/DrPixelCat/ApexPathing/commit/812e284c8b4cf7f34837eb5fc77f52222b97ce7b) (2026-09-23); substantive accounts: Joel H | Recorded: 12 calls |
| [tuning/follower/tracking/ProfiledTrackingOptimizer.java](../../TeamCode/src/main/java/tuning/follower/tracking/ProfiledTrackingOptimizer.java) | Individual human implementation author unverified. | [`a1806769`](https://github.com/DrPixelCat/ApexPathing/commit/a1806769bcb710c7c6eb3c6cba239bb795af9439) (2026-10-02); substantive accounts: Joel H | Recorded: 12 calls |
| [tuning/follower/tracking/ProfiledTrackingPhase.java](../../TeamCode/src/main/java/tuning/follower/tracking/ProfiledTrackingPhase.java) | Joel H: directed automatic braking-tuning workflow and signed-velocity-error evaluation. | [`a1806769`](https://github.com/DrPixelCat/ApexPathing/commit/a1806769bcb710c7c6eb3c6cba239bb795af9439) (2026-10-02); substantive accounts: Joel H | Recorded: 33 calls |
| [tuning/follower/TunerContext.java](../../TeamCode/src/main/java/tuning/follower/TunerContext.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks; Dylan B. - 18597 RoboClovers - Delta.  | [`66b7edfb`](https://github.com/DrPixelCat/ApexPathing/commit/66b7edfbcff8e8c288d7eda685263e8fabd363b6) (2026-07-07); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 2 calls |
| [tuning/follower/TuningMeasurements.java](../../TeamCode/src/main/java/tuning/follower/TuningMeasurements.java) | Individual human implementation author unverified. | [`5232d34c`](https://github.com/DrPixelCat/ApexPathing/commit/5232d34cb58f24a7d5220862639e5825a3dcfe28) (2026-10-03); substantive accounts: Joel H | Recorded: 1 calls |
| [tuning/follower/TuningPhase.java](../../TeamCode/src/main/java/tuning/follower/TuningPhase.java) | Original non-Joel tags: Sohum Arora - 22985 Paraducks; Dylan B. - 18597 RoboClovers - Delta.  | [`66b7edfb`](https://github.com/DrPixelCat/ApexPathing/commit/66b7edfbcff8e8c288d7eda685263e8fabd363b6) (2026-07-07); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 5 calls |
| [tuning/localizer/calibration/CalibrationCandidate.java](../../TeamCode/src/main/java/tuning/localizer/calibration/CalibrationCandidate.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 2 calls |
| [tuning/localizer/calibration/CalibrationSnapshot.java](../../TeamCode/src/main/java/tuning/localizer/calibration/CalibrationSnapshot.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 1 calls |
| [tuning/localizer/calibration/DirectionTrial.java](../../TeamCode/src/main/java/tuning/localizer/calibration/DirectionTrial.java) | Individual human implementation author unverified. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 1 calls |
| [tuning/localizer/calibration/SpinTrial.java](../../TeamCode/src/main/java/tuning/localizer/calibration/SpinTrial.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 2 calls |
| [tuning/localizer/LocalizationTunerContext.java](../../TeamCode/src/main/java/tuning/localizer/LocalizationTunerContext.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 7 calls |
| [tuning/localizer/LocalizerAdapter.java](../../TeamCode/src/main/java/tuning/localizer/LocalizerAdapter.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 3 calls |
| [tuning/localizer/LocalizerAdapters.java](../../TeamCode/src/main/java/tuning/localizer/LocalizerAdapters.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 12 calls |
| [tuning/localizer/phases/DirectionsPhase.java](../../TeamCode/src/main/java/tuning/localizer/phases/DirectionsPhase.java) | Individual human implementation author unverified. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 6 calls |
| [tuning/localizer/phases/DistancePhase.java](../../TeamCode/src/main/java/tuning/localizer/phases/DistancePhase.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 6 calls |
| [tuning/localizer/phases/FilterMetrics.java](../../TeamCode/src/main/java/tuning/localizer/phases/FilterMetrics.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 3 calls |
| [tuning/localizer/phases/FilterPhase.java](../../TeamCode/src/main/java/tuning/localizer/phases/FilterPhase.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 10 calls |
| [tuning/localizer/phases/RelativeImuAccumulator.java](../../TeamCode/src/main/java/tuning/localizer/phases/RelativeImuAccumulator.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 3 calls |
| [tuning/localizer/phases/SpinPhase.java](../../TeamCode/src/main/java/tuning/localizer/phases/SpinPhase.java) | Individual human implementation author unverified. | [`016b397e`](https://github.com/DrPixelCat/ApexPathing/commit/016b397e5ac35fc26ce54fcad538e9fd56c3bd01) (2026-09-12); substantive accounts: Joel H | Recorded: 23 calls |
| [tuning/localizer/TuningPhase.java](../../TeamCode/src/main/java/tuning/localizer/TuningPhase.java) | Individual human implementation author unverified. | [`f0dc87ad`](https://github.com/DrPixelCat/ApexPathing/commit/f0dc87ad5856d844b314c4a07a043fa3a6aac7b3) (2026-09-10); substantive accounts: Joel H | Recorded: 11 calls |
| [tuning/TunerContext.java](../../TeamCode/src/main/java/tuning/TunerContext.java) | Individual human implementation author unverified. | [`66b7edfb`](https://github.com/DrPixelCat/ApexPathing/commit/66b7edfbcff8e8c288d7eda685263e8fabd363b6) (2026-07-07); substantive accounts: Dylan B., Joel H, Sohum Arora | Recorded: 12 calls |
| [tuning/TuningCsvWriter.java](../../TeamCode/src/main/java/tuning/TuningCsvWriter.java) | Individual human implementation author unverified. | [`80173bff`](https://github.com/DrPixelCat/ApexPathing/commit/80173bffad23d5573b680f5ba0be86c1bd78ba35) (2026-08-12); substantive accounts: Joel H | Recorded: 4 calls |
| [util/DeltaTime.java](../../TeamCode/src/main/java/util/DeltaTime.java) | Joel H: specified a shared nanosecond delta-time helper and its direct timestamp API. | Uncommitted new file | Recorded: 1 calls |

## Direct evidence for scoped Joel H credits

### controllers/FeedbackController.java

- Joel H specified the fluent component API, static-term behavior, and coefficient validation; session `01a0a04a-101c-7fe2-800d-1a6cea465755`, user-message lines 130, 157, 171, 183, 258.
- Joel H requested integral control with anti-windup; session `01a121ec-7a84-74a2-b713-42c5966f26fb`, user-message lines 12, 49.

### controllers/TurnController.java

- Joel H directed displacement-profile turn behavior and the quick-turn versus stored-profile distinction; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 1510.

### drivetrains/TranslationAxis.java

- Joel H directed calibration-axis ownership and placement of the shared translation axis near the drivetrain; session `01a1178a-eb65-7b12-baae-f19d8f695a87`, user-message lines 12, 102, 233, 245.

### follower/Follower.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.
- Joel H specified public slowed/stuck status, a dynamic stuck timeout, and more tolerant endpoint completion; session `01a082e4-14e9-7da0-81ea-185375f9201c`, user-message lines 9, 123.

### follower/FollowerDiagnostics.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.

### follower/FollowerStuckWatchdog.java

- Joel H specified public slowed/stuck status, a dynamic stuck timeout, and more tolerant endpoint completion; session `01a082e4-14e9-7da0-81ea-185375f9201c`, user-message lines 9, 123.

### follower/HolonomicPathFollower.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.
- Joel H directed Road Runner-inspired braking behavior and comparative validation; session `01a0f827-dfbd-7310-a314-27dee99100b8`, user-message lines 12, 204, 216, 584.

### follower/MotionFeedforward.java

- Joel H directed Road Runner-inspired braking behavior and comparative validation; session `01a0f827-dfbd-7310-a314-27dee99100b8`, user-message lines 12, 204, 216, 584.

### follower/PathCompletionPolicy.java

- Joel H specified public slowed/stuck status, a dynamic stuck timeout, and more tolerant endpoint completion; session `01a082e4-14e9-7da0-81ea-185375f9201c`, user-message lines 9, 123.

### follower/TankPathFollower.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.
- Joel H directed Road Runner-inspired braking behavior and comparative validation; session `01a0f827-dfbd-7310-a314-27dee99100b8`, user-message lines 12, 204, 216, 584.

### follower/TurnExecutor.java

- Joel H directed displacement-profile turn behavior and the quick-turn versus stored-profile distinction; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 1510.

### geometry/Dist.java

- Joel H specified a unitless Scalar API based on Dist and shared distance arithmetic; session `01a0a04a-101c-7fe2-800d-1a6cea465755`, user-message lines 9.

### geometry/Scalar.java

- Joel H specified a unitless Scalar API based on Dist and shared distance arithmetic; session `01a0a04a-101c-7fe2-800d-1a6cea465755`, user-message lines 9.

### localizers/BaseLocalizer.java

- Joel H directed shared Kalman localization, automatic tuning, and separation of filtering from pose estimation; session `01a101f0-fed2-7fe2-9c8f-70cb84584776`, user-message lines 12, 134, 155, 690, 709, 739.
- Joel H approved default Kalman velocity integration and filter lifecycle requirements; session `019ffe9f-c203-7f92-a5e2-43fa3e7c2de6`, user-message lines 701.

### localizers/OdometryPod.java

- Joel H directed calibration-axis ownership and placement of the shared translation axis near the drivetrain; session `01a1178a-eb65-7b12-baae-f19d8f695a87`, user-message lines 12, 102, 233, 245.

### localizers/TankDriveEncoders.java

- Joel H requested a separate localization tuner and a multi-motor tank-encoder/IMU localizer; session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, user-message lines 49, 68, 90, 168, 209.

### localizers/util/AdaptiveKalmanFilter.java

- Joel H approved default Kalman velocity integration and filter lifecycle requirements; session `019ffe9f-c203-7f92-a5e2-43fa3e7c2de6`, user-message lines 701.

### localizers/util/DataFilter.java

- Joel H directed the timestamp-update API and use of the shared delta-time helper; session `01a122d7-636c-7ab3-a6bb-78b6d2e275c7`, user-message lines 12, 34.

### localizers/util/KalmanFilter.java

- Joel H directed shared Kalman localization, automatic tuning, and separation of filtering from pose estimation; session `01a101f0-fed2-7fe2-9c8f-70cb84584776`, user-message lines 12, 134, 155, 690, 709, 739.

### localizers/util/PoseEstimator.java

- Joel H directed shared Kalman localization, automatic tuning, and separation of filtering from pose estimation; session `01a101f0-fed2-7fe2-9c8f-70cb84584776`, user-message lines 12, 134, 155, 690, 709, 739.

### org/firstinspires/ftc/teamcode/apexpathing/LocalizationTuner.java

- Joel H requested a separate localization tuner and a multi-motor tank-encoder/IMU localizer; session `01a0870f-ec8a-7a81-b49a-f00a8b1cc9b7`, user-message lines 49, 68, 90, 168, 209.

### paths/builders/HolonomicPathBuilder.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.

### paths/builders/PathBuilder.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.

### paths/builders/TankPathBuilder.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.

### paths/builders/TurnBuilder.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.

### paths/heading/HolonomicInterpolator.java

- Joel H specified FACING_POINT interpolation and its angular-offset behavior; session `019f48d7-b199-75c2-9422-3cfb88a0dac3`, user-message lines 6, 206, 335.

### paths/movements/Path.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.

### paths/movements/Turn.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.

### trajectory/generators/BaseProfileGenerator.java

- Joel H specified power-constrained profile optimization requirements and drivetrain-specific validation; session `019f2428-2020-7842-902c-743f4bd95173`, user-message lines 6, 235, 495.

### trajectory/generators/MecanumProfileGenerator.java

- Joel H specified power-constrained profile optimization requirements and drivetrain-specific validation; session `019f2428-2020-7842-902c-743f4bd95173`, user-message lines 6, 235, 495.

### trajectory/generators/SwerveProfileGenerator.java

- Joel H specified power-constrained profile optimization requirements and drivetrain-specific validation; session `019f2428-2020-7842-902c-743f4bd95173`, user-message lines 6, 235, 495.

### trajectory/generators/TankProfileGenerator.java

- Joel H specified power-constrained profile optimization requirements and drivetrain-specific validation; session `019f2428-2020-7842-902c-743f4bd95173`, user-message lines 6, 235, 495.

### trajectory/generators/TurnProfileGenerator.java

- Joel H directed displacement-profile turn behavior and the quick-turn versus stored-profile distinction; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 1510.

### trajectory/OnlineTrajectoryPlanner.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.

### trajectory/StoppingHorizon.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.

### trajectory/TrajectoryField2D.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.

### trajectory/TrajectorySample.java

- Joel H specified online lookahead versus stored trajectory behavior and shared vector-reference requirements; session `01a0fe37-2ea7-7331-9e0c-fe675ad9cb01`, user-message lines 12, 232, 1435, 1510.

### tuning/follower/characterization/MotionCharacterizationPhase.java

- Joel H specified fewer alternating trials and moving validation into dedicated tests; session `01a116fe-1282-7610-906a-7aac27ed0ad6`, user-message lines 42.

### tuning/follower/feedback/PDSRoutine.java

- Joel H specified iterative PD-tuning requirements and subsequent workflow refinements; session `019ffd65-bddf-7582-94ef-e71ff4926458`, user-message lines 9, 441, 712.

### tuning/follower/sysid/FeedforwardTuner.java

- Joel H specified WPILib-style SysId, separate drive/heading phases, and result graphs; session `01a0c9ed-d6e3-7413-92f0-cecb70e88287`, user-message lines 9, 73.

### tuning/follower/tracking/ProfiledTrackingPhase.java

- Joel H directed automatic braking-tuning workflow and signed-velocity-error evaluation; session `01a0e42c-e6bc-7340-ae3b-2aa4be2362a7`, user-message lines 9, 815, 986.

### util/DeltaTime.java

- Joel H specified a shared nanosecond delta-time helper and its direct timestamp API; session `01a121ca-3fbc-7b30-8e53-0024f074d1ea`, user-message lines 12, 160.
- Joel H specified the minimum returned interval and simpler error handling; session `01a122d7-636c-7ab3-a6bb-78b6d2e275c7`, user-message lines 95.
- Joel H requested simpler delta-time error handling; session `01a12402-b1b5-7462-b3e4-6b9d83186baa`, user-message lines 12.

## Bundled FTC SDK and sample provenance

These sources came with the SDK; the repository importer is not credited as their author. The original legal notices remain intact. Notice names below identify copyright provenance and must not be read as a verified list of individual implementation authors.

| File | Retained upstream notices |
| --- | --- |
| [org/firstinspires/ftc/robotcontroller/external/samples/BasicOmniOpMode_Linear.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/BasicOmniOpMode_Linear.java) | Copyright (c) 2021 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/BasicOpMode_Iterative.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/BasicOpMode_Iterative.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/BasicOpMode_Linear.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/BasicOpMode_Linear.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTag.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTag.java) | Copyright (c) 2023 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagEasy.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagEasy.java) | Copyright (c) 2023 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagLocalization.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagLocalization.java) | Copyright (c) 2024 Dryw Wade. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagMultiPortal.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagMultiPortal.java) | Copyright (c) 2024 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagOptimizeExposure.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagOptimizeExposure.java) | Copyright (c) 2023 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagSwitchableCameras.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagSwitchableCameras.java) | Copyright (c) 2023 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptBlackboard.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptBlackboard.java) | Copyright (c) 2025 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptExploringIMUOrientation.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptExploringIMUOrientation.java) | Copyright (c) 2022 REV Robotics, FIRST |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptGamepadEdgeDetection.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptGamepadEdgeDetection.java) | Copyright (c) 2024 Miriam Sinton-Remes |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptGamepadRumble.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptGamepadRumble.java) | No individual copyright/author notice in this file; FTC SDK origin retained. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptGamepadTouchpad.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptGamepadTouchpad.java) | No individual copyright/author notice in this file; FTC SDK origin retained. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptLEDStick.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptLEDStick.java) | Copyright (c) 2021-24 Alan Smith |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java) | Copyright (c) 2019 Phil Malone. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptNullOp.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptNullOp.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptRampMotorSpeed.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptRampMotorSpeed.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptRevLED.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptRevLED.java) | Copyright (c) 2024 Alan Smith |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptRevSPARKMini.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptRevSPARKMini.java) | Copyright (c) 2018 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptScanServo.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptScanServo.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptSoundsASJava.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptSoundsASJava.java) | Copyright (c) 2018 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptSoundsOnBotJava.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptSoundsOnBotJava.java) | Copyright (c) 2018 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptSoundsSKYSTONE.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptSoundsSKYSTONE.java) | Copyright (c) 2018 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptTelemetry.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptTelemetry.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptVisionColorLocator_Circle.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptVisionColorLocator_Circle.java) | Copyright (c) 2024 Phil Malone |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptVisionColorLocator_Rectangle.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptVisionColorLocator_Rectangle.java) | Copyright (c) 2024 Phil Malone |
| [org/firstinspires/ftc/robotcontroller/external/samples/ConceptVisionColorSensor.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptVisionColorSensor.java) | Copyright (c) 2024 Phil Malone |
| [org/firstinspires/ftc/robotcontroller/external/samples/externalhardware/ConceptExternalHardwareClass.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/externalhardware/ConceptExternalHardwareClass.java) | Copyright (c) 2022 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/externalhardware/RobotHardware.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/externalhardware/RobotHardware.java) | Copyright (c) 2022 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveByEncoder_Linear.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveByEncoder_Linear.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveByGyro_Linear.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveByGyro_Linear.java) | Copyright (c) 2022 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveByTime_Linear.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveByTime_Linear.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveToAprilTagOmni.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveToAprilTagOmni.java) | Copyright (c) 2023 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveToAprilTagTank.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveToAprilTagTank.java) | Copyright (c) 2023 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveToLine_Linear.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveToLine_Linear.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/RobotTeleopMecanumFieldRelativeDrive.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotTeleopMecanumFieldRelativeDrive.java) | Copyright (c) 2025 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/RobotTeleopPOV_Linear.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotTeleopPOV_Linear.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/RobotTeleopTank_Iterative.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotTeleopTank_Iterative.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SampleRevBlinkinLedDriver.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SampleRevBlinkinLedDriver.java) | Copyright (c) 2018 Craig MacFarlane |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorAndyMarkIMUNonOrthogonal.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorAndyMarkIMUNonOrthogonal.java) | Copyright (c) 2025 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorAndyMarkIMUOrthogonal.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorAndyMarkIMUOrthogonal.java) | Copyright (c) 2025 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorAndyMarkTOF.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorAndyMarkTOF.java) | Copyright (c) 2025 FIRST |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorBNO055IMU.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorBNO055IMU.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorBNO055IMUCalibration.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorBNO055IMUCalibration.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorColor.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorColor.java) | Copyright (c) 2017-2020 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorDigitalTouch.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorDigitalTouch.java) | Copyright (c) 2024 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorGoBildaPinpoint.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorGoBildaPinpoint.java) | Copyright (c) 2025 Alan Smith |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorHuskyLens.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorHuskyLens.java) | Copyright (c) 2023 FIRST |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorIMUNonOrthogonal.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorIMUNonOrthogonal.java) | Copyright (c) 2022 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorIMUOrthogonal.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorIMUOrthogonal.java) | Copyright (c) 2022 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorKLNavxMicro.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorKLNavxMicro.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorLimelight3A.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorLimelight3A.java) | Copyright (c) 2024 Limelight Vision |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorMRColor.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorMRColor.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorMRGyro.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorMRGyro.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorMROpticalDistance.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorMROpticalDistance.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorMRRangeSensor.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorMRRangeSensor.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorOctoQuad.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorOctoQuad.java) | Copyright (c) 2025 DigitalChickenLabs |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorOctoQuadAdv.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorOctoQuadAdv.java) | Copyright (c) 2024 DigitalChickenLabs |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorOctoQuadLocalization.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorOctoQuadLocalization.java) | Copyright (c) 2025 DigitalChickenLabs |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorREV2mDistance.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorREV2mDistance.java) | Copyright (c) 2018 FIRST |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorSparkFunOTOS.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorSparkFunOTOS.java) | Copyright (c) 2024 SparkFun Electronics |
| [org/firstinspires/ftc/robotcontroller/external/samples/SensorTouch.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorTouch.java) | Copyright (c) 2017 FIRST. All rights reserved. |
| [org/firstinspires/ftc/robotcontroller/external/samples/UtilityCameraFrameCapture.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/UtilityCameraFrameCapture.java) | Copyright (c) 2023 FIRST |
| [org/firstinspires/ftc/robotcontroller/external/samples/UtilityOctoQuadConfigMenu.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/UtilityOctoQuadConfigMenu.java) | Copyright (c) 2024 DigitalChickenLabs; Copyright (c) 2023 OpenFTC Team |
| [org/firstinspires/ftc/robotcontroller/internal/FtcOpModeRegister.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/internal/FtcOpModeRegister.java) | Copyright (c) 2014, 2015 Qualcomm Technologies Inc |
| [org/firstinspires/ftc/robotcontroller/internal/FtcRobotControllerActivity.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/internal/FtcRobotControllerActivity.java) | Copyright (c) 2014, 2015 Qualcomm Technologies Inc |
| [org/firstinspires/ftc/robotcontroller/internal/PermissionValidatorWrapper.java](../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/internal/PermissionValidatorWrapper.java) | Copyright (c) 2018 Craig MacFarlane |

## Remaining provenance gaps

- Exact hand-written versus AI-produced portions cannot be reconstructed for every class. The ledger leaves these unknown rather than guessing from code style or commit size.
- Octoquad.Driver resembles the Digital Chicken Labs driver family, but an exact donor revision and copied portions were not established. This remains an upstream-provenance question. The manufacturer’s [source repository](https://github.com/DigitalChickenLabs/OctoQuad) establishes the project family, not an exact code-copy match.

## Maintaining the audit

Edit `credits.json` only when new evidence supports a changed contribution. Keep role descriptions specific. Preserve unrelated original author tags. Use independent Git contribution evidence for other people; a tag in a Joel-only new-class commit does not establish their contribution. Record AI changes and direct guidance separately; approval, review, and ownership do not by themselves prove hand-written implementation.

The one-time collection, JavaDoc editing, rendering, and validation scripts used for this completed audit are [preserved in Git history](https://github.com/DrPixelCat/ApexPathing/tree/26d84367d42c914adac61e97f9672aafdfbce199/tools). They are not needed to build, test, or run Apex. Maintain the reviewed ledger and reports directly; preserve the original snapshots when comparing against this audit’s validation.

Generated evidence snapshots are compressed losslessly and can be read with standard gzip tools. The editable credit ledger, reports, and validation results remain plain text. See [snapshot storage](SNAPSHOTS.md) for details.
