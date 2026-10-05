$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$reviewGradle = Join-Path $repoRoot 'gradlew.bat'
$reviewArguments = @(':TeamCode:testDebugUnitTest', '--console=plain')
$reviewPatterns = @('geometry.*', 'controllers.*', 'paths.*', 'trajectory.*',
    'follower.*', 'localizers.*', 'feedforward.*', 'tuning.*', 'storage.*',
    'drivetrains.*', '*FollowerReviewRegressionTest', '*DualActuatedProfilesTest',
    '*CentripetalVectorSimulationTest',
    '*FollowerTunerTelemetryTest.stopDiscardsUnacceptedManualEdits',
    'org.firstinspires.ftc.teamcode.apexpathing.*')
foreach ($reviewPattern in $reviewPatterns) {
    $reviewArguments += @('--tests', $reviewPattern)
}
Push-Location $repoRoot
try {
    & $reviewGradle @reviewArguments
    if ($LASTEXITCODE -ne 0) { throw 'Java review regressions failed.' }
} finally {
    Pop-Location
}
