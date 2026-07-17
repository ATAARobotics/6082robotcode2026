parameter([Parameter(ValueFromRemainingArguments=$true)][string[]]$GradleArgs)
$ErrorActionPreference = 'Stop'
$RepoRoot = git rev-parse --show-toplevel
Set-Location -LiteralPath $RepoRoot
& ./gradlew installGitHooks @GradleArgs
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }