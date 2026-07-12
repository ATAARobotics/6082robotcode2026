#!/usr/bin/env bash
# Install git hooks (delegates to the Gradle installGitHooks task).
set -euo pipefail
REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"
exec ./gradlew installGitHooks "$@"