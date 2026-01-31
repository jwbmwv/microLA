#!/usr/bin/env bash
set -euo pipefail
NODE=/home/jbaldwin/.nvm/versions/node/v24.13.0/bin/node
ML_SCRIPT=/home/jbaldwin/.nvm/versions/node/v24.13.0/lib/node_modules/markdownlint-cli2/markdownlint-cli2.mjs

echo "Running markdownlint --fix on tracked .md files..."
git ls-files -z -- '*.md' | while IFS= read -r -d '' file; do
  echo "Fixing: $file"
  "$NODE" "$ML_SCRIPT" --config .markdownlint.json --fix "$file" || true
done

echo "Regenerating report..."
./scripts/run_markdownlint2.sh

echo "Modified files (git status --porcelain):"
git status --porcelain
