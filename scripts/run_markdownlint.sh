#!/usr/bin/env bash
set -euo pipefail
NODE=/home/jbaldwin/.nvm/versions/node/v24.13.0/bin/node
ML_SCRIPT=/home/jbaldwin/.nvm/versions/node/v24.13.0/lib/node_modules/markdownlint-cli2/markdownlint-cli2.mjs
OUT=markdownlint_report.txt
: > "$OUT"
# List tracked markdown files to avoid generated/build files
git ls-files -z -- '*.md' | while IFS= read -r -d '' file; do
  printf '--- %s ---\n' "$file" >> "$OUT"
  "$NODE" "$ML_SCRIPT" --config .markdownlint.json "$file" >> "$OUT" 2>&1 || true
done
# Show first 200 rule occurrences (if any)
grep -nE 'MD[0-9]{3}' "$OUT" | sed -n '1,200p' || true
ls -la "$OUT"
