#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TARGET_DIRS=("$REPO_ROOT/design" "$REPO_ROOT/examples")
FORMATS=(png svg)

PLANTUML_CMD=()
if command -v plantuml >/dev/null 2>&1; then
  PLANTUML_CMD=(plantuml)
elif [[ -n "${PLANTUML_JAR:-}" && -f "${PLANTUML_JAR}" ]]; then
  PLANTUML_CMD=(java -jar "${PLANTUML_JAR}")
elif [[ -f /opt/plantuml/plantuml.jar ]]; then
  PLANTUML_CMD=(java -jar /opt/plantuml/plantuml.jar)
else
  echo "Error: PlantUML not found. Install the 'plantuml' command or set PLANTUML_JAR." >&2
  exit 1
fi

PDF_VIA_SVG=false
if command -v rsvg-convert >/dev/null 2>&1; then
  PDF_VIA_SVG=true
fi

echo "Rendering PlantUML diagrams under: ${TARGET_DIRS[*]}"
for dir in "${TARGET_DIRS[@]}"; do
  [[ -d "$dir" ]] || continue
  shopt -s nullglob
  for file in "$dir"/*.puml; do
    echo "  -> $file"
    for format in "${FORMATS[@]}"; do
      "${PLANTUML_CMD[@]}" "-t${format}" "$file"
    done

    pdf_file="${file%.puml}.pdf"
    if [[ "$PDF_VIA_SVG" == true ]]; then
      rsvg-convert -f pdf -o "$pdf_file" "${file%.puml}.svg"
    else
      "${PLANTUML_CMD[@]}" -tpdf "$file"
    fi

    if [[ ! -s "$pdf_file" ]]; then
      rm -f "$pdf_file"
      echo "Error: failed to render non-empty PDF for $file" >&2
      exit 1
    fi
  done
done

echo "Diagram rendering complete (png, svg, pdf)."
