#!/usr/bin/env python3
"""
Extract and compile code snippets from markdown documentation.

This script finds C++ code blocks in markdown files and attempts to compile them
to ensure documentation stays in sync with the actual library API.
"""

import argparse
import re
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import List, Tuple, Optional

class CodeSnippet:
    """Represents a code snippet extracted from markdown."""

    def __init__(self, code: str, file: Path, line: int, lang: str):
        self.code = code
        self.file = file
        self.line = line
        self.lang = lang
        self.is_complete = self._check_completeness()

    def _check_completeness(self) -> bool:
        """Check if snippet is a complete, compilable program."""
        # Must have main() to be compilable
        if 'int main(' not in self.code and 'auto main(' not in self.code:
            return False
        # Should have includes
        if '#include' not in self.code:
            return False
        return True

    def __repr__(self):
        return f"CodeSnippet({self.file}:{self.line}, complete={self.is_complete})"


def extract_snippets_from_file(md_file: Path) -> List[CodeSnippet]:
    """Extract code snippets from a markdown file."""
    snippets = []

    with open(md_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Match fenced code blocks with optional language specifier
    # Pattern: ```cpp or ```c++ or ``` followed by code and closing ```
    pattern = r'```(?:cpp|c\+\+|\.cpp)?\s*\n(.*?)\n```'

    for match in re.finditer(pattern, content, re.DOTALL):
        code = match.group(1)

        # Determine line number
        line_num = content[:match.start()].count('\n') + 1

        # Only consider C++ code (heuristic: has #include or looks like C++)
        if '#include' in code or 'namespace' in code or '::' in code or 'std::' in code:
            snippet = CodeSnippet(code, md_file, line_num, 'cpp')
            snippets.append(snippet)

    return snippets


def create_test_wrapper(snippet: CodeSnippet, include_dir: Path) -> str:
    """Wrap incomplete snippet in a minimal test harness."""

    # Check if snippet already has includes and main
    if snippet.is_complete:
        return snippet.code

    # Create wrapper for incomplete snippets
    wrapper = f"""// Auto-generated test wrapper for snippet at {snippet.file}:{snippet.line}
#include <iostream>
#include <cmath>

// Original snippet:
{snippet.code}

// Minimal main to check compilation
int main() {{
    // Snippet wrapped in test harness
    return 0;
}}
"""
    return wrapper


def compile_snippet(snippet: CodeSnippet, include_dir: Path, compiler: str = 'g++',
                   std: str = 'c++20', verbose: bool = False) -> Tuple[bool, str]:
    """
    Compile a code snippet.

    Returns:
        Tuple of (success: bool, error_message: str)
    """
    # Skip snippets that are clearly not meant to compile (e.g., pseudocode)
    skip_patterns = [
        '...',           # Ellipsis indicating omitted code
        '/* ... */',     # Comment ellipsis
        '// ...',        # Comment ellipsis
        'Lines',         # "Lines X-Y omitted"
        'existing code', # Placeholder text
    ]

    for pattern in skip_patterns:
        if pattern in snippet.code:
            if verbose:
                print(f"  Skipping snippet with placeholder '{pattern}' at {snippet.file}:{snippet.line}")
            return True, f"Skipped (contains placeholder '{pattern}')"

    # Skip incomplete snippets that are obviously just API demonstrations
    # These are valuable documentation but not meant to be compiled standalone
    if not snippet.is_complete:
        # Check if it's a simple variable declaration/usage pattern
        lines = snippet.code.strip().split('\n')
        # If very short and no control flow, probably just an API demo
        if len(lines) < 15 and not any(kw in snippet.code for kw in ['for', 'while', 'if', 'class', 'struct']):
            if verbose:
                print(f"  Skipping short API demo snippet at {snippet.file}:{snippet.line}")
            return True, "Skipped (short API demo)"

    # Only compile complete snippets
    if not snippet.is_complete:
        if verbose:
            print(f"  Skipping incomplete snippet at {snippet.file}:{snippet.line}")
        return True, "Skipped (incomplete, no main())"

    # Create temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.cpp', delete=False) as f:
        temp_file = Path(f.name)
        f.write(snippet.code)

    try:
        # Compile command
        cmd = [
            compiler,
            f'-std={std}',
            f'-I{include_dir}',
            '-fsyntax-only',  # Only check syntax, don't generate executable
            '-Wall',
            '-Wextra',
            '-Werror=return-type',
            str(temp_file)
        ]

        if verbose:
            print(f"  Compiling: {' '.join(cmd)}")

        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=10
        )

        if result.returncode == 0:
            return True, "OK"
        else:
            error_msg = result.stderr or result.stdout
            return False, error_msg

    except subprocess.TimeoutExpired:
        return False, "Compilation timeout"
    except Exception as e:
        return False, f"Compilation error: {e}"
    finally:
        # Clean up temp file
        temp_file.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(
        description='Extract and compile code snippets from markdown documentation'
    )
    parser.add_argument(
        'markdown_files',
        nargs='*',
        help='Markdown files to check (default: all .md files in docs/ and root)'
    )
    parser.add_argument(
        '--include-dir',
        type=Path,
        default=Path('include'),
        help='Include directory for microLA headers (default: include/)'
    )
    parser.add_argument(
        '--compiler',
        default='g++',
        help='Compiler to use (default: g++)'
    )
    parser.add_argument(
        '--std',
        default='c++20',
        help='C++ standard (default: c++20)'
    )
    parser.add_argument(
        '--verbose',
        '-v',
        action='store_true',
        help='Verbose output'
    )
    parser.add_argument(
        '--fail-fast',
        action='store_true',
        help='Stop on first failure'
    )

    args = parser.parse_args()

    # Determine which markdown files to check
    if args.markdown_files:
        md_files = [Path(f) for f in args.markdown_files]
    else:
        # Default: check common documentation files
        root = Path('.')
        md_files = list(root.glob('*.md')) + list((root / 'docs').glob('*.md'))
        # Exclude some files that don't contain code
        exclude_patterns = ['CHANGELOG', 'LICENSE', 'NOTICE', 'CONTRIBUTING']
        md_files = [f for f in md_files if not any(excl in f.name for excl in exclude_patterns)]

    if not md_files:
        print("No markdown files found")
        return 1

    print(f"Checking {len(md_files)} markdown files...")
    print(f"Include directory: {args.include_dir}")
    print(f"Compiler: {args.compiler} -std={args.std}")
    print()

    total_snippets = 0
    failed_snippets = []
    skipped_snippets = 0

    for md_file in sorted(md_files):
        if not md_file.exists():
            print(f"Warning: {md_file} not found, skipping")
            continue

        snippets = extract_snippets_from_file(md_file)

        if not snippets:
            continue

        print(f"📄 {md_file}: {len(snippets)} snippet(s)")

        for i, snippet in enumerate(snippets, 1):
            total_snippets += 1

            if args.verbose:
                print(f"  [{i}/{len(snippets)}] Line {snippet.line} (complete={snippet.is_complete})")

            success, message = compile_snippet(
                snippet,
                args.include_dir,
                args.compiler,
                args.std,
                args.verbose
            )

            if "Skipped" in message:
                skipped_snippets += 1
                if args.verbose:
                    print(f"    ⊘ {message}")
            elif success:
                if args.verbose:
                    print(f"    ✓ {message}")
            else:
                print(f"  ✗ FAILED at {md_file}:{snippet.line}")
                print(f"    {message}")
                failed_snippets.append((snippet, message))

                if args.fail_fast:
                    return 1

        print()

    # Summary
    print("=" * 70)
    print(f"Total snippets checked: {total_snippets}")
    print(f"Passed: {total_snippets - len(failed_snippets) - skipped_snippets}")
    print(f"Skipped: {skipped_snippets}")
    print(f"Failed: {len(failed_snippets)}")

    if failed_snippets:
        print()
        print("Failed snippets:")
        for snippet, msg in failed_snippets:
            print(f"  - {snippet.file}:{snippet.line}")
        return 1

    print()
    print("✓ All code snippets compile successfully!")
    return 0


if __name__ == '__main__':
    sys.exit(main())
