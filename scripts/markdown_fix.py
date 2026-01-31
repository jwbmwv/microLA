#!/usr/bin/env python3
import sys
from pathlib import Path

ROOT = Path('.').resolve()
EXCLUDE_DIRS = {'.git', 'node_modules', 'build', 'build-std11', 'build-std14', 'build-std17', 'build-std20'}

def should_skip(path: Path):
    for part in path.parts:
        if part in EXCLUDE_DIRS:
            return True
    return False


def fix_file(path: Path):
    changed = False
    text = path.read_text(encoding='utf-8')
    lines = text.splitlines()
    out = []
    i = 0
    n = len(lines)
    while i < n:
        line = lines[i]
        # remove trailing spaces
        new_line = line.rstrip()
        # ensure blank line before headings
        if new_line.lstrip().startswith('#'):
            if out and out[-1].strip() != '':
                out.append('')
                changed = True
        # ensure blank line before fenced code block
        if new_line.startswith('```'):
            if out and out[-1].strip() != '':
                out.append('')
                changed = True
            out.append(new_line)
            i += 1
            # copy until closing fence
            while i < n:
                l = lines[i].rstrip()
                out.append(l)
                if l.startswith('```'):
                    # ensure blank line after closing fence
                    if i+1 < n and lines[i+1].strip() != '':
                        out.append('')
                        changed = True
                    i += 1
                    break
                i += 1
            continue
        # handle headings: ensure blank line after heading
        if new_line.lstrip().startswith('#'):
            out.append(new_line)
            if i+1 < n and lines[i+1].strip() != '':
                out.append('')
                changed = True
            i += 1
            continue
        out.append(new_line)
        if new_line != line:
            changed = True
        i += 1
    new_text = '\n'.join(out) + ('\n' if out and not out[-1].endswith('\n') else '')
    if changed:
        path.write_text(new_text, encoding='utf-8')
    return changed


def main():
    md_files = list(ROOT.rglob('*.md'))
    modified = []
    for p in md_files:
        if should_skip(p):
            continue
        try:
            if fix_file(p):
                modified.append(str(p))
        except Exception as e:
            print(f'Error processing {p}: {e}', file=sys.stderr)
    if modified:
        print('Modified files:')
        for m in modified:
            print(m)
    else:
        print('No files modified')

if __name__ == '__main__':
    main()
