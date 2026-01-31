#!/usr/bin/env python3
"""Convert SVG files to PDF format."""

import subprocess
import sys
from pathlib import Path

def convert_svg_to_pdf_cairosvg(svg_file, pdf_file):
    """Convert SVG to PDF using cairosvg."""
    try:
        import cairosvg
        cairosvg.svg2pdf(url=str(svg_file), write_to=str(pdf_file))
        return True
    except ImportError:
        return False
    except Exception as e:
        print(f"Error with cairosvg: {e}")
        return False

def convert_svg_to_pdf_svglib(svg_file, pdf_file):
    """Convert SVG to PDF using svglib + reportlab."""
    try:
        from svglib.svglib import svg2rlg
        from reportlab.graphics import renderPDF
        drawing = svg2rlg(str(svg_file))
        if drawing:
            renderPDF.drawToFile(drawing, str(pdf_file))
            return True
        return False
    except ImportError:
        return False
    except Exception as e:
        print(f"Error with svglib: {e}")
        return False

def convert_svg_to_pdf_rsvg(svg_file, pdf_file):
    """Convert SVG to PDF using rsvg-convert command-line tool."""
    try:
        result = subprocess.run(
            ['rsvg-convert', '-f', 'pdf', '-o', str(pdf_file), str(svg_file)],
            capture_output=True,
            check=True
        )
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False

def convert_svg_to_pdf_inkscape(svg_file, pdf_file):
    """Convert SVG to PDF using Inkscape command-line."""
    try:
        result = subprocess.run(
            ['inkscape', str(svg_file), '--export-filename=' + str(pdf_file)],
            capture_output=True,
            check=True
        )
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False

def main():
    design_dir = Path(__file__).parent
    svg_files = list(design_dir.glob('*.svg'))

    if not svg_files:
        print("No SVG files found!")
        return 1

    # Try different conversion methods in order of preference
    converters = [
        ("cairosvg", convert_svg_to_pdf_cairosvg),
        ("svglib", convert_svg_to_pdf_svglib),
        ("rsvg-convert", convert_svg_to_pdf_rsvg),
        ("inkscape", convert_svg_to_pdf_inkscape),
    ]

    working_converter = None
    for name, converter in converters:
        test_svg = svg_files[0]
        test_pdf = test_svg.with_suffix('.pdf.test')
        if converter(test_svg, test_pdf):
            working_converter = (name, converter)
            test_pdf.unlink(missing_ok=True)
            print(f"Using {name} for conversion")
            break
        test_pdf.unlink(missing_ok=True)

    if not working_converter:
        print("ERROR: No PDF converter available!")
        print("Please install one of: cairosvg, svglib+reportlab, rsvg-convert, or inkscape")
        print("\nYou can install cairosvg with: pip install cairosvg")
        return 1

    name, converter = working_converter
    success_count = 0

    for svg_file in svg_files:
        pdf_file = svg_file.with_suffix('.pdf')
        print(f"Converting {svg_file.name} to {pdf_file.name}...", end=' ')

        if converter(svg_file, pdf_file):
            print("✓")
            success_count += 1
        else:
            print("✗")

    print(f"\nSuccessfully converted {success_count}/{len(svg_files)} files to PDF")
    return 0 if success_count == len(svg_files) else 1

if __name__ == '__main__':
    sys.exit(main())
