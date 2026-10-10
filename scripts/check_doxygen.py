#!/usr/bin/env python3
"""Check function and template documentation against Doxygen XML signatures."""

import argparse
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


def text(element: ET.Element, path: str) -> str:
    """Return the flattened text of a child, or an empty string if absent.

    Args:
        element: XML element containing the child.
        path: ElementTree path identifying the child.

    Returns:
        Whitespace-trimmed text, including nested references.
    """
    child = element.find(path)
    return "".join(child.itertext()).strip() if child is not None else ""


def template_names(element: ET.Element) -> list[str]:
    """Extract named template parameters, excluding empty specialization slots.

    Args:
        element: Doxygen declaration or compound element.

    Returns:
        Names of type, non-type, and parameter-pack template parameters.
    """
    names = []
    for parameter in element.findall("./templateparamlist/param"):
        name = text(parameter, "declname") or text(parameter, "defname")
        if not name:
            tokens = text(parameter, "type").split()
            candidate = tokens[-1].strip(".") if len(tokens) > 1 else ""
            unnamed_types = {"class", "typename", "bool", "char", "int", "long", "short",
                             "signed", "unsigned", "float", "double"}
            if re.fullmatch(r"[A-Za-z_]\w*", candidate) and candidate not in unnamed_types:
                name = candidate
        if name:
            names.append(name)
    return names


def return_type(element: ET.Element) -> str:
    """Identify a function's return type without inventing constructor returns.

    Args:
        element: Doxygen member declaration.

    Returns:
        Return type text, or an empty string for constructors and destructors.
    """
    name = text(element, "name")
    qualified = text(element, "qualifiedname")
    owner = qualified.rsplit("::", 1)[0].rsplit("::", 1)[-1].split("<", 1)[0]
    if name.startswith("~") or name == owner:
        return ""
    arguments = text(element, "argsstring")
    if "->" in arguments:
        result = arguments.rsplit("->", 1)[1]
    else:
        result = text(element, "type")
    result = re.split(r"\brequires\b", result, maxsplit=1)[0].strip()
    result = re.sub(r"\b(?:constexpr|consteval|static|inline|explicit)\b", "", result).strip()
    if not result and name.startswith("operator "):
        result = name.removeprefix("operator ").strip()
    return result


def documented_names(element: ET.Element, kind: str) -> set[str]:
    """Collect names with nonempty parameter descriptions of the specified kind.

    Args:
        element: Declaration whose detailed documentation is inspected.
        kind: Doxygen parameter-list kind: param, templateparam, or retval.

    Returns:
        Names associated with nonempty descriptions.
    """
    names = set()
    for item in element.findall(
        f"./detaileddescription//parameterlist[@kind='{kind}']/parameteritem"
    ):
        if text(item, "parameterdescription"):
            names.update(
                name.text.strip()
                for name in item.findall("./parameternamelist/parametername")
                if name.text
            )
    return names


def missing_documentation(element: ET.Element) -> list[str]:
    """Compare a function or template signature with its own documentation.

    Args:
        element: Doxygen function, template alias, variable, or compound.

    Returns:
        Missing brief, template, parameter, return, and Boolean-outcome entries.
    """
    missing = []
    if not text(element, "briefdescription"):
        missing.append("brief")
    documented_templates = documented_names(element, "templateparam")
    missing.extend(
        f"tparam:{name}" for name in template_names(element) if name not in documented_templates
    )
    if element.get("kind") != "function":
        return missing
    documented_parameters = documented_names(element, "param")
    for parameter in element.findall("./param"):
        name = text(parameter, "declname") or text(parameter, "defname")
        if name and name not in documented_parameters:
            missing.append(f"param:{name}")
    result = return_type(element)
    if result and result != "void":
        return_sections = element.findall("./detaileddescription//simplesect[@kind='return']")
        has_return = any("".join(section.itertext()).strip() for section in return_sections)
        if not has_return and not documented_names(element, "retval"):
            missing.append("return")
    if result == "bool":
        outcomes = documented_names(element, "retval")
        missing.extend(f"retval:{value}" for value in ("true", "false") if value not in outcomes)
    return missing


def declarations(xml_directory: Path) -> list[ET.Element]:
    """Collect unique library functions and templates from Doxygen's XML files.

    Args:
        xml_directory: Directory containing Doxygen compound XML files.

    Returns:
        Declarations located under include/microla, including private helpers.
    """
    result = []
    seen = set()
    for path in sorted(xml_directory.glob("*.xml")):
        root = ET.parse(path).getroot()
        for element in root.findall("./compounddef") + root.findall(".//memberdef"):
            identifier = element.get("id")
            location = element.find("location")
            if identifier in seen or location is None:
                continue
            seen.add(identifier)
            source = location.get("file", "").replace("\\", "/")
            if not (source.startswith(("include/microla/", "microla/")) or "/include/microla/" in source):
                continue
            if element.get("kind") == "function" or template_names(element):
                result.append(element)
    return result


def check_xml(xml_directory: Path) -> int:
    """Report documentation gaps in one generated library inventory.

    Args:
        xml_directory: Doxygen XML output directory.

    Returns:
        Zero for complete documentation, one for gaps or an empty inventory.
    """
    entries = declarations(xml_directory)
    if not entries:
        print("No library functions or templates found in the XML inventory.", file=sys.stderr)
        return 1
    failures = 0
    for entry in entries:
        missing = missing_documentation(entry)
        if missing:
            location = entry.find("location")
            assert location is not None
            name = text(entry, "qualifiedname") or text(entry, "compoundname") or text(entry, "name")
            print(f"{location.get('file')}:{location.get('line')}: {name}: {', '.join(missing)}")
            failures += 1
    print(f"Doxygen coverage: {len(entries)} declarations checked; {failures} incomplete.")
    return int(failures != 0)


def generate_and_check() -> int:
    """Generate fresh inventories for hosted, safe/NEON, and embedded branches.

    Returns:
        Zero when all configurations have complete signature documentation,
        otherwise one, including when Doxygen cannot run successfully.
    """
    repository = Path(__file__).resolve().parents[1]
    configurations = {
        "hosted": "MICROLA_HAS_DYNAMIC_ALLOC=1",
        "safe-neon": "MICROLA_SAFE_MODE=1 CONFIG_MICROLA_NEON=1 MICROLA_HAS_DYNAMIC_ALLOC=1",
        "embedded": "MICROLA_EMBEDDED=1 MICROLA_HAS_DYNAMIC_ALLOC=0",
    }
    failed = 0
    with tempfile.TemporaryDirectory(prefix="microla-doxygen-") as directory:
        for name, definitions in configurations.items():
            output = Path(directory) / name
            configuration = f'''
@INCLUDE = "{repository / 'Doxyfile'}"
INPUT = "{repository / 'include/microla'}"
OUTPUT_DIRECTORY = "{output}"
RECURSIVE = YES
FILE_PATTERNS = *.hpp
GENERATE_HTML = NO
GENERATE_LATEX = NO
GENERATE_XML = YES
EXTRACT_ALL = YES
EXTRACT_PRIVATE = YES
EXTRACT_STATIC = YES
QUIET = YES
WARN_LOGFILE = "{output / 'warnings.log'}"
PREDEFINED += {definitions} "MICROLA_DYNAMIC_ALLOC_ONLY(...)=__VA_ARGS__"
'''
            print(f"Checking {name} documentation:", flush=True)
            try:
                subprocess.run(
                    ["doxygen", "-"], input=configuration, text=True, cwd=repository, check=True
                )
            except (OSError, subprocess.CalledProcessError) as error:
                print(f"Doxygen failed: {error}", file=sys.stderr)
                return 1
            failed |= check_xml(output / "xml")
    return failed


def main() -> int:
    """Run the command-line coverage check or generate fresh inventories.

    Returns:
        Zero for complete documentation, one for failed coverage or generation.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xml_directory", nargs="?", type=Path, help="Doxygen XML output directory")
    parser.add_argument("--generate", action="store_true", help="Generate and check all library profiles")
    arguments = parser.parse_args()
    if arguments.generate and arguments.xml_directory is not None:
        parser.error("choose --generate or an XML directory, not both")
    if arguments.generate:
        return generate_and_check()
    if arguments.xml_directory is None:
        parser.error("provide --generate or an XML directory")
    return check_xml(arguments.xml_directory)


if __name__ == "__main__":
    raise SystemExit(main())
