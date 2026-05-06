#!/usr/bin/env python3

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
MKDOCS_PATH = REPO_ROOT / "mkdocs.yml"


@dataclass(frozen=True)
class Mapping:
    src_root: Path
    docs_root: Path
    nav_title: str | None
    begin_marker: str
    end_marker: str
    extra_docs: tuple[Path, ...] = ()


MAPPINGS = [
    Mapping(
        src_root=REPO_ROOT / "src/rtr/system/physics/ipc",
        docs_root=REPO_ROOT / "docs/documentation/system/physics/ipc",
        nav_title="IPC",
        begin_marker="# BEGIN AUTO-GENERATED PHYSICS NAV",
        end_marker="# END AUTO-GENERATED PHYSICS NAV",
        extra_docs=(
            REPO_ROOT / "docs/documentation/system/physics/ipc/model/mesh_to_tet.md",
            REPO_ROOT / "docs/documentation/system/physics/ipc/model/mesh_to_tet.zh.md",
            REPO_ROOT / "docs/documentation/system/physics/ipc/model/tet_to_mesh.md",
            REPO_ROOT / "docs/documentation/system/physics/ipc/model/tet_to_mesh.zh.md",
        ),
    ),
    Mapping(
        src_root=REPO_ROOT / "src/rtr/system/render",
        docs_root=REPO_ROOT / "docs/documentation/system/render",
        nav_title=None,
        begin_marker="# BEGIN AUTO-GENERATED RENDER NAV",
        end_marker="# END AUTO-GENERATED RENDER NAV",
        extra_docs=(
            REPO_ROOT / "docs/documentation/system/render/camera-coordinate-conventions.md",
            REPO_ROOT / "docs/documentation/system/render/camera-coordinate-conventions.zh.md",
        ),
    ),
    Mapping(
        src_root=REPO_ROOT / "src/rtr/editor/render",
        docs_root=REPO_ROOT / "docs/documentation/editor/render",
        nav_title=None,
        begin_marker="# BEGIN AUTO-GENERATED EDITOR RENDER NAV",
        end_marker="# END AUTO-GENERATED EDITOR RENDER NAV",
    ),
]

ACRONYMS = {
    "ipc": "IPC",
    "obj": "OBJ",
    "dof": "DOF",
}


def title_from_stem(name: str) -> str:
    parts = re.split(r"[_-]+", name)
    titled = []
    for part in parts:
        lower = part.lower()
        if lower in ACRONYMS:
            titled.append(ACRONYMS[lower])
        elif part:
            titled.append(part.capitalize())
    return " ".join(titled)


def relative_docs_path(path: Path) -> str:
    return path.relative_to(REPO_ROOT / "docs").as_posix()


def expected_docs(mapping: Mapping) -> set[Path]:
    expected: set[Path] = set()

    for src_dir in sorted(p for p in mapping.src_root.rglob("*") if p.is_dir()):
        rel = src_dir.relative_to(mapping.src_root)
        docs_dir = mapping.docs_root / rel
        expected.add(docs_dir / "overview.md")
        expected.add(docs_dir / "overview.zh.md")

    expected.add(mapping.docs_root / "overview.md")
    expected.add(mapping.docs_root / "overview.zh.md")

    for header in sorted(mapping.src_root.rglob("*.hpp")):
        rel = header.relative_to(mapping.src_root)
        docs_dir = mapping.docs_root / rel.parent
        expected.add(docs_dir / f"{header.stem}.md")
        expected.add(docs_dir / f"{header.stem}.zh.md")

    return expected


def validate_mapping(mapping: Mapping) -> list[str]:
    errors: list[str] = []
    expected = expected_docs(mapping)

    for doc in sorted(expected):
        if not doc.exists():
            errors.append(f"missing documentation file: {doc.relative_to(REPO_ROOT)}")

    actual = {
        path
        for path in mapping.docs_root.rglob("*.md")
        if path.is_file()
    }

    for doc in sorted(actual - expected - set(mapping.extra_docs)):
        errors.append(f"orphan documentation file: {doc.relative_to(REPO_ROOT)}")

    return errors


def build_dir_nav(src_dir: Path, docs_dir: Path, indent: int) -> list[str]:
    lines: list[str] = []
    indent_str = " " * indent
    lines.append(f"{indent_str}- Overview: {relative_docs_path(docs_dir / 'overview.md')}")

    child_dirs = sorted(
        [path for path in src_dir.iterdir() if path.is_dir()],
        key=lambda p: p.name,
    )
    child_files = sorted(
        [path for path in src_dir.iterdir() if path.is_file() and path.suffix == ".hpp"],
        key=lambda p: p.name,
    )

    for child_dir in child_dirs:
        lines.append(f"{indent_str}- {title_from_stem(child_dir.name)}:")
        lines.extend(build_dir_nav(child_dir, docs_dir / child_dir.name, indent + 4))

    for child_file in child_files:
        lines.append(
            f"{indent_str}- {title_from_stem(child_file.stem)}: "
            f"{relative_docs_path(docs_dir / f'{child_file.stem}.md')}"
        )

    return lines


def generate_nav_block(mapping: Mapping, anchor_indent: int) -> list[str]:
    if mapping.nav_title is None:
        return build_dir_nav(mapping.src_root, mapping.docs_root, anchor_indent)

    indent = " " * anchor_indent
    lines = [f"{indent}- {mapping.nav_title}:"]
    lines.extend(build_dir_nav(mapping.src_root, mapping.docs_root, anchor_indent + 4))
    return lines


def update_mkdocs(mapping: Mapping, check_only: bool) -> list[str]:
    text = MKDOCS_PATH.read_text(encoding="utf-8")
    begin_match = re.search(rf"^(?P<indent>\s*){re.escape(mapping.begin_marker)}\s*$", text, re.MULTILINE)
    end_match = re.search(rf"^(?P<indent>\s*){re.escape(mapping.end_marker)}\s*$", text, re.MULTILINE)
    errors: list[str] = []

    if begin_match is None or end_match is None:
        return [f"missing nav anchor for mapping {mapping.nav_title} in mkdocs.yml"]
    if begin_match.start() >= end_match.start():
        return [f"invalid nav anchor order for mapping {mapping.nav_title} in mkdocs.yml"]

    begin_indent = len(begin_match.group("indent"))
    end_indent = len(end_match.group("indent"))
    if begin_indent != end_indent:
        return [f"nav anchor indentation mismatch for mapping {mapping.nav_title}"]

    generated_lines = generate_nav_block(mapping, begin_indent)
    replacement = (
        text[: begin_match.end()]
        + "\n"
        + "\n".join(generated_lines)
        + "\n"
        + text[end_match.start() :]
    )

    if check_only:
        if replacement != text:
            errors.append("mkdocs.yml auto-generated navigation is out of date")
        return errors

    if replacement != text:
        MKDOCS_PATH.write_text(replacement, encoding="utf-8")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate mkdocs navigation from source tree mappings.")
    parser.add_argument("--check", action="store_true", help="Validate docs and nav without modifying files.")
    args = parser.parse_args()

    errors: list[str] = []
    for mapping in MAPPINGS:
        errors.extend(validate_mapping(mapping))
        errors.extend(update_mkdocs(mapping, check_only=args.check))

    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
