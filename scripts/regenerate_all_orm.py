"""Regenerates all ORMatic-generated SQLAlchemy interfaces.

Pass ``--migrate`` to also generate Alembic migration scripts that capture
the schema delta between the newly generated metadata and any existing
database. The generated scripts land in each package's
``orm/alembic/versions/`` directory and must be reviewed before applying.

Apply a generated migration with::

    alembic -c <package>/src/<package>/orm/alembic.ini upgrade head
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent


@dataclass
class SubPackage:
    """Describes one subpackage that owns an ORMatic-generated interface."""

    generate_orm_script: Path
    """Path to the generate_orm.py script for this subpackage."""

    ormatic_interface: Path
    """Path to the generated ormatic_interface.py file."""

    alembic_ini: Path
    """Path to the alembic.ini file for this subpackage."""


SUBPACKAGES: list[SubPackage] = [
    SubPackage(
        generate_orm_script=ROOT / "semantic_digital_twin/scripts/generate_orm.py",
        ormatic_interface=ROOT / "semantic_digital_twin/src/semantic_digital_twin/orm/ormatic_interface.py",
        alembic_ini=ROOT / "semantic_digital_twin/src/semantic_digital_twin/orm/alembic.ini",
    ),
    SubPackage(
        generate_orm_script=ROOT / "coraplex/scripts/generate_orm.py",
        ormatic_interface=ROOT / "coraplex/src/coraplex/orm/ormatic_interface.py",
        alembic_ini=ROOT / "coraplex/src/coraplex/orm/alembic.ini",
    ),
    SubPackage(
        generate_orm_script=ROOT / "experiments/scripts/generate_orm.py",
        ormatic_interface=ROOT / "experiments/src/experiments/orm/ormatic_interface.py",
        alembic_ini=ROOT / "experiments/src/experiments/orm/alembic.ini",
    ),
]


def clear_file(path: Path) -> None:
    """Erase the contents of a file without deleting it."""
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")
    path.write_text("", encoding="utf-8")


def regenerate(script: Path) -> None:
    """Run a generate_orm.py script in its own directory."""
    if script.name != "generate_orm.py":
        raise ValueError(f"Expected a generate_orm.py file, got: {script}")
    if not script.exists():
        raise FileNotFoundError(f"Generator not found: {script}")
    subprocess.run([sys.executable, str(script)], cwd=script.parent, check=True)


def generate_migration(alembic_ini: Path, message: str) -> None:
    """Ask Alembic to diff the current metadata and write a migration script."""
    subprocess.run(
        [
            sys.executable, "-m", "alembic",
            "-c", str(alembic_ini),
            "revision", "--autogenerate",
            "-m", message,
        ],
        check=True,
    )


def main() -> None:
    """Entry point: regenerate ORM interfaces and optionally create migrations."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--migrate",
        action="store_true",
        help=(
            "After regenerating, run 'alembic revision --autogenerate' for each "
            "subpackage to produce migration scripts from the schema delta."
        ),
    )
    parser.add_argument(
        "--message", "-m",
        default="schema change",
        help="Message embedded in generated migration file names (default: 'schema change').",
    )
    args = parser.parse_args()

    for package in SUBPACKAGES:
        print(f"Clearing {package.ormatic_interface.name} for {package.generate_orm_script.parent.parent.name}...")
        clear_file(package.ormatic_interface)

    for package in SUBPACKAGES:
        name = package.generate_orm_script.parent.parent.name
        print(f"Regenerating ORM for {name}...")
        regenerate(package.generate_orm_script)

    if args.migrate:
        for package in SUBPACKAGES:
            name = package.generate_orm_script.parent.parent.name
            print(f"Generating Alembic migration for {name}...")
            generate_migration(package.alembic_ini, args.message)
        print(
            "\nMigrations written. Review the generated files in each package's "
            "orm/alembic/versions/ directory, then apply with:\n"
            "  alembic -c <package>/src/<package>/orm/alembic.ini upgrade head"
        )


if __name__ == "__main__":
    main()
