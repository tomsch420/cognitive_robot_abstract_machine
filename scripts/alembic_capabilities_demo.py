"""Demonstration of what Alembic autogenerate can and cannot handle automatically.

Each scenario modifies a SQLAlchemy model, runs ``alembic revision --autogenerate``,
and shows the generated migration.  Scenarios that require manual intervention are
clearly marked and the required fix is shown inline.

Run with::

    python3 scripts/alembic_capabilities_demo.py

No project dependencies are required — only ``alembic`` and ``sqlalchemy``.
"""
from __future__ import annotations

import shutil
import subprocess
import sys
import textwrap
import tempfile
from dataclasses import dataclass
from pathlib import Path


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

SECTION_WIDTH = 72

ALEMBIC_INI_TEMPLATE = """\
[alembic]
sqlalchemy.url = sqlite:///{db_path}
script_location = {alembic_dir}

[loggers]
keys = root,sqlalchemy,alembic

[handlers]
keys = console

[formatters]
keys = generic

[logger_root]
level = WARN
handlers = console
qualname =

[logger_sqlalchemy]
level = WARN
handlers =
qualname = sqlalchemy.engine

[logger_alembic]
level = INFO
handlers =
qualname = alembic

[handler_console]
class = StreamHandler
args = (sys.stderr,)
level = NOTSET
formatter = generic

[formatter_generic]
format = %(levelname)-5.5s [%(name)s] %(message)s
datefmt = %H:%M:%S
"""

ENV_PY_TEMPLATE = """\
from __future__ import annotations
from logging.config import fileConfig
from sqlalchemy import engine_from_config, pool
from alembic import context
import sys
sys.path.insert(0, "{model_dir}")
from model import Base

config = context.config
if config.config_file_name:
    fileConfig(config.config_file_name)
target_metadata = Base.metadata

def run_migrations_online():
    connectable = engine_from_config(
        config.get_section(config.config_ini_section, {{}}),
        prefix="sqlalchemy.",
        poolclass=pool.NullPool,
    )
    with connectable.connect() as connection:
        context.configure(connection=connection, target_metadata=target_metadata)
        with context.begin_transaction():
            context.run_migrations()

run_migrations_online()
"""

SCRIPT_MAKO = """\
\"\"\"${message}

Revision ID: ${up_revision}
Revises: ${down_revision | comma,n}
Create Date: ${create_date}
\"\"\"
from typing import Sequence, Union
from alembic import op
import sqlalchemy as sa
${imports if imports else ""}

revision: str = ${repr(up_revision)}
down_revision: Union[str, Sequence[str], None] = ${repr(down_revision)}
branch_labels: Union[str, Sequence[str], None] = ${repr(branch_labels)}
depends_on: Union[str, Sequence[str], None] = ${repr(depends_on)}

def upgrade() -> None:
    ${upgrades if upgrades else "pass"}

def downgrade() -> None:
    ${downgrades if downgrades else "pass"}
"""


@dataclass
class Workspace:
    """Temporary directory tree for one scenario."""

    root: Path
    model_file: Path
    alembic_ini: Path
    alembic_dir: Path
    versions_dir: Path
    db_path: Path

    @classmethod
    def create(cls) -> "Workspace":
        root = Path(tempfile.mkdtemp(prefix="alembic_demo_"))
        alembic_dir = root / "alembic"
        versions_dir = alembic_dir / "versions"
        versions_dir.mkdir(parents=True)
        db_path = root / "demo.db"
        model_file = root / "model.py"

        ini_path = root / "alembic.ini"
        ini_path.write_text(
            ALEMBIC_INI_TEMPLATE.format(db_path=db_path, alembic_dir=alembic_dir)
        )
        (alembic_dir / "env.py").write_text(
            ENV_PY_TEMPLATE.format(model_dir=root)
        )
        (alembic_dir / "script.py.mako").write_text(SCRIPT_MAKO)

        return cls(
            root=root,
            model_file=model_file,
            alembic_ini=ini_path,
            alembic_dir=alembic_dir,
            versions_dir=versions_dir,
            db_path=db_path,
        )

    def destroy(self) -> None:
        shutil.rmtree(self.root, ignore_errors=True)

    def write_model(self, source: str) -> None:
        self.model_file.write_text(textwrap.dedent(source))

    def alembic(self, *args: str) -> tuple[int, str]:
        """Run an alembic sub-command; return (returncode, combined output)."""
        result = subprocess.run(
            [sys.executable, "-m", "alembic", "-c", str(self.alembic_ini), *args],
            capture_output=True,
            text=True,
        )
        output = result.stdout + result.stderr
        return result.returncode, output

    def latest_migration(self) -> Path | None:
        files = sorted(self.versions_dir.glob("*.py"))
        return files[-1] if files else None

    def migration_body(self) -> str:
        """Return just the upgrade/downgrade bodies from the latest migration."""
        path = self.latest_migration()
        if not path:
            return "(no migration generated)"
        lines = path.read_text().splitlines()
        # Keep only from 'def upgrade' onward
        start = next(
            (i for i, l in enumerate(lines) if l.startswith("def upgrade")), 0
        )
        return "\n".join(lines[start:])


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------

def header(title: str) -> None:
    print()
    print("=" * SECTION_WIDTH)
    print(f"  {title}")
    print("=" * SECTION_WIDTH)


def subheader(title: str) -> None:
    print()
    print(f"  --- {title} ---")


def show(label: str, text: str) -> None:
    print(f"\n  [{label}]")
    for line in text.strip().splitlines():
        print(f"    {line}")


def verdict(can: bool, note: str = "") -> None:
    icon = "✓  CAN" if can else "✗  CANNOT"
    suffix = f"  ({note})" if note else ""
    print(f"\n  >>> {icon} do this automatically{suffix}\n")


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------

def scenario_add_column() -> None:
    header("SCENARIO 1 — Add a column")
    ws = Workspace.create()
    try:
        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                name: Mapped[str]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "initial")
        ws.alembic("upgrade", "head")

        # Add a new column
        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                name: Mapped[str]
                max_speed: Mapped[float]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "add max_speed")
        show("Generated migration", ws.migration_body())
        verdict(True)
    finally:
        ws.destroy()


def scenario_remove_column() -> None:
    header("SCENARIO 2 — Remove a column")
    ws = Workspace.create()
    try:
        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                name: Mapped[str]
                legacy_field: Mapped[str]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "initial")
        ws.alembic("upgrade", "head")

        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                name: Mapped[str]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "drop legacy_field")
        show("Generated migration", ws.migration_body())
        verdict(True)
    finally:
        ws.destroy()


def scenario_add_table() -> None:
    header("SCENARIO 3 — Add a new table")
    ws = Workspace.create()
    try:
        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                name: Mapped[str]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "initial")
        ws.alembic("upgrade", "head")

        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped, relationship
            from sqlalchemy import ForeignKey
            from typing import List
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                name: Mapped[str]
                tasks: Mapped[List["Task"]] = relationship(back_populates="robot")
            class Task(Base):
                __tablename__ = "task"
                id: Mapped[int] = mapped_column(primary_key=True)
                description: Mapped[str]
                robot_id: Mapped[int] = mapped_column(ForeignKey("robot.id"))
                robot: Mapped["Robot"] = relationship(back_populates="tasks")
        """)
        ws.alembic("revision", "--autogenerate", "-m", "add task table")
        show("Generated migration", ws.migration_body())
        verdict(True)
    finally:
        ws.destroy()


def scenario_drop_table() -> None:
    header("SCENARIO 4 — Drop a table")
    ws = Workspace.create()
    try:
        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                name: Mapped[str]
            class ObsoleteLog(Base):
                __tablename__ = "obsolete_log"
                id: Mapped[int] = mapped_column(primary_key=True)
                message: Mapped[str]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "initial")
        ws.alembic("upgrade", "head")

        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                name: Mapped[str]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "drop obsolete_log")
        show("Generated migration", ws.migration_body())
        verdict(True)
    finally:
        ws.destroy()


def scenario_add_index() -> None:
    header("SCENARIO 5 — Add an index")
    ws = Workspace.create()
    try:
        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                name: Mapped[str]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "initial")
        ws.alembic("upgrade", "head")

        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            from sqlalchemy import Index
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                __table_args__ = (Index("ix_robot_name", "name"),)
                id: Mapped[int] = mapped_column(primary_key=True)
                name: Mapped[str]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "index on name")
        show("Generated migration", ws.migration_body())
        verdict(True)
    finally:
        ws.destroy()


def scenario_change_nullable() -> None:
    header("SCENARIO 6 — Change nullable constraint")
    ws = Workspace.create()
    try:
        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            from typing import Optional
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                description: Mapped[Optional[str]]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "initial")
        ws.alembic("upgrade", "head")

        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                description: Mapped[str]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "description not nullable")
        show("Generated migration", ws.migration_body())
        print()
        print("  NOTE: SQLite does not enforce NOT NULL via ALTER — on PostgreSQL")
        print("  this correctly emits ALTER COLUMN ... SET NOT NULL.")
        verdict(True, "detects the change; enforcement depends on the database")
    finally:
        ws.destroy()


def scenario_rename_column() -> None:
    header("SCENARIO 7 — Rename a column  [MANUAL INTERVENTION REQUIRED]")
    ws = Workspace.create()
    try:
        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                old_name: Mapped[str]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "initial")
        ws.alembic("upgrade", "head")

        # Rename: Alembic sees a drop + add, not a rename — data would be lost
        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                new_name: Mapped[str]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "rename old_name to new_name")
        show("Generated migration (WRONG — data loss!)", ws.migration_body())
        print()
        print("  PROBLEM: Alembic generates a DROP + ADD instead of RENAME.")
        print("  All data in 'old_name' would be destroyed.")
        print()
        print("  FIX: Replace the generated body with:")
        print("    def upgrade():")
        print("        op.alter_column('robot', 'old_name', new_column_name='new_name')")
        print("    def downgrade():")
        print("        op.alter_column('robot', 'new_name', new_column_name='old_name')")
        verdict(False, "generates DROP+ADD; must be replaced with op.alter_column")
    finally:
        ws.destroy()


def scenario_rename_table() -> None:
    header("SCENARIO 8 — Rename a table  [MANUAL INTERVENTION REQUIRED]")
    ws = Workspace.create()
    try:
        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            class Base(DeclarativeBase): pass
            class OldRobot(Base):
                __tablename__ = "old_robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                name: Mapped[str]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "initial")
        ws.alembic("upgrade", "head")

        ws.write_model("""\
            from sqlalchemy.orm import DeclarativeBase, mapped_column, Mapped
            class Base(DeclarativeBase): pass
            class Robot(Base):
                __tablename__ = "robot"
                id: Mapped[int] = mapped_column(primary_key=True)
                name: Mapped[str]
        """)
        ws.alembic("revision", "--autogenerate", "-m", "rename old_robot to robot")
        show("Generated migration (WRONG — data loss!)", ws.migration_body())
        print()
        print("  PROBLEM: Alembic generates DROP TABLE + CREATE TABLE.")
        print("  All rows are destroyed.")
        print()
        print("  FIX: Replace the generated body with:")
        print("    def upgrade():")
        print("        op.rename_table('old_robot', 'robot')")
        print("    def downgrade():")
        print("        op.rename_table('robot', 'old_robot')")
        verdict(False, "generates DROP+CREATE; must be replaced with op.rename_table")
    finally:
        ws.destroy()


def scenario_data_migration() -> None:
    header("SCENARIO 9 — Data migration  [ALWAYS MANUAL]")
    print("""
  Alembic autogenerate only tracks schema changes, not data.
  Suppose a column is split: 'full_name' → 'first_name' + 'last_name'.

  The schema part (adding two columns, dropping one) is detected, but
  Alembic cannot know that the old data should be split on whitespace.

  The migration must be written manually:

    from alembic import op
    import sqlalchemy as sa
    from sqlalchemy.sql import table, column

    def upgrade():
        # 1. Schema changes (Alembic can generate these)
        op.add_column('robot', sa.Column('first_name', sa.String()))
        op.add_column('robot', sa.Column('last_name', sa.String()))

        # 2. Data migration (always manual)
        robot = table('robot',
            column('full_name', sa.String),
            column('first_name', sa.String),
            column('last_name', sa.String),
        )
        connection = op.get_bind()
        for row in connection.execute(robot.select()):
            parts = (row.full_name or "").split(" ", 1)
            connection.execute(
                robot.update()
                .where(robot.c.full_name == row.full_name)
                .values(first_name=parts[0], last_name=parts[1] if len(parts) > 1 else "")
            )

        # 3. Drop the old column
        op.drop_column('robot', 'full_name')

    def downgrade():
        op.add_column('robot', sa.Column('full_name', sa.String()))
        # reverse data migration omitted for brevity
        op.drop_column('robot', 'first_name')
        op.drop_column('robot', 'last_name')
""")
    verdict(False, "schema changes detected; data transformation always requires manual code")


def scenario_custom_type_change() -> None:
    header("SCENARIO 10 — Custom SQLAlchemy type change  [MANUAL INTERVENTION REQUIRED]")
    print("""
  When ORMatic maps a Python type to a custom SQLAlchemy type (e.g.
  TrimeshType → LargeBinary), Alembic compares the rendered SQL types.

  If the underlying SQL type does not change (both map to LargeBinary),
  Alembic sees no difference even if the serialization logic changed.
  The migration must be written manually to transform the stored bytes.

  If the SQL type DOES change (e.g. LargeBinary → JSON), Alembic detects
  it but generates only the DDL — the data transformation is manual:

    def upgrade():
        op.alter_column('body', 'mesh_data',
                         existing_type=sa.LargeBinary(),
                         type_=sa.JSON(),
                         postgresql_using='convert_from(mesh_data, \\'UTF8\\')::json')

  The `postgresql_using` clause (or equivalent) is always hand-written.
""")
    verdict(False, "DDL change detected; data conversion clause always manual")


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

def print_summary() -> None:
    header("SUMMARY")
    rows = [
        ("Add a column",               True,  ""),
        ("Remove a column",            True,  ""),
        ("Add a table",                True,  ""),
        ("Drop a table",               True,  ""),
        ("Add / drop an index",        True,  ""),
        ("Change nullable",            True,  "depends on DB; SQLite ignores it"),
        ("Rename a column",            False, "generates DROP+ADD → use op.alter_column"),
        ("Rename a table",             False, "generates DROP+CREATE → use op.rename_table"),
        ("Data migration",             False, "always manual; Alembic is schema-only"),
        ("Custom type serialization",  False, "SQL type change detected; conversion clause manual"),
    ]
    print()
    print(f"  {'Scenario':<35} {'Auto?':<8} Notes")
    print(f"  {'-'*35} {'-'*7} {'-'*28}")
    for label, can, note in rows:
        icon = "✓ YES" if can else "✗ NO "
        print(f"  {label:<35} {icon:<8} {note}")
    print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print()
    print("Alembic autogenerate capabilities — live demonstration")
    print("SQLAlchemy", __import__("sqlalchemy").__version__,
          "/ Alembic", __import__("alembic").__version__)

    scenario_add_column()
    scenario_remove_column()
    scenario_add_table()
    scenario_drop_table()
    scenario_add_index()
    scenario_change_nullable()
    scenario_rename_column()
    scenario_rename_table()
    scenario_data_migration()
    scenario_custom_type_change()
    print_summary()
