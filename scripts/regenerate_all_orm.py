from pathlib import Path
import subprocess
import sys


def regenerate(generate_orm_path: str) -> None:
    """
    Runs the provided generate_orm.py file in a subprocess.
    """
    generate_orm = Path(generate_orm_path).resolve()
    folder = generate_orm.parent

    if generate_orm.name != "generate_orm.py":
        raise ValueError(f"Expected a generate_orm.py file, got: {generate_orm}")

    if not generate_orm.exists():
        raise FileNotFoundError(f"Generator not found: {generate_orm}")

    subprocess.run(
        [sys.executable, str(generate_orm)],
        cwd=folder,
        check=True,
    )


def clear_file(file_path: str) -> None:
    """
    Deletes the contents of a file without deleting the file itself.
    """
    path = Path(file_path).resolve()
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")
    path.write_text("", encoding="utf-8")


clear_file(
    "../semantic_digital_twin/src/semantic_digital_twin/orm/ormatic_interface.py"
)
clear_file("../coraplex/src/coraplex/orm/ormatic_interface.py")
clear_file("../experiments/src/experiments/orm/ormatic_interface.py")

regenerate("../semantic_digital_twin/scripts/generate_orm.py")
regenerate("../coraplex/scripts/generate_orm.py")
regenerate("../experiments/scripts/generate_orm.py")
