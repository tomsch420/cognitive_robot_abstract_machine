import sys

import pytest

import experiments.montessori.generate_insertion_experience as generate_insertion_experience
from experiments.montessori.montessori_demo import DEFAULT_ROBOT_CLASS


def test_parse_arguments_defaults():
    old_argv = sys.argv
    sys.argv = ["generate_insertion_experience"]
    try:
        arguments = generate_insertion_experience._parse_arguments()
    finally:
        sys.argv = old_argv

    assert arguments.runs == 100
    assert arguments.headless is False
    assert arguments.database_uri == generate_insertion_experience.DEFAULT_DATABASE_URI


def test_parse_arguments_reads_given_options():
    old_argv = sys.argv
    sys.argv = [
        "generate_insertion_experience",
        "--runs",
        "5",
        "--headless",
        "--database-uri",
        "sqlite:///custom.db",
    ]
    try:
        arguments = generate_insertion_experience._parse_arguments()
    finally:
        sys.argv = old_argv

    assert arguments.runs == 5
    assert arguments.headless is True
    assert arguments.database_uri == "sqlite:///custom.db"


def test_parse_arguments_falls_back_to_the_environment_variable(monkeypatch):
    monkeypatch.setenv("MONTESSORI_EXPERIENCE_DATABASE_URI", "sqlite:///from_env.db")
    old_argv = sys.argv
    sys.argv = ["generate_insertion_experience"]
    try:
        arguments = generate_insertion_experience._parse_arguments()
    finally:
        sys.argv = old_argv

    assert arguments.database_uri == "sqlite:///from_env.db"


def test_main_raises_if_rclpy_is_not_installed(monkeypatch):
    monkeypatch.setattr(generate_insertion_experience, "rclpy_installed", lambda: False)
    old_argv = sys.argv
    sys.argv = ["generate_insertion_experience"]
    try:
        with pytest.raises(RuntimeError, match="rclpy"):
            generate_insertion_experience.main()
    finally:
        sys.argv = old_argv


def test_main_raises_if_the_default_robot_is_not_installed(monkeypatch):
    monkeypatch.setattr(generate_insertion_experience, "rclpy_installed", lambda: True)
    monkeypatch.setattr(
        generate_insertion_experience, "robot_installed", lambda robot_class: False
    )
    old_argv = sys.argv
    sys.argv = ["generate_insertion_experience"]
    try:
        with pytest.raises(RuntimeError, match=DEFAULT_ROBOT_CLASS.__name__):
            generate_insertion_experience.main()
    finally:
        sys.argv = old_argv
