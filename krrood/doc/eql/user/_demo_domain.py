from dataclasses import dataclass


@dataclass(unsafe_hash=True)
class Robot:
    name: str
    battery: int
