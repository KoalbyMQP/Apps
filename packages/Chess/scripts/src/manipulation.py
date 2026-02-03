"""
Manipulation wrapper: placeholders for IK repo. Will expose move / pose control.

Once the IK repo is available, install it (e.g.):
  pip install "git+https://github.com/YOUR_ORG/IK_REPO.git@branch#subdirectory=path"

Then uncomment the imports below and implement the methods.
"""

# Placeholder: uncomment when IK package is available.
# from ik_package.solver import Solver
# from ik_package.robot import Robot


class Manipulation:
    """Thin wrapper around IK repo: move arm to position, etc."""

    def __init__(self) -> None:
        pass

    def move_to_square(self, square: str) -> None:
        """Placeholder: move arm to chess square (e.g. 'A1')."""
        raise NotImplementedError("IK repo not installed; add repo and implement.")

    def home(self) -> None:
        """Placeholder: move arm to home position."""
        raise NotImplementedError("IK repo not installed; add repo and implement.")


def main() -> None:
    print("Manipulation: placeholder (IK repo not wired up yet).")
    m = Manipulation()
    print("  Manipulation() OK; move_to_square / home will raise until implemented.")


if __name__ == "__main__":
    main()
