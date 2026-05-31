import json
import os
import sys
import shutil
import subprocess
import importlib.util
from enum import Enum
from pathlib import Path
from textwrap import dedent
from typing import List
import configparser

COLOR_ESC = '\033['
COLOR_RESET = f'{COLOR_ESC}0m'
COLOR_GREEN = f'{COLOR_ESC}32m'
COLOR_RED = f'{COLOR_ESC}31m'
COLOR_CYAN = f'{COLOR_ESC}36m'
COLOR_GRAY = f'{COLOR_ESC}30;1m'

REPO_ROOT = Path(__file__).resolve().parent.parent
BUILD_DIR = REPO_ROOT / "build"
PLATFORMIO_BUILD_DIR = REPO_ROOT / ".pio" / "build"


class DeviceConfiguration:
    def __init__(self,  platform: str, board: str, platformio_board: str) -> None:
        self.platform = platform
        self.board = board
        self.platformio_board = platformio_board

    def filename(self) -> str:
        return f"{self.board}-firmware.bin"

    def __str__(self) -> str:
        return f"{self.platform}@{self.board}"


def get_matrix() -> List[DeviceConfiguration]:
    matrix: List[DeviceConfiguration] = []

    config = configparser.ConfigParser()
    config.read(REPO_ROOT / "platformio.ini")
    for section in config.sections():
        split = section.split(":")
        if len(split) != 2 or split[0] != 'env':
            continue

        board = split[1]
        platform = config[section]["platform"]
        platformio_board = config[section]["board"]

        matrix.append(DeviceConfiguration(
            platform,
            board,
            platformio_board))

    return matrix


def prepare() -> None:
    print(f"🡢 {COLOR_CYAN}Preparation{COLOR_RESET}")
    if BUILD_DIR.exists():
        print(f"  🡢 {COLOR_GRAY}Removing existing build folder...{COLOR_RESET}")
        shutil.rmtree(BUILD_DIR)
    print(f"  🡢 {COLOR_GRAY}Creating build folder...{COLOR_RESET}")
    BUILD_DIR.mkdir()

    print(f"  🡢 {COLOR_GREEN}Success!{COLOR_RESET}")


def resolve_platformio_command() -> List[str]:
    override = os.environ.get("PLATFORMIO_CMD")
    if override:
        return [override]

    for candidate in ("platformio", "pio"):
        resolved = shutil.which(candidate)
        if resolved:
            return [resolved]

    python_dir = Path(sys.executable).resolve().parent
    scripts_dir = python_dir / ("Scripts" if os.name == "nt" else "bin")
    for candidate in ("platformio", "pio"):
        suffix = ".exe" if os.name == "nt" else ""
        resolved = scripts_dir / f"{candidate}{suffix}"
        if resolved.exists():
            return [str(resolved)]

    if importlib.util.find_spec("platformio") is not None:
        return [sys.executable, "-m", "platformio"]

    raise FileNotFoundError(
        "PlatformIO was not found. Install it into the active Python environment "
        "or set PLATFORMIO_CMD to the executable path."
    )


def build() -> int:
    print(f"🡢 {COLOR_CYAN}Build{COLOR_RESET}")

    failed_builds: List[str] = []
    code = 0

    matrix = get_matrix()

    for device in matrix:
        print(f"  🡢 {COLOR_CYAN}Building for {device.platform}{COLOR_RESET}")

        status = build_for_device(device)

        if not status:
            failed_builds.append(device.board)

    if len(failed_builds) > 0:
        print(f"  🡢 {COLOR_RED}Failed!{COLOR_RESET}")

        for failed_build in failed_builds:
            print(f"    🡢 {COLOR_RED}{failed_build}{COLOR_RESET}")

        code = 1
    else:
        print(f"  🡢 {COLOR_GREEN}Success!{COLOR_RESET}")

    return code


def build_for_device(device: DeviceConfiguration) -> bool:
    success = True

    print(f"::group::Build {device}")

    try:
        platformio_command = resolve_platformio_command()
    except FileNotFoundError as error:
        print(f"    🡢 {COLOR_RED}{error}{COLOR_RESET}")
        print("::endgroup::")
        return False

    result = subprocess.run(
        [*platformio_command, "run", "-e", device.board],
        cwd=REPO_ROOT,
        check=False,
    )
    code = result.returncode

    if code == 0:
        shutil.copy(
            PLATFORMIO_BUILD_DIR / device.board / "firmware.bin",
            BUILD_DIR / device.filename(),
        )

        print(f"    🡢 {COLOR_GREEN}Success!{COLOR_RESET}")
    else:
        success = False

        print(f"    🡢 {COLOR_RED}Failed!{COLOR_RESET}")

    print("::endgroup::")

    return success


def main() -> None:
    prepare()
    code = build()

    sys.exit(code)


if __name__ == "__main__":
    main()
