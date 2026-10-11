#!/usr/bin/env python3
"""Проверить оба самостоятельных проекта: одинаковый core, API и реальные GPIO."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default="g++", help="нативный компилятор C++11")
    args = parser.parse_args()
    compiler = shutil.which(args.cxx)
    if not compiler:
        parser.error("Компилятор не найден: " + args.cxx)
    root = Path(__file__).resolve().parents[1]
    first = root / "arduino-actuator-1"
    second = root / "arduino-actuator-2"
    # У проектов свои обычные файлы, не symlink и не include из соседней папки.
    # Дублирующийся core должен оставаться одинаковым при дальнейших изменениях.
    files = {p.relative_to(first / "src") for p in (first / "src").rglob("*") if p.is_file()}
    other = {p.relative_to(second / "src") for p in (second / "src").rglob("*") if p.is_file()}
    assert files == other, "Состав src проектов отличается"
    for file in files - {Path("main.cpp")}:
        assert (first / "src" / file).read_bytes() == (second / "src" / file).read_bytes(), file
    assert (first / "arduino-actuator-1.ino").read_bytes() == (second / "arduino-actuator-2.ino").read_bytes()
    for project, node, count in ((first, 14, 4), (second, 17, 2)):
        config = (project / "BoardConfig.h").read_text(encoding="utf-8")
        assert "API_ACTUATOR_BOARD" not in config, "Переключателя платы быть не должно"
        assert f"#define API_LOCAL_NODE {node}" in config
        assert f"#define API_FAN_COUNT {count}" in config
    with tempfile.TemporaryDirectory(prefix="actuator-project-tests-") as directory:
        build = Path(directory)

        def check(name, source, flags):
            executable = build / (name + ".exe")
            subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(root / "test/stubs"), *flags,
                            str(root / "test" / source), "-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True)

        for board in (1, 2):
            folder = f"arduino-actuator-{board}"
            check(f"console-{board}", "console_test.cpp",
                  [f'-DACTUATOR_TEST_HEADER="../{folder}/src/ActuatorTestInput.h"'])
            check(f"api-{board}", "hardware_api_test.cpp",
                  [f'-DACTUATOR_API_HEADER="../{folder}/src/ActuatorApi.h"'])
            check(f"gpio-{board}", "board_io_test.cpp",
                  [f'-DACTUATOR_BOARD_IO_HEADER="../{folder}/src/BoardIO.h"'])
        check("invalid-pins", "board_io_test.cpp",
              ['-DHARDWARE_API_BOARD_HEADER="' + (root / "test/compile_only_board.h").as_posix() + '"'])
    print("Оба проекта: core синхронизирован, API/GPIO/RPM/реле и ошибочная карта проверены.")


if __name__ == "__main__":
    main()
