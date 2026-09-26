"""自研 C 代码的可读性提示和基础风险门禁。

本脚本只检查 Application、Bsp、Modules 下的自研代码，跳过第三方与自动
生成代码。文件长度、函数长度和圈复杂度只是人工评审信号，不会因为统计
数字阻止构建。只有能够明确判定的源码损坏（例如未解决的合并冲突）才报错。
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOTS = ("Application", "Bsp", "Modules")
EXCLUDED_PARTS = {"c_library_v2-master"}

FILE_REVIEW_LINES = 800
FILE_HIGH_REVIEW_LINES = 1200
FUNCTION_REVIEW_LINES = 50
FUNCTION_HIGH_REVIEW_LINES = 80
COMPLEXITY_REVIEW_LIMIT = 15
COMPLEXITY_HIGH_REVIEW_LIMIT = 20

MERGE_CONFLICT_PATTERN = re.compile(r"(?m)^\s*(?:<<<<<<< .+|=======|>>>>>>> .+)$")


@dataclass(frozen=True)
class FunctionMetric:
    path: str
    name: str
    line: int
    code_lines: int
    complexity: int


def iter_custom_files(suffix: str) -> list[Path]:
    files: list[Path] = []
    for root_name in SOURCE_ROOTS:
        for path in (PROJECT_ROOT / root_name).rglob(f"*{suffix}"):
            if not EXCLUDED_PARTS.intersection(path.parts):
                files.append(path)
    return sorted(files)


def strip_comments_and_literals(source: str) -> str:
    """以空格替换注释和字面量，同时保留换行，便于统计源码行号。"""
    output: list[str] = []
    index = 0
    state = "code"

    while index < len(source):
        char = source[index]
        next_char = source[index + 1] if index + 1 < len(source) else ""

        if state == "code":
            if char == "/" and next_char == "*":
                output.extend((" ", " "))
                index += 2
                state = "block_comment"
                continue
            if char == "/" and next_char == "/":
                output.extend((" ", " "))
                index += 2
                state = "line_comment"
                continue
            if char in {'"', "'"}:
                output.append(" ")
                index += 1
                state = "string" if char == '"' else "character"
                continue
            output.append(char)
            index += 1
            continue

        if state == "block_comment":
            if char == "*" and next_char == "/":
                output.extend((" ", " "))
                index += 2
                state = "code"
            else:
                output.append("\n" if char == "\n" else " ")
                index += 1
            continue

        if state == "line_comment":
            output.append("\n" if char == "\n" else " ")
            index += 1
            if char == "\n":
                state = "code"
            continue

        quote = '"' if state == "string" else "'"
        if char == "\\" and index + 1 < len(source):
            output.extend((" ", " "))
            index += 2
        elif char == quote:
            output.append(" ")
            index += 1
            state = "code"
        else:
            output.append("\n" if char == "\n" else " ")
            index += 1

    return "".join(output)


FUNCTION_PATTERN = re.compile(
    r"(?m)^[ \t]*(?:static[ \t]+)?(?:inline[ \t]+)?"
    r"(?:[A-Za-z_]\w*(?:[ \t]+|[ \t]*\*[ \t]*))+"
    r"(?P<name>[A-Za-z_]\w*)[ \t]*\([^;{}]*?\)[ \t]*(?:\n[ \t]*)?\{"
)


def measure_functions(path: Path, source: str) -> list[FunctionMetric]:
    metrics: list[FunctionMetric] = []
    relative_path = path.relative_to(PROJECT_ROOT).as_posix()

    for match in FUNCTION_PATTERN.finditer(source):
        name = match.group("name")
        opening_brace = source.find("{", match.start())
        depth = 1
        cursor = opening_brace + 1

        while cursor < len(source) and depth > 0:
            depth += int(source[cursor] == "{")
            depth -= int(source[cursor] == "}")
            cursor += 1

        if depth != 0:
            continue

        body = source[opening_brace:cursor]
        code_lines = sum(bool(line.strip()) for line in body.splitlines())
        complexity = 1 + len(
            re.findall(r"\b(?:if|for|while)\s*\(|\bcase\b|&&|\|\||(?<!\?)\?(?!\?)", body)
        )
        metrics.append(
            FunctionMetric(
                path=relative_path,
                name=name,
                line=source.count("\n", 0, match.start()) + 1,
                code_lines=code_lines,
                complexity=complexity,
            )
        )

    return metrics


def main() -> int:
    errors: list[str] = []
    reviews: list[str] = []
    all_metrics: list[FunctionMetric] = []
    source_files = iter_custom_files(".c")

    for path in source_files:
        relative_path = path.relative_to(PROJECT_ROOT).as_posix()
        try:
            raw_source = path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as error:
            errors.append(f"{relative_path}: 无法读取 UTF-8 源码：{error}")
            continue

        if "\0" in raw_source:
            errors.append(f"{relative_path}: 源码包含 NUL 字节")
        if MERGE_CONFLICT_PATTERN.search(raw_source):
            errors.append(f"{relative_path}: 存在未解决的版本控制合并冲突标记")

        source = strip_comments_and_literals(raw_source)
        code_lines = sum(bool(line.strip()) for line in source.splitlines())

        if code_lines > FILE_HIGH_REVIEW_LINES:
            reviews.append(f"{relative_path}: 有效代码 {code_lines} 行，建议重点检查职责、依赖和状态所有权")
        elif code_lines > FILE_REVIEW_LINES:
            reviews.append(f"{relative_path}: 有效代码 {code_lines} 行，建议检查是否仍保持单一模块职责")

        all_metrics.extend(measure_functions(path, source))

    for metric in all_metrics:
        location = f"{metric.path}:{metric.line} {metric.name}"

        if metric.code_lines > FUNCTION_HIGH_REVIEW_LINES:
            reviews.append(f"{location}: {metric.code_lines} 行，建议重点检查是否混合多个职责")
        elif metric.code_lines > FUNCTION_REVIEW_LINES:
            reviews.append(f"{location}: {metric.code_lines} 行，建议继续检查职责")

        if metric.complexity > COMPLEXITY_HIGH_REVIEW_LIMIT:
            reviews.append(f"{location}: 圈复杂度约 {metric.complexity}，建议重点检查状态和分支职责")
        elif metric.complexity > COMPLEXITY_REVIEW_LIMIT:
            reviews.append(f"{location}: 圈复杂度约 {metric.complexity}，需要重点评审")

    print(
        f"检查完成：{len(source_files)} 个 C 文件，"
        f"{len(all_metrics)} 个函数，{len(reviews)} 个评审提示，{len(errors)} 个错误。"
    )

    for review in sorted(set(reviews)):
        print(f"[REVIEW] {review}")

    for error in sorted(set(errors)):
        print(f"[ERROR] {error}", file=sys.stderr)

    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
