import os
import re

source_dir = "/home/ws/src/evaluator/evaluator"
target_dir = "/home/ws/src/evaluator/explanation"


def extract_relevant_lines(file_content):
    lines = file_content.splitlines()
    result_lines = []
    in_docstring = False
    in_declaration = False
    in_import = False
    docstring_delimiter = None
    at_file_beginning = True

    for line in lines:
        stripped_line = line.strip()
        # Check for the start of a docstring

        if stripped_line.startswith('"""') or stripped_line.startswith("'''"):
            result_lines.append(line)
            at_file_beginning = False
            if in_docstring:
                if stripped_line.endswith(docstring_delimiter):
                    in_docstring = False

                    docstring_delimiter = None
            else:
                docstring_delimiter = stripped_line[:3]
                if (
                    stripped_line.endswith(docstring_delimiter)
                    and len(stripped_line) > 3
                ):
                    in_docstring = False
                    docstring_delimiter = None
                else:
                    in_docstring = True

        elif in_docstring:
            result_lines.append(line)

        elif stripped_line.startswith("import ") or stripped_line.startswith("from "):
            result_lines.append(line)
            at_file_beginning = False
            if "(" in stripped_line and ")" not in stripped_line:
                in_import = True

        elif in_import:
            result_lines.append(line)
            if ")" in stripped_line:
                in_import = False

        elif stripped_line.startswith("class ") or stripped_line.startswith("def "):
            # Start of a new class or function declaration
            result_lines.append(line)
            in_declaration = True

            # Check if the declaration is multi-line
            if stripped_line.endswith(":"):
                in_declaration = False
            else:
                in_declaration = True
            if not at_file_beginning:
                result_lines[-1] = "\n" + result_lines[-1]
            at_file_beginning = False

        elif in_declaration:
            # Continue capturing multi-line declarations
            result_lines.append(line)
            if stripped_line.endswith(":"):
                in_declaration = False
        else:
            # End capturing lines if not in a declaration or docstring
            in_declaration = False

    return "\n".join(result_lines)


def process_files(src_dir, dest_dir):
    for root, _, files in os.walk(src_dir):
        for file in files:
            if file.endswith(".py"):
                src_file_path = os.path.join(root, file)
                dest_file_path = os.path.join(dest_dir, file)

                with open(src_file_path, "r") as src_file:
                    file_content = src_file.read()

                relevant_content = extract_relevant_lines(file_content)

                os.makedirs(os.path.dirname(dest_file_path), exist_ok=True)
                with open(dest_file_path, "w") as dest_file:
                    dest_file.write(relevant_content)


if __name__ == "__main__":
    src_directory = "/home/ws/src/evaluator/evaluator"
    dest_directory = "/home/ws/src/evaluator/explanation"

    if not os.path.exists(dest_directory):
        os.makedirs(dest_directory)

    process_files(src_directory, dest_directory)
