import os
import sys
import re
import subprocess
from collections import defaultdict
from PIL import ImageFont


def find_cpp_files(root_folder):
    cpp_files = []
    for root, _, files in os.walk(root_folder):
        for file in files:
            if file.endswith(".cpp"):
                cpp_files.append(os.path.join(root, file))
    return cpp_files


def parse_instantiate_lines(cpp_files):
    pattern = re.compile(r"^INSTANTIATE_TEST\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\);")
    lp_table = defaultdict(set)
    milp_table = defaultdict(set)

    for file in cpp_files:
        with open(file, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                match = pattern.search(line)
                if match:
                    solver, test, model = match.groups()
                    if "_lp_" in model:
                        lp_table[test].add(solver.replace("_lp", ""))
                    elif "_milp_" in model:
                        milp_table[test].add(solver.replace("_milp", ""))

    return lp_table, milp_table


formated_test_names = [
    ("LpModelTest", "LP Model"),
    ("MilpModelTest", "MILP Model"),
    ("ReadableObjectiveTest", "Read objective"),
    ("ReadableVariablesBoundsTest", "Read variables bounds"),
    ("ModifiableObjectiveTest", "Increment objective"),
    ("ModifiableVariablesBoundsTest", "Modify variable bounds"),
    ("NamedVariablesTest", "Named variables"),
    ("AddColumnTest", "Add column"),
    ("CuttingStockTest", "Cutting stock example"),
    ("LpStatusTest", "LP status"),
    ("CandidateSolutionCallbackTest", "Candidate solution callback"),
    ("TravellingSalesmanTest", "Travelling Salesman example"),
    ("DualSolutionTest", "Dual solution"),
    ("SudokuTest", "Sudoku example"),
]


def format_test_names(table):
    test_names = []
    formated_names = {}
    for name, formated_name in formated_test_names:
        if name in table.keys():
            test_names.append(name)
            formated_names[name] = formated_name
    for name in table.keys():
        if name not in test_names:
            test_names.append(name)
            formated_names[name] = name
    return test_names, formated_names


def generate_latex_table(table):
    all_solvers = sorted({solver for solvers in table.values() for solver in solvers})
    test_names, formated_names = format_test_names(table)

    latex = []
    latex.append("\\renewcommand{\\arraystretch}{1.2}")
    latex.append("\\begin{NiceTabular}{" + "l" + "c" * len(all_solvers) + "}")
    header = (
        "Tested Feature & "
        + " & ".join([f"\\Rot{{{solver}}}" for solver in all_solvers])
        + " \\\\"
    )
    latex.append(header)
    latex.append("\\midrule")

    for test in test_names:
        row_name = formated_names[test]
        row = (
            row_name
            + " & "
            + " & ".join(
                "$\\checkmark$" if solver in table[test] else ""
                for solver in all_solvers
            )
            + " \\\\"
        )
        latex.append(row)

    latex.append("\\CodeAfter\n")
    for i in range(len(all_solvers)):
        latex.append(f"\\MixedRule{{{i+2}}}")
    latex.append("\\end{NiceTabular}\n")
    return "\n".join(latex)


def write_and_compile_latex(table, output_path, output_filename, res, width):
    latex_doc = [
        """\\documentclass[border=5pt]{standalone}
\\usepackage{booktabs}
\\usepackage{amssymb}
\\usepackage{xparse}
\\usepackage{nicematrix}
\\usepackage{tikz}
\\usetikzlibrary{calc}
                 
\\usepackage{fontspec}
 
\\setmainfont{DejaVu Sans}

\\ExplSyntaxOn
\\NewExpandableDocumentCommand { \\ValuePlusOne } { m } 
{ \\int_eval:n { \\int_use:c { c @ #1 } + 1 } }
\\NewExpandableDocumentCommand { \\Sec } { m } 
{ \\fp_eval:n { secd ( #1 ) } }
\\NewDocumentCommand { \\Rot } { m }
{ 
    \\hbox_to_wd:nn { 1 em }
    { 
        \\hbox_overlap_right:n 
        { 
            \\skip_horizontal:n { \\fp_to_dim:n { 7 * cosd (\\Angle) } } 
            \\rotatebox{\\Angle}{#1}
        } 
    } 
}
\\ExplSyntaxOff

\\NewDocumentCommand { \\MixedRule } { m }
{
    \\begin{tikzpicture}
    \\coordinate (a) at (2-|#1) ;
    \\coordinate (b) at (1-|#1) ;
    \\draw (a) -- ($(a)!\\Sec{90-\\Angle}!\\Angle-90:(b)$) ;
    %
    \\draw (2-|#1) -- (\\ValuePlusOne{iRow}-|#1) ;
    \\end{tikzpicture}
}

\\begin{document}     
        
\\def\\Angle{60}
"""
    ]

    if table:
        latex_doc.append(generate_latex_table(table))

    latex_doc.append("\\end{document}")

    tex_path = f"{output_path}/{output_filename}.tex"
    with open(tex_path, "w") as f:
        f.write("\n".join(latex_doc))

    try:
        subprocess.run(
            ["xelatex", f"-output-directory={output_path}", tex_path], check=True
        )
        subprocess.run(
            ["xelatex", f"-output-directory={output_path}", tex_path], check=True
        )
        subprocess.run(
            [
                "pdftocairo",
                "-png",
                "-transp",
                "-r",
                str(res),
                "-W",
                str(width),
                "-singlefile",
                f"{output_path}/{output_filename}.pdf",
                f"{output_path}/{output_filename}_light",
            ],
            check=True,
        )
        subprocess.run(
            [
                "convert",
                f"{output_path}/{output_filename}_light.png",
                "-channel",
                "RGB",
                "-negate",
                "+channel",
                f"{output_path}/{output_filename}_dark.png",
            ],
            check=True,
        )

        print(f"\n✅ PDF generated: {output_path}/{output_filename}.pdf")
    except subprocess.CalledProcessError:
        print(
            "❌ Error compiling LaTeX. Make sure `pdflatex` is installed and available in PATH."
        )


def compute_names_max_width(names, font_size=10, res=600):
    font = ImageFont.truetype("DejaVuSans", int(res / 92 * font_size))
    return max([font.getsize(name)[0] for name in names])


def compute_width(table, res):
    return int(
        (
            94
            + compute_names_max_width(format_test_names(table)[1])
            + (len(table.values()) + 1) * 183
            + 94
        )
        * 600
        / res
    )


def main():
    root_path = os.path.dirname(sys.argv[0])
    tests_folder = f"{root_path}/../../test/solvers"
    cpp_files = find_cpp_files(tests_folder)
    lp_table, milp_table = parse_instantiate_lines(cpp_files)

    res = 600
    width = max(compute_width(lp_table, res), compute_width(milp_table, res))

    write_and_compile_latex(lp_table, root_path, "lp_table", res, width)
    write_and_compile_latex(milp_table, root_path, "milp_table", res, width)


if __name__ == "__main__":
    main()
