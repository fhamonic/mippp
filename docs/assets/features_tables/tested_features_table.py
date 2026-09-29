import os
import sys
import re
import subprocess
from collections import defaultdict
from PIL import Image


def find_cpp_files(root_folder):
    cpp_files = []
    for root, _, files in os.walk(root_folder):
        for file in files:
            if file == "dumb.cpp":
                continue
            if file.endswith(".cpp"):
                cpp_files.append(os.path.join(root, file))
    return cpp_files


# A QP model is continuous, so its column joins the LP table, labelled apart
# from the same solver's LP class.
def column_label(solver):
    return re.sub(r"_qp$", " QP", re.sub(r"_(lp|milp)$", "", solver))


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
                    if "_lp_" in model or "_qp_" in model:
                        lp_table[test].add(column_label(solver))
                    elif "_milp_" in model:
                        milp_table[test].add(column_label(solver))

    return lp_table, milp_table


formated_test_names = [
    ("LpModelTest", "LP Model"),
    ("MilpModelTest", "MILP Model"),
    ("QpModelTest", "QP Model"),
    ("EnumerableEntitiesTest", "Enumerate variables and constraints"),
    ("ReadableObjectiveTest", "Read objective"),
    ("ReadableQuadraticObjectiveTest", "Read quadratic objective"),
    ("ReadableVariablesBoundsTest", "Read variables bounds"),
    ("ReadableConstraintsTest", "Read constraints"),
    ("ReadableConstraintBoundsTest", "Read constraint bounds"),
    ("RangedConstraintsTest", "Ranged constraints"),
    ("ModifiableObjectiveTest", "Increment objective"),
    ("ModifiableVariablesBoundsTest", "Modify variable bounds"),
    ("ModifiableConstraintBoundsTest", "Modify constraint bounds"),
    ("IisTest", "IIS, native"),
    ("IisByDeletionTest", "IIS, deletion filter"),
    ("NamedVariablesTest", "Named variables"),
    ("NamedConstraintsTest", "Named constraints"),
    ("LpStatusTest", "LP status"),
    ("AddColumnTest", "Add column"),
    ("RemoveVariableTest", "Remove variable"),
    ("DualSolutionTest", "Dual solution"),
    ("ReducedCostsTest", "Reduced costs"),
    ("ColumnManagerTest", "Column manager"),
    ("CuttingStockTest", "Cutting stock example"),
    ("CandidateSolutionCallbackTest", "Candidate solution callback"),
    ("LazyConstraintsTest", "Lazy constraints"),
    ("TravellingSalesmanTest", "Travelling Salesman example"),
    ("TimeLimitTest", "Time limit"),
    ("TimeLimitIncumbentTest", "Incumbent at time limit"),
    ("IterationLimitTest", "Iteration limit"),
    ("OptimalityToleranceTest", "Optimality tolerance"),
    ("MipGapTest", "MIP gap"),
    ("IntegralityToleranceTest", "Integrality tolerance"),
    ("VerbosityTest", "Verbosity control"),
    ("MipStartTest", "MIP start"),
    ("SudokuTest", "Sudoku example"),
    ("LpFuzzyTest", "Fuzzing tested"),
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


def write_and_compile_latex(table, output_path, output_filename, res):
    latex_doc = ["""\\documentclass[border=5pt]{standalone}
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
"""]

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
                "-singlefile",
                f"{output_path}/{output_filename}.pdf",
                f"{output_path}/{output_filename}_light",
            ],
            check=True,
        )

        print(f"\n✅ PDF generated: {output_path}/{output_filename}.pdf")
    except subprocess.CalledProcessError:
        print(
            "❌ Error generating the table. Make sure `xelatex` and `pdftocairo` are installed and available in PATH."
        )
        sys.exit(1)


# The standalone page is wider than what is drawn on it, by an amount that
# varies with the table, so the drawn area is measured on the image. Both
# tables get one width, the right margin mirroring the left, so that the pages
# show them at the same scale.
def crop_to_common_width(png_paths):
    images = [Image.open(path) for path in png_paths]
    boxes = [image.getchannel("A").getbbox() for image in images]
    for path, image, (_, _, right, _) in zip(png_paths, images, boxes):
        if right == image.width:
            sys.exit(f"❌ {path}: the table reaches the edge of the page")
    width = max(left + right for left, _, right, _ in boxes)
    for path, image in zip(png_paths, images):
        image.crop((0, 0, width, image.height)).save(path)


def write_dark_variant(light_path, dark_path):
    subprocess.run(
        ["convert", light_path, "-channel", "RGB", "-negate", "+channel", dark_path],
        check=True,
    )


def main():
    root_path = os.path.dirname(sys.argv[0])
    tests_folder = f"{root_path}/../../../test/solvers"
    cpp_files = find_cpp_files(tests_folder)
    lp_table, milp_table = parse_instantiate_lines(cpp_files)

    res = 600
    names = ["lp_table", "milp_table"]
    for name, table in zip(names, [lp_table, milp_table]):
        write_and_compile_latex(table, root_path, name, res)

    crop_to_common_width([f"{root_path}/{name}_light.png" for name in names])
    for name in names:
        write_dark_variant(
            f"{root_path}/{name}_light.png", f"{root_path}/{name}_dark.png"
        )


if __name__ == "__main__":
    main()
