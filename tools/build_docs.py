# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Builds the docs site: docs/*.md to _site/*.html with one shared
# shell. No framework, no theme gem, no front matter in the
# sources; the markdown files stay clean for the repo view and
# this script owns the site view. Requires: pip install markdown.

import pathlib

import markdown

ROOT = pathlib.Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
OUT = ROOT / "_site"

PAGES = [
    ("index", "Maul3D", DOCS / "index.md"),
    ("manual", "The manual", DOCS / "manual.md"),
    ("changelog", "The changelog", DOCS / "changelog.md"),
    ("samples", "Samples", DOCS / "samples.md"),
]

SHELL = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title} | Maul3D</title>
<style>
:root {{ color-scheme: light dark; }}
body {{ max-width: 46rem; margin: 0 auto; padding: 1rem 1.25rem 4rem;
       font: 16px/1.6 system-ui, sans-serif; }}
nav {{ padding: 0.75rem 0; border-bottom: 1px solid #8884;
      margin-bottom: 1.5rem; }}
nav a {{ margin-right: 1.25rem; text-decoration: none; font-weight: 600; }}
pre {{ overflow-x: auto; padding: 0.75rem; border: 1px solid #8884;
      border-radius: 6px; }}
code {{ font-size: 0.92em; }}
table {{ border-collapse: collapse; }}
td, th {{ border: 1px solid #8886; padding: 0.3rem 0.6rem; }}
h1, h2 {{ line-height: 1.25; }}
</style>
</head>
<body>
<nav>
<a href="index.html">Maul3D</a>
<a href="manual.html">Manual</a>
<a href="samples.html">Samples</a>
<a href="changelog.html">Changelog</a>
<a href="https://github.com/siracozmen01/Maul3D">GitHub</a>
</nav>
{body}
</body>
</html>
"""


def main():
    OUT.mkdir(exist_ok=True)
    md = markdown.Markdown(extensions=["fenced_code", "tables", "toc"])
    for name, title, source in PAGES:
        body = md.reset().convert(source.read_text(encoding="utf-8"))
        page = SHELL.format(title=title, body=body)
        (OUT / (name + ".html")).write_text(page, encoding="utf-8")
        print("built", name + ".html")


if __name__ == "__main__":
    main()
