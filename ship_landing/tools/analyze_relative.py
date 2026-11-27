#!/usr/bin/env python3
"""Summarise a relative_check recording: python3 tools/analyze_relative.py results/relative_check"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ship_landing.relative_analysis import load, to_markdown  # noqa: E402

folder = Path(sys.argv[1] if len(sys.argv) > 1 else 'results/relative_check')
text = to_markdown(load(folder / 'relative.csv'), title=f'Camera-only pad tracking against the truth: {folder.name}')
(folder / 'summary.md').write_text(text)
print(text)
