#!/usr/bin/env python3
"""Summarise a stick_check recording: python3 tools/analyze_sticks.py results/stick_check"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ship_landing.stick_analysis import load, to_markdown  # noqa: E402

folder = Path(sys.argv[1] if len(sys.argv) > 1 else 'results/stick_check')
text = to_markdown(load(folder / 'sticks.csv'), title=f'Stick check: {folder.name}')
(folder / 'summary.md').write_text(text)
print(text)
