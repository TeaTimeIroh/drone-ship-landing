#!/usr/bin/env python3
"""Summarise a hold_over_pad recording: python3 tools/analyze_hold.py results/hold"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ship_landing.hold_analysis import load, to_markdown  # noqa: E402

folder = Path(sys.argv[1] if len(sys.argv) > 1 else 'results/hold')
text = to_markdown(load(folder / 'hold.csv'), title=f'Holding over the pad by lean angle: {folder.name}')
(folder / 'summary.md').write_text(text)
print(text)
