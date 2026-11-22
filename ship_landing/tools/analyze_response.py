#!/usr/bin/env python3
"""Summarise a response_check recording: python3 tools/analyze_response.py results/response_gps"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ship_landing.response_analysis import load, to_markdown  # noqa: E402

folder = Path(sys.argv[1] if len(sys.argv) > 1 else 'results/response')
text = to_markdown(load(folder / 'response.csv'), title=f'Response to velocity commands: {folder.name}')
(folder / 'summary.md').write_text(text)
print(text)
