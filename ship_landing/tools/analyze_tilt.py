"""Analyse a tilt_check recording: does the tilt correction put the pad where it really is?

    python3 tools/analyze_tilt.py results/<tilt_check folder>

Reads detections.csv and poses.csv, writes summary.md and tilt_check.png into the same
folder and prints the summary.
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ship_landing.tilt_analysis import Geometry, analyse, load_log, to_markdown  # noqa: E402

# Two categorical colours that stay distinguishable for colour-blind readers; the two
# lines also differ in style, so the plot does not rely on colour alone.
RAW, LEVELLED = '#eb6834', '#2a78d6'
SURFACE, INK, INK_SOFT, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'


def draw(result, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    start = result.series[0][0]
    times = [row[0] - start for row in result.series]
    fig, (top, bottom) = plt.subplots(
        2, 1, figsize=(8.0, 5.6), dpi=160, sharex=True, gridspec_kw={'height_ratios': [2.2, 1.0]})
    fig.patch.set_facecolor(SURFACE)
    top.plot(times, [row[2] for row in result.series], color=RAW, linewidth=2.0,
             linestyle=(0, (5, 2)), label='camera as it is (tilt ignored)')
    top.plot(times, [row[3] for row in result.series], color=LEVELLED, linewidth=2.0,
             label='corrected with the drone\'s attitude')
    top.set_ylabel('Error in pad position (m)', color=INK_SOFT)
    top.set_ylim(bottom=0.0)
    top.set_title('Where the camera puts the pad, against where it is', color=INK, fontsize=11, loc='left')
    bottom.plot(times, [row[1] for row in result.series], color=INK_SOFT, linewidth=1.5)
    bottom.set_ylabel('Tilt (degrees)', color=INK_SOFT)
    bottom.set_xlabel('Time into the recording (s)', color=INK_SOFT)
    bottom.set_ylim(bottom=0.0)
    for ax in (top, bottom):
        ax.set_facecolor(SURFACE)
        ax.tick_params(colors=INK_SOFT, labelsize=9)
        ax.grid(True, color=GRID, linewidth=0.6)
        for spine in ax.spines.values():
            spine.set_color(GRID)
    # Below the plots, so the legend can never cover the lines.
    legend = fig.legend(*top.get_legend_handles_labels(), loc='lower center', ncol=2,
                        frameon=False, fontsize=9, bbox_to_anchor=(0.5, -0.04))
    for text in legend.get_texts():
        text.set_color(INK_SOFT)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE, bbox_inches='tight', pad_inches=0.2)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('folder', type=Path, help='tilt_check output folder')
    parser.add_argument('--pad-east', type=float, default=3.0)
    parser.add_argument('--pad-north', type=float, default=2.0)
    args = parser.parse_args()

    for name in ('detections.csv', 'poses.csv'):
        if not (args.folder / name).exists():
            sys.exit(f'{args.folder / name} not found. Did the tilt_check flight run?')
    detections, poses, note = load_log(args.folder)
    geometry = Geometry(pad_east=args.pad_east, pad_north=args.pad_north)
    result = analyse(detections, poses, geometry)
    text = to_markdown(result, note)
    (args.folder / 'summary.md').write_text(text)
    print(text)
    if result.series:
        try:
            draw(result, args.folder / 'tilt_check.png')
            print(f'Plot: {args.folder / "tilt_check.png"}')
        except ImportError:
            print('matplotlib is not installed, so no plot was drawn.')
    print(f'Summary: {args.folder / "summary.md"}')


if __name__ == '__main__':
    main()
