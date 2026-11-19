"""Summarise moving-pad landing trials and draw where the drone touched down on the pad.

    python3 tools/analyze_moving_trials.py results/<run>/batch_*.csv --out results/<run>

Writes summary.md and touchdowns.png into the output folder and prints the summary.
The plot is drawn relative to the way the pad was travelling at touchdown: to the
right is ahead of the pad centre, up is to the left of its travel.
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ship_landing.moving_trial_stats import CHANGING, load_rows, summarise, to_markdown  # noqa: E402
from ship_landing.trial_stats import MARKER_HALF, PAD_HALF  # noqa: E402

# Two categorical colours that stay distinguishable for colour-blind readers; each group
# also has its own marker shape, so the plot does not rely on colour alone.
STEADY = ('#2a78d6', 'o')
CHANGED = ('#eb6834', '^')
SURFACE, INK, INK_SOFT, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'


def draw(rows, summary, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    landed = [r for r in rows if r['outcome'] == 'landed' and r['along'] is not None]
    if not landed:
        return False
    stats = summary['error']
    reach = max([abs(r['along']) for r in landed] + [abs(r['across']) for r in landed] + [stats['p95']])
    limit = max(0.05, 1.3 * reach)

    fig, ax = plt.subplots(figsize=(6.4, 6.4), dpi=160)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    ax.add_patch(Rectangle((-PAD_HALF, -PAD_HALF), 2 * PAD_HALF, 2 * PAD_HALF,
                           facecolor='#f1f0ec', edgecolor=INK_SOFT, linewidth=1.0, zorder=1))
    ax.add_patch(Rectangle((-MARKER_HALF, -MARKER_HALF), 2 * MARKER_HALF, 2 * MARKER_HALF,
                           facecolor='none', edgecolor=INK_SOFT, linewidth=1.0, linestyle=(0, (4, 3)), zorder=2))
    ax.axhline(0, color=GRID, linewidth=1.0, zorder=2)
    ax.axvline(0, color=GRID, linewidth=1.0, zorder=2)
    for radius in (stats['median'], stats['p95']):
        ax.add_patch(Circle((0, 0), radius, facecolor='none', edgecolor=INK_SOFT, linewidth=1.0, zorder=3))
    ax.text(0.02, 0.98, f'inner ring: half of the landings, {100 * stats["median"]:.1f} cm\n'
                        f'outer ring: 95% of the landings, {100 * stats["p95"]:.1f} cm',
            transform=ax.transAxes, va='top', ha='left', fontsize=9, color=INK_SOFT, zorder=6)

    ax.plot([summary['bias_along']], [summary['bias_across']], marker='+', markersize=14, markeredgewidth=2.0,
            color=INK, zorder=7, linestyle='none', label='average touchdown')
    groups = [
        ('pad held its velocity', [r for r in landed if r['pad_velocity_change'] is not None
                                   and r['pad_velocity_change'] < CHANGING], STEADY),
        ('pad changed its velocity', [r for r in landed if r['pad_velocity_change'] is None
                                      or r['pad_velocity_change'] >= CHANGING], CHANGED),
    ]
    for name, group, (colour, marker) in groups:
        if group:
            ax.scatter([r['along'] for r in group], [r['across'] for r in group], s=58, marker=marker,
                       color=colour, edgecolors=SURFACE, linewidths=1.6, zorder=5,
                       label=f'{name} ({len(group)})')

    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_aspect('equal')
    ax.set_xlabel('Ahead of the pad centre, the way the pad was travelling (m)', color=INK_SOFT)
    ax.set_ylabel('To the left of its travel (m)', color=INK_SOFT)
    ax.tick_params(colors=INK_SOFT, labelsize=9)
    ax.grid(True, color=GRID, linewidth=0.6, zorder=0)
    for spine in ax.spines.values():
        spine.set_color(GRID)
    shown = 'the pad edge (solid) and marker edge (dashed) are outside this view' if limit < MARKER_HALF else \
        'solid square: 1.25 m pad, dashed: 1.0 m marker'
    ax.set_title(f'Touchdown points on the moving pad, {len(landed)} landings\n{shown}',
                 color=INK, fontsize=11, loc='left')
    # Below the plot, so the legend can never sit on top of a touchdown point.
    legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.11), ncol=3, frameon=False, fontsize=9,
                       handletextpad=0.3, columnspacing=1.4)
    for text in legend.get_texts():
        text.set_color(INK_SOFT)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE, bbox_inches='tight', pad_inches=0.2)
    plt.close(fig)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('csv', nargs='+', help='CSV files written by the moving_pad_trials node')
    parser.add_argument('--out', default='.', help='folder for summary.md and touchdowns.png')
    args = parser.parse_args()

    rows = load_rows(args.csv)
    if not rows:
        sys.exit('No trials found in the given files.')
    summary = summarise(rows)
    report = to_markdown(summary)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'summary.md').write_text(report)
    print(report)
    if summary['error'] is not None and draw(rows, summary, out / 'touchdowns.png'):
        print(f'Wrote {out / "summary.md"} and {out / "touchdowns.png"}')
    else:
        print(f'Wrote {out / "summary.md"}')


if __name__ == '__main__':
    main()
