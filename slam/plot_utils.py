import matplotlib.patheffects as path_effects
from matplotlib.artist import Artist

BACKGROUND_COLOR: str = "#fff9ed"


def outline(artist: Artist, lw: float = 1, color: str = BACKGROUND_COLOR):
    artist.set_path_effects(
        [path_effects.withStroke(linewidth=lw, foreground=color,)]
    )
