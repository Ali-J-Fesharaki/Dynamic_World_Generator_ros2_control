"""Enhanced color utilities with QColor support and expanded palette."""
from PyQt5.QtGui import QColor


# Full color palette — maps display names to (R, G, B) float tuples (0–1)
COLOR_PALETTE = {
    "Black":   (0.0,  0.0,  0.0),
    "Gray":    (0.5,  0.5,  0.5),
    "White":   (1.0,  1.0,  1.0),
    "Red":     (0.9,  0.2,  0.2),
    "Blue":    (0.2,  0.4,  0.9),
    "Green":   (0.2,  0.8,  0.3),
    "Orange":  (1.0,  0.6,  0.0),
    "Purple":  (0.6,  0.2,  0.8),
    "Yellow":  (1.0,  0.9,  0.2),
    "Cyan":    (0.0,  0.8,  0.8),
    "Brown":   (0.55, 0.35, 0.17),
    "Pink":    (0.9,  0.4,  0.6),
}

# Hex colors for UI display (used by the color picker widget)
COLOR_HEX = {
    "Black":   "#1A1A1A",
    "Gray":    "#808080",
    "White":   "#F0F0F0",
    "Red":     "#E63946",
    "Blue":    "#3366E6",
    "Green":   "#33CC4D",
    "Orange":  "#FF9900",
    "Purple":  "#9933CC",
    "Yellow":  "#FFE633",
    "Cyan":    "#00CCCC",
    "Brown":   "#8C5A2B",
    "Pink":    "#E6668F",
}


def get_color(color_name):
    """Map color name to (R, G, B) float tuple for SDF/Gazebo.
    
    Backwards compatible: returns the float tuple used by the existing
    QColor.fromRgbF() calls and SDF material generation.
    """
    return COLOR_PALETTE.get(color_name, COLOR_PALETTE["Gray"])


def get_qcolor(color_name):
    """Map color name to a QColor instance."""
    rgb = COLOR_PALETTE.get(color_name, COLOR_PALETTE["Gray"])
    return QColor.fromRgbF(*rgb)


def get_hex_color(color_name):
    """Map color name to hex string for stylesheets."""
    return COLOR_HEX.get(color_name, COLOR_HEX["Gray"])


def get_all_colors():
    """Return dict of all color names to hex values."""
    return dict(COLOR_HEX)


def get_color_names():
    """Return list of all available color names in display order."""
    return list(COLOR_PALETTE.keys())