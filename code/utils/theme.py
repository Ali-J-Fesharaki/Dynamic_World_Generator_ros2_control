"""
Centralized Theme Module for Dynamic World Generator Wizard.
Provides color palette, global stylesheet, and font setup.
"""
from PyQt5.QtGui import QFont, QFontDatabase, QColor
from PyQt5.QtCore import Qt
import os

# ── Color Palette ──────────────────────────────────────────────

class Colors:
    """Application-wide color constants."""
    # Backgrounds
    BG_DARKEST    = "#0D1117"
    BG_DARK       = "#161B22"
    BG_PRIMARY    = "#1A1A2E"
    BG_SURFACE    = "#16213E"
    BG_ELEVATED   = "#1F2937"
    BG_CARD       = "#1E293B"
    BG_CARD_HOVER = "#263548"

    # Accent
    PRIMARY       = "#4A90E2"
    PRIMARY_HOVER = "#6AB0F3"
    PRIMARY_DARK  = "#3A7BD5"
    SECONDARY     = "#64FFDA"

    # Semantic
    SUCCESS       = "#4CAF50"
    WARNING       = "#FF9800"
    ERROR         = "#F44336"
    INFO          = "#29B6F6"

    # Text
    TEXT_PRIMARY   = "#E0E0E0"
    TEXT_SECONDARY = "#9E9E9E"
    TEXT_DISABLED  = "#555555"
    TEXT_ACCENT    = "#4A90E2"

    # Borders
    BORDER         = "#2D3748"
    BORDER_LIGHT   = "#3D4A5C"
    BORDER_FOCUS   = "#4A90E2"

    # Canvas
    CANVAS_BG      = "#0D1117"
    CANVAS_GRID    = "#1C2333"
    CANVAS_AXIS    = "#2D3748"

    # Navigation
    NAV_BG         = "#0F1923"
    NAV_ACTIVE     = "#4A90E2"
    NAV_COMPLETED  = "#4CAF50"
    NAV_INACTIVE   = "#3D4A5C"

    # Misc
    SHADOW         = "rgba(0, 0, 0, 0.3)"
    OVERLAY        = "rgba(0, 0, 0, 0.6)"
    GLASS          = "rgba(30, 41, 59, 0.85)"

    # Status Log Panel
    LOG_BG         = "#111827"
    LOG_HEADER     = "#1F2937"
    LOG_TERMINAL   = "#0A0A0A"
    OVERLAY_DIM    = "rgba(0, 0, 0, 0.4)"

    # Canvas Toolbar
    TOOLBAR_BG     = "rgba(17, 24, 39, 0.92)"
    TOOLBAR_BTN_HOVER = "#374151"


# ── Dimensions ─────────────────────────────────────────────────

class Dims:
    """Consistent spacing and sizing constants."""
    RADIUS_SM  = 4
    RADIUS_MD  = 8
    RADIUS_LG  = 12
    RADIUS_XL  = 16

    PAD_XS  = 4
    PAD_SM  = 8
    PAD_MD  = 12
    PAD_LG  = 16
    PAD_XL  = 24

    SPACING_SM = 6
    SPACING_MD = 10
    SPACING_LG = 16

    BUTTON_HEIGHT     = 40
    BUTTON_HEIGHT_LG  = 48
    INPUT_HEIGHT      = 36
    NAV_ICON_SIZE     = 32


# ── Font Setup ─────────────────────────────────────────────────

def setup_fonts():
    """Load custom fonts. Returns the primary font family name."""
    # Try loading Inter if bundled; fallback to system fonts
    font_dir = os.path.join(os.path.dirname(__file__), "..", "..", "fonts")
    if os.path.isdir(font_dir):
        for fname in os.listdir(font_dir):
            if fname.lower().endswith((".ttf", ".otf")):
                QFontDatabase.addApplicationFont(os.path.join(font_dir, fname))

    # Check if Inter is available
    db = QFontDatabase()
    families = db.families()
    if "Inter" in families:
        return "Inter"
    # Fallbacks
    for fallback in ["Segoe UI", "Roboto", "Noto Sans", "Ubuntu", "DejaVu Sans"]:
        if fallback in families:
            return fallback
    return "sans-serif"


# ── Global Stylesheet ──────────────────────────────────────────

def _build_stylesheet(font_family):
    """Build the complete QSS stylesheet string."""
    return f"""
    /* ─── Global ─── */
    * {{
        font-family: "{font_family}", sans-serif;
        font-size: 11pt;
    }}

    QWidget {{
        background-color: {Colors.BG_PRIMARY};
        color: {Colors.TEXT_PRIMARY};
    }}

    /* ─── QWizard ─── */
    QWizard {{
        background-color: {Colors.BG_DARK};
    }}
    QWizard > QWidget {{
        background-color: {Colors.BG_PRIMARY};
    }}

    /* ─── Buttons ─── */
    QPushButton {{
        background-color: {Colors.BG_ELEVATED};
        color: {Colors.TEXT_PRIMARY};
        border: 1px solid {Colors.BORDER};
        border-radius: {Dims.RADIUS_MD}px;
        padding: {Dims.PAD_SM}px {Dims.PAD_LG}px;
        min-height: {Dims.BUTTON_HEIGHT}px;
        font-weight: 500;
    }}
    QPushButton:hover {{
        background-color: {Colors.BG_CARD_HOVER};
        border-color: {Colors.BORDER_LIGHT};
    }}
    QPushButton:pressed {{
        background-color: {Colors.BG_SURFACE};
    }}
    QPushButton:disabled {{
        background-color: {Colors.BG_DARK};
        color: {Colors.TEXT_DISABLED};
        border-color: {Colors.BG_DARK};
    }}

    QPushButton[cssClass="primary"] {{
        background-color: {Colors.PRIMARY};
        color: white;
        border: none;
        font-weight: 600;
    }}
    QPushButton[cssClass="primary"]:hover {{
        background-color: {Colors.PRIMARY_HOVER};
    }}
    QPushButton[cssClass="primary"]:pressed {{
        background-color: {Colors.PRIMARY_DARK};
    }}

    QPushButton[cssClass="success"] {{
        background-color: {Colors.SUCCESS};
        color: white;
        border: none;
        font-weight: 600;
    }}
    QPushButton[cssClass="success"]:hover {{
        background-color: #66BB6A;
    }}

    QPushButton[cssClass="danger"] {{
        background-color: transparent;
        color: {Colors.ERROR};
        border: 1px solid {Colors.ERROR};
    }}
    QPushButton[cssClass="danger"]:hover {{
        background-color: {Colors.ERROR};
        color: white;
    }}

    /* ─── Inputs ─── */
    QLineEdit, QSpinBox, QDoubleSpinBox {{
        background-color: {Colors.BG_DARK};
        color: {Colors.TEXT_PRIMARY};
        border: 1px solid {Colors.BORDER};
        border-radius: {Dims.RADIUS_SM}px;
        padding: {Dims.PAD_SM}px {Dims.PAD_MD}px;
        min-height: {Dims.INPUT_HEIGHT}px;
        selection-background-color: {Colors.PRIMARY};
    }}
    QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {{
        border-color: {Colors.BORDER_FOCUS};
    }}
    QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled {{
        background-color: {Colors.BG_PRIMARY};
        color: {Colors.TEXT_DISABLED};
    }}
    QLineEdit[cssClass="error"] {{
        border-color: {Colors.ERROR};
    }}

    /* ─── ComboBox ─── */
    QComboBox {{
        background-color: {Colors.BG_DARK};
        color: {Colors.TEXT_PRIMARY};
        border: 1px solid {Colors.BORDER};
        border-radius: {Dims.RADIUS_SM}px;
        padding: {Dims.PAD_SM}px {Dims.PAD_MD}px;
        min-height: {Dims.INPUT_HEIGHT}px;
    }}
    QComboBox:hover {{
        border-color: {Colors.BORDER_LIGHT};
    }}
    QComboBox::drop-down {{
        border: none;
        width: 30px;
    }}
    QComboBox::down-arrow {{
        image: none;
        border-left: 5px solid transparent;
        border-right: 5px solid transparent;
        border-top: 6px solid {Colors.TEXT_SECONDARY};
        margin-right: 10px;
    }}
    QComboBox QAbstractItemView {{
        background-color: {Colors.BG_DARK};
        color: {Colors.TEXT_PRIMARY};
        border: 1px solid {Colors.BORDER};
        selection-background-color: {Colors.PRIMARY};
        selection-color: white;
        outline: none;
    }}

    /* ─── ListWidget ─── */
    QListWidget {{
        background-color: {Colors.BG_DARK};
        color: {Colors.TEXT_PRIMARY};
        border: 1px solid {Colors.BORDER};
        border-radius: {Dims.RADIUS_SM}px;
        padding: {Dims.PAD_XS}px;
        outline: none;
    }}
    QListWidget::item {{
        padding: {Dims.PAD_SM}px {Dims.PAD_MD}px;
        border-radius: {Dims.RADIUS_SM}px;
        margin: 1px 0px;
    }}
    QListWidget::item:selected {{
        background-color: {Colors.PRIMARY};
        color: white;
    }}
    QListWidget::item:hover:!selected {{
        background-color: {Colors.BG_ELEVATED};
    }}

    /* ─── Labels ─── */
    QLabel {{
        background-color: transparent;
        color: {Colors.TEXT_PRIMARY};
        border: none;
    }}
    QLabel[cssClass="heading"] {{
        font-size: 16pt;
        font-weight: 700;
        color: {Colors.TEXT_PRIMARY};
    }}
    QLabel[cssClass="subheading"] {{
        font-size: 12pt;
        font-weight: 600;
        color: {Colors.TEXT_SECONDARY};
    }}
    QLabel[cssClass="caption"] {{
        font-size: 9pt;
        color: {Colors.TEXT_SECONDARY};
    }}
    QLabel[cssClass="section"] {{
        font-size: 10pt;
        font-weight: 600;
        color: {Colors.PRIMARY};
        padding-top: 8px;
        padding-bottom: 4px;
    }}
    QLabel[cssClass="accent"] {{
        color: {Colors.SECONDARY};
        font-weight: 600;
    }}

    /* ─── Frames / Cards ─── */
    QFrame[cssClass="card"] {{
        background-color: {Colors.BG_CARD};
        border: 1px solid {Colors.BORDER};
        border-radius: {Dims.RADIUS_LG}px;
    }}
    QFrame[cssClass="card"]:hover {{
        border-color: {Colors.BORDER_LIGHT};
    }}
    QFrame[cssClass="card-selected"] {{
        background-color: {Colors.BG_CARD};
        border: 2px solid {Colors.PRIMARY};
        border-radius: {Dims.RADIUS_LG}px;
    }}
    QFrame[cssClass="separator"] {{
        background-color: {Colors.BORDER};
        max-height: 1px;
        min-height: 1px;
    }}

    /* ─── Scrollbars ─── */
    QScrollBar:vertical {{
        background: {Colors.BG_DARK};
        width: 10px;
        margin: 0;
        border-radius: 5px;
    }}
    QScrollBar::handle:vertical {{
        background: {Colors.BORDER_LIGHT};
        min-height: 30px;
        border-radius: 5px;
    }}
    QScrollBar::handle:vertical:hover {{
        background: {Colors.TEXT_SECONDARY};
    }}
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
        height: 0;
    }}
    QScrollBar:horizontal {{
        background: {Colors.BG_DARK};
        height: 10px;
        margin: 0;
        border-radius: 5px;
    }}
    QScrollBar::handle:horizontal {{
        background: {Colors.BORDER_LIGHT};
        min-width: 30px;
        border-radius: 5px;
    }}
    QScrollBar::handle:horizontal:hover {{
        background: {Colors.TEXT_SECONDARY};
    }}
    QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
        width: 0;
    }}

    /* ─── ToolTip ─── */
    QToolTip {{
        background-color: {Colors.BG_ELEVATED};
        color: {Colors.TEXT_PRIMARY};
        border: 1px solid {Colors.BORDER};
        border-radius: {Dims.RADIUS_SM}px;
        padding: {Dims.PAD_SM}px;
        font-size: 10pt;
    }}

    /* ─── MessageBox ─── */
    QMessageBox {{
        background-color: {Colors.BG_PRIMARY};
    }}
    QMessageBox QLabel {{
        color: {Colors.TEXT_PRIMARY};
        font-size: 11pt;
    }}

    /* ─── Menu ─── */
    QMenu {{
        background-color: {Colors.BG_DARK};
        color: {Colors.TEXT_PRIMARY};
        border: 1px solid {Colors.BORDER};
        border-radius: {Dims.RADIUS_SM}px;
        padding: {Dims.PAD_XS}px;
    }}
    QMenu::item {{
        padding: {Dims.PAD_SM}px {Dims.PAD_LG}px;
        border-radius: {Dims.RADIUS_SM}px;
    }}
    QMenu::item:selected {{
        background-color: {Colors.PRIMARY};
        color: white;
    }}
    QMenu::separator {{
        height: 1px;
        background-color: {Colors.BORDER};
        margin: 4px 8px;
    }}

    /* ─── Wizard-specific overrides ─── */
    QWizard QPushButton#qt_wizard_commit,
    QWizard QPushButton#qt_wizard_finish {{
        background-color: {Colors.PRIMARY};
        color: white;
        border: none;
        font-weight: 600;
        border-radius: {Dims.RADIUS_MD}px;
        padding: {Dims.PAD_SM}px {Dims.PAD_XL}px;
    }}
    QWizard QPushButton#qt_wizard_commit:hover,
    QWizard QPushButton#qt_wizard_finish:hover {{
        background-color: {Colors.PRIMARY_HOVER};
    }}
    QWizard QPushButton#qt_wizard_cancel {{
        background-color: transparent;
        color: {Colors.TEXT_SECONDARY};
        border: 1px solid {Colors.BORDER};
    }}
    QWizard QPushButton#qt_wizard_cancel:hover {{
        border-color: {Colors.ERROR};
        color: {Colors.ERROR};
    }}

    /* ─── QTextEdit (read-only terminal) ─── */
    QTextEdit[cssClass="terminal"] {{
        background-color: {Colors.LOG_TERMINAL};
        color: {Colors.TEXT_PRIMARY};
        border: 1px solid {Colors.BORDER};
        border-radius: {Dims.RADIUS_SM}px;
        font-family: "JetBrains Mono", "Fira Code", "Consolas", monospace;
        font-size: 10pt;
        padding: {Dims.PAD_SM}px;
        selection-background-color: {Colors.PRIMARY};
    }}

    /* ─── Floating toolbar buttons ─── */
    QPushButton[cssClass="toolbar-btn"] {{
        background-color: transparent;
        color: {Colors.TEXT_SECONDARY};
        border: none;
        border-radius: {Dims.RADIUS_SM}px;
        font-size: 14pt;
        padding: 4px;
        min-height: 32px;
        min-width: 32px;
    }}
    QPushButton[cssClass="toolbar-btn"]:hover {{
        background-color: {Colors.TOOLBAR_BTN_HOVER};
        color: {Colors.TEXT_PRIMARY};
    }}
    QPushButton[cssClass="toolbar-btn"]:pressed {{
        background-color: {Colors.PRIMARY};
        color: white;
    }}
    """


# ── Public API ─────────────────────────────────────────────────

def apply_global_stylesheet(app):
    """Apply the complete dark theme to a QApplication instance."""
    font_family = setup_fonts()
    app.setFont(QFont(font_family, 11))
    stylesheet = _build_stylesheet(font_family)
    app.setStyleSheet(stylesheet)


def make_primary_button(button):
    """Mark a QPushButton as primary-styled."""
    button.setProperty("cssClass", "primary")
    button.style().unpolish(button)
    button.style().polish(button)


def make_danger_button(button):
    """Mark a QPushButton as danger-styled."""
    button.setProperty("cssClass", "danger")
    button.style().unpolish(button)
    button.style().polish(button)


def make_success_button(button):
    """Mark a QPushButton as success-styled."""
    button.setProperty("cssClass", "success")
    button.style().unpolish(button)
    button.style().polish(button)


def make_section_label(label):
    """Mark a QLabel as a section header."""
    label.setProperty("cssClass", "section")
    label.style().unpolish(label)
    label.style().polish(label)


def make_heading_label(label):
    """Mark a QLabel as a heading."""
    label.setProperty("cssClass", "heading")
    label.style().unpolish(label)
    label.style().polish(label)


def make_subheading_label(label):
    """Mark a QLabel as a subheading."""
    label.setProperty("cssClass", "subheading")
    label.style().unpolish(label)
    label.style().polish(label)


def make_caption_label(label):
    """Mark a QLabel as a caption."""
    label.setProperty("cssClass", "caption")
    label.style().unpolish(label)
    label.style().polish(label)


def make_card_frame(frame):
    """Mark a QFrame as a card."""
    frame.setProperty("cssClass", "card")
    frame.style().unpolish(frame)
    frame.style().polish(frame)


def make_card_selected_frame(frame):
    """Mark a QFrame as a selected card."""
    frame.setProperty("cssClass", "card-selected")
    frame.style().unpolish(frame)
    frame.style().polish(frame)
