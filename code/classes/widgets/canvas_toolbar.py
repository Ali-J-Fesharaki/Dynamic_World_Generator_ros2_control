"""
Canvas Toolbar — Floating overlay with zoom, fit, rotate, and grid controls.
Inspired by the grid-map-editor's NavToolsPanel.qml.

Positioned in the top-right corner of the canvas area.
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame,
                             QToolTip, QSizePolicy)
from PyQt5.QtCore import Qt, pyqtSignal
from utils.theme import Colors, Dims


class _ToolBtn(QPushButton):
    """Small icon button for the toolbar."""
    def __init__(self, icon_text, tooltip, parent=None):
        super().__init__(icon_text, parent)
        self.setFixedSize(36, 36)
        self.setCursor(Qt.PointingHandCursor)
        self.setToolTip(tooltip)
        self.setProperty("cssClass", "toolbar-btn")
        # Inline style as fallback (the global QSS will also style via cssClass)
        self.setStyleSheet(
            f"QPushButton{{background:transparent; color:{Colors.TEXT_SECONDARY}; "
            f"border:none; border-radius:{Dims.RADIUS_SM}px; font-size:14pt; "
            f"min-height:32px; min-width:32px;}}"
            f"QPushButton:hover{{background:{Colors.TOOLBAR_BTN_HOVER}; color:{Colors.TEXT_PRIMARY};}}"
            f"QPushButton:pressed{{background:{Colors.PRIMARY}; color:white;}}")


class CanvasToolbar(QWidget):
    """Floating toolbar overlay for canvas controls."""

    # Signals
    zoomInRequested = pyqtSignal()
    zoomOutRequested = pyqtSignal()
    fitAllRequested = pyqtSignal()
    rotateCWRequested = pyqtSignal()
    rotateCCWRequested = pyqtSignal()
    gridToggled = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._grid_on = True
        self.setFixedWidth(48)
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(
            f"CanvasToolbar {{"
            f"background: {Colors.TOOLBAR_BG}; "
            f"border: 1px solid {Colors.BORDER}; "
            f"border-radius: {Dims.RADIUS_MD}px;"
            f"}}")

        lay = QVBoxLayout()
        lay.setContentsMargins(6, 8, 6, 8)
        lay.setSpacing(4)

        # Zoom In
        btn_zi = _ToolBtn("🔍", "Zoom In")
        btn_zi.clicked.connect(self.zoomInRequested.emit)
        lay.addWidget(btn_zi)

        # Zoom Out
        btn_zo = _ToolBtn("🔎", "Zoom Out")
        btn_zo.clicked.connect(self.zoomOutRequested.emit)
        lay.addWidget(btn_zo)

        # Separator
        sep1 = QFrame()
        sep1.setFixedHeight(1)
        sep1.setStyleSheet(f"background:{Colors.BORDER};")
        lay.addWidget(sep1)

        # Fit All
        btn_fit = _ToolBtn("⊞", "Fit to View")
        btn_fit.clicked.connect(self.fitAllRequested.emit)
        lay.addWidget(btn_fit)

        # Separator
        sep2 = QFrame()
        sep2.setFixedHeight(1)
        sep2.setStyleSheet(f"background:{Colors.BORDER};")
        lay.addWidget(sep2)

        # Rotate CW
        btn_cw = _ToolBtn("↻", "Rotate 90° CW")
        btn_cw.clicked.connect(self.rotateCWRequested.emit)
        lay.addWidget(btn_cw)

        # Rotate CCW
        btn_ccw = _ToolBtn("↺", "Rotate 90° CCW")
        btn_ccw.clicked.connect(self.rotateCCWRequested.emit)
        lay.addWidget(btn_ccw)

        # Separator
        sep3 = QFrame()
        sep3.setFixedHeight(1)
        sep3.setStyleSheet(f"background:{Colors.BORDER};")
        lay.addWidget(sep3)

        # Grid Toggle
        self.grid_btn = _ToolBtn("#", "Toggle Grid")
        self.grid_btn.setCheckable(True)
        self.grid_btn.setChecked(True)
        self.grid_btn.clicked.connect(self._on_grid_toggled)
        lay.addWidget(self.grid_btn)

        self.setLayout(lay)
        self.adjustSize()

    def _on_grid_toggled(self, checked):
        self._grid_on = checked
        self.gridToggled.emit(checked)
        # Visual feedback
        if checked:
            self.grid_btn.setStyleSheet(
                f"QPushButton{{background:transparent; color:{Colors.TEXT_SECONDARY}; "
                f"border:none; border-radius:{Dims.RADIUS_SM}px; font-size:14pt; "
                f"min-height:32px; min-width:32px;}}"
                f"QPushButton:hover{{background:{Colors.TOOLBAR_BTN_HOVER}; color:{Colors.TEXT_PRIMARY};}}")
        else:
            self.grid_btn.setStyleSheet(
                f"QPushButton{{background:{Colors.BG_ELEVATED}; color:{Colors.TEXT_DISABLED}; "
                f"border:none; border-radius:{Dims.RADIUS_SM}px; font-size:14pt; "
                f"min-height:32px; min-width:32px;}}"
                f"QPushButton:hover{{background:{Colors.TOOLBAR_BTN_HOVER}; color:{Colors.TEXT_PRIMARY};}}")

    def position_in_parent(self):
        """Reposition self to top-right of parent widget."""
        if self.parentWidget():
            pr = self.parentWidget().rect()
            self.move(pr.width() - self.width() - 12, 12)
