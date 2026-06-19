"""
Reusable color picker widget — displays color swatches for selection.
"""
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QSizePolicy
from PyQt5.QtCore import pyqtSignal, Qt, QSize
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush
from utils.color_utils import get_hex_color, get_color_names


class ColorSwatch(QWidget):
    """A single clickable color swatch circle."""
    clicked = pyqtSignal(str)

    def __init__(self, color_name, hex_color, size=28, parent=None):
        super().__init__(parent)
        self.color_name = color_name
        self.hex_color = hex_color
        self._selected = False
        self._hovered = False
        self._size = size
        self.setFixedSize(size + 8, size + 8)
        self.setCursor(Qt.PointingHandCursor)
        self.setToolTip(color_name)

    @property
    def selected(self):
        return self._selected

    @selected.setter
    def selected(self, value):
        self._selected = value
        self.update()

    def enterEvent(self, event):
        self._hovered = True
        self.update()

    def leaveEvent(self, event):
        self._hovered = False
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(self.color_name)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        center_x = self.width() / 2
        center_y = self.height() / 2
        radius = self._size / 2

        # Draw selection ring
        if self._selected:
            painter.setPen(QPen(QColor("#4A90E2"), 2.5))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(int(center_x - radius - 2), int(center_y - radius - 2),
                                int(radius * 2 + 4), int(radius * 2 + 4))

        # Draw hover ring
        elif self._hovered:
            painter.setPen(QPen(QColor("#9E9E9E"), 1.5))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(int(center_x - radius - 1), int(center_y - radius - 1),
                                int(radius * 2 + 2), int(radius * 2 + 2))

        # Draw swatch circle
        color = QColor(self.hex_color)
        painter.setPen(QPen(QColor("#3D4A5C"), 1))
        painter.setBrush(QBrush(color))
        painter.drawEllipse(int(center_x - radius), int(center_y - radius),
                            int(radius * 2), int(radius * 2))

        # Draw white border for very dark colors
        if color.lightnessF() < 0.15:
            painter.setPen(QPen(QColor("#555555"), 1))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(int(center_x - radius), int(center_y - radius),
                                int(radius * 2), int(radius * 2))

        painter.end()


class ColorPicker(QWidget):
    """A row of color swatches for selecting a color by name."""
    colorSelected = pyqtSignal(str)

    def __init__(self, default_color="Gray", swatch_size=28, parent=None):
        super().__init__(parent)
        self._current_color = default_color

        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        # Section label
        label = QLabel("Color")
        label.setProperty("cssClass", "section")
        layout.addWidget(label)

        # Swatches in a flow layout (2 rows of 6)
        row1 = QHBoxLayout()
        row1.setSpacing(4)
        row1.setContentsMargins(0, 0, 0, 0)
        row2 = QHBoxLayout()
        row2.setSpacing(4)
        row2.setContentsMargins(0, 0, 0, 0)

        self.swatches = {}
        color_names = get_color_names()

        for i, name in enumerate(color_names):
            swatch = ColorSwatch(name, get_hex_color(name), swatch_size)
            swatch.clicked.connect(self._on_swatch_clicked)
            if name == default_color:
                swatch.selected = True
            self.swatches[name] = swatch
            if i < 6:
                row1.addWidget(swatch)
            else:
                row2.addWidget(swatch)

        # Add stretch to keep swatches left-aligned
        row1.addStretch()
        row2.addStretch()

        layout.addLayout(row1)
        layout.addLayout(row2)
        self.setLayout(layout)

    @property
    def current_color(self):
        return self._current_color

    def _on_swatch_clicked(self, color_name):
        # Deselect previous
        if self._current_color in self.swatches:
            self.swatches[self._current_color].selected = False

        # Select new
        self._current_color = color_name
        self.swatches[color_name].selected = True
        self.colorSelected.emit(color_name)

    def set_color(self, color_name):
        """Programmatically set the selected color."""
        if color_name in self.swatches:
            self._on_swatch_clicked(color_name)
