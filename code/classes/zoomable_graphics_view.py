"""
Enhanced zoomable graphics view with dark theme, dot grid, 
coordinate tracking, context menu, rotation, fit-to-view,
and crosshair cursor support.
"""
from PyQt5.QtWidgets import QGraphicsView, QLabel, QMenu, QAction
from PyQt5.QtCore import Qt, QPointF, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush
from utils.theme import Colors


class ZoomableGraphicsView(QGraphicsView):
    """A graphics view with zoom, pan, dark grid, rotation, and coordinate readout."""

    # Emits (x_meters, y_meters) on mouse move
    coordinateChanged = pyqtSignal(float, float)
    # Emits current rotation angle in degrees
    rotationChanged = pyqtSignal(int)

    def __init__(self, scene):
        super().__init__(scene)
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setBackgroundBrush(QColor(Colors.CANVAS_BG))
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)

        # Allow infinite panning by setting a massive scene rect (10km x 10km)
        self.setSceneRect(-500000, -500000, 1000000, 1000000)

        # Zoom state
        self._zoom_level = 1.0
        self._min_zoom = 0.1
        self._max_zoom = 10.0

        # Pan state
        self.is_panning = False
        self.last_pan_point = QPointF()

        # Grid settings
        self._show_grid = True
        self._grid_spacing = 10  # pixels (= 0.1m)

        # Rotation state (0, 90, 180, 270)
        self._rotation_angle = 0

        # Placement mode cursor
        self._crosshair_mode = False

        # Scale label (overlay in corner)
        self.scale_label = QLabel("", self)
        self.scale_label.setStyleSheet(
            f"background: {Colors.GLASS}; "
            f"font-size: 9pt; font-weight: 600; "
            f"color: {Colors.TEXT_SECONDARY}; "
            f"padding: 4px 10px; "
            f"border-radius: 4px; "
            f"border: 1px solid {Colors.BORDER};"
        )
        self._update_zoom_label()
        self.scale_label.setGeometry(10, 10, 130, 24)

        # Enable mouse tracking for coordinate readout
        self.setMouseTracking(True)

        # Context menu
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self._show_context_menu)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.scale_label.move(10, 10)

    def set_crosshair_mode(self, enabled):
        """Enable/disable crosshair cursor for placement mode."""
        self._crosshair_mode = enabled
        self.setCursor(Qt.CrossCursor if enabled else Qt.ArrowCursor)

    # ── Zoom ───────────────────────────────────────────────────

    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            factor = 1.2
        else:
            factor = 1.0 / 1.2

        new_zoom = self._zoom_level * factor
        if self._min_zoom <= new_zoom <= self._max_zoom:
            self._zoom_level = new_zoom
            # Zoom towards cursor position
            old_pos = self.mapToScene(event.pos())
            self.scale(factor, factor)
            new_pos = self.mapToScene(event.pos())
            delta = new_pos - old_pos
            self.translate(delta.x(), delta.y())
            self._update_zoom_label()

    def zoom_in(self):
        """Zoom in by 20%, centered on viewport."""
        factor = 1.2
        new_zoom = self._zoom_level * factor
        if new_zoom <= self._max_zoom:
            self._zoom_level = new_zoom
            self.scale(factor, factor)
            self._update_zoom_label()

    def zoom_out(self):
        """Zoom out by 20%, centered on viewport."""
        factor = 1.0 / 1.2
        new_zoom = self._zoom_level * factor
        if new_zoom >= self._min_zoom:
            self._zoom_level = new_zoom
            self.scale(factor, factor)
            self._update_zoom_label()

    def _update_zoom_label(self):
        pct = int(self._zoom_level * 100)
        rot_str = f"  ∠{self._rotation_angle}°" if self._rotation_angle != 0 else ""
        # At 100% zoom, 1 pixel = 1 cm (grid spacing 10px = 0.1m)
        self.scale_label.setText(f"⊞ {pct}%  •  10px = 0.1m{rot_str}")
        self.scale_label.adjustSize()

    # ── Pan ────────────────────────────────────────────────────

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self.is_panning = True
            self.last_pan_point = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        # Emit coordinate for status bar
        scene_pos = self.mapToScene(event.pos())
        x_m = scene_pos.x() / 100.0
        y_m = -scene_pos.y() / 100.0
        self.coordinateChanged.emit(x_m, y_m)

        if self.is_panning:
            delta = event.pos() - self.last_pan_point
            self.horizontalScrollBar().setValue(int(self.horizontalScrollBar().value() - delta.x()))
            self.verticalScrollBar().setValue(int(self.verticalScrollBar().value() - delta.y()))
            self.last_pan_point = event.pos()
            event.accept()
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MiddleButton and self.is_panning:
            self.is_panning = False
            if self._crosshair_mode:
                self.setCursor(Qt.CrossCursor)
            else:
                self.setCursor(Qt.ArrowCursor)
            event.accept()
        else:
            super().mouseReleaseEvent(event)

    # ── Rotation ──────────────────────────────────────────────

    def rotate_cw(self):
        """Rotate the view 90° clockwise."""
        self._rotation_angle = (self._rotation_angle + 90) % 360
        self.rotate(90)
        self._update_zoom_label()
        self.rotationChanged.emit(self._rotation_angle)

    def rotate_ccw(self):
        """Rotate the view 90° counter-clockwise."""
        self._rotation_angle = (self._rotation_angle - 90) % 360
        self.rotate(-90)
        self._update_zoom_label()
        self.rotationChanged.emit(self._rotation_angle)

    # ── Fit to View ───────────────────────────────────────────

    def fit_all(self):
        """Fit all scene content into the viewport."""
        scene_rect = self.scene().itemsBoundingRect()
        if scene_rect.isNull():
            # No items — just center on origin
            self.centerOn(0, 0)
            return
        # Add some padding
        margin = 40
        scene_rect.adjust(-margin, -margin, margin, margin)
        self.fitInView(scene_rect, Qt.KeepAspectRatio)
        # Update internal zoom level to match
        t = self.transform()
        self._zoom_level = t.m11()  # horizontal scale factor
        self._update_zoom_label()

    # ── Grid Toggle ───────────────────────────────────────────

    def set_grid_visible(self, visible):
        """Show or hide the grid."""
        self._show_grid = visible
        self.viewport().update()

    # ── Grid Drawing ───────────────────────────────────────────

    def drawBackground(self, painter, rect):
        """Draw dark background with dot grid."""
        # Fill background
        painter.fillRect(rect, QColor(Colors.CANVAS_BG))

        if not self._show_grid:
            return

        spacing = self._grid_spacing

        # Get visible rect
        left = int(rect.left()) - (int(rect.left()) % spacing)
        top = int(rect.top()) - (int(rect.top()) % spacing)

        # Draw dot grid
        dot_color = QColor(Colors.CANVAS_GRID)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(dot_color))

        x = left
        while x <= rect.right():
            y = top
            while y <= rect.bottom():
                # Larger dots at major grid lines (every 100px = 1m)
                if x % 100 == 0 and y % 100 == 0:
                    painter.setBrush(QBrush(QColor(Colors.CANVAS_AXIS)))
                    painter.drawEllipse(QPointF(x, y), 2.0, 2.0)
                    painter.setBrush(QBrush(dot_color))
                else:
                    painter.drawEllipse(QPointF(x, y), 0.8, 0.8)
                y += spacing
            x += spacing

        # Draw origin crosshair
        axis_pen = QPen(QColor(Colors.CANVAS_AXIS), 1, Qt.DashLine)
        painter.setPen(axis_pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawLine(int(rect.left()), 0, int(rect.right()), 0)
        painter.drawLine(0, int(rect.top()), 0, int(rect.bottom()))

    # ── Context Menu ───────────────────────────────────────────

    def _show_context_menu(self, pos):
        menu = QMenu(self)

        reset_action = QAction("⟲  Reset Zoom", self)
        reset_action.triggered.connect(self._reset_zoom)
        menu.addAction(reset_action)

        fit_action = QAction("⊞  Fit to View", self)
        fit_action.triggered.connect(self.fit_all)
        menu.addAction(fit_action)

        center_action = QAction("◎  Center View", self)
        center_action.triggered.connect(self._center_view)
        menu.addAction(center_action)

        menu.addSeparator()

        rot_cw_action = QAction("↻  Rotate 90° CW", self)
        rot_cw_action.triggered.connect(self.rotate_cw)
        menu.addAction(rot_cw_action)

        rot_ccw_action = QAction("↺  Rotate 90° CCW", self)
        rot_ccw_action.triggered.connect(self.rotate_ccw)
        menu.addAction(rot_ccw_action)

        menu.addSeparator()

        grid_action = QAction("⊞  Toggle Grid", self)
        grid_action.setCheckable(True)
        grid_action.setChecked(self._show_grid)
        grid_action.triggered.connect(self._toggle_grid)
        menu.addAction(grid_action)

        menu.exec_(self.mapToGlobal(pos))

    def _reset_zoom(self):
        from PyQt5.QtGui import QTransform
        self.setTransform(QTransform().rotate(self._rotation_angle))
        self._zoom_level = 1.0
        self._update_zoom_label()

    def _center_view(self):
        self.centerOn(0, 0)

    def _toggle_grid(self, checked):
        self._show_grid = checked
        self.viewport().update()