"""
Status & Log Panel — Slide-out drawer from the right edge.
Inspired by the grid-map-editor's StatusPanel.qml.

Shows:
  - Terminal log area with timestamped, color-coded messages
  - World stats summary (walls, obstacles, dynamic objects)
  - Copy All / Clear controls
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                             QPushButton, QTextEdit, QFrame, QApplication,
                             QGraphicsOpacityEffect)
from PyQt5.QtCore import (Qt, QPropertyAnimation, QEasingCurve, QRect,
                          pyqtSignal, pyqtProperty, QDateTime)
from PyQt5.QtGui import QColor, QPainter
from utils.theme import Colors, Dims


class _DimOverlay(QWidget):
    """Semi-transparent dark overlay behind the panel."""
    clicked = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._opacity = 0.0
        self.setAttribute(Qt.WA_TransparentForMouseEvents, False)
        self.hide()

    def get_opacity(self):
        return self._opacity

    def set_opacity(self, v):
        self._opacity = v
        self.update()

    opacity_value = pyqtProperty(float, get_opacity, set_opacity)

    def paintEvent(self, event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(0, 0, 0, int(self._opacity * 255)))
        p.end()

    def mousePressEvent(self, event):
        self.clicked.emit()
        event.accept()


class StatusLogPanel(QWidget):
    """Right-side slide-out drawer with terminal logs and world stats."""

    def __init__(self, parent=None, app_state=None):
        super().__init__(parent)
        self.app_state = app_state
        self._is_open = False

        # Panel doesn't paint its own background — children handle it
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background: transparent;")

        # ── Dark overlay ──────────────────────────────────────
        self.overlay = _DimOverlay(parent)
        self.overlay.clicked.connect(self.close_panel)
        self.overlay.hide()

        # ── The panel itself ──────────────────────────────────
        self.panel = QFrame(parent)
        self.panel.setStyleSheet(
            f"QFrame#logPanel {{"
            f"background: {Colors.LOG_BG}; "
            f"border-left: 1px solid {Colors.BORDER};"
            f"}}")
        self.panel.setObjectName("logPanel")
        self.panel.hide()

        lay = QVBoxLayout()
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        # ── Header ────────────────────────────────────────────
        header = QFrame()
        header.setFixedHeight(50)
        header.setStyleSheet(f"background: {Colors.LOG_HEADER};")
        hdr_lay = QHBoxLayout()
        hdr_lay.setContentsMargins(20, 0, 16, 0)

        title = QLabel("System Status & Logs")
        title.setStyleSheet(
            f"font-size: 14pt; font-weight: 700; color: {Colors.TEXT_PRIMARY}; "
            f"background: transparent;")
        hdr_lay.addWidget(title)
        hdr_lay.addStretch()

        close_btn = QPushButton("✕")
        close_btn.setFixedSize(32, 32)
        close_btn.setCursor(Qt.PointingHandCursor)
        close_btn.setStyleSheet(
            f"QPushButton{{background:transparent; color:{Colors.TEXT_SECONDARY}; "
            f"font-size:16pt; border:none; border-radius:4px;}}"
            f"QPushButton:hover{{background:{Colors.BG_ELEVATED}; color:{Colors.TEXT_PRIMARY};}}")
        close_btn.clicked.connect(self.close_panel)
        hdr_lay.addWidget(close_btn)
        header.setLayout(hdr_lay)
        lay.addWidget(header)

        # ── Terminal area ─────────────────────────────────────
        term_frame = QFrame()
        term_frame.setStyleSheet(f"background: {Colors.LOG_TERMINAL}; border-radius: 4px;")
        term_lay = QVBoxLayout()
        term_lay.setContentsMargins(15, 10, 15, 10)
        term_lay.setSpacing(0)

        # Terminal toolbar
        tb_row = QHBoxLayout()
        tb_label = QLabel("TERMINAL")
        tb_label.setStyleSheet(
            f"color: #4ADE80; font-size: 9pt; font-weight: 700; "
            f"letter-spacing: 1px; background: transparent;")
        tb_row.addWidget(tb_label)
        tb_row.addStretch()

        copy_btn = QPushButton("Copy All")
        copy_btn.setCursor(Qt.PointingHandCursor)
        copy_btn.setStyleSheet(
            f"QPushButton{{background:transparent; color:{Colors.TEXT_SECONDARY}; "
            f"font-size:10pt; border:none; padding: 4px 8px; border-radius:4px;}}"
            f"QPushButton:hover{{background:{Colors.BG_ELEVATED};}}")
        copy_btn.clicked.connect(self._copy_all)
        tb_row.addWidget(copy_btn)

        clear_btn = QPushButton("Clear")
        clear_btn.setCursor(Qt.PointingHandCursor)
        clear_btn.setStyleSheet(
            f"QPushButton{{background:transparent; color:{Colors.TEXT_SECONDARY}; "
            f"font-size:10pt; border:none; padding: 4px 8px; border-radius:4px;}}"
            f"QPushButton:hover{{background:{Colors.BG_ELEVATED};}}")
        clear_btn.clicked.connect(self._clear_logs)
        tb_row.addWidget(clear_btn)

        term_lay.addLayout(tb_row)

        # Log output
        self.terminal = QTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setProperty("cssClass", "terminal")
        self.terminal.setStyleSheet(
            f"QTextEdit{{background:{Colors.LOG_TERMINAL}; color:{Colors.TEXT_PRIMARY}; "
            f"border:none; font-family:'JetBrains Mono','Fira Code','Consolas',monospace; "
            f"font-size:10pt; padding:8px;}}")
        term_lay.addWidget(self.terminal, 1)

        term_frame.setLayout(term_lay)
        lay.addWidget(term_frame, 1)

        # ── World Stats area ──────────────────────────────────
        stats_frame = QFrame()
        stats_frame.setFixedHeight(130)
        stats_frame.setStyleSheet(
            f"background: {Colors.LOG_HEADER}; border-radius: 4px; margin: 12px;")
        stats_lay = QVBoxLayout()
        stats_lay.setContentsMargins(16, 12, 16, 12)
        stats_lay.setSpacing(8)

        stats_title = QLabel("WORLD STATISTICS")
        stats_title.setStyleSheet(
            f"color: {Colors.TEXT_SECONDARY}; font-size: 9pt; font-weight: 700; "
            f"letter-spacing: 0.5px; background: transparent;")
        stats_lay.addWidget(stats_title)

        self.stats_label = QLabel("No world loaded")
        self.stats_label.setWordWrap(True)
        self.stats_label.setStyleSheet(
            f"color: {Colors.TEXT_PRIMARY}; font-size: 11pt; background: transparent;")
        stats_lay.addWidget(self.stats_label)
        stats_lay.addStretch()
        stats_frame.setLayout(stats_lay)
        lay.addWidget(stats_frame)

        self.panel.setLayout(lay)

        # Animations
        self._panel_anim = QPropertyAnimation(self.panel, b"geometry")
        self._panel_anim.setDuration(300)
        self._panel_anim.setEasingCurve(QEasingCurve.OutCubic)

        self._overlay_anim = QPropertyAnimation(self.overlay, b"opacity_value")
        self._overlay_anim.setDuration(250)

        # Initial log
        self.add_log("System initialized.", "info")
        self.add_log("Welcome to DWG Wizard.", "info")

    # ── Public API ─────────────────────────────────────────────

    def add_log(self, msg, msg_type="info"):
        """Add a timestamped, color-coded log entry."""
        now = QDateTime.currentDateTime().toString("HH:mm:ss")
        color_map = {
            "info": Colors.TEXT_PRIMARY,
            "success": "#10B981",
            "warning": "#FBBF24",
            "error": "#EF4444",
        }
        color = color_map.get(msg_type, Colors.TEXT_PRIMARY)
        line = (f'<span style="color:#6B7280;">[{now}]</span> '
                f'<span style="color:{color};">{msg}</span>')
        self.terminal.append(line)

    def update_stats(self):
        """Pull world stats from AppState."""
        if not self.app_state or not self.app_state.world_manager:
            self.stats_label.setText("No world loaded")
            return
        wm = self.app_state.world_manager
        if not wm.world_name:
            self.stats_label.setText("No world loaded")
            return
        walls = sum(1 for m in wm.models if m["type"] == "wall" and m.get("status") != "removed")
        obstacles = sum(1 for m in wm.models
                        if m["type"] in ("box", "cylinder", "sphere") and m.get("status") != "removed")
        dynamic = sum(1 for m in wm.models
                      if m["type"] in ("box", "cylinder", "sphere")
                      and m.get("status") != "removed"
                      and "motion" in m.get("properties", {}))
        self.stats_label.setText(
            f"<b style='color:{Colors.TEXT_PRIMARY};'>🌍 {wm.world_name}</b><br>"
            f"<span style='color:{Colors.TEXT_SECONDARY};'>"
            f"Simulator: <b>Gazebo {wm.version.title()}</b><br>"
            f"Walls: <b>{walls}</b>  •  "
            f"Obstacles: <b>{obstacles}</b>  •  "
            f"Dynamic: <b>{dynamic}</b>"
            f"</span>")

    def toggle(self):
        """Toggle the panel open/closed."""
        if self._is_open:
            self.close_panel()
        else:
            self.open_panel()

    def open_panel(self):
        """Slide the panel in from the right."""
        if self._is_open:
            return
        self._is_open = True
        self.update_stats()

        parent = self.panel.parentWidget()
        if not parent:
            return
        pr = parent.rect()
        panel_width = int(pr.width() * 0.45)
        panel_width = max(400, min(panel_width, 700))

        # Show overlay
        self.overlay.setGeometry(pr)
        self.overlay.show()
        self.overlay.raise_()
        self._overlay_anim.setStartValue(0.0)
        self._overlay_anim.setEndValue(0.4)
        self._overlay_anim.start()

        # Slide panel in
        start_rect = QRect(pr.width(), 0, panel_width, pr.height())
        end_rect = QRect(pr.width() - panel_width, 0, panel_width, pr.height())
        self.panel.setGeometry(start_rect)
        self.panel.show()
        self.panel.raise_()

        self._panel_anim.setStartValue(start_rect)
        self._panel_anim.setEndValue(end_rect)
        self._panel_anim.start()

    def close_panel(self):
        """Slide the panel out to the right."""
        if not self._is_open:
            return
        self._is_open = False

        parent = self.panel.parentWidget()
        if not parent:
            return
        pr = parent.rect()

        # Fade overlay
        self._overlay_anim.setStartValue(0.4)
        self._overlay_anim.setEndValue(0.0)
        self._overlay_anim.start()
        self._overlay_anim.finished.connect(self._hide_overlay_once)

        # Slide panel out
        current_rect = self.panel.geometry()
        end_rect = QRect(pr.width(), 0, current_rect.width(), pr.height())
        self._panel_anim.setStartValue(current_rect)
        self._panel_anim.setEndValue(end_rect)
        self._panel_anim.finished.connect(self._hide_panel_once)
        self._panel_anim.start()

    def reposition(self, parent_rect):
        """Reposition overlay and panel on parent resize."""
        self.overlay.setGeometry(parent_rect)
        if self._is_open:
            panel_width = self.panel.width()
            self.panel.setGeometry(
                parent_rect.width() - panel_width, 0,
                panel_width, parent_rect.height())

    # ── Private ────────────────────────────────────────────────

    def _hide_overlay_once(self):
        if not self._is_open:
            self.overlay.hide()
        try:
            self._overlay_anim.finished.disconnect(self._hide_overlay_once)
        except TypeError:
            pass

    def _hide_panel_once(self):
        if not self._is_open:
            self.panel.hide()
        try:
            self._panel_anim.finished.disconnect(self._hide_panel_once)
        except TypeError:
            pass

    def _copy_all(self):
        clipboard = QApplication.clipboard()
        clipboard.setText(self.terminal.toPlainText())
        self.add_log("Logs copied to clipboard.", "success")

    def _clear_logs(self):
        self.terminal.clear()
        self.add_log("Logs cleared.", "info")
