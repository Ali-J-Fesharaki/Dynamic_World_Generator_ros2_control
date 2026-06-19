"""
Main Window — Professional desktop application shell.
Replaces QWizard with QMainWindow + icon sidebar + stacked panels.

Architecture:
  ┌──────────────────────────────────────────────────────┐
  │  TopBar: branding + world badge + simulation badge   │
  ├────┬─────────────────────────────────────────────────┤
  │ S  │                                                 │
  │ i  │         Active Panel                            │
  │ d  │         (Setup / Walls / Obstacles / Export)     │
  │ e  │                                                 │
  │ b  │                                                 │
  │ a  │                                                 │
  │ r  │                                                 │
  ├────┴─────────────────────────────────────────────────┤
  │  StatusBar: world • coordinates • last action        │
  └──────────────────────────────────────────────────────┘
"""
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QStackedWidget, QLabel, QFrame, QPushButton,
                             QSizePolicy, QGraphicsPixmapItem, QGraphicsRectItem,
                             QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsTextItem,
                             QApplication, QSplitter, QButtonGroup)
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, QRectF, QLineF, QPointF
from PyQt5.QtGui import QPainter, QColor, QPen, QPixmap
from classes.zoomable_graphics_view import ZoomableGraphicsView
from classes.widgets.canvas_toolbar import CanvasToolbar
from classes.app_state import AppState
from classes.panels.setup_panel import SetupPanel
from classes.panels.walls_panel import WallsPanel
from classes.panels.obstacles_panel import ObstaclesPanel
from classes.panels.export_panel import ExportPanel
from classes.widgets.status_log_panel import StatusLogPanel
from utils.color_utils import get_color
from utils.theme import Colors, Dims
import math, yaml, os


# ── Sidebar and Splash removed in favor of Unified Layout ────────────────────────────────────────────────────



# ── Top Bar ────────────────────────────────────────────────────

class TopBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(48)
        self.setStyleSheet(
            f"background:{Colors.BG_DARK}; border-bottom:1px solid {Colors.BORDER};")

        lay = QHBoxLayout()
        lay.setContentsMargins(Dims.PAD_LG, 0, Dims.PAD_LG, 0)
        lay.setSpacing(Dims.SPACING_LG)

        # Brand
        brand = QLabel("DWG Wizard")
        brand.setStyleSheet(
            f"font-size:14pt; font-weight:700; color:{Colors.TEXT_PRIMARY}; background:transparent;")
        lay.addWidget(brand)

        ver = QLabel("v2.0")
        ver.setStyleSheet(
            f"font-size:9pt; color:{Colors.SECONDARY}; font-weight:600; background:transparent;")
        lay.addWidget(ver)

        lay.addStretch()

        # World badge
        self.world_badge = QLabel("No world")
        self.world_badge.setStyleSheet(
            f"background:{Colors.BG_ELEVATED}; color:{Colors.TEXT_SECONDARY}; "
            f"font-size:9pt; padding:4px 12px; border-radius:10px; "
            f"border:1px solid {Colors.BORDER};")
        lay.addWidget(self.world_badge)

        # Sim badge
        self.sim_badge = QLabel("No simulator")
        self.sim_badge.setStyleSheet(
            f"background:{Colors.BG_ELEVATED}; color:{Colors.TEXT_SECONDARY}; "
            f"font-size:9pt; padding:4px 12px; border-radius:10px; "
            f"border:1px solid {Colors.BORDER};")
        lay.addWidget(self.sim_badge)

        self.setLayout(lay)

    def set_world(self, name):
        self.world_badge.setText(f"🌍 {name}")
        self.world_badge.setStyleSheet(
            f"background:{Colors.BG_ELEVATED}; color:{Colors.SUCCESS}; "
            f"font-size:9pt; font-weight:600; padding:4px 12px; border-radius:10px; "
            f"border:1px solid {Colors.BORDER};")

    def set_sim(self, sim, version):
        self.sim_badge.setText(f"⚙ Gazebo {version.title()}")
        self.sim_badge.setStyleSheet(
            f"background:{Colors.BG_ELEVATED}; color:{Colors.PRIMARY}; "
            f"font-size:9pt; font-weight:600; padding:4px 12px; border-radius:10px; "
            f"border:1px solid {Colors.BORDER};")


# ── Status Bar ─────────────────────────────────────────────────

class StatusBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(30)
        self.setStyleSheet(
            f"background:{Colors.BG_DARK}; border-top:1px solid {Colors.BORDER};")

        lay = QHBoxLayout()
        lay.setContentsMargins(Dims.PAD_MD, 0, Dims.PAD_MD, 0)
        lay.setSpacing(Dims.PAD_LG)

        self.coord_label = QLabel("X: 0.00m  Y: 0.00m")
        self.coord_label.setStyleSheet(
            f"color:{Colors.TEXT_SECONDARY}; font-size:9pt; font-family:monospace; background:transparent;")
        lay.addWidget(self.coord_label)

        lay.addStretch()

        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet(
            f"color:{Colors.SECONDARY}; font-size:9pt; font-weight:600; background:transparent;")
        lay.addWidget(self.status_label)

        self.setLayout(lay)

    def set_coords(self, x, y):
        self.coord_label.setText(f"X: {x:+.2f}m  Y: {y:+.2f}m")

    def set_status(self, msg, kind="info"):
        c = {
            "info": Colors.SECONDARY, "success": Colors.SUCCESS,
            "warning": Colors.WARNING, "error": Colors.ERROR
        }.get(kind, Colors.TEXT_SECONDARY)
        self.status_label.setStyleSheet(
            f"color:{c}; font-size:9pt; font-weight:600; background:transparent;")
        self.status_label.setText(msg)


# ── Splash Overlay Removed ─────────────────────────────────────────────


# ── Canvas Refresh (free function) ─────────────────────────────

def refresh_canvas(app):
    """Redraw all models on the shared QGraphicsScene."""
    scene = app.scene
    scene.clear()
    app.wall_items.clear()
    app.obstacle_items.clear()
    app.path_items.clear()

    # Background map
    wm = app.world_manager
    if wm and wm.map_path:
        pix = QPixmap(wm.map_path)
        if not pix.isNull():
            item = QGraphicsPixmapItem(pix)
            yml = os.path.splitext(wm.map_path)[0] + ".yaml"
            if os.path.exists(yml):
                try:
                    with open(yml) as f:
                        md = yaml.safe_load(f)
                    res = md.get("resolution", 0.05)
                    origin = md.get("origin", [0, 0, 0])
                    sc = res * 100
                    item.setScale(sc)
                    theta = origin[2] if len(origin) > 2 else 0
                    item.setRotation(-math.degrees(theta))
                    sx = origin[0] * 100
                    sy = -origin[1] * 100
                    item.setPos(sx, sy - pix.height() * sc)
                except Exception:
                    item.setPos(-pix.width() / 2, -pix.height() / 2)
            else:
                item.setPos(-pix.width() / 2, -pix.height() / 2)
            item.setZValue(-10)
            scene.addItem(item)

    if not wm:
        return

    for model in wm.models:
        if model.get("status") == "removed":
            continue

        if model["type"] == "wall":
            st = QPointF(model["properties"]["start"][0] * 100,
                         -model["properties"]["start"][1] * 100)
            en = QPointF(model["properties"]["end"][0] * 100,
                         -model["properties"]["end"][1] * 100)
            rgb = get_color(model["properties"]["color"])
            qc = QColor.fromRgbF(*rgb)
            th = max(int(model["properties"]["width"] * 100), 2)
            line = QGraphicsLineItem(QLineF(st, en))
            line.setPen(QPen(qc, th))
            scene.addItem(line)
            txt = QGraphicsTextItem(model["name"])
            txt.setDefaultTextColor(QColor(Colors.TEXT_SECONDARY))
            txt.setPos((st + en) / 2)
            scene.addItem(txt)
            app.wall_items[model["name"]] = (line, txt)

        elif model["type"] in ("box", "cylinder", "sphere", "person"):
            pos = model["properties"]["position"]
            sz = model["properties"]["size"] if model["type"] != "person" else [1,1,1]
            ctr = QPointF(pos[0] * 100, -pos[1] * 100)
            if model["type"] == "box":
                W, L, _ = sz
                hw = round((W / 2) * 100 / 10) * 10
                hl = round((L / 2) * 100 / 10) * 10
                it = QGraphicsRectItem(QRectF(ctr.x() - hw, ctr.y() - hl, 2 * hw, 2 * hl))
            elif model["type"] == "person":
                R = 0.25
                rp = R * 100
                it = QGraphicsEllipseItem(
                    QRectF(ctr.x() - rp, ctr.y() - rp, 2 * rp, 2 * rp))
            else:
                R = sz[0]
                rp = R * 100
                it = QGraphicsEllipseItem(
                    QRectF(ctr.x() - rp, ctr.y() - rp, 2 * rp, 2 * rp))
            it.setPen(QPen(QColor(Colors.BORDER_LIGHT), 2))
            if model["type"] == "person":
                it.setBrush(QColor(Colors.PRIMARY))
            else:
                rgb = get_color(model["properties"]["color"])
                it.setBrush(QColor.fromRgbF(*rgb))
            scene.addItem(it)
            txt = QGraphicsTextItem(model["name"])
            txt.setDefaultTextColor(QColor(Colors.TEXT_PRIMARY))
            txt.setPos(ctr)
            scene.addItem(txt)
            app.obstacle_items[model["name"]] = (it, txt)

        motion = model["properties"].get("motion")
        if motion:
            t = motion["type"]
            col = {"linear": "#FF6B6B", "elliptical": "#51CF66", "polygon": "#339AF0"}[t]
            items = []
            if t == "linear":
                p1 = QPointF(motion["path"][0][0] * 100, -motion["path"][0][1] * 100)
                p2 = QPointF(motion["path"][1][0] * 100, -motion["path"][1][1] * 100)
                ln = QGraphicsLineItem(QLineF(p1, p2))
                ln.setPen(QPen(QColor(col), 2, Qt.DashLine))
                scene.addItem(ln)
                items.append(ln)
            elif t == "elliptical":
                cm = model["properties"]["position"][:2]
                c = QPointF(cm[0] * 100, -cm[1] * 100)
                el = QGraphicsEllipseItem(
                    QRectF(-motion["semi_major"] * 100, -motion["semi_minor"] * 100,
                           2 * motion["semi_major"] * 100, 2 * motion["semi_minor"] * 100))
                el.setPos(c)
                el.setRotation(-math.degrees(motion["angle"]))
                el.setPen(QPen(QColor(col), 2, Qt.DashLine))
                scene.addItem(el)
                items.append(el)
            elif t == "polygon":
                pts = [QPointF(p[0] * 100, -p[1] * 100) for p in motion["path"]]
                for i in range(len(pts)):
                    ln = QGraphicsLineItem(
                        QLineF(pts[i], pts[(i + 1) % len(pts)]))
                    ln.setPen(QPen(QColor(col), 2, Qt.DashLine))
                    scene.addItem(ln)
                    items.append(ln)
            app.path_items[model["name"]] = items


# ── Main Window ────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DWG Wizard — Dynamic World Generator")
        self.resize(1700, 920)
        self.setMinimumSize(1024, 700)

        # Shared state
        self.app = AppState()

        # Central widget
        central = QWidget()
        outer = QVBoxLayout()
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # Top bar
        self.top_bar = TopBar()
        outer.addWidget(self.top_bar)

        # Unified Splitter Body
        self.splitter = QSplitter(Qt.Horizontal)
        self.splitter.setHandleWidth(1)
        self.splitter.setStyleSheet(f"QSplitter::handle {{ background: {Colors.BORDER}; }}")
        
        # ── Left SidePanel ──
        self.side_panel = QWidget()
        self.side_panel.setMinimumWidth(320)
        self.side_panel.setMaximumWidth(400)
        self.side_panel.setStyleSheet(f"QWidget {{ background: {Colors.NAV_BG}; }}")
        sp_lay = QVBoxLayout(self.side_panel)
        sp_lay.setContentsMargins(16, 16, 16, 16)
        sp_lay.setSpacing(16)
        
        # Segmented Tabs
        self.tab_group = QButtonGroup(self)
        self.tab_group.setExclusive(True)
        tab_row = QHBoxLayout()
        tab_row.setSpacing(0)
        
        self.tabs = []
        for i, text in enumerate(["Setup", "Walls", "Obstacles", "Export"]):
            btn = QPushButton(text)
            btn.setCheckable(True)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setFixedHeight(36)
            rad = f"border-top-left-radius: 4px; border-bottom-left-radius: 4px;" if i == 0 else \
                  f"border-top-right-radius: 4px; border-bottom-right-radius: 4px;" if i == 3 else ""
            btn.setStyleSheet(
                f"QPushButton {{"
                f"  background: {Colors.BG_DARK}; color: {Colors.TEXT_SECONDARY};"
                f"  border: 1px solid {Colors.BORDER}; font-size: 11pt; {rad}"
                f"}}"
                f"QPushButton:checked {{"
                f"  background: {Colors.PRIMARY}; color: white; border: 1px solid {Colors.PRIMARY};"
                f"}}"
            )
            self.tab_group.addButton(btn, i)
            tab_row.addWidget(btn)
            self.tabs.append(btn)
        
        self.tab_group.buttonClicked[int].connect(self._nav)
        
        tab_container = QWidget()
        tab_container.setLayout(tab_row)
        tab_container.setStyleSheet("background: transparent;")
        sp_lay.addWidget(tab_container)
        
        # Stacked Panels
        self.stack = QStackedWidget()
        
        # Shared Canvas
        self.view = ZoomableGraphicsView(self.app.scene)
        self.view.coordinateChanged.connect(lambda x, y: self.app.coordinateUpdate.emit(x, y))
        
        # Pass view to the panels that need it
        self.setup_panel = SetupPanel(self.app)
        self.walls_panel = WallsPanel(self.app, self.view)
        self.obstacles_panel = ObstaclesPanel(self.app, self.view)
        self.export_panel = ExportPanel(self.app)

        self.stack.addWidget(self.setup_panel)
        self.stack.addWidget(self.walls_panel)
        self.stack.addWidget(self.obstacles_panel)
        self.stack.addWidget(self.export_panel)

        sp_lay.addWidget(self.stack, 1)
        self.splitter.addWidget(self.side_panel)
        
        # ── Right Canvas Area ──
        self.canvas_area = QWidget()
        cv_lay = QVBoxLayout(self.canvas_area)
        cv_lay.setContentsMargins(0, 0, 0, 0)
        cv_lay.addWidget(self.view)
        
        self.splitter.addWidget(self.canvas_area)
        self.splitter.setSizes([360, 1340])
        
        outer.addWidget(self.splitter, 1)

        # Status bar
        self.status_bar = StatusBar()
        outer.addWidget(self.status_bar)

        central.setLayout(outer)
        self.setCentralWidget(central)

        # ── Toolbar ─────────────────────────────────────────────────
        self.toolbar = CanvasToolbar(self.view)
        self.toolbar.setParent(self.canvas_area)
        self.toolbar.zoomInRequested.connect(self.view.zoom_in)
        self.toolbar.zoomOutRequested.connect(self.view.zoom_out)
        self.toolbar.fitAllRequested.connect(self.view.fit_all)
        self.toolbar.rotateCWRequested.connect(self.view.rotate_cw)
        self.toolbar.rotateCCWRequested.connect(self.view.rotate_ccw)
        self.toolbar.gridToggled.connect(self.view.set_grid_visible)
        self.toolbar.raise_()

        # ── Status Log Panel (slide-out drawer) ─────────────
        self.log_panel = StatusLogPanel(parent=central, app_state=self.app)

        # Floating toggle button for log panel
        self.log_toggle_btn = QPushButton("📋", central)
        self.log_toggle_btn.setFixedSize(44, 44)
        self.log_toggle_btn.setCursor(Qt.PointingHandCursor)
        self.log_toggle_btn.setToolTip("System Status & Logs")
        self.log_toggle_btn.setStyleSheet(
            f"QPushButton{{background:{Colors.BG_ELEVATED}; color:{Colors.TEXT_SECONDARY}; "
            f"font-size:16pt; border:1px solid {Colors.BORDER}; "
            f"border-radius:22px;}}"
            f"QPushButton:hover{{background:{Colors.PRIMARY}; color:white; "
            f"border-color:{Colors.PRIMARY};}}")
        self.log_toggle_btn.clicked.connect(self.log_panel.toggle)
        self.log_toggle_btn.raise_()

        # ── Connections ────────────────────────────────────────
        self.tabs[0].setChecked(True)
        self._nav(0)
        
        # Lock design panels until world is loaded
        for i in range(1, 4):
            self.tabs[i].setEnabled(False)

        self.setup_panel.panelReady.connect(self._unlock_panels)
        self.app.simulationSet.connect(self.top_bar.set_sim)
        self.app.worldLoaded.connect(self.top_bar.set_world)
        self.app.statusMessage.connect(self.status_bar.set_status)
        self.app.statusMessage.connect(self.log_panel.add_log)
        self.app.coordinateUpdate.connect(self.status_bar.set_coords)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        cr = self.centralWidget().rect()
        if hasattr(self, "log_panel"):
            self.log_panel.reposition(cr)
        if hasattr(self, "log_toggle_btn"):
            self.log_toggle_btn.move(cr.width() - 56, cr.height() - 56)
        if hasattr(self, "toolbar"):
            self.toolbar.position_in_parent()

    def _nav(self, index):
        if not self.tabs[index].isEnabled():
            self.status_bar.set_status("Complete setup first (select sim + load world)", "warning")
            return
            
        self.stack.setCurrentIndex(index)
        
        # Notify panels of active state
        self.walls_panel.set_active(index == 1)
        self.obstacles_panel.set_active(index == 2)
        
        # Ensure correct check state in case nav was called programmatically
        if not self.tabs[index].isChecked():
            self.tabs[index].setChecked(True)

    def _unlock_panels(self):
        for i in range(1, 4):
            self.tabs[i].setEnabled(True)
        self.status_bar.set_status("All panels unlocked — start designing!", "success")

    def closeEvent(self, event):
        self.app.cleanup()
        event.accept()
