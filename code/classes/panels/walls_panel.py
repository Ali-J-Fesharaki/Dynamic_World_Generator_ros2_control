"""
Walls Panel — Draw walls on the shared canvas.
"""
from PyQt5.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QPushButton,
                             QListWidget, QMessageBox, QLabel, QDoubleSpinBox,
                             QFrame, QAbstractItemView, QScrollArea)
from PyQt5.QtCore import Qt, QEvent, QPointF
from classes.widgets.color_picker import ColorPicker
from utils.theme import Colors, Dims, make_primary_button, make_danger_button, make_section_label


class WallsPanel(QWidget):
    def __init__(self, app_state, shared_view):
        super().__init__()
        self.app = app_state
        self.view = shared_view
        self.is_active = False
        self.view.viewport().installEventFilter(self)
        self._start_pt = None
        self._preview_line = None

        layout = QVBoxLayout()
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)

        # ── Left sidebar ──────────────────────────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet(f"QScrollArea{{border:none; background:{Colors.BG_SURFACE};}}")

        left = QWidget()
        left.setObjectName("lp")
        left.setStyleSheet(f"QWidget#lp{{background:{Colors.BG_SURFACE};}}")
        lv = QVBoxLayout()
        lv.setContentsMargins(Dims.PAD_LG, Dims.PAD_LG, Dims.PAD_LG, Dims.PAD_LG)
        lv.setSpacing(Dims.SPACING_SM)

        # Properties
        s = QLabel("Wall Properties"); make_section_label(s); lv.addWidget(s)

        for label, attr, lo, hi, default, step, sfx, dec in [
            ("Width",  "width_spin",  0.01, 10,  0.2, 0.05, " m", 2),
            ("Height", "height_spin", 0.1,  50,  1.5, 0.1,  " m", 1),
        ]:
            r = QHBoxLayout()
            lb = QLabel(label)
            lb.setStyleSheet(f"color:{Colors.TEXT_SECONDARY}; font-size:10pt; background:transparent;")
            lb.setFixedWidth(50)
            r.addWidget(lb)
            sp = QDoubleSpinBox()
            sp.setRange(lo, hi); sp.setValue(default); sp.setSingleStep(step)
            sp.setSuffix(sfx); sp.setDecimals(dec)
            r.addWidget(sp)
            setattr(self, attr, sp)
            lv.addLayout(r)

        self.color_picker = ColorPicker(default_color="Gray")
        lv.addWidget(self.color_picker)

        lv.addWidget(self._sep())

        # Walls list
        s2 = QLabel("Walls"); make_section_label(s2); lv.addWidget(s2)
        self.wall_list = QListWidget()
        self.wall_list.setMinimumHeight(80)
        self.wall_list.setSelectionMode(QAbstractItemView.SingleSelection)
        lv.addWidget(self.wall_list, 1)

        rm = QPushButton("🗑  Remove Selected")
        make_danger_button(rm)
        rm.clicked.connect(self._remove)
        lv.addWidget(rm)

        lv.addWidget(self._sep())

        ap = QPushButton("▶  Apply & Preview")
        make_primary_button(ap)
        ap.setMinimumHeight(Dims.BUTTON_HEIGHT_LG)
        ap.clicked.connect(self._apply)
        lv.addWidget(ap)

        hint = QLabel("Click twice on canvas to draw a wall\n(start → end)")
        hint.setStyleSheet(f"font-size:9pt; color:{Colors.TEXT_DISABLED}; background:transparent; font-style:italic; padding-top:4px;")
        hint.setAlignment(Qt.AlignCenter)
        lv.addWidget(hint)

        lv.addStretch()
        left.setLayout(lv)
        scroll.setWidget(left)
        scroll.setMinimumWidth(220)
        scroll.setMaximumWidth(340)

        layout.addWidget(scroll)
        self.setLayout(layout)

    def set_active(self, active):
        self.is_active = active
        if active:
            self._refresh()

    def showEvent(self, e):
        super().showEvent(e)
        if self.is_active:
            self._refresh()

    def _refresh(self):
        self.wall_list.clear()
        if not self.app.world_manager:
            return
        from classes.main_window import refresh_canvas
        refresh_canvas(self.app)
        for m in self.app.world_manager.models:
            if m.get("status") != "removed" and m["type"] == "wall":
                self.wall_list.addItem(m["name"])

    def _sep(self):
        f = QFrame(); f.setFixedHeight(1)
        f.setStyleSheet(f"background:{Colors.BORDER};")
        return f

    def snap(self, pt, sp=10):
        return QPointF(round(pt.x()/sp)*sp, round(pt.y()/sp)*sp)

    def eventFilter(self, obj, event):
        if not self.is_active:
            return super().eventFilter(obj, event)
        if obj != self.view.viewport() or not self.app.world_manager:
            return super().eventFilter(obj, event)

        if event.type() == QEvent.MouseButtonPress and event.button() == Qt.LeftButton:
            pt = self.snap(self.view.mapToScene(event.pos()))
            if self._start_pt is None:
                self._start_pt = pt
                self.view.set_crosshair_mode(True)
                self.app.status("Click end point for wall…", "info")
                
                # Create preview line
                from PyQt5.QtWidgets import QGraphicsLineItem
                from PyQt5.QtGui import QPen, QColor
                from PyQt5.QtCore import QLineF
                from utils.color_utils import get_color
                
                rgb = get_color(self.color_picker.current_color)
                qc = QColor.fromRgbF(*rgb)
                qc.setAlphaF(0.5)  # semi-transparent preview
                th = max(int(self.width_spin.value() * 100), 2)
                
                self._preview_line = QGraphicsLineItem(QLineF(self._start_pt, self._start_pt))
                self._preview_line.setPen(QPen(qc, th))
                self.app.scene.addItem(self._preview_line)
            else:
                end = pt
                wm = self.app.world_manager
                name = f"wall_{len(wm.models)+1}"
                wm.add_model({
                    "name": name, "type": "wall",
                    "properties": {
                        "start": (self._start_pt.x()/100, -self._start_pt.y()/100),
                        "end":   (end.x()/100, -end.y()/100),
                        "width": self.width_spin.value(),
                        "height": self.height_spin.value(),
                        "color": self.color_picker.current_color,
                    },
                    "status": "new",
                })
                self._start_pt = None
                if self._preview_line:
                    self.app.scene.removeItem(self._preview_line)
                    self._preview_line = None
                    
                self.view.set_crosshair_mode(False)
                self._refresh()
                self.app.status(f"Added {name}", "success")
            return True

        elif event.type() == QEvent.MouseMove and self._start_pt is not None:
            pt = self.snap(self.view.mapToScene(event.pos()))
            if self._preview_line:
                from PyQt5.QtCore import QLineF
                self._preview_line.setLine(QLineF(self._start_pt, pt))
            return False

        return super().eventFilter(obj, event)

    def _remove(self):
        wm = self.app.world_manager
        if not wm: return
        item = self.wall_list.currentItem()
        if not item: return
        name = item.text()
        if name in self.app.wall_items:
            for i in self.app.wall_items[name]:
                self.app.scene.removeItem(i)
            del self.app.wall_items[name]
        for m in wm.models:
            if m["name"] == name:
                m["status"] = "removed"; break
        self.wall_list.takeItem(self.wall_list.row(item))
        self.app.status(f"Removed {name}", "warning")

    def _apply(self):
        wm = self.app.world_manager
        if not wm:
            QMessageBox.warning(self, "Error", "No world loaded.")
            return
        try:
            wm.apply_changes()
            self._refresh()
            self.app.status("Wall changes applied", "success")
            
            # Launch preview
            from utils.config import PROJECT_ROOT
            import subprocess, os
            
            # Try bash first, fallback to zsh
            setup = os.path.join(PROJECT_ROOT, "code", "control_ws", "install", "setup.bash")
            shell = "bash"
            if not os.path.exists(setup):
                setup = os.path.join(PROJECT_ROOT, "code", "control_ws", "install", "setup.zsh")
                shell = "zsh"
                
            cmd = (f"source {setup} && ros2 launch dynamic_obstacle_gz_spawning "
                   f"multi_obstacle_world.launch.py world_name:={wm.world_name}")
            subprocess.Popen(["gnome-terminal", "--", shell, "-c", f"{cmd}; exec {shell}"])
            self.app.status("Launched Gazebo Preview", "success")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))
