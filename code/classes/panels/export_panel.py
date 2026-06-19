"""
Export Panel — World summary, upcoming features, and final export controls.
Replaces the old Coming Soon page with a useful summary + future roadmap.
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                             QFrame, QPushButton, QMessageBox, QScrollArea)
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import Qt
from utils.config import FUTURE_IMAGES_DIR, PROJECT_ROOT
from utils.theme import Colors, Dims, make_primary_button, make_section_label
import os


class _FeatureCard(QFrame):
    """A frosted-glass style coming-soon feature card."""
    def __init__(self, title, img_path, description):
        super().__init__()
        self.setStyleSheet(
            f"QFrame{{background:{Colors.BG_CARD}; border:1px solid {Colors.BORDER}; "
            f"border-radius:{Dims.RADIUS_LG}px;}}")
        self.setMinimumWidth(200)
        self.setMaximumWidth(320)

        lay = QVBoxLayout()
class ExportPanel(QWidget):
    """Summary of the current world."""

    def __init__(self, app_state):
        super().__init__()
        self.app = app_state

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(f"QScrollArea{{border:none; background:{Colors.BG_PRIMARY};}}")

        content = QWidget()
        root = QVBoxLayout()
        root.setContentsMargins(Dims.PAD_MD, Dims.PAD_MD, Dims.PAD_MD, Dims.PAD_MD)
        root.setSpacing(Dims.SPACING_MD)

        # ── World Summary ─────────────────────────────────────
        hdr = QLabel("World Summary")
        hdr.setStyleSheet(
            f"font-size:20pt; font-weight:700; color:{Colors.TEXT_PRIMARY}; background:transparent;")
        root.addWidget(hdr)

        self.summary_label = QLabel("No world loaded yet.")
        self.summary_label.setWordWrap(True)
        self.summary_label.setStyleSheet(
            f"font-size:11pt; color:{Colors.TEXT_SECONDARY}; background:{Colors.BG_CARD}; "
            f"border:1px solid {Colors.BORDER}; border-radius:{Dims.RADIUS_MD}px; "
            f"padding:16px;")
        root.addWidget(self.summary_label)

        # Quick action
        qa_row = QHBoxLayout()
        qa_row.setSpacing(Dims.SPACING_SM)

        self.launch_btn = QPushButton("🚀  Launch in Gazebo")
        make_primary_button(self.launch_btn)
        self.launch_btn.setMinimumHeight(Dims.BUTTON_HEIGHT_LG)
        self.launch_btn.clicked.connect(self._launch)
        qa_row.addWidget(self.launch_btn)

        self.export_btn = QPushButton("💾  Export YAML")
        self.export_btn.setMinimumHeight(Dims.BUTTON_HEIGHT_LG)
        self.export_btn.clicked.connect(self._export)
        qa_row.addWidget(self.export_btn)

        qa_row.addStretch()
        root.addLayout(qa_row)

        # ── Separator ──────────────────────────────────────────
        sep = QFrame()
        sep.setFixedHeight(1)
        sep.setStyleSheet(f"background:{Colors.BORDER};")
        root.addWidget(sep)

        root.addStretch()

        # ── Credits ────────────────────────────────────────────
        credits = QLabel(
            "Dynamic World Generator Wizard v2.0  •  ROS 2 Control\n"
            "Special thanks to Professor Sousso KELOUWANI\n"
            "ali.jafari.fesh@gmail.com")
        credits.setStyleSheet(
            f"font-size:9pt; color:{Colors.TEXT_DISABLED}; background:transparent; "
            f"padding-top:16px;")
        credits.setAlignment(Qt.AlignCenter)
        root.addWidget(credits)

        content.setLayout(root)
        scroll.setWidget(content)

        main = QVBoxLayout()
        main.setContentsMargins(0, 0, 0, 0)
        main.addWidget(scroll)
        self.setLayout(main)

    def showEvent(self, e):
        super().showEvent(e)
        self._update_summary()

    def _update_summary(self):
        wm = self.app.world_manager
        if not wm or not wm.world_name:
            self.summary_label.setText("No world loaded yet. Go to Setup to create or load a world.")
            return

        walls = sum(1 for m in wm.models if m["type"] == "wall" and m.get("status") != "removed")
        obstacles = sum(1 for m in wm.models
                        if m["type"] in ("box", "cylinder", "sphere") and m.get("status") != "removed")
        dynamic = sum(1 for m in wm.models
                      if m["type"] in ("box", "cylinder", "sphere")
                      and m.get("status") != "removed"
                      and "motion" in m.get("properties", {}))

        self.summary_label.setText(
            f"<b style='color:{Colors.TEXT_PRIMARY}; font-size:13pt;'>"
            f"🌍 {wm.world_name}</b><br><br>"
            f"<span style='color:{Colors.TEXT_SECONDARY};'>"
            f"Simulator: <b>Gazebo {wm.version.title()}</b><br>"
            f"Walls: <b>{walls}</b><br>"
            f"Obstacles: <b>{obstacles}</b>  ({dynamic} with motion paths)<br>"
            f"</span>")

    def _launch(self):
        wm = self.app.world_manager
        if not wm or not wm.world_name:
            QMessageBox.warning(self, "Error", "No world loaded.")
            return
        try:
            wm.apply_changes()
            from ament_index_python.packages import get_package_share_directory
            import yaml
            # Quick export
            cfg = os.path.join(get_package_share_directory("dynamic_obstacle_gz_spawning"),
                               "config", "obstacles.yaml")
            self._do_export(cfg)
            
            has_dynamic = any("motion" in m.get("properties", {}) for m in wm.models if m["type"] in ("box","cylinder","sphere") and m.get("status") != "removed")
            
            self.app.launch_preview(use_ros_launch=has_dynamic)
            
        except Exception as exc:
            QMessageBox.critical(self, "Error", str(exc))

    def _export(self):
        self._do_export()

    def _do_export(self, path=None):
        wm = self.app.world_manager
        if not wm:
            return
        import yaml
        from ament_index_python.packages import get_package_share_directory
        obs = []
        for m in wm.models:
            if m.get("status") == "removed" or m["type"] not in ("box", "cylinder", "sphere"):
                continue
            if "motion" not in m.get("properties", {}):
                continue
            o = {"name": m["name"], "type": m["type"],
                 "color": m["properties"].get("color", "gray").lower(),
                 "enabled": True,
                 "x_pose": float(m["properties"]["position"][0]),
                 "y_pose": float(m["properties"]["position"][1]),
                 "z_pose": float(m["properties"]["position"][2]),
                 "size": list(m["properties"]["size"])}
            if "motion" in m["properties"]:
                mt = m["properties"]["motion"]
                o["motion"] = {"type": mt["type"], "velocity": float(mt["velocity"]),
                               "std": float(mt["std"])}
                if "path" in mt:
                    o["motion"]["path"] = [
                        [p[0] - float(m["properties"]["position"][0]),
                         p[1] - float(m["properties"]["position"][1])]
                        for p in mt["path"]]
                if mt["type"] == "elliptical":
                    o["motion"]["semi_major"] = float(mt["semi_major"])
                    o["motion"]["semi_minor"] = float(mt["semi_minor"])
                    o["motion"]["angle"] = float(mt["angle"])
            obs.append(o)
        if path is None:
            path = os.path.join(get_package_share_directory("dynamic_obstacle_gz_spawning"),
                                "config", "obstacles.yaml")
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w") as f:
                yaml.dump({"obstacles": obs}, f, default_flow_style=None, sort_keys=False)
            self.app.status(f"Exported to {os.path.basename(path)}", "success")
        except Exception as exc:
            QMessageBox.critical(self, "Error", str(exc))
