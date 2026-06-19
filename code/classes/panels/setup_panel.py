"""
Setup Panel — Simulation selection + World create / load.
First panel the user interacts with after the splash screen.
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
                             QLineEdit, QPushButton, QSizePolicy, QMessageBox)
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import Qt, pyqtSignal
from utils.config import INTRO_IMAGES_DIR, WORLDS_GAZEBO_DIR, PROJECT_ROOT
from utils.theme import Colors, Dims, make_primary_button, make_section_label
import os, shutil, sys, subprocess
from xml.etree import ElementTree as ET


class SimCard(QFrame):
    """Clickable simulation-platform card."""
    clicked = pyqtSignal(str)

    def __init__(self, sid, title, img_path, features, recommended=False, enabled=True):
        super().__init__()
        self.sid = sid
        self._enabled = enabled
        self._selected = False
        self.setObjectName("SimCard")
        self.setCursor(Qt.PointingHandCursor if enabled else Qt.ForbiddenCursor)

        self.setMinimumWidth(200)
        self.setMaximumWidth(380)
        self.setMinimumHeight(140)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        lay = QVBoxLayout()
        lay.setContentsMargins(Dims.PAD_LG, Dims.PAD_LG, Dims.PAD_LG, Dims.PAD_LG)
        lay.setSpacing(Dims.SPACING_SM)

        if recommended:
            badge = QLabel("★  RECOMMENDED")
            badge.setStyleSheet(
                f"background-color: {Colors.PRIMARY}; color: white; font-size: 8pt; "
                f"font-weight: 700; padding: 3px 12px; border-radius: 10px;")
            badge.setAlignment(Qt.AlignCenter)
            lay.addWidget(badge, alignment=Qt.AlignCenter)

        # Image removed per user request

        ttl = QLabel(title)
        ttl.setStyleSheet(f"font-size:13pt; font-weight:700; color:{Colors.TEXT_PRIMARY}; background:transparent;")
        ttl.setAlignment(Qt.AlignCenter)
        lay.addWidget(ttl)
        lay.addSpacing(5)

        for f in features:
            fl = QLabel(f"  {f}")
            fl.setStyleSheet(f"font-size:10pt; color:{Colors.TEXT_SECONDARY}; background:transparent; padding:2px 0;")
            lay.addWidget(fl)

        if not enabled:
            ol = QLabel("🔒  Coming Soon")
            ol.setStyleSheet(f"font-size:11pt; font-weight:600; color:{Colors.TEXT_DISABLED}; background:transparent; padding-top:6px;")
            ol.setAlignment(Qt.AlignCenter)
            lay.addWidget(ol)

        lay.addStretch()

        self.sel_label = QLabel("Click to select" if enabled else "")
        self.sel_label.setStyleSheet(f"font-size:8pt; color:{Colors.TEXT_DISABLED}; background:transparent; font-style:italic;")
        self.sel_label.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.sel_label)

        self.setLayout(lay)
        self._restyle()

    @property
    def selected(self):
        return self._selected

    @selected.setter
    def selected(self, v):
        self._selected = v
        self._restyle()

    def _restyle(self):
        if self._selected:
            self.setStyleSheet(
                f"#SimCard {{background:{Colors.BG_CARD}; border:2px solid {Colors.PRIMARY}; border-radius:{Dims.RADIUS_LG}px;}}")
            self.sel_label.setText("✓  Selected")
            self.sel_label.setStyleSheet(f"font-size:10pt; color:{Colors.SUCCESS}; background:transparent; font-weight:600;")
        elif not self._enabled:
            self.setStyleSheet(
                f"#SimCard {{background:{Colors.BG_DARK}; border:1px solid {Colors.BORDER}; border-radius:{Dims.RADIUS_LG}px;}}")
        else:
            self.setStyleSheet(
                f"#SimCard {{background:{Colors.BG_CARD}; border:1px solid {Colors.BORDER}; border-radius:{Dims.RADIUS_LG}px;}}")
            self.sel_label.setText("Click to select")
            self.sel_label.setStyleSheet(f"font-size:8pt; color:{Colors.TEXT_DISABLED}; background:transparent; font-style:italic;")

    def enterEvent(self, e):
        if self._enabled and not self._selected:
            self.setStyleSheet(
                f"#SimCard {{background:{Colors.BG_CARD_HOVER}; border:1px solid {Colors.BORDER_LIGHT}; border-radius:{Dims.RADIUS_LG}px;}}")

    def leaveEvent(self, e):
        self._restyle()

    def mousePressEvent(self, e):
        if self._enabled and e.button() == Qt.LeftButton:
            self.clicked.emit(self.sid)


class SetupPanel(QWidget):
    """Combined simulation selection + world creation/loading."""
    panelReady = pyqtSignal()  # emitted when a world is loaded → unlock other panels

    def __init__(self, app_state):
        super().__init__()
        self.app = app_state

        root = QVBoxLayout()
        root.setContentsMargins(Dims.PAD_MD, Dims.PAD_MD, Dims.PAD_MD, Dims.PAD_MD)
        root.setSpacing(Dims.SPACING_MD)

        # ── Header ─────────────────────────────────────────────
        hdr = QLabel("Project Setup")
        hdr.setStyleSheet(f"font-size:20pt; font-weight:700; color:{Colors.TEXT_PRIMARY}; background:transparent;")
        root.addWidget(hdr)
        sub = QLabel("Select a simulator and create or load a world to get started")
        sub.setStyleSheet(f"font-size:11pt; color:{Colors.TEXT_SECONDARY}; background:transparent;")
        root.addWidget(sub)
        root.addSpacing(4)

        # ── Simulation Cards ──────────────────────────────────
        sec1 = QLabel("Simulation Platform")
        make_section_label(sec1)
        root.addWidget(sec1)

        cards = QVBoxLayout()
        cards.setSpacing(Dims.SPACING_MD)

        self.harmonic_card = SimCard("harmonic", "Gazebo Harmonic",
            os.path.join(INTRO_IMAGES_DIR, "harmonic.png"),
            ["✓ Latest features", "✓ Full ROS 2 Control", "✓ Superior collisions"],
            recommended=True)
        self.harmonic_card.clicked.connect(lambda: self._select_sim("harmonic"))

        self.fortress_card = SimCard("fortress", "Gazebo Fortress",
            os.path.join(INTRO_IMAGES_DIR, "fortress.jpeg"),
            ["✓ Stable LTS", "✓ ROS 2 Control", "✓ Wide community"])
        self.fortress_card.clicked.connect(lambda: self._select_sim("fortress"))

        self.isaac_card = SimCard("isaac", "Isaac Sim",
            os.path.join(INTRO_IMAGES_DIR, "isaacsim_450_gray.png"),
            ["○ NVIDIA Omniverse", "○ AI training", "○ Digital twins"], enabled=True)
        self.isaac_card.clicked.connect(lambda: self._select_sim("isaacsim"))

        cards.addWidget(self.harmonic_card)
        cards.addWidget(self.fortress_card)
        cards.addWidget(self.isaac_card)
        root.addLayout(cards)

        # ── World ──────────────────────────────────────────────
        sep = QFrame(); sep.setFixedHeight(1)
        sep.setStyleSheet(f"background:{Colors.BORDER};")
        root.addWidget(sep)

        sec2 = QLabel("World")
        make_section_label(sec2)
        root.addWidget(sec2)

        world_col = QVBoxLayout()
        world_col.setSpacing(Dims.SPACING_SM)
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("Enter world name …")
        self.name_input.setMinimumHeight(Dims.BUTTON_HEIGHT)
        world_col.addWidget(self.name_input)

        btns_row = QHBoxLayout()
        btns_row.setSpacing(Dims.SPACING_SM)

        self.create_btn = QPushButton("⊕  Create New")
        make_primary_button(self.create_btn)
        self.create_btn.setMinimumHeight(Dims.BUTTON_HEIGHT)
        self.create_btn.clicked.connect(self._create_world)
        btns_row.addWidget(self.create_btn)

        self.load_btn = QPushButton("📂  Load Existing")
        self.load_btn.setMinimumHeight(Dims.BUTTON_HEIGHT)
        self.load_btn.clicked.connect(self._load_world)
        btns_row.addWidget(self.load_btn)

        world_col.addLayout(btns_row)
        root.addLayout(world_col)

        # Status
        self.info_label = QLabel("")
        self.info_label.setStyleSheet(
            f"font-size:10pt; color:{Colors.TEXT_SECONDARY}; background:transparent; padding-top:4px;")
        root.addWidget(self.info_label)

        root.addStretch()
        self.setLayout(root)

    # ── Actions ────────────────────────────────────────────────

    def _select_sim(self, version):
        self.harmonic_card.selected = (version == "harmonic")
        self.fortress_card.selected = (version == "fortress")
        self.isaac_card.selected = (version == "isaacsim")
        if version == "isaacsim":
            self.app.set_simulation("isaacsim", "5.1")
        else:
            self.app.set_simulation("gazebo", version)

    def _create_world(self):
        wm = self.app.world_manager
        if not wm:
            QMessageBox.warning(self, "Error", "Select a simulation platform first.")
            return
        name = self.name_input.text().strip()
        if not name:
            QMessageBox.warning(self, "Error", "Enter a world name.")
            return
        try:
            if wm.sim_type == "gazebo":
                empty = os.path.join(WORLDS_GAZEBO_DIR, wm.version, "empty_world.sdf")
                if not os.path.exists(empty):
                    raise FileNotFoundError(f"Template not found: {empty}")
                dst_src = os.path.join(PROJECT_ROOT, 'code', 'control_ws', 'src',
                                   'dynamic_obstacle_gz_spawning', 'worlds', f"{name}.sdf")
                shutil.copyfile(empty, dst_src)
                tree = ET.parse(dst_src)
                we = tree.getroot().find("world")
                if we is not None:
                    we.set("name", name)
                tree.write(dst_src, encoding="utf-8", xml_declaration=True)

                dst_install = os.path.join(PROJECT_ROOT, 'code', 'control_ws', 'install',
                                   'dynamic_obstacle_gz_spawning', 'share',
                                   'dynamic_obstacle_gz_spawning', 'worlds', f"{name}.sdf")
                os.makedirs(os.path.dirname(dst_install), exist_ok=True)
                shutil.copyfile(dst_src, dst_install)
            elif wm.sim_type == "isaacsim":
                from utils.config import PROJECT_ROOT
                empty = os.path.join(PROJECT_ROOT, "worlds", "isaacsim", wm.version, "empty_world.usd")
                if not os.path.exists(empty):
                    # fallback
                    empty = os.path.join(PROJECT_ROOT, "worlds", "isaacsim", "empty_world.usd")
                if not os.path.exists(empty):
                    raise FileNotFoundError(f"Template not found: {empty}")
                # We only need to create an empty world file in the project worlds directory for now.
                # Isaac Sim spawning script loads the empty template directly.
                dst_src = os.path.join(PROJECT_ROOT, 'worlds', 'isaacsim', f"{name}.usd")
                os.makedirs(os.path.dirname(dst_src), exist_ok=True)
                shutil.copyfile(empty, dst_src)
            
            wm.load_world(name)
            self.info_label.setText(f"✓  Created and loaded world: {name}")
            self.info_label.setStyleSheet(
                f"font-size:10pt; color:{Colors.SUCCESS}; background:transparent; font-weight:600;")
            self.app.worldLoaded.emit(name)
            self.panelReady.emit()
            self.app.status(f"Created world: {name}", "success")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def _load_world(self):
        wm = self.app.world_manager
        if not wm:
            QMessageBox.warning(self, "Error", "Select a simulation platform first.")
            return
        name = self.name_input.text().strip()
        if not name:
            QMessageBox.warning(self, "Error", "Enter a world name.")
            return
        try:
            wm.load_world(name)
            # Auto-generate map if missing
            maps_dir = os.path.join(PROJECT_ROOT, 'code', 'control_ws', 'src',
                                    'dynamic_obstacle_gz_spawning', 'maps')
            pgm = os.path.join(maps_dir, f"{name}.pgm")
            yml = os.path.join(maps_dir, f"{name}.yaml")
            if not os.path.exists(pgm) or not os.path.exists(yml):
                sdf = os.path.join(PROJECT_ROOT, 'code', 'control_ws', 'src',
                                   'dynamic_obstacle_gz_spawning', 'worlds', f"{name}.sdf")
                if os.path.exists(sdf):
                    try:
                        cli = os.path.join(PROJECT_ROOT, 'code', 'classes', 'sdf2map_cli.py')
                        subprocess.run([sys.executable, cli, sdf, '-o', maps_dir, '--skip-missing'],
                                       capture_output=True, text=True, timeout=60)
                    except Exception:
                        pass
                
                # Copy the existing SDF to install directory to ensure Gazebo can find it
                if os.path.exists(sdf):
                    dst_install = os.path.join(PROJECT_ROOT, 'code', 'control_ws', 'install',
                                       'dynamic_obstacle_gz_spawning', 'share',
                                       'dynamic_obstacle_gz_spawning', 'worlds', f"{name}.sdf")
                    os.makedirs(os.path.dirname(dst_install), exist_ok=True)
                    import shutil
                    shutil.copyfile(sdf, dst_install)
            wm.map_path = pgm
            self.info_label.setText(f"✓  Loaded world: {name}")
            self.info_label.setStyleSheet(
                f"font-size:10pt; color:{Colors.SUCCESS}; background:transparent; font-weight:600;")
            self.app.worldLoaded.emit(name)
            self.panelReady.emit()
            self.app.status(f"Loaded world: {name}", "success")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))
