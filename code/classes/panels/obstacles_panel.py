"""
Obstacles Panel — Create obstacles AND assign motion paths in one unified view.
"""
from PyQt5.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QPushButton,
                             QListWidget, QMessageBox, QLabel, QDoubleSpinBox,
                             QFrame, QButtonGroup, QAbstractItemView, QScrollArea,
                             QGraphicsLineItem, QGraphicsEllipseItem, QGraphicsRectItem)
from PyQt5.QtCore import Qt, QEvent, QPointF, QLineF, QRectF
from PyQt5.QtGui import QPen, QColor, QBrush
from classes.widgets.color_picker import ColorPicker
from utils.theme import Colors, Dims, make_primary_button, make_section_label, make_danger_button
from utils.color_utils import get_color
from utils.config import PROJECT_ROOT
import math, yaml, os, subprocess
from ament_index_python.packages import get_package_share_directory


class _Toggle(QPushButton):
    def __init__(self, label, tid, accent=None):
        super().__init__(label)
        self.type_id = tid
        self.accent = accent or Colors.PRIMARY
        self.setCheckable(True); self.setCursor(Qt.PointingHandCursor)
        self._rs()
    def _rs(self):
        if self.isChecked():
            self.setStyleSheet(
                f"QPushButton{{background:{self.accent}; color:white; border:none; "
                f"border-radius:{Dims.RADIUS_MD}px; font-size:10pt; font-weight:600; padding:5px 4px;}}")
        else:
            self.setStyleSheet(
                f"QPushButton{{background:{Colors.BG_DARK}; color:{Colors.TEXT_SECONDARY}; "
                f"border:1px solid {Colors.BORDER}; border-radius:{Dims.RADIUS_MD}px; "
                f"font-size:10pt; padding:5px 4px;}}"
                f"QPushButton:hover{{background:{Colors.BG_ELEVATED};}}")
    def nextCheckState(self):
        if not self.isChecked(): self.setChecked(True)
    def setChecked(self, c):
        super().setChecked(c); self._rs()


def _spin(label, lo, hi, val, step, sfx, dec=2, w=45, tip=""):
    r = QHBoxLayout()
    lb = QLabel(label)
    lb.setStyleSheet(f"color:{Colors.TEXT_SECONDARY};font-size:10pt;background:transparent;")
    lb.setFixedWidth(w); r.addWidget(lb)
    sp = QDoubleSpinBox()
    sp.setRange(lo,hi); sp.setValue(val); sp.setSingleStep(step)
    sp.setSuffix(sfx); sp.setDecimals(dec)
    if tip: sp.setToolTip(tip)
    r.addWidget(sp)
    return r, lb, sp

def _sep():
    f=QFrame(); f.setFixedHeight(1); f.setStyleSheet(f"background:{Colors.BORDER};"); return f


class ObstaclesPanel(QWidget):
    def __init__(self, app_state, shared_view):
        super().__init__()
        self.app = app_state
        self.view = shared_view
        self.is_active = False
        self.view.viewport().installEventFilter(self)
        self.current_obstacle = None
        self.current_motion = "linear"
        self.points = []
        self.click_mode = None  # None | "place" | "path"
        self._preview_item = None

        layout = QVBoxLayout()
        layout.setSpacing(0); layout.setContentsMargins(0,0,0,0)

        # ── Sidebar ───────────────────────────────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet(f"QScrollArea{{border:none;background:{Colors.BG_SURFACE};}}")

        lw = QWidget(); lw.setObjectName("lp")
        lw.setStyleSheet(f"QWidget#lp{{background:{Colors.BG_SURFACE};}}")
        lv = QVBoxLayout()
        lv.setContentsMargins(Dims.PAD_LG,Dims.PAD_LG,Dims.PAD_LG,Dims.PAD_LG)
        lv.setSpacing(Dims.SPACING_SM)

        # ─ Add Obstacle ───────────────────────────────────────
        s = QLabel("Add Obstacle"); make_section_label(s); lv.addWidget(s)

        tr = QHBoxLayout(); tr.setSpacing(Dims.SPACING_SM)
        self.type_group = QButtonGroup(self); self.type_group.setExclusive(True)
        self.box_btn = _Toggle("📦 Box","box"); self.box_btn.setChecked(True); self.box_btn.setFixedHeight(44)
        self.cyl_btn = _Toggle("⬤ Cyl","cylinder"); self.cyl_btn.setFixedHeight(44)
        self.sph_btn = _Toggle("⚪ Sph","sphere"); self.sph_btn.setFixedHeight(44)
        for b in (self.box_btn, self.cyl_btn, self.sph_btn):
            self.type_group.addButton(b); tr.addWidget(b)
        lv.addLayout(tr)
        self.type_group.buttonClicked.connect(self._type_changed)

        r, self.w_lbl, self.w_spin = _spin("Width",0.01,50,1,.1," m")
        lv.addLayout(r)
        r, self.l_lbl, self.l_spin = _spin("Length",0.01,50,1,.1," m")
        lv.addLayout(r)
        r, self.h_lbl, self.h_spin = _spin("Height",0.01,50,1,.1," m")
        lv.addLayout(r)
        r, self.r_lbl, self.r_spin = _spin("Radius",0.01,25,.5,.1," m")
        lv.addLayout(r)

        self.color_picker = ColorPicker(default_color="Gray")
        lv.addWidget(self.color_picker)

        self.place_btn = QPushButton("⊕  Place on Canvas")
        make_primary_button(self.place_btn)
        self.place_btn.clicked.connect(self._start_place)
        lv.addWidget(self.place_btn)

        lv.addWidget(_sep())

        # ─ Obstacles List ─────────────────────────────────────
        s2 = QLabel("Obstacles"); make_section_label(s2); lv.addWidget(s2)
        self.obs_list = QListWidget()
        self.obs_list.setMinimumHeight(60); self.obs_list.setMaximumHeight(140)
        self.obs_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.obs_list.itemClicked.connect(self._obs_selected)
        lv.addWidget(self.obs_list)

        rm = QPushButton("🗑  Remove")
        make_danger_button(rm); rm.clicked.connect(self._remove)
        lv.addWidget(rm)

        lv.addWidget(_sep())

        # ─ Motion Path ────────────────────────────────────────
        self.motion_box = QWidget()
        mb = QVBoxLayout(); mb.setContentsMargins(0,0,0,0); mb.setSpacing(Dims.SPACING_SM)
        s3 = QLabel("Motion Path"); make_section_label(s3); mb.addWidget(s3)

        mr = QHBoxLayout(); mr.setSpacing(Dims.SPACING_SM)
        self.mg = QButtonGroup(self); self.mg.setExclusive(True)
        self.lin_btn = _Toggle("⟷ Linear","linear","#FF6B6B"); self.lin_btn.setChecked(True); self.lin_btn.setFixedHeight(32)
        self.ell_btn = _Toggle("⬮ Ellip","elliptical","#51CF66"); self.ell_btn.setFixedHeight(32)
        self.pol_btn = _Toggle("⬡ Poly","polygon","#339AF0"); self.pol_btn.setFixedHeight(32)
        for b in (self.lin_btn, self.ell_btn, self.pol_btn):
            self.mg.addButton(b); mr.addWidget(b)
        mb.addLayout(mr)
        self.mg.buttonClicked.connect(self._motion_changed)

        r, _, self.vel_spin = _spin("Vel",0.01,100,1,.5," m/s")
        mb.addLayout(r)
        r, _, self.std_spin = _spin("Std",0,10,.1,.05,"",dec=2)
        mb.addLayout(r)
        r, self.maj_lbl, self.maj_spin = _spin("Major",0.1,50,2,.5," m",1)
        mb.addLayout(r)
        r, self.min_lbl, self.min_spin = _spin("Minor",0.1,50,1,.5," m",1)
        mb.addLayout(r)

        self.instr = QLabel("Select an obstacle to configure motion")
        self.instr.setWordWrap(True)
        self.instr.setStyleSheet(
            f"font-size:9pt;color:{Colors.SECONDARY};background:{Colors.BG_DARK};"
            f"border:1px solid {Colors.BORDER};border-radius:{Dims.RADIUS_SM}px;padding:8px;")
        mb.addWidget(self.instr)

        pbr = QHBoxLayout(); pbr.setSpacing(Dims.SPACING_SM)
        self.sp_btn = QPushButton("▶  Start Path"); make_primary_button(self.sp_btn)
        self.sp_btn.clicked.connect(self._start_path); pbr.addWidget(self.sp_btn)
        self.fp_btn = QPushButton("■  Finish")
        self.fp_btn.clicked.connect(self._finish_path); pbr.addWidget(self.fp_btn)
        self.cl_btn = QPushButton("✕"); make_danger_button(self.cl_btn)
        self.cl_btn.setFixedWidth(36)
        self.cl_btn.clicked.connect(self._clear_motion); pbr.addWidget(self.cl_btn)
        mb.addLayout(pbr)

        self.motion_box.setLayout(mb)
        lv.addWidget(self.motion_box)

        lv.addWidget(_sep())

        # ─ Actions ────────────────────────────────────────────
        s4 = QLabel("Actions"); make_section_label(s4); lv.addWidget(s4)
        ab = QPushButton("▶  Apply & Preview"); make_primary_button(ab)
        ab.setMinimumHeight(Dims.BUTTON_HEIGHT_LG)
        ab.clicked.connect(self._launch); lv.addWidget(ab)
        eb = QPushButton("💾  Export YAML")
        eb.clicked.connect(lambda: self._export()); lv.addWidget(eb)

        lv.addStretch()
        lw.setLayout(lv); scroll.setWidget(lw)
        scroll.setMinimumWidth(220); scroll.setMaximumWidth(360)

        layout.addWidget(scroll)
        self.setLayout(layout)
        self._type_changed(); self._motion_changed()

    def set_active(self, active):
        self.is_active = active
        if active:
            self._refresh()

    # ── Show / refresh ─────────────────────────────────────────
    def showEvent(self, e):
        super().showEvent(e)
        if self.is_active:
            self._refresh()

    def _refresh(self):
        self.obs_list.clear()
        wm = self.app.world_manager
        if not wm: return
        from classes.main_window import refresh_canvas
        refresh_canvas(self.app)
        for m in wm.models:
            if m.get("status")!="removed" and m["type"] in ("box","cylinder","sphere"):
                self.obs_list.addItem(m["name"])

    def _s(self, m, k="info"): self.app.status(m, k)
    def snap(self, p, sp=10): return QPointF(round(p.x()/sp)*sp, round(p.y()/sp)*sp)

    # ── Type toggle ────────────────────────────────────────────
    def _type_changed(self, _=None):
        c = self.type_group.checkedButton()
        if not c: return
        t = c.type_id
        for w in (self.w_spin, self.w_lbl): w.setEnabled(t=="box")
        for w in (self.l_spin, self.l_lbl): w.setEnabled(t=="box")
        for w in (self.h_spin, self.h_lbl): w.setEnabled(t!="sphere")
        for w in (self.r_spin, self.r_lbl): w.setEnabled(t!="box")

    # ── Place ──────────────────────────────────────────────────
    def _start_place(self):
        self.click_mode = "place"
        self.view.set_crosshair_mode(True)
        self.instr.setText("Click canvas to place obstacle…")
        self._s("Click canvas to place obstacle", "info")

    def _do_place(self, pt):
        c = self.type_group.checkedButton()
        if not c: return
        t = c.type_id
        wm = self.app.world_manager
        if t=="box":
            sz = (self.w_spin.value(), self.l_spin.value(), self.h_spin.value())
            pz = sz[2]/2
        elif t=="cylinder":
            sz = (self.r_spin.value(), self.h_spin.value()); pz = sz[1]/2
        else:
            sz = (self.r_spin.value(),); pz = sz[0]
        name = f"{t}_{len(wm.models)+1}"
        if self._preview_item:
            self.app.scene.removeItem(self._preview_item)
            self._preview_item = None
            
        wm.add_model({"name":name,"type":t,"properties":{
            "position":(pt.x()/100, -pt.y()/100, pz),
            "size":sz, "color":self.color_picker.current_color},"status":"new"})
        self._refresh()
        self._s(f"Placed {name}", "success")
        self.click_mode = None; self.view.set_crosshair_mode(False)

    # ── Remove ─────────────────────────────────────────────────
    def _remove(self):
        wm = self.app.world_manager
        if not wm: return
        item = self.obs_list.currentItem()
        if not item: return
        name = item.text()
        for store in (self.app.obstacle_items, self.app.path_items):
            if name in store:
                entries = store[name]
                if isinstance(entries, tuple): entries = list(entries)
                if not isinstance(entries, list): entries = [entries]
                for e in entries:
                    self.app.scene.removeItem(e)
                del store[name]
        for m in wm.models:
            if m["name"]==name: m["status"]="removed"; break
        self.obs_list.takeItem(self.obs_list.row(item))
        if self.current_obstacle==name: self.current_obstacle=None
        self._s(f"Removed {name}", "warning")

    # ── Select obstacle ────────────────────────────────────────
    def _obs_selected(self, item):
        self.current_obstacle = item.text()
        wm = self.app.world_manager
        model = next((m for m in wm.models if m["name"]==self.current_obstacle), None)
        if not model: return
        if "motion" in model["properties"]:
            mot = model["properties"]["motion"]
            bm = {"linear":self.lin_btn,"elliptical":self.ell_btn,"polygon":self.pol_btn}
            if mot["type"] in bm: bm[mot["type"]].setChecked(True); self._motion_changed()
            self.current_motion = mot["type"]
            self.vel_spin.setValue(mot["velocity"]); self.std_spin.setValue(mot["std"])
            if mot["type"]=="elliptical":
                self.maj_spin.setValue(mot["semi_major"]); self.min_spin.setValue(mot["semi_minor"])
            if mot["type"] in ("linear","polygon"):
                self.points = [QPointF(x*100,-y*100) for x,y in mot["path"]]
            elif mot["type"]=="elliptical":
                cm = model["properties"]["position"][:2]
                c = QPointF(cm[0]*100,-cm[1]*100)
                a = mot["angle"]
                self.points = [c+QPointF(mot["semi_major"]*100*math.cos(a), -mot["semi_major"]*100*math.sin(a))]
            self._draw_path(close=(mot["type"]=="polygon"))
            self.instr.setText(f"Path loaded for {self.current_obstacle}")
        else:
            self._clear_gfx(); self.points = []
            self.vel_spin.setValue(1); self.std_spin.setValue(.1)
            self.maj_spin.setValue(2); self.min_spin.setValue(1)
            self.instr.setText("No motion — click 'Start Path'")
        self._s(f"Selected: {self.current_obstacle}", "info")

    # ── Motion type ────────────────────────────────────────────
    def _motion_changed(self, _=None):
        c = self.mg.checkedButton()
        if not c: return
        self.current_motion = c.type_id
        self._clear_gfx(); self.points = []
        ie = self.current_motion=="elliptical"
        for w in (self.maj_spin, self.maj_lbl, self.min_spin, self.min_lbl): w.setEnabled(ie)
        hints = {"linear":"Click 2 pts for linear path","elliptical":"Click 1 pt for ellipse orientation","polygon":"Click multiple pts, then 'Finish'"}
        self.instr.setText(hints.get(self.current_motion,""))

    # ── Path ───────────────────────────────────────────────────
    def _start_path(self):
        if not self.current_obstacle:
            self.instr.setText("⚠ Select an obstacle first"); return
        self.click_mode = "path"; self.points = []
        self._clear_gfx(); self.view.set_crosshair_mode(True)
        hints = {"linear":"Click 2 pts…","elliptical":"Click 1 pt…","polygon":"Click pts, then 'Finish'…"}
        self.instr.setText(hints.get(self.current_motion,"")); self._s("Defining path…","info")

    def _finish_path(self):
        self.click_mode = None; self.view.set_crosshair_mode(False)
        self._draw_path(close=self.current_motion=="polygon")
        self._store(); self.instr.setText(f"✓ Path saved for {self.current_obstacle}")
        self._s(f"Path saved for {self.current_obstacle}","success")

    def _clear_motion(self):
        if not self.current_obstacle: return
        self._clear_gfx(); self.points = []
        model = next((m for m in self.app.world_manager.models if m["name"]==self.current_obstacle),None)
        if model and "motion" in model["properties"]: del model["properties"]["motion"]
        self.instr.setText("Motion cleared"); self._s("Motion cleared","warning")

    # ── Canvas events ──────────────────────────────────────────
    def eventFilter(self, obj, event):
        if not self.is_active: return super().eventFilter(obj,event)
        if obj != self.view.viewport() or not self.app.world_manager: return super().eventFilter(obj,event)
        
        if event.type() == QEvent.MouseMove and self.click_mode == "path" and self.points:
            pt = self.snap(self.view.mapToScene(event.pos()))
            col = {"linear":"#FF6B6B","elliptical":"#51CF66","polygon":"#339AF0"}[self.current_motion]
            pen = QPen(QColor(col), 2, Qt.DashLine)
            
            if self.current_motion in ("linear", "polygon"):
                if not self._preview_item:
                    self._preview_item = QGraphicsLineItem(QLineF(self.points[-1], pt))
                    self._preview_item.setPen(pen)
                    self.app.scene.addItem(self._preview_item)
                else:
                    self._preview_item.setLine(QLineF(self.points[-1], pt))
                    
            elif self.current_motion == "elliptical":
                smaj, smin = self.maj_spin.value(), self.min_spin.value()
                model = next(m for m in self.app.world_manager.models if m["name"]==self.current_obstacle)
                cm = model["properties"]["position"][:2]
                center = QPointF(cm[0]*100, -cm[1]*100)
                d = pt - center
                angle = math.degrees(math.atan2(-d.y(), d.x()))
                
                if not self._preview_item:
                    self._preview_item = QGraphicsEllipseItem(QRectF(-smaj*100,-smin*100,2*smaj*100,2*smin*100))
                    self._preview_item.setPen(pen)
                    self.app.scene.addItem(self._preview_item)
                
                self._preview_item.setPos(center)
                self._preview_item.setRotation(-angle)
            return False

        if event.type() == QEvent.MouseMove and self.click_mode == "place":
            pt = self.snap(self.view.mapToScene(event.pos()))
            c = self.type_group.checkedButton()
            if not c: return False
            t = c.type_id
            
            pen = QPen(QColor(Colors.BORDER_LIGHT), 2)
            rgb = get_color(self.color_picker.current_color)
            brush = QBrush(QColor.fromRgbF(rgb[0], rgb[1], rgb[2], 0.5)) # Semi-transparent
            
            if self._preview_item:
                self.app.scene.removeItem(self._preview_item)
                self._preview_item = None
                
            if t == "box":
                w, l = self.w_spin.value() * 100, self.l_spin.value() * 100
                hw, hl = w/2, l/2
                self._preview_item = QGraphicsRectItem(QRectF(pt.x() - hw, pt.y() - hl, w, l))
            else:
                r = self.r_spin.value() * 100
                self._preview_item = QGraphicsEllipseItem(QRectF(pt.x() - r, pt.y() - r, 2*r, 2*r))
                
            self._preview_item.setPen(pen)
            self._preview_item.setBrush(brush)
            self.app.scene.addItem(self._preview_item)
            return False

        if event.type() != QEvent.MouseButtonPress or event.button() != Qt.LeftButton:
            return super().eventFilter(obj,event)
            
        pt = self.snap(self.view.mapToScene(event.pos()))
        if self.click_mode=="place": self._do_place(pt); return True
        if self.click_mode=="path":
            self.points.append(pt)
            mt = self.current_motion
            
            if mt=="linear" and len(self.points)==2:
                self.click_mode=None; self.view.set_crosshair_mode(False)
                if self._preview_item:
                    self.app.scene.removeItem(self._preview_item)
                    self._preview_item = None
                self._draw_path(); self._store()
                self.instr.setText(f"✓ Linear path set")
            elif mt=="elliptical" and len(self.points)==1:
                self.click_mode=None; self.view.set_crosshair_mode(False)
                if self._preview_item:
                    self.app.scene.removeItem(self._preview_item)
                    self._preview_item = None
                self._draw_path(); self._store()
                self.instr.setText(f"✓ Elliptical path set")
            elif mt=="polygon":
                if self._preview_item:
                    self.app.scene.removeItem(self._preview_item)
                    self._preview_item = None
                self._draw_path()
                self.instr.setText(f"{len(self.points)} pts — click more or 'Finish'")
            return True
        return super().eventFilter(obj,event)

    # ── Draw / clear path graphics ─────────────────────────────
    def _clear_gfx(self):
        if self.current_obstacle and self.current_obstacle in self.app.path_items:
            for i in self.app.path_items[self.current_obstacle]: self.app.scene.removeItem(i)
            del self.app.path_items[self.current_obstacle]

    def _draw_path(self, close=False):
        self._clear_gfx()
        items = []
        col = {"linear":"#FF6B6B","elliptical":"#51CF66","polygon":"#339AF0"}[self.current_motion]
        pen = QPen(QColor(col), 2, Qt.DashLine)
        if self.current_motion=="linear" and len(self.points)==2:
            ln = QGraphicsLineItem(QLineF(self.points[0], self.points[1]))
            ln.setPen(pen); self.app.scene.addItem(ln); items.append(ln)
        elif self.current_motion=="elliptical" and len(self.points)==1:
            smaj, smin = self.maj_spin.value(), self.min_spin.value()
            model = next(m for m in self.app.world_manager.models if m["name"]==self.current_obstacle)
            cm = model["properties"]["position"][:2]
            center = QPointF(cm[0]*100, -cm[1]*100)
            d = self.points[0] - center
            angle = math.degrees(math.atan2(-d.y(), d.x()))
            el = QGraphicsEllipseItem(QRectF(-smaj*100,-smin*100,2*smaj*100,2*smin*100))
            el.setPos(center); el.setRotation(-angle); el.setPen(pen)
            self.app.scene.addItem(el); items.append(el)
        elif self.current_motion=="polygon" and len(self.points)>=2:
            for i in range(len(self.points)-1):
                ln = QGraphicsLineItem(QLineF(self.points[i], self.points[i+1]))
                ln.setPen(pen); self.app.scene.addItem(ln); items.append(ln)
            if close and len(self.points)>=3:
                ln = QGraphicsLineItem(QLineF(self.points[-1], self.points[0]))
                ln.setPen(pen); self.app.scene.addItem(ln); items.append(ln)
        if self.current_obstacle:
            self.app.path_items[self.current_obstacle] = items

    # ── Store motion ───────────────────────────────────────────
    def _store(self):
        if not self.current_obstacle or not self.points: return
        vel, std = self.vel_spin.value(), self.std_spin.value()
        if vel<=0: QMessageBox.warning(self,"Error","Velocity must be positive."); return
        if self.current_motion=="linear" and len(self.points)==2:
            s,e = self.points
            length = math.hypot((e.x()-s.x())/100, (e.y()-s.y())/100)
            if vel > length/0.001*0.5:
                QMessageBox.warning(self,"Velocity Too High",
                    f"Max recommended: {length/0.001*0.5:.0f} m/s for length {length:.2f} m"); return
        model = next(m for m in self.app.world_manager.models if m["name"]==self.current_obstacle)
        mot = {"type":self.current_motion,"velocity":vel,"std":std}
        if self.current_motion in ("linear","polygon"):
            mot["path"] = [(p.x()/100, -p.y()/100) for p in self.points]
        elif self.current_motion=="elliptical":
            cm = model["properties"]["position"][:2]
            center = QPointF(cm[0]*100, -cm[1]*100)
            d = self.points[0]-center
            mot["semi_major"] = self.maj_spin.value()
            mot["semi_minor"] = self.min_spin.value()
            mot["angle"] = math.atan2(-d.y(), d.x())
        model["properties"]["motion"] = mot
        model["status"] = "updated" if model.get("status") else "new"

    # ── Launch / Export ─────────────────────────────────────────
    def _launch(self):
        wm = self.app.world_manager
        if not wm: QMessageBox.warning(self,"Error","No world loaded."); return
        for m in wm.models:
            if m["type"] in ("box","cylinder","sphere"): m["external_spawn"]=True
        try:
            wm.apply_changes()
            
            cfg = os.path.join(get_package_share_directory("dynamic_obstacle_gz_spawning"),"config","obstacles.yaml")
            self._export(cfg)
            self._refresh()
            
            has_dynamic = any("motion" in m.get("properties", {}) for m in wm.models if m["type"] in ("box","cylinder","sphere") and m.get("status") != "removed")
            
            self.app.launch_preview(use_ros_launch=has_dynamic)
            
        except Exception as e:
            QMessageBox.critical(self,"Error",str(e))

    def _export(self, path=None):
        wm = self.app.world_manager
        if not wm: QMessageBox.warning(self,"Error","No world loaded."); return
        obs = []
        for m in wm.models:
            if m.get("status")=="removed" or m["type"]=="wall" or m["type"] not in ("box","cylinder","sphere"):
                continue
            if "motion" not in m.get("properties", {}):
                continue
            o = {"name":m["name"],"type":m["type"],"color":m["properties"].get("color","gray").lower(),
                 "enabled":True,"x_pose":float(m["properties"]["position"][0]),
                 "y_pose":float(m["properties"]["position"][1]),
                 "z_pose":float(m["properties"]["position"][2]),"size":list(m["properties"]["size"])}
            if "motion" in m["properties"]:
                mt = m["properties"]["motion"]
                o["motion"] = {"type":mt["type"],"velocity":float(mt["velocity"]),"std":float(mt["std"])}
                if "path" in mt:
                    o["motion"]["path"] = [[p[0]-float(m["properties"]["position"][0]),
                                            p[1]-float(m["properties"]["position"][1])] for p in mt["path"]]
                if mt["type"]=="elliptical":
                    o["motion"]["semi_major"]=float(mt["semi_major"])
                    o["motion"]["semi_minor"]=float(mt["semi_minor"])
                    o["motion"]["angle"]=float(mt["angle"])
            obs.append(o)
        data = {"obstacles":obs}
        if path is None:
            if wm.world_path:
                path = os.path.join(get_package_share_directory("dynamic_obstacle_gz_spawning"),"config","obstacles.yaml")
            else: return
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path,"w") as f: yaml.dump(data,f,default_flow_style=None,sort_keys=False)
            self._s(f"Exported {os.path.basename(path)}","success")
        except Exception as e:
            QMessageBox.critical(self,"Error",str(e))
