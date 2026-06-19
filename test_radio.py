import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QRadioButton, QButtonGroup
from PyQt5.QtCore import Qt

class _Toggle(QRadioButton):
    def __init__(self, label):
        super().__init__(label)
        self.setCursor(Qt.PointingHandCursor)
        self.toggled.connect(self._rs)
        self._rs()
        
    def _rs(self):
        base_style = "QRadioButton::indicator { width: 0px; height: 0px; background: transparent; border: none; }"
        if self.isChecked():
            self.setStyleSheet(base_style + "QRadioButton { background: blue; color: white; border-radius: 5px; padding: 5px; text-align: center; }")
        else:
            self.setStyleSheet(base_style + "QRadioButton { background: gray; color: black; border-radius: 5px; padding: 5px; text-align: center; }")

app = QApplication(sys.argv)
w = QWidget()
l = QVBoxLayout(w)
bg = QButtonGroup(w)
bg.setExclusive(True)
b1 = _Toggle("Box")
b2 = _Toggle("Cylinder")
b3 = _Toggle("Sphere")

bg.addButton(b1)
bg.addButton(b2)
bg.addButton(b3)

l.addWidget(b1)
l.addWidget(b2)
l.addWidget(b3)

b1.setChecked(True)
w.show()
sys.exit(app.exec_())
