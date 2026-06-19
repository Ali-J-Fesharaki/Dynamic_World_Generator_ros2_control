import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QButtonGroup

class _Toggle(QPushButton):
    def __init__(self, label):
        super().__init__(label)
        self.setCheckable(True)
        self.toggled.connect(self._on_toggled)
        self._rs()
    def _on_toggled(self, checked):
        self._rs()
    def _rs(self):
        if self.isChecked():
            self.setStyleSheet("background: blue; color: white;")
        else:
            self.setStyleSheet("background: gray; color: black;")

app = QApplication(sys.argv)
w = QWidget()
l = QVBoxLayout(w)
bg = QButtonGroup(w)
bg.setExclusive(True)
b1 = _Toggle("B1"); b2 = _Toggle("B2")
bg.addButton(b1); bg.addButton(b2)
b1.setChecked(True)  # Set AFTER adding to group!

def print_state():
    print("B1:", b1.isChecked(), "B2:", b2.isChecked())

print_state()
b2.click()
print_state()
sys.exit(0)
