import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QButtonGroup
from PyQt5.QtCore import Qt, QTimer

class _Toggle(QPushButton):
    def __init__(self, label):
        super().__init__(label)
        self.setCheckable(True)
        self.toggled.connect(lambda _: self._rs())
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
b1.setChecked(True)
bg.addButton(b1); bg.addButton(b2)
l.addWidget(b1); l.addWidget(b2)

def auto_test():
    print("Pre-click:")
    print("B1:", b1.isChecked(), "B2:", b2.isChecked())
    
    # Simulate a real mouse click using QTest
    from PyQt5.QtTest import QTest
    QTest.mouseClick(b2, Qt.LeftButton)
    
    print("Post-click:")
    print("B1:", b1.isChecked(), "B1 style has blue:", "blue" in b1.styleSheet())
    print("B2:", b2.isChecked(), "B2 style has blue:", "blue" in b2.styleSheet())
    sys.exit(0)

QTimer.singleShot(100, auto_test)
w.show()
sys.exit(app.exec_())
