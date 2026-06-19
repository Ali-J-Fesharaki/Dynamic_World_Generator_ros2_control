import sys, gc
from PyQt5.QtWidgets import QApplication, QPushButton

class Btn(QPushButton):
    def __init__(self):
        super().__init__("Test")
        self.setCheckable(True)
        self.toggled.connect(lambda _: self.my_slot())
    def my_slot(self):
        print("Slot called!")

app = QApplication(sys.argv)
b = Btn()
b.show()
gc.collect()
b.click()
gc.collect()
sys.exit(0)
