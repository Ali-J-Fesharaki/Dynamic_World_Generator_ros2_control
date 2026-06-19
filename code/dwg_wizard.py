"""
Dynamic World Generator Wizard — Entry Point.
Launches the main window application with the global dark theme applied.
"""
import sys
import os

# Ensure the code directory is in the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon
from classes.main_window import MainWindow
from utils.theme import apply_global_stylesheet

def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Dynamic World Generator Wizard")
    app.setOrganizationName("DWG")

    # Apply global dark theme
    apply_global_stylesheet(app)

    # Set application icon if available
    icon_path = os.path.join(os.path.dirname(__file__), "..", "images", "intro", "icon.png")
    if os.path.exists(icon_path):
        app.setWindowIcon(QIcon(icon_path))

    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()