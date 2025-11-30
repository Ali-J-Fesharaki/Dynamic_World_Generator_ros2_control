from PyQt5.QtWidgets import QWizardPage, QLineEdit, QHBoxLayout, QWidget, QVBoxLayout, QLabel, QPushButton, QFrame
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal
from utils.config import INTRO_IMAGES_DIR
import os

class SimSelectionPage(QWizardPage):
    simulationSelected = pyqtSignal(str, str)

    def __init__(self):
        # Initialize wizard page with title and hidden fields
        super().__init__()
        self.setTitle("Select Simulation Platform")
        self._simulation = ""
        self._sim_version = ""

        # Register hidden fields for simulation and version
        self.simulation_field = QLineEdit()
        self.simulation_field.setVisible(False)
        self.sim_version_field = QLineEdit()
        self.sim_version_field.setVisible(False)
        self.registerField("simulation*", self.simulation_field)
        self.registerField("sim_version", self.sim_version_field)

        # Setup main layout with Gazebo and Isaac Sim sections
        layout = QHBoxLayout()

        # Gazebo section with Harmonic and Fortress options
        gazebo_widget = QWidget()
        gazebo_layout = QVBoxLayout()

        # Harmonic option
        harmonic_widget = QWidget()
        harmonic_layout = QVBoxLayout()
        harmonic_label = QLabel("Gazebo Harmonic (Recommended)")
        harmonic_label.setAlignment(Qt.AlignCenter)
        harmonic_label.setFont(QFont("Arial", 18, QFont.Bold | QFont.StyleItalic))
        harmonic_label.setStyleSheet("color: red;")
        harmonic_image_label = QLabel()
        harmonic_image_path = os.path.join(INTRO_IMAGES_DIR, "harmonic.png")
        if os.path.exists(harmonic_image_path):
            pixmap = QPixmap(harmonic_image_path).scaled(290, 290, Qt.KeepAspectRatio)
            harmonic_image_label.setPixmap(pixmap)
        else:
            harmonic_image_label.setText("Harmonic image not found")
        harmonic_image_label.setFixedSize(290, 290)
        harmonic_image_label.setAlignment(Qt.AlignCenter)
        self.harmonic_button = QPushButton("Select Harmonic")
        self.harmonic_button.setFont(QFont("Arial", 14))
        self.harmonic_button.setFixedHeight(50)
        self.harmonic_button.clicked.connect(lambda: self.select_gazebo_version("harmonic"))
        harmonic_layout.addWidget(harmonic_label)
        harmonic_layout.addSpacing(10)
        harmonic_layout.addWidget(harmonic_image_label, alignment=Qt.AlignCenter)
        harmonic_layout.addSpacing(10)
        harmonic_layout.addWidget(self.harmonic_button, alignment=Qt.AlignCenter)
        harmonic_layout.addStretch(1)
        harmonic_widget.setLayout(harmonic_layout)
        gazebo_layout.addWidget(harmonic_widget)

        # Horizontal separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        gazebo_layout.addWidget(separator)

        # Fortress option
        fortress_widget = QWidget()
        fortress_layout = QVBoxLayout()
        fortress_label = QLabel("Gazebo Fortress")
        fortress_label.setAlignment(Qt.AlignCenter)
        fortress_label.setFont(QFont("Arial", 18, QFont.Bold | QFont.StyleItalic))
        fortress_label.setStyleSheet("color: red;")
        fortress_image_label = QLabel()
        fortress_image_path = os.path.join(INTRO_IMAGES_DIR, "fortress.jpeg")
        if os.path.exists(fortress_image_path):
            pixmap = QPixmap(fortress_image_path).scaled(290, 290, Qt.KeepAspectRatio)
            fortress_image_label.setPixmap(pixmap)
        else:
            fortress_image_label.setText("Fortress image not found")
        fortress_image_label.setFixedSize(290, 290)
        fortress_image_label.setAlignment(Qt.AlignCenter)
        self.fortress_button = QPushButton("Select Fortress")
        self.fortress_button.setFont(QFont("Arial", 14))
        self.fortress_button.setFixedHeight(50)
        self.fortress_button.clicked.connect(lambda: self.select_gazebo_version("fortress"))
        fortress_layout.addWidget(fortress_label)
        fortress_layout.addSpacing(10)
        fortress_layout.addWidget(fortress_image_label, alignment=Qt.AlignCenter)
        fortress_layout.addSpacing(10)
        fortress_layout.addWidget(self.fortress_button, alignment=Qt.AlignCenter)
        fortress_layout.addStretch(1)
        fortress_widget.setLayout(fortress_layout)
        gazebo_layout.addWidget(fortress_widget)

        gazebo_layout.addStretch(1)
        gazebo_widget.setLayout(gazebo_layout)
        layout.addWidget(gazebo_widget, stretch=1)

        # Vertical divider
        divider = QFrame()
        divider.setFrameShape(QFrame.VLine)
        divider.setFrameShadow(QFrame.Sunken)
        layout.addWidget(divider, stretch=0)

        # Isaac Sim section (now enabled with version selection)
        isaac_widget = QWidget()
        isaac_layout = QVBoxLayout()
        
        # Isaac Sim 4.5 option
        isaac_45_widget = QWidget()
        isaac_45_layout = QVBoxLayout()
        isaac_45_label = QLabel("Isaac Sim 4.5.0")
        isaac_45_label.setAlignment(Qt.AlignCenter)
        isaac_45_label.setFont(QFont("Arial", 18, QFont.Bold | QFont.StyleItalic))
        isaac_45_label.setStyleSheet("color: #76B900;")  # NVIDIA green
        isaac_45_image_label = QLabel()
        isaac_45_image_path = os.path.join(INTRO_IMAGES_DIR, "isaacsim_450.png")
        # Try colored version first, fall back to gray
        if not os.path.exists(isaac_45_image_path):
            isaac_45_image_path = os.path.join(INTRO_IMAGES_DIR, "isaacsim_450_gray.png")
        if os.path.exists(isaac_45_image_path):
            pixmap = QPixmap(isaac_45_image_path).scaled(290, 290, Qt.KeepAspectRatio)
            isaac_45_image_label.setPixmap(pixmap)
        else:
            isaac_45_image_label.setText("Isaac Sim 4.5 image not found")
        isaac_45_image_label.setFixedSize(290, 290)
        isaac_45_image_label.setAlignment(Qt.AlignCenter)
        self.isaac_45_button = QPushButton("Select Isaac Sim 4.5")
        self.isaac_45_button.setFont(QFont("Arial", 14))
        self.isaac_45_button.setFixedHeight(50)
        self.isaac_45_button.clicked.connect(lambda: self.select_isaacsim_version("450"))
        isaac_45_layout.addWidget(isaac_45_label)
        isaac_45_layout.addSpacing(10)
        isaac_45_layout.addWidget(isaac_45_image_label, alignment=Qt.AlignCenter)
        isaac_45_layout.addSpacing(10)
        isaac_45_layout.addWidget(self.isaac_45_button, alignment=Qt.AlignCenter)
        isaac_45_layout.addStretch(1)
        isaac_45_widget.setLayout(isaac_45_layout)
        isaac_layout.addWidget(isaac_45_widget)

        # Horizontal separator for Isaac Sim versions
        isaac_separator = QFrame()
        isaac_separator.setFrameShape(QFrame.HLine)
        isaac_separator.setFrameShadow(QFrame.Sunken)
        isaac_layout.addWidget(isaac_separator)

        # Isaac Sim 5.0 option (future)
        isaac_50_widget = QWidget()
        isaac_50_layout = QVBoxLayout()
        isaac_50_label = QLabel("Isaac Sim 5.0.0 (Coming Soon)")
        isaac_50_label.setAlignment(Qt.AlignCenter)
        isaac_50_label.setFont(QFont("Arial", 18, QFont.Bold | QFont.StyleItalic))
        isaac_50_label.setStyleSheet("color: gray;")
        isaac_50_image_label = QLabel()
        isaac_50_image_path = os.path.join(INTRO_IMAGES_DIR, "isaacsim_500.png")
        if not os.path.exists(isaac_50_image_path):
            isaac_50_image_path = os.path.join(INTRO_IMAGES_DIR, "isaacsim_450_gray.png")
        if os.path.exists(isaac_50_image_path):
            pixmap = QPixmap(isaac_50_image_path).scaled(290, 290, Qt.KeepAspectRatio)
            isaac_50_image_label.setPixmap(pixmap)
        else:
            isaac_50_image_label.setText("Isaac Sim 5.0 image not found")
        isaac_50_image_label.setFixedSize(290, 290)
        isaac_50_image_label.setAlignment(Qt.AlignCenter)
        self.isaac_50_button = QPushButton("Select Isaac Sim 5.0")
        self.isaac_50_button.setFont(QFont("Arial", 14))
        self.isaac_50_button.setFixedHeight(50)
        self.isaac_50_button.setEnabled(False)  # Disabled until 5.0 is ready
        isaac_50_layout.addWidget(isaac_50_label)
        isaac_50_layout.addSpacing(10)
        isaac_50_layout.addWidget(isaac_50_image_label, alignment=Qt.AlignCenter)
        isaac_50_layout.addSpacing(10)
        isaac_50_layout.addWidget(self.isaac_50_button, alignment=Qt.AlignCenter)
        isaac_50_layout.addStretch(1)
        isaac_50_widget.setLayout(isaac_50_layout)
        isaac_layout.addWidget(isaac_50_widget)

        isaac_layout.addStretch(1)
        isaac_widget.setLayout(isaac_layout)
        layout.addWidget(isaac_widget, stretch=1)

        self.setLayout(layout)

        # Apply button stylesheets
        button_style = """
            QPushButton {
                background-color: #4A90E2;
                color: white;
                padding: 15px;
                font-size: 14pt;
            }
            QPushButton:hover {
                background-color: #6AB0F3;
                transform: scale(1.05);
            }
        """
        isaac_button_style = """
            QPushButton {
                background-color: #76B900;
                color: white;
                padding: 15px;
                font-size: 14pt;
            }
            QPushButton:hover {
                background-color: #8ED100;
                transform: scale(1.05);
            }
        """
        disabled_button_style = """
            QPushButton {
                background-color: #A9A9A9;
                color: white;
                padding: 15px;
                font-size: 14pt;
            }
        """
        self.fortress_button.setStyleSheet(button_style)
        self.harmonic_button.setStyleSheet(button_style)
        self.isaac_45_button.setStyleSheet(isaac_button_style)
        self.isaac_50_button.setStyleSheet(disabled_button_style)

    def select_gazebo_version(self, version):
        # Select Gazebo version and emit signal
        self._simulation = "gazebo"
        self._sim_version = version
        self.simulation_field.setText("gazebo")
        self.sim_version_field.setText(version)
        self.simulationSelected.emit("gazebo", version)
        self.completeChanged.emit()

    def select_isaacsim_version(self, version):
        # Select Isaac Sim version and emit signal
        self._simulation = "isaacsim"
        self._sim_version = version
        self.simulation_field.setText("isaacsim")
        self.sim_version_field.setText(version)
        self.simulationSelected.emit("isaacsim", version)
        self.completeChanged.emit()

    def isComplete(self):
        # Check if a valid simulation and version are selected
        if self._simulation == "gazebo" and self._sim_version in ["fortress", "harmonic"]:
            return True
        if self._simulation == "isaacsim" and self._sim_version in ["450", "500"]:
            return True
        return False