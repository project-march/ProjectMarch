"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QFileDialog
import yaml
import os
import datetime

from .joint import Joint

class Window(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Mujoco PID Tuner")
        self.setGeometry(100, 100, 400, 300)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.joints = {
            'HAA': Joint('HAA'),
            'HFE': Joint('HFE'),
            'KFE': Joint('KFE'),
            'ADPF': Joint('ADPF'),
        }

        self.kp = []
        self.kd = []
        self.ki = []

    def configure_ui(self) -> None:

        self.layout = QHBoxLayout()
        self.central_widget.setLayout(self.layout)

        # Set the layout for each joint with equal width
        for _, joint in enumerate(self.joints.values()):
            self.layout.addLayout(joint.create_layout())

        # Set the layout for the menu
        self.layout.addLayout(self.create_menu())

            # Load gains from a file
        default_gains_file = get_package_share_directory('march_control') + '/config/mujoco/low_level_controller_tunings.yaml'
        self.load_yaml(default_gains_file)

    def create_menu(self) -> None:
        """
        TODO:
        - Create a button to set the gains to the default values
        - Create a button and functionality to save the gains to a file
        - Create a button and functionality to load the gains from a file
        - Create a functionality to store the gains as YAML
        """
        self.layout = QVBoxLayout()
        
        # Add buttons to the layout
        self.set_button = QPushButton('Set Gains')
        self.save_button = QPushButton('Save Gains')
        self.load_button = QPushButton('Load Gains')

        self.set_button.clicked.connect(self.set_gains)
        self.save_button.clicked.connect(self.save_gains)
        self.load_button.clicked.connect(self.load_gains)

        self.layout.addWidget(self.set_button)
        self.layout.addWidget(self.save_button)
        self.layout.addWidget(self.load_button)

        return self.layout

    def get_gains(self):
        self.kp = []
        self.kd = []
        self.ki = []

        for joint in self.joints.values():
            p, d, i = joint.get_gains()
            self.kp.append(p)
            self.kd.append(d)
            self.ki.append(i)

        return self.kp, self.kd, self.ki
    
    def set_gains(self):
        kp, kd, ki = self.get_gains()
        self.callback(kp, kd, ki)
    
    def save_gains(self):
        output_path_dir = os.path.join(os.path.dirname(__file__), '../outputs')
        output_path = os.path.join(output_path_dir, 'gains_' + datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '.yaml')

        if not os.path.exists(output_path_dir):
            os.makedirs(output_path_dir)
        
        data = {
            'mujoco_sim': {
                'ros__parameters': {
                    'position': {
                        'P': self.kp * 2,
                        'D': self.kd * 2,
                        'I': self.ki * 2,
                    },
                    'torque': {
                        'P': [0.0] * len(self.kp) * 2,
                        'D': [0.0] * len(self.kd) * 2,
                    }
                }
            }
        }

        with open(output_path, 'w') as output_file:
            yaml.dump(data, output_file, default_flow_style=False)
        print(f'Gains have been saved to {output_path}')

    def load_yaml(self, file_path):
        with open(file_path, 'r') as input_file:
            data = yaml.load(input_file, Loader=yaml.FullLoader)
        
        position_gains = data['mujoco_sim']['ros__parameters']['position']
        # torque_gains = data['mujoco_sim']['ros__parameters']['torque']

        position_kp = position_gains['P'][0:len(self.joints)]
        position_kd = position_gains['D'][0:len(self.joints)]
        position_ki = position_gains['I'][0:len(self.joints)]

        for i, joint in enumerate(self.joints.values()):
            joint.set_gains(position_kp[i], position_kd[i], position_ki[i])

        print(f'Gains have been loaded from {file_path}')
        print(f'Position KP: {position_kp}')
        print(f'Position KD: {position_kd}')
        print(f'Position KI: {position_ki}')

    def load_gains(self):
        file_path, _ = QFileDialog.getOpenFileName(self, 'Open Gains File', os.path.join(os.path.dirname(__file__), '../outputs'), 'YAML Files (*.yaml)')
        
        if file_path == '' or file_path is None:
            print('No file selected')
            return
        
        self.load_yaml(file_path)

    def set_callback(self, callback) -> None:
        self.callback = callback