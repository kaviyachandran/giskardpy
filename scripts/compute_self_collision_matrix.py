from __future__ import annotations
import traceback
from typing import Set, Tuple, List, Optional, Dict
import rospy
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidget, QCheckBox, QWidget, \
    QVBoxLayout, QHBoxLayout, QPushButton, QLineEdit, QFileDialog, QMessageBox, QProgressBar, QLabel, QDialog, \
    QDialogButtonBox, QComboBox, QFrame
from PyQt5.QtCore import Qt
import xml.etree.ElementTree as ET
import pandas as pd
from collections import defaultdict
import sys
import os

from std_msgs.msg import ColorRGBA

from giskardpy import identifier
from giskardpy.model.better_pybullet_syncer import BetterPyBulletSyncer
from giskardpy.model.collision_world_syncer import DisableCollisionReason
from giskardpy.model.ros_msg_visualization import ROSMsgVisualization
from giskardpy.model.utils import robot_name_from_urdf_string
from giskardpy.model.world import WorldTree
from giskardpy.my_types import PrefixName

reason_color_map = {
    DisableCollisionReason.Never: (163, 177, 233),  # blue
    DisableCollisionReason.Adjacent: (233, 163, 163),  # red
    DisableCollisionReason.AlmostAlways: (233, 163, 231),  # purple
    DisableCollisionReason.Default: (233, 231, 163),  # yellow
    DisableCollisionReason.Unknown: (153, 76, 0),  # brown
    None: (255, 255, 255),
}


class ReasonCheckBox(QCheckBox):
    row: int
    column: int
    table: Table

    def __init__(self, table: Table, row: int, column: int) -> None:
        super().__init__()
        self.reason = None
        self.row = row
        self.column = column
        self.table = table

    def connect_callback(self):
        self.stateChanged.connect(self.checkbox_callback)

    def sync_reason(self):
        reason = self.table.reason_from_index(self.row, self.column)
        self.setChecked(reason is not None)
        self.setStyleSheet(f'background-color: rgb{reason_color_map[reason]};')

    def checkbox_callback(self, state, update_range: bool = True):
        if update_range:
            self.table.selectedRanges()
            for range_ in self.table.selectedRanges():
                for row in range(range_.topRow(), range_.bottomRow() + 1):
                    for column in range(range_.leftColumn(), range_.rightColumn() + 1):
                        item = self.table.get_widget(row, column)
                        if state != item.checkState():
                            item.checkbox_callback(state, False)
        link1 = self.table.table_id_to_link_name(self.row)
        link2 = self.table.table_id_to_link_name(self.column)
        if state == Qt.Checked:
            reason = DisableCollisionReason.Unknown
        else:
            reason = None
        self.table.update_reason(link1, link2, reason)


class Table(QTableWidget):
    _reasons: Dict[Tuple[PrefixName, PrefixName], DisableCollisionReason]
    world: WorldTree

    def __init__(self, world: WorldTree, collision_scene: BetterPyBulletSyncer):
        super().__init__()
        self.cellClicked.connect(self.table_item_callback)
        self.world = world
        self.collision_scene = collision_scene
        self.ros_visualizer = ROSMsgVisualization('map')

    def get_widget(self, row, column):
        return self.cellWidget(row, column).layout().itemAt(0).widget()

    def prefix_reasons_to_str_reasons(self, reasons: Dict[Tuple[PrefixName, PrefixName], DisableCollisionReason]) \
            -> Dict[Tuple[str, str], DisableCollisionReason]:
        return {(x[0].short_name, x[1].short_name): reason for x, reason in reasons.items()}

    @property
    def str_reasons(self) -> Dict[Tuple[str, str], DisableCollisionReason]:
        return self.prefix_reasons_to_str_reasons(self._reasons)

    @property
    def reasons(self) -> Dict[Tuple[PrefixName, PrefixName], DisableCollisionReason]:
        return self._reasons

    def table_id_to_link_name(self, index: int) -> str:
        return self.link_names[index]

    def sort_links(self, link1: str, link2: str) -> Tuple[str, str]:
        return tuple(sorted((link1, link2)))

    def update_reason(self, link1: str, link2: str, new_reason: Optional[DisableCollisionReason]):
        link1 = self.world.search_for_link_name(link1)
        link2 = self.world.search_for_link_name(link2)
        key = self.sort_links(link1, link2)
        if new_reason is None:
            if key in self._reasons:
                del self._reasons[key]
        else:
            self._reasons[key] = new_reason
        row = self.link_names.index(link1.short_name)
        column = self.link_names.index(link2.short_name)
        self.get_widget(row, column).sync_reason()
        self.get_widget(column, row).sync_reason()

    def reason_from_index(self, row, column):
        link1 = self.table_id_to_link_name(row)
        link2 = self.table_id_to_link_name(column)
        key = tuple(sorted((link1, link2)))
        r_key = (key[1], key[0])
        reasons = self.str_reasons
        if key in reasons:
            return reasons[key]
        elif r_key in reasons:
            return reasons[r_key]
        return None

    def table_item_callback(self, row, column):
        self.ros_visualizer.clear_marker()
        self.collision_scene.sync()
        for link_name in self.world.link_names_with_collisions:
            self.world.links[link_name].dye_collisions(self.world.default_link_color)
        link1 = self.world.search_for_link_name(self.link_names[row])
        link2 = self.world.search_for_link_name(self.link_names[column])
        key = self.sort_links(link1, link2)
        reason = self.reasons.get(key, None)
        color = reason_color_map[reason]
        color_msg = ColorRGBA(color[0] / 255, color[1] / 255, color[2] / 255, 1)
        self.world.links[link1].dye_collisions(color_msg)
        self.world.links[link2].dye_collisions(color_msg)
        self.world.reset_cache()
        self.ros_visualizer.publish_markers()

    @property
    def link_names(self) -> List[str]:
        return list(sorted(x.short_name for x in self.world.link_names_with_collisions))

    def add_table_item(self, row, column):
        checkbox = ReasonCheckBox(self, row, column)
        checkbox.sync_reason()
        checkbox.connect_callback()
        if row == column:
            checkbox.setDisabled(True)

        widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(checkbox)
        layout.setAlignment(checkbox, Qt.AlignCenter)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        widget.setLayout(layout)
        self.setCellWidget(row, column, widget)

    def update_table(self, reasons: Dict[Tuple[PrefixName, PrefixName], DisableCollisionReason]):
        self._reasons = {self.sort_links(*k): v for k, v in reasons.items()}
        self.clear()
        self.setRowCount(len(self.link_names))
        self.setColumnCount(len(self.link_names))
        self.setHorizontalHeaderLabels(self.link_names)
        self.setVerticalHeaderLabels(self.link_names)

        for x, link1 in enumerate(self.link_names):
            for y, link2 in enumerate(self.link_names):
                self.add_table_item(x, y)

        num_rows = self.rowCount()

        widths = []

        for x in range(num_rows):
            item = self.item(x, 0)
            if item is not None:
                widths.append(item.sizeHint().width())
        if widths:
            self.setColumnWidth(0, max(widths))


def get_readable_color(red: float, green: float, blue: float) -> Tuple[int, int, int]:
    luminance = ((0.299 * red) + (0.587 * green) + (0.114 * blue)) / 255
    if luminance > 0.5:
        return 0, 0, 0
    else:
        return 255, 255, 255


class MyProgressBar(QProgressBar):
    def set_progress(self, value: int, text: Optional[str] = None):
        value = min(max(value, 0), 100)
        self.setValue(value)
        if text is not None:
            self.setFormat(f'{text}: %p%')
        self.parent().repaint()


class ComputeSelfCollisionMatrixParameterDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle('Set Parameters')

        self.parameters = {}

        self.layout = QVBoxLayout(self)
        self.layout.addWidget(QLabel('Set Thresholds for computing self collision matrix'))
        self.horizontalLine = QFrame()
        self.horizontalLine.setFrameShape(QFrame.HLine)
        self.horizontalLine.setFrameShadow(QFrame.Sunken)
        self.layout.addWidget(self.horizontalLine)
        self.layout.addLayout(
            self.make_parameter_entry('Add link pair when they are this close in default joint state:',
                                      0.0,
                                      'distance_threshold_zero'))
        self.layout.addLayout(
            self.make_parameter_entry('Add link pair when they are closer than this in 95% of checks:',
                                      0.005,
                                      'distance_threshold_always'))
        self.layout.addLayout(self.make_parameter_entry('distance_threshold_never_initial:',
                                                        0.05,
                                                        'distance_threshold_never_initial'))
        self.layout.addLayout(self.make_parameter_entry('distance_threshold_never_min:',
                                                        -0.02,
                                                        'distance_threshold_never_min'))
        self.layout.addLayout(self.make_parameter_entry('distance_threshold_never_range:',
                                                        0.05,
                                                        'distance_threshold_never_range'))
        self.layout.addLayout(self.make_parameter_entry('distance_threshold_never_zero:',
                                                        0.0,
                                                        'distance_threshold_never_zero'))

        self.buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
        self.layout.addWidget(self.buttonBox)

    def make_parameter_entry(self, text: str, default: float, parameter_name: str) -> QVBoxLayout:
        inner_box = QHBoxLayout()
        edit = QLineEdit(self)
        inner_box.addWidget(edit)
        inner_box.addWidget(QLabel('m'))
        edit.setText(str(default))
        edit.setValidator(QDoubleValidator(self))

        outer_box = QVBoxLayout()
        outer_box.addWidget(QLabel(text))
        outer_box.addLayout(inner_box)
        self.parameters[parameter_name] = edit
        return outer_box

    def get_parameter_map(self) -> Dict[str, float]:
        params = {param_name: float(edit.text()) for param_name, edit in self.parameters.items()}
        return params


class RosparamSelectionDialog(QDialog):
    default_option = '/robot_description'

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("File Selection")

        self.layout = QVBoxLayout(self)

        self.label = QLabel("Please select an option:")
        self.layout.addWidget(self.label)

        self.combo_box = QComboBox(self)
        self.combo_box.setEditable(True)  # Make the combo box editable
        self.layout.addWidget(self.combo_box)

        # Add the options to the combobox
        self.combo_box.addItems(rospy.get_param_names())
        if rospy.has_param(self.default_option):
            self.combo_box.setCurrentText(self.default_option)

        self.buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
        self.layout.addWidget(self.buttonBox)

    def get_selected_option(self):
        return self.combo_box.currentText()


class Application(QMainWindow):
    __srdf_path: Optional[str]

    def __init__(self):
        super().__init__()
        self.__srdf_path = None
        self.world = WorldTree.empty_world()
        self.world.default_link_color = ColorRGBA(0.5, 0.5, 0.5, 1)
        self.collision_scene = BetterPyBulletSyncer.empty(self.world)
        self.df = pd.DataFrame()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Self Collision Matrix Tool')
        self.setMinimumSize(800, 600)

        self.progress = MyProgressBar(self)

        self.table = Table(self.world, self.collision_scene)

        layout = QVBoxLayout()
        layout.addLayout(self._urdf_box_layout())
        self.horizontalLine = QFrame()
        self.horizontalLine.setFrameShape(QFrame.HLine)
        self.horizontalLine.setFrameShadow(QFrame.Sunken)
        layout.addWidget(self.horizontalLine)
        layout.addLayout(self._srdf_box_layout())
        layout.addWidget(self.progress)
        layout.addLayout(self._legend_box_layout())
        layout.addWidget(self.table)

        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        self.progress.set_progress(0, 'Load urdf')

    def _srdf_box_layout(self) -> QHBoxLayout:
        self.load_srdf_button = QPushButton('Load from srdf')
        self.load_srdf_button.clicked.connect(self.load_srdf)
        self.compute_srdf_button = QPushButton('Compute self collision matrix')
        self.compute_srdf_button.clicked.connect(self.compute_self_collision_matrix)
        self.save_srdf_button = QPushButton('Save as srdf')
        self.save_srdf_button.clicked.connect(self.save_srdf)
        srdf_bottoms = QHBoxLayout()
        srdf_bottoms.addWidget(self.compute_srdf_button)
        srdf_bottoms.addWidget(self.load_srdf_button)
        srdf_bottoms.addWidget(self.save_srdf_button)
        self.disable_srdf_buttons()
        return srdf_bottoms

    def _urdf_box_layout(self) -> QHBoxLayout:
        self.load_urdf_file_button = QPushButton('Load urdf from file')
        self.load_urdf_file_button.clicked.connect(self.load_urdf_from_path)
        self.load_urdf_param_button = QPushButton('Load urdf from parameter server')
        self.load_urdf_param_button.clicked.connect(self.load_urdf_from_paramserver)
        self.urdf_progress = MyProgressBar(self)
        self.urdf_progress.set_progress(0, 'No urdf loaded')
        urdf_section = QHBoxLayout()
        urdf_section.addWidget(self.load_urdf_file_button)
        urdf_section.addWidget(self.load_urdf_param_button)
        urdf_section.addWidget(self.urdf_progress)
        return urdf_section

    def _legend_box_layout(self) -> QHBoxLayout:
        legend = QHBoxLayout()

        for reason, color in reason_color_map.items():
            if reason is not None:
                label = QLabel(reason.name)
            else:
                label = QLabel('check collision')
            label.setStyleSheet(f'background-color: rgb{color}; color: rgb{get_readable_color(*color)};')
            if reason == DisableCollisionReason.Never:
                label.setToolTip('These links are never in contact.')
            elif reason == DisableCollisionReason.Unknown:
                label.setToolTip('This link pair was disabled for an unknown reason.')
            elif reason == DisableCollisionReason.Adjacent:
                label.setToolTip('This link pair is only connected by joints that cannot move.')
            elif reason == DisableCollisionReason.Default:
                label.setToolTip('This link pair is in collision in the robot\'s default state.')
            elif reason == DisableCollisionReason.AlmostAlways:
                label.setToolTip('This link pair is almost always in collision.')
            else:
                label.setToolTip('Collisions will be computed.')
            legend.addWidget(label)
        return legend

    def compute_self_collision_matrix(self):
        dialog = ComputeSelfCollisionMatrixParameterDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            parameters = dialog.get_parameter_map()
            reasons = self.collision_scene.compute_self_collision_matrix(self.group_name,
                                                                         save_to_tmp=False,
                                                                         progress_callback=self.progress.set_progress,
                                                                         **parameters)
            self.table.update_table(reasons)
            self.progress.set_progress(100, 'Done checking collisions')
        else:
            self.progress.set_progress(0, 'Canceled collision checking')

    def load_urdf_from_paramserver(self):
        dialog = RosparamSelectionDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            robot_description = dialog.get_selected_option()
            if rospy.has_param(robot_description):
                urdf = rospy.get_param(robot_description)
                self.load_urdf(urdf, robot_description)
            else:
                QMessageBox.critical(self, 'Error', f'Parameter not found: \n{robot_description}')

    def load_urdf(self, urdf: str, progress_str: str):
        self.world._clear()
        self.urdf_progress.set_progress(0, f'Loading {progress_str}')
        group_name = robot_name_from_urdf_string(urdf)
        self.urdf_progress.set_progress(10, f'Parsing {progress_str}')
        self.world.add_urdf(urdf, group_name)
        self.world.god_map.set_data(identifier.controlled_joints, self.world.movable_joint_names)
        self.urdf_progress.set_progress(50, f'Applying vhacd to concave meshes of {progress_str}')
        self.collision_scene.sync()
        self.urdf_progress.set_progress(80, f'Updating table {progress_str}')
        reasons = {(link_name, link_name): DisableCollisionReason.Adjacent for link_name in self.world.link_names}
        self.table.update_table(reasons)
        self.set_tmp_srdf_path()
        self.enable_srdf_buttons()
        self.urdf_progress.set_progress(100, f'Loaded {progress_str}')

    def set_tmp_srdf_path(self):
        self.__srdf_path = self.collision_scene.get_path_to_self_collision_matrix(self.group_name)

    def disable_srdf_buttons(self):
        self.__disable_srdf_buttons(True)

    def enable_srdf_buttons(self):
        self.__disable_srdf_buttons(False)

    def __disable_srdf_buttons(self, active: bool):
        self.save_srdf_button.setDisabled(active)
        self.load_srdf_button.setDisabled(active)
        self.compute_srdf_button.setDisabled(active)

    def load_srdf(self):
        srdf_file = self.get_srdf_path_with_dialog(False)
        if srdf_file is None:
            return
        try:
            if os.path.isfile(srdf_file):
                reasons = self.collision_scene.load_black_list_from_srdf(srdf_file, self.group_name, False)
                self.table.update_table(reasons)
                self.progress.set_progress(100, f'Loaded {srdf_file}')
            else:
                QMessageBox.critical(self, 'Error', f'File does not exist: \n{srdf_file}')
        except Exception as e:
            traceback.print_exc()
            QMessageBox.critical(self, 'Error', str(e))

    def load_urdf_from_path(self):
        options = QFileDialog.Options()
        options |= QFileDialog.ReadOnly
        urdf_file, _ = QFileDialog.getOpenFileName(
            self,
            "QFileDialog.getOpenFileName()",
            '',
            "urdf files (*.urdf);;All files (*)",
            options=options
        )
        if urdf_file:
            if not os.path.isfile(urdf_file):
                QMessageBox.critical(self, 'Error', f'File does not exist: \n{urdf_file}')
                return

            with open(urdf_file, 'r') as f:
                self.load_urdf(f.read(), urdf_file)

    @property
    def group_name(self):
        return list(self.world.group_names)[0]

    def get_srdf_path_with_dialog(self, save: bool) -> str:
        options = QFileDialog.Options()
        options |= QFileDialog.ReadOnly
        if save:
            srdf_file, _ = QFileDialog.getSaveFileName(
                self,
                "QFileDialog.getSaveFileName()",
                self.__srdf_path,
                "srdf files (*.srdf);;All files (*)",
                options=options
            )
        else:
            srdf_file, _ = QFileDialog.getOpenFileName(
                self,
                "QFileDialog.getOpenFileName()",
                self.__srdf_path,
                "srdf files (*.srdf);;All files (*)",
                options=options
            )

        if srdf_file:
            self.__srdf_path = srdf_file
        else:
            srdf_file = None

        return srdf_file

    def save_srdf(self):
        srdf_path = self.get_srdf_path_with_dialog(True)
        if srdf_path is not None:
            self.collision_scene.save_black_list(self.world.groups[self.group_name],
                                                 self.table.reasons,
                                                 file_name=srdf_path)
            self.progress.set_progress(100, f'Saved {self.__srdf_path}')


def main():
    # Display DataFrame in PyQt5 GUI
    app = QApplication(sys.argv)
    window = Application()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    rospy.init_node('self_collision_matrix_updater')
    main()
