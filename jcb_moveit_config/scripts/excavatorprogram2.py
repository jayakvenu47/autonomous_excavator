#! /usr/bin/env python3
from planning_path import ControlRobot
from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_ExcavatorProgram(object):
    def setupUi(self, ExcavatorProgram):
        ExcavatorProgram.setObjectName("ExcavatorProgram")
        ExcavatorProgram.resize(920, 300)

        self.input_x = QtWidgets.QDoubleSpinBox(ExcavatorProgram)
        self.input_x.setGeometry(QtCore.QRect(140, 10, 170, 60))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.input_x.setFont(font)
        self.input_x.setMinimum(-99.99)
        self.input_x.setObjectName("input_x")
        self.input_y = QtWidgets.QDoubleSpinBox(ExcavatorProgram)
        self.input_y.setGeometry(QtCore.QRect(430, 10, 170, 60))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.input_y.setFont(font)
        self.input_y.setMinimum(-99.99)
        self.input_y.setObjectName("input_y")
        self.input_z = QtWidgets.QDoubleSpinBox(ExcavatorProgram)
        self.input_z.setGeometry(QtCore.QRect(740, 10, 170, 60))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.input_z.setFont(font)
        self.input_z.setMinimum(-99.99)
        self.input_z.setProperty("value", 0.0)
        self.input_z.setObjectName("input_z")
        self.send_data = QtWidgets.QPushButton(ExcavatorProgram)
        self.send_data.setGeometry(QtCore.QRect(55, 190, 350, 90))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.send_data.setFont(font)
        self.send_data.setObjectName("send_data")
        self.x_label = QtWidgets.QLabel(ExcavatorProgram)
        self.x_label.setGeometry(QtCore.QRect(90, 10, 110, 50))
        font = QtGui.QFont()
        font.setPointSize(27)
        self.x_label.setFont(font)
        self.x_label.setObjectName("x_label")
        self.y_label = QtWidgets.QLabel(ExcavatorProgram)
        self.y_label.setGeometry(QtCore.QRect(380, 10, 110, 50))
        font = QtGui.QFont()
        font.setPointSize(27)
        self.y_label.setFont(font)
        self.y_label.setLineWidth(3)
        self.y_label.setObjectName("y_label")
        self.z_label = QtWidgets.QLabel(ExcavatorProgram)
        self.z_label.setGeometry(QtCore.QRect(690, 10, 110, 50))
        font = QtGui.QFont()
        font.setPointSize(27)
        self.z_label.setFont(font)
        self.z_label.setLineWidth(3)
        self.z_label.setObjectName("z_label")
        self.pitch_label = QtWidgets.QLabel(ExcavatorProgram)
        self.pitch_label.setGeometry(QtCore.QRect(10, 110, 141, 50))
        font = QtGui.QFont()
        font.setPointSize(27)
        self.pitch_label.setFont(font)
        self.pitch_label.setObjectName("pitch_label")
        self.input_pitch = QtWidgets.QDoubleSpinBox(ExcavatorProgram)
        self.input_pitch.setGeometry(QtCore.QRect(140, 110, 170, 60))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.input_pitch.setFont(font)
        self.input_pitch.setMinimum(-99.99)
        self.input_pitch.setObjectName("input_pitch")
        self.input_roll = QtWidgets.QDoubleSpinBox(ExcavatorProgram)
        self.input_roll.setGeometry(QtCore.QRect(430, 110, 170, 60))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.input_roll.setFont(font)
        self.input_roll.setMinimum(-99.99)
        self.input_roll.setObjectName("input_roll")
        self.input_yaw = QtWidgets.QDoubleSpinBox(ExcavatorProgram)
        self.input_yaw.setGeometry(QtCore.QRect(740, 110, 170, 60))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.input_yaw.setFont(font)
        self.input_yaw.setMinimum(-99.99)
        self.input_yaw.setObjectName("input_yaw")
        self.roll_label = QtWidgets.QLabel(ExcavatorProgram)
        self.roll_label.setGeometry(QtCore.QRect(330, 110, 110, 50))
        font = QtGui.QFont()
        font.setPointSize(27)
        self.roll_label.setFont(font)
        self.roll_label.setObjectName("roll_label")
        self.yaw_label = QtWidgets.QLabel(ExcavatorProgram)
        self.yaw_label.setGeometry(QtCore.QRect(620, 110, 110, 50))
        font = QtGui.QFont()
        font.setPointSize(27)
        self.yaw_label.setFont(font)
        self.yaw_label.setLineWidth(10)
        self.yaw_label.setObjectName("yaw_label")
        self.clear_data = QtWidgets.QPushButton(ExcavatorProgram)
        self.clear_data.setGeometry(QtCore.QRect(515, 190, 350, 90))
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(False)
        font.setWeight(50)
        self.clear_data.setFont(font)
        self.clear_data.setObjectName("clear_data")

        self.retranslateUi(ExcavatorProgram)

        self.send_data.clicked.connect(self.pass_values)
        self.clear_data.clicked.connect(self.clear_values)


        QtCore.QMetaObject.connectSlotsByName(ExcavatorProgram)


    def clear_values(self):
        #sets the values back to 0,00 when clear button is clicked
        self.input_x.setValue(0)
        self.input_y.setValue(0)
        self.input_z.setValue(0)
        self.input_pitch.setValue(0)
        self.input_roll.setValue(0)
        self.input_yaw.setValue(0)

    def pass_values(self):
        #prints out the inserted values when send button is clicked
        x = float(self.input_x.text())
        y = float(self.input_y.text())
        z = float(self.input_z.text())
        pitch = float(self.input_pitch.text())
        roll = float(self.input_roll.text())
        yaw = float(self.input_yaw.text())
        control_robot = ControlRobot()
        plan = control_robot.plan(x, y,
            z, roll, pitch, yaw)
        control_robot.pub_positions(plan)
        control_robot.go(plan)

    def retranslateUi(self, ExcavatorProgram):
        _translate = QtCore.QCoreApplication.translate
        ExcavatorProgram.setWindowTitle(_translate("ExcavatorProgram", "ExcavatorProgram"))
        self.send_data.setText(_translate("ExcavatorProgram", "Send"))
        self.x_label.setText(_translate("ExcavatorProgram", "X"))
        self.y_label.setText(_translate("ExcavatorProgram", "Y"))
        self.z_label.setText(_translate("ExcavatorProgram", "Z"))
        self.pitch_label.setText(_translate("ExcavatorProgram", "Pitch"))
        self.roll_label.setText(_translate("ExcavatorProgram", "Roll"))
        self.yaw_label.setText(_translate("ExcavatorProgram", "Yaw"))
        self.clear_data.setText(_translate("ExcavatorProgram", "Clear"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    ExcavatorProgram = QtWidgets.QDialog()
    ui = Ui_ExcavatorProgram()
    ui.setupUi(ExcavatorProgram)
    ExcavatorProgram.show()
    sys.exit(app.exec_())
