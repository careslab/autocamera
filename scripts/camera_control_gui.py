# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'camera_control_gui.ui'
#
# Created: Wed Nov  8 11:39:16 2017
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName(_fromUtf8("Dialog"))
        Dialog.resize(672, 613)
        Dialog.setMinimumSize(QtCore.QSize(672, 613))
        Dialog.setMaximumSize(QtCore.QSize(1000, 1000))
        Dialog.setMouseTracking(True)
        self.groupBoxOperationMode = QtGui.QGroupBox(Dialog)
        self.groupBoxOperationMode.setGeometry(QtCore.QRect(10, 230, 161, 71))
        self.groupBoxOperationMode.setObjectName(_fromUtf8("groupBoxOperationMode"))
        self.radioButtonSimulation = QtGui.QRadioButton(self.groupBoxOperationMode)
        self.radioButtonSimulation.setGeometry(QtCore.QRect(10, 20, 116, 22))
        self.radioButtonSimulation.setObjectName(_fromUtf8("radioButtonSimulation"))
        self.radioButtonHardware = QtGui.QRadioButton(self.groupBoxOperationMode)
        self.radioButtonHardware.setGeometry(QtCore.QRect(10, 40, 116, 22))
        self.radioButtonHardware.setObjectName(_fromUtf8("radioButtonHardware"))
        self.groupBoxCameraControlMethod = QtGui.QGroupBox(Dialog)
        self.groupBoxCameraControlMethod.setGeometry(QtCore.QRect(10, 300, 171, 131))
        self.groupBoxCameraControlMethod.setAutoFillBackground(False)
        self.groupBoxCameraControlMethod.setObjectName(_fromUtf8("groupBoxCameraControlMethod"))
        self.radioButtonAutocamera = QtGui.QRadioButton(self.groupBoxCameraControlMethod)
        self.radioButtonAutocamera.setGeometry(QtCore.QRect(10, 40, 116, 22))
        self.radioButtonAutocamera.setObjectName(_fromUtf8("radioButtonAutocamera"))
        self.radioButtonClutchAndMove = QtGui.QRadioButton(self.groupBoxCameraControlMethod)
        self.radioButtonClutchAndMove.setGeometry(QtCore.QRect(10, 60, 151, 22))
        self.radioButtonClutchAndMove.setObjectName(_fromUtf8("radioButtonClutchAndMove"))
        self.radioButtonJoystick = QtGui.QRadioButton(self.groupBoxCameraControlMethod)
        self.radioButtonJoystick.setGeometry(QtCore.QRect(10, 80, 141, 22))
        self.radioButtonJoystick.setObjectName(_fromUtf8("radioButtonJoystick"))
        self.radioButtonTeleop = QtGui.QRadioButton(self.groupBoxCameraControlMethod)
        self.radioButtonTeleop.setGeometry(QtCore.QRect(10, 20, 116, 22))
        self.radioButtonTeleop.setObjectName(_fromUtf8("radioButtonTeleop"))
        self.radioButtonOculus = QtGui.QRadioButton(self.groupBoxCameraControlMethod)
        self.radioButtonOculus.setGeometry(QtCore.QRect(10, 100, 141, 22))
        self.radioButtonOculus.setObjectName(_fromUtf8("radioButtonOculus"))
        self.groupBoxPower = QtGui.QGroupBox(Dialog)
        self.groupBoxPower.setGeometry(QtCore.QRect(10, 20, 120, 181))
        self.groupBoxPower.setObjectName(_fromUtf8("groupBoxPower"))
        self.pushButtonHome = QtGui.QPushButton(self.groupBoxPower)
        self.pushButtonHome.setGeometry(QtCore.QRect(10, 20, 98, 27))
        self.pushButtonHome.setObjectName(_fromUtf8("pushButtonHome"))
        self.pushButtonPowerOff = QtGui.QPushButton(self.groupBoxPower)
        self.pushButtonPowerOff.setGeometry(QtCore.QRect(10, 80, 98, 27))
        self.pushButtonPowerOff.setObjectName(_fromUtf8("pushButtonPowerOff"))
        self.pushButtonExit = QtGui.QPushButton(self.groupBoxPower)
        self.pushButtonExit.setGeometry(QtCore.QRect(10, 140, 98, 27))
        self.pushButtonExit.setObjectName(_fromUtf8("pushButtonExit"))
        self.pushButtonPowerOn = QtGui.QPushButton(self.groupBoxPower)
        self.pushButtonPowerOn.setGeometry(QtCore.QRect(10, 50, 98, 27))
        self.pushButtonPowerOn.setObjectName(_fromUtf8("pushButtonPowerOn"))
        self.pushButtonReset = QtGui.QPushButton(self.groupBoxPower)
        self.pushButtonReset.setGeometry(QtCore.QRect(10, 110, 98, 27))
        self.pushButtonReset.setObjectName(_fromUtf8("pushButtonReset"))
        self.groupBoxAutocameraParams = QtGui.QGroupBox(Dialog)
        self.groupBoxAutocameraParams.setGeometry(QtCore.QRect(220, 20, 461, 80))
        self.groupBoxAutocameraParams.setObjectName(_fromUtf8("groupBoxAutocameraParams"))
        self.horizontalSliderInnerzone = QtGui.QSlider(self.groupBoxAutocameraParams)
        self.horizontalSliderInnerzone.setGeometry(QtCore.QRect(138, 23, 231, 29))
        self.horizontalSliderInnerzone.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSliderInnerzone.setObjectName(_fromUtf8("horizontalSliderInnerzone"))
        self.horizontalSliderDeadzone = QtGui.QSlider(self.groupBoxAutocameraParams)
        self.horizontalSliderDeadzone.setGeometry(QtCore.QRect(138, 53, 231, 29))
        self.horizontalSliderDeadzone.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSliderDeadzone.setObjectName(_fromUtf8("horizontalSliderDeadzone"))
        self.labelInnerzoneValue = QtGui.QLabel(self.groupBoxAutocameraParams)
        self.labelInnerzoneValue.setGeometry(QtCore.QRect(378, 26, 41, 17))
        self.labelInnerzoneValue.setObjectName(_fromUtf8("labelInnerzoneValue"))
        self.labelDeadzoneValue = QtGui.QLabel(self.groupBoxAutocameraParams)
        self.labelDeadzoneValue.setGeometry(QtCore.QRect(378, 55, 41, 17))
        self.labelDeadzoneValue.setObjectName(_fromUtf8("labelDeadzoneValue"))
        self.label_3 = QtGui.QLabel(self.groupBoxAutocameraParams)
        self.label_3.setGeometry(QtCore.QRect(10, 29, 121, 20))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.label_4 = QtGui.QLabel(self.groupBoxAutocameraParams)
        self.label_4.setGeometry(QtCore.QRect(10, 56, 131, 20))
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.pushButtonRecord = QtGui.QPushButton(Dialog)
        self.pushButtonRecord.setGeometry(QtCore.QRect(13, 540, 131, 27))
        self.pushButtonRecord.setObjectName(_fromUtf8("pushButtonRecord"))
        self.labelSubjectInfo = QtGui.QLabel(Dialog)
        self.labelSubjectInfo.setGeometry(QtCore.QRect(90, 513, 111, 20))
        self.labelSubjectInfo.setText(_fromUtf8(""))
        self.labelSubjectInfo.setObjectName(_fromUtf8("labelSubjectInfo"))
        self.labelSubjectNumber = QtGui.QLabel(Dialog)
        self.labelSubjectNumber.setGeometry(QtCore.QRect(12, 490, 131, 16))
        self.labelSubjectNumber.setObjectName(_fromUtf8("labelSubjectNumber"))
        self.spinBoxSubjectNumber = QtGui.QSpinBox(Dialog)
        self.spinBoxSubjectNumber.setGeometry(QtCore.QRect(19, 510, 60, 27))
        self.spinBoxSubjectNumber.setMinimum(1)
        self.spinBoxSubjectNumber.setMaximum(20)
        self.spinBoxSubjectNumber.setObjectName(_fromUtf8("spinBoxSubjectNumber"))
        self.labelPatternNumber = QtGui.QLabel(Dialog)
        self.labelPatternNumber.setGeometry(QtCore.QRect(12, 430, 131, 16))
        self.labelPatternNumber.setObjectName(_fromUtf8("labelPatternNumber"))
        self.spinBoxPatternNumber = QtGui.QSpinBox(Dialog)
        self.spinBoxPatternNumber.setGeometry(QtCore.QRect(20, 450, 60, 27))
        self.spinBoxPatternNumber.setMinimum(1)
        self.spinBoxPatternNumber.setMaximum(5)
        self.spinBoxPatternNumber.setObjectName(_fromUtf8("spinBoxPatternNumber"))
        self.labelTimer = QtGui.QLabel(Dialog)
        self.labelTimer.setGeometry(QtCore.QRect(159, 545, 66, 17))
        self.labelTimer.setObjectName(_fromUtf8("labelTimer"))

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(_translate("Dialog", "da Vinci Camera Controller", None))
        self.groupBoxOperationMode.setTitle(_translate("Dialog", "Operation Mode", None))
        self.radioButtonSimulation.setText(_translate("Dialog", "Simulation", None))
        self.radioButtonHardware.setText(_translate("Dialog", "Hardware", None))
        self.groupBoxCameraControlMethod.setTitle(_translate("Dialog", "Camera Control Method", None))
        self.radioButtonAutocamera.setText(_translate("Dialog", "Autocamera", None))
        self.radioButtonClutchAndMove.setText(_translate("Dialog", "Clutch and Move", None))
        self.radioButtonJoystick.setText(_translate("Dialog", "Joystick Control", None))
        self.radioButtonTeleop.setText(_translate("Dialog", "Teleop", None))
        self.radioButtonOculus.setText(_translate("Dialog", "Oculus", None))
        self.groupBoxPower.setTitle(_translate("Dialog", "Power", None))
        self.pushButtonHome.setText(_translate("Dialog", "Home", None))
        self.pushButtonPowerOff.setText(_translate("Dialog", "Power Off", None))
        self.pushButtonExit.setText(_translate("Dialog", "Exit", None))
        self.pushButtonPowerOn.setText(_translate("Dialog", "Power On", None))
        self.pushButtonReset.setText(_translate("Dialog", "Reset", None))
        self.groupBoxAutocameraParams.setTitle(_translate("Dialog", "Autocamera Zoom Parameters", None))
        self.labelInnerzoneValue.setText(_translate("Dialog", "0", None))
        self.labelDeadzoneValue.setText(_translate("Dialog", "0", None))
        self.label_3.setText(_translate("Dialog", "Inner Zone Radius", None))
        self.label_4.setText(_translate("Dialog", "Dead Zone Radius", None))
        self.pushButtonRecord.setText(_translate("Dialog", "Record", None))
        self.labelSubjectNumber.setText(_translate("Dialog", "Subject Number", None))
        self.labelPatternNumber.setText(_translate("Dialog", "Pattern Number", None))
        self.labelTimer.setText(_translate("Dialog", "0:0:0", None))

