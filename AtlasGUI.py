# -*- coding: utf-8 -*-
import sys
import Queue
import glob
import serial
import serial.tools.list_ports
import math
import time
from math import pi
from scipy.optimize import fsolve

from PyQt4 import QtCore,  QtGui,  QtOpenGL
QtWidgets = QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *


from OCC.BRep import BRep_Builder,  BRep_Tool_Triangulation
from OCC.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.BRepMesh import brepmesh_Mesh
from OCC.TopoDS import topods_Face,  topods_Edge
from OCC.TopExp import TopExp_Explorer
from OCC.TopAbs import TopAbs_FACE

from OCC.TopoDS import TopoDS_Shape
from OCC.TopoDS import TopoDS_Compound
from OCC.StlAPI import StlAPI_Reader
from OCC.gp import gp_Ax1,  gp_Pnt,  gp_Dir,  gp_Vec,  gp_Trsf,  gp_Quaternion
from OCC.TopLoc import TopLoc_Location
from OCC.AIS import (AIS_WireFrame,  AIS_Shaded)
from OCC.Aspect import (Aspect_GFM_NONE, Aspect_GFM_HOR, Aspect_GFM_VER, 
                    Aspect_GFM_DIAG1, Aspect_GFM_DIAG2, Aspect_GFM_CORNER1, 
                    Aspect_GFM_CORNER2, Aspect_GFM_CORNER3, 
                    Aspect_GFM_CORNER4)
from OCC.Display.qtWidgetDisplay import qtViewer3d

from AtlasSerialConnector import SerialConnectorThread
from AtlasDataBridge import DataBridge
from EbUtils import get_all_from_queue,  get_item_from_queue

from OCC.Quantity import (Quantity_NOC_BLACK,  Quantity_NOC_WHITE, 
                          Quantity_NOC_MATRAGRAY,  Quantity_NOC_AZURE, 
                          Quantity_NOC_BURLYWOOD,  Quantity_NOC_DARKGREEN, 
                          Quantity_NOC_STEELBLUE,  Quantity_NOC_STEELBLUE2, 
                          Quantity_NOC_STEELBLUE3,  Quantity_NOC_STEELBLUE4, 
                          Quantity_NOC_ROYALBLUE,  Quantity_NOC_CHOCOLATE, 
                          Quantity_NOC_CHOCOLATE1,  Quantity_NOC_CHOCOLATE2, 
                          Quantity_NOC_CHOCOLATE3,  Quantity_NOC_CHOCOLATE4, 
                          Quantity_NOC_DARKGOLDENROD,  Quantity_NOC_DARKGOLDENROD1, 
                          Quantity_NOC_DARKGOLDENROD2,  Quantity_NOC_DARKGOLDENROD3, 
                          Quantity_NOC_DARKGOLDENROD4,  Quantity_NOC_DARKSLATEGRAY1, 
                          Quantity_NOC_DARKSLATEGRAY4,  Quantity_NOC_GRAY42, 
                          Quantity_NOC_ANTIQUEWHITE4,  Quantity_NOC_DARKSLATEGRAY4, 
                          Quantity_NOC_DARKORANGE,  Quantity_NOC_DARKORANGE1, 
                          Quantity_NOC_DARKORANGE2,  Quantity_NOC_DARKORANGE3, 
                          Quantity_NOC_DARKORANGE4,  Quantity_NOC_FIREBRICK, 
                          Quantity_NOC_DEEPSKYBLUE4,  Quantity_NOC_DODGERBLUE4, 
                          Quantity_NOC_BROWN,  Quantity_NOC_BISQUE, 
                          Quantity_NOC_FIREBRICK4, 
                          Quantity_NOC_WHEAT4,  Quantity_NOC_GRAY28, 
                          Quantity_NOC_GRAY21,  Quantity_NOC_GRAY86,  Quantity_NOC_GRAY99)

displayMode = AIS_Shaded
displayShapeColor = Quantity_NOC_DARKORANGE4
displayEdgeColor = Quantity_NOC_FIREBRICK


enableDispEdge = [0,  1,  1,  1,  0,  0,  0]

backgroundColor1 = Quantity_NOC_GRAY99
backgroundColor2 = Quantity_NOC_GRAY86


class Ui_MainWindow(object):
    ### from the QT designer
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1121, 747)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.groupBox_Comm = QtGui.QGroupBox(self.centralwidget)
        self.groupBox_Comm.setMinimumSize(QtCore.QSize(425, 58))
        self.groupBox_Comm.setMaximumSize(QtCore.QSize(500, 58))
        self.groupBox_Comm.setAlignment(QtCore.Qt.AlignJustify|QtCore.Qt.AlignVCenter)
        self.groupBox_Comm.setObjectName(_fromUtf8("groupBox_Comm"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.groupBox_Comm)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.label_29 = QtGui.QLabel(self.groupBox_Comm)
        self.label_29.setObjectName(_fromUtf8("label_29"))
        self.horizontalLayout.addWidget(self.label_29)
        self.comboBox_SerialPort = QtGui.QComboBox(self.groupBox_Comm)
        self.comboBox_SerialPort.setMinimumSize(QtCore.QSize(100, 0))
        self.comboBox_SerialPort.setSizeAdjustPolicy(QtGui.QComboBox.AdjustToContents)
        self.comboBox_SerialPort.setObjectName(_fromUtf8("comboBox_SerialPort"))
        self.horizontalLayout.addWidget(self.comboBox_SerialPort)
        self.refreshBtn_SerialPort = QtGui.QPushButton(self.groupBox_Comm)
        self.refreshBtn_SerialPort.setMaximumSize(QtCore.QSize(70, 16777215))
        self.refreshBtn_SerialPort.setObjectName(_fromUtf8("refreshBtn_SerialPort"))
        self.horizontalLayout.addWidget(self.refreshBtn_SerialPort)
        self.connectBtn_SerialPort = QtGui.QPushButton(self.groupBox_Comm)
        self.connectBtn_SerialPort.setMaximumSize(QtCore.QSize(70, 16777215))
        self.connectBtn_SerialPort.setObjectName(_fromUtf8("connectBtn_SerialPort"))
        self.horizontalLayout.addWidget(self.connectBtn_SerialPort)
        self.stopBtn_SerialPort = QtGui.QPushButton(self.groupBox_Comm)
        self.stopBtn_SerialPort.setMaximumSize(QtCore.QSize(70, 16777215))
        self.stopBtn_SerialPort.setObjectName(_fromUtf8("stopBtn_SerialPort"))
        self.horizontalLayout.addWidget(self.stopBtn_SerialPort)
        self.horizontalLayout_3.addWidget(self.groupBox_Comm)
        self.groupBox = QtGui.QGroupBox(self.centralwidget)
        self.groupBox.setMinimumSize(QtCore.QSize(300, 58))
        self.groupBox.setMaximumSize(QtCore.QSize(304, 58))
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.horizontalLayout_9 = QtGui.QHBoxLayout(self.groupBox)
        self.horizontalLayout_9.setObjectName(_fromUtf8("horizontalLayout_9"))
        self.dispRadioBtn_Mode = QtGui.QRadioButton(self.groupBox)
        self.dispRadioBtn_Mode.setObjectName(_fromUtf8("dispRadioBtn_Mode"))
        self.horizontalLayout_9.addWidget(self.dispRadioBtn_Mode)
        self.ctrlRadioBtn_Mode = QtGui.QRadioButton(self.groupBox)
        self.ctrlRadioBtn_Mode.setObjectName(_fromUtf8("ctrlRadioBtn_Mode"))
        self.horizontalLayout_9.addWidget(self.ctrlRadioBtn_Mode)
        self.radioButton_3 = QtGui.QRadioButton(self.groupBox)
        self.radioButton_3.setObjectName(_fromUtf8("radioButton_3"))
        self.horizontalLayout_9.addWidget(self.radioButton_3)
        self.horizontalLayout_3.addWidget(self.groupBox)
        self.groupBox_CtrlOpts = QtGui.QGroupBox(self.centralwidget)
        self.groupBox_CtrlOpts.setMinimumSize(QtCore.QSize(0, 58))
        self.groupBox_CtrlOpts.setMaximumSize(QtCore.QSize(16777215, 58))
        self.groupBox_CtrlOpts.setAlignment(QtCore.Qt.AlignJustify|QtCore.Qt.AlignVCenter)
        self.groupBox_CtrlOpts.setObjectName(_fromUtf8("groupBox_CtrlOpts"))
        self.horizontalLayout_11 = QtGui.QHBoxLayout(self.groupBox_CtrlOpts)
        self.horizontalLayout_11.setObjectName(_fromUtf8("horizontalLayout_11"))
        self.freeplayRadioBtn_ctrlops = QtGui.QRadioButton(self.groupBox_CtrlOpts)
        self.freeplayRadioBtn_ctrlops.setObjectName(_fromUtf8("freeplayRadioBtn_ctrlops"))
        self.horizontalLayout_11.addWidget(self.freeplayRadioBtn_ctrlops)
        self.fixedeeflocRadioBtn_ctrlops = QtGui.QRadioButton(self.groupBox_CtrlOpts)
        self.fixedeeflocRadioBtn_ctrlops.setObjectName(_fromUtf8("fixedeeflocRadioBtn_ctrlops"))
        self.horizontalLayout_11.addWidget(self.fixedeeflocRadioBtn_ctrlops)
        self.fixedeefangRadioBtn_ctrlops = QtGui.QRadioButton(self.groupBox_CtrlOpts)
        self.fixedeefangRadioBtn_ctrlops.setObjectName(_fromUtf8("fixedeefangRadioBtn_ctrlops"))
        self.horizontalLayout_11.addWidget(self.fixedeefangRadioBtn_ctrlops)
        self.horizontalLayout_3.addWidget(self.groupBox_CtrlOpts)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.splitter = QtGui.QSplitter(self.centralwidget)
        self.splitter.setOrientation(QtCore.Qt.Horizontal)
        self.splitter.setObjectName(_fromUtf8("splitter"))
        self.groupBox_Servos = QtGui.QGroupBox(self.splitter)
        self.groupBox_Servos.setMinimumSize(QtCore.QSize(220, 582))
        self.groupBox_Servos.setAlignment(QtCore.Qt.AlignJustify|QtCore.Qt.AlignVCenter)
        self.groupBox_Servos.setObjectName(_fromUtf8("groupBox_Servos"))
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.groupBox_Servos)
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label_30 = QtGui.QLabel(self.groupBox_Servos)
        self.label_30.setObjectName(_fromUtf8("label_30"))
        self.horizontalLayout_2.addWidget(self.label_30)
        self.lcdNumber_Servo5 = QtGui.QLCDNumber(self.groupBox_Servos)
        self.lcdNumber_Servo5.setObjectName(_fromUtf8("lcdNumber_Servo5"))
        self.horizontalLayout_2.addWidget(self.lcdNumber_Servo5)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        self.horizontalSlider_Servo5 = QtGui.QSlider(self.groupBox_Servos)
        self.horizontalSlider_Servo5.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_Servo5.setObjectName(_fromUtf8("horizontalSlider_Servo5"))
        self.verticalLayout_2.addWidget(self.horizontalSlider_Servo5)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.label_31 = QtGui.QLabel(self.groupBox_Servos)
        self.label_31.setObjectName(_fromUtf8("label_31"))
        self.horizontalLayout_4.addWidget(self.label_31)
        self.lcdNumber_Servo4 = QtGui.QLCDNumber(self.groupBox_Servos)
        self.lcdNumber_Servo4.setObjectName(_fromUtf8("lcdNumber_Servo4"))
        self.horizontalLayout_4.addWidget(self.lcdNumber_Servo4)
        self.verticalLayout_2.addLayout(self.horizontalLayout_4)
        self.horizontalSlider_Servo4 = QtGui.QSlider(self.groupBox_Servos)
        self.horizontalSlider_Servo4.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_Servo4.setObjectName(_fromUtf8("horizontalSlider_Servo4"))
        self.verticalLayout_2.addWidget(self.horizontalSlider_Servo4)
        self.horizontalLayout_5 = QtGui.QHBoxLayout()
        self.horizontalLayout_5.setObjectName(_fromUtf8("horizontalLayout_5"))
        self.label_32 = QtGui.QLabel(self.groupBox_Servos)
        self.label_32.setObjectName(_fromUtf8("label_32"))
        self.horizontalLayout_5.addWidget(self.label_32)
        self.lcdNumber_Servo3 = QtGui.QLCDNumber(self.groupBox_Servos)
        self.lcdNumber_Servo3.setObjectName(_fromUtf8("lcdNumber_Servo3"))
        self.horizontalLayout_5.addWidget(self.lcdNumber_Servo3)
        self.verticalLayout_2.addLayout(self.horizontalLayout_5)
        self.drivingRadioBtn_servo3 = QtGui.QRadioButton(self.groupBox_Servos)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.drivingRadioBtn_servo3.sizePolicy().hasHeightForWidth())
        self.drivingRadioBtn_servo3.setSizePolicy(sizePolicy)
        self.drivingRadioBtn_servo3.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.drivingRadioBtn_servo3.setObjectName(_fromUtf8("drivingRadioBtn_servo3"))
        self.verticalLayout_2.addWidget(self.drivingRadioBtn_servo3)
        self.horizontalSlider_Servo3 = QtGui.QSlider(self.groupBox_Servos)
        self.horizontalSlider_Servo3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_Servo3.setObjectName(_fromUtf8("horizontalSlider_Servo3"))
        self.verticalLayout_2.addWidget(self.horizontalSlider_Servo3)
        self.horizontalLayout_6 = QtGui.QHBoxLayout()
        self.horizontalLayout_6.setObjectName(_fromUtf8("horizontalLayout_6"))
        self.label_33 = QtGui.QLabel(self.groupBox_Servos)
        self.label_33.setObjectName(_fromUtf8("label_33"))
        self.horizontalLayout_6.addWidget(self.label_33)
        self.lcdNumber_Servo2 = QtGui.QLCDNumber(self.groupBox_Servos)
        self.lcdNumber_Servo2.setObjectName(_fromUtf8("lcdNumber_Servo2"))
        self.horizontalLayout_6.addWidget(self.lcdNumber_Servo2)
        self.verticalLayout_2.addLayout(self.horizontalLayout_6)
        self.drivingRadioBtn_servo2 = QtGui.QRadioButton(self.groupBox_Servos)
        self.drivingRadioBtn_servo2.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.drivingRadioBtn_servo2.setObjectName(_fromUtf8("drivingRadioBtn_servo2"))
        self.verticalLayout_2.addWidget(self.drivingRadioBtn_servo2)
        self.horizontalSlider_Servo2 = QtGui.QSlider(self.groupBox_Servos)
        self.horizontalSlider_Servo2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_Servo2.setObjectName(_fromUtf8("horizontalSlider_Servo2"))
        self.verticalLayout_2.addWidget(self.horizontalSlider_Servo2)
        self.horizontalLayout_7 = QtGui.QHBoxLayout()
        self.horizontalLayout_7.setObjectName(_fromUtf8("horizontalLayout_7"))
        self.label_34 = QtGui.QLabel(self.groupBox_Servos)
        self.label_34.setObjectName(_fromUtf8("label_34"))
        self.horizontalLayout_7.addWidget(self.label_34)
        self.lcdNumber_Servo1 = QtGui.QLCDNumber(self.groupBox_Servos)
        self.lcdNumber_Servo1.setObjectName(_fromUtf8("lcdNumber_Servo1"))
        self.horizontalLayout_7.addWidget(self.lcdNumber_Servo1)
        self.verticalLayout_2.addLayout(self.horizontalLayout_7)
        self.drivingRadioBtn_servo1 = QtGui.QRadioButton(self.groupBox_Servos)
        self.drivingRadioBtn_servo1.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.drivingRadioBtn_servo1.setObjectName(_fromUtf8("drivingRadioBtn_servo1"))
        self.verticalLayout_2.addWidget(self.drivingRadioBtn_servo1)
        self.horizontalSlider_Servo1 = QtGui.QSlider(self.groupBox_Servos)
        self.horizontalSlider_Servo1.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_Servo1.setObjectName(_fromUtf8("horizontalSlider_Servo1"))
        self.verticalLayout_2.addWidget(self.horizontalSlider_Servo1)
        self.horizontalLayout_8 = QtGui.QHBoxLayout()
        self.horizontalLayout_8.setObjectName(_fromUtf8("horizontalLayout_8"))
        self.label_35 = QtGui.QLabel(self.groupBox_Servos)
        self.label_35.setObjectName(_fromUtf8("label_35"))
        self.horizontalLayout_8.addWidget(self.label_35)
        self.lcdNumber_Servo0 = QtGui.QLCDNumber(self.groupBox_Servos)
        self.lcdNumber_Servo0.setObjectName(_fromUtf8("lcdNumber_Servo0"))
        self.horizontalLayout_8.addWidget(self.lcdNumber_Servo0)
        self.verticalLayout_2.addLayout(self.horizontalLayout_8)
        self.horizontalSlider_Servo0 = QtGui.QSlider(self.groupBox_Servos)
        self.horizontalSlider_Servo0.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_Servo0.setObjectName(_fromUtf8("horizontalSlider_Servo0"))
        self.verticalLayout_2.addWidget(self.horizontalSlider_Servo0)
        self.groupBox_Visual = QtGui.QGroupBox(self.splitter)
        self.groupBox_Visual.setMinimumSize(QtCore.QSize(806, 608))
        self.groupBox_Visual.setAlignment(QtCore.Qt.AlignJustify|QtCore.Qt.AlignVCenter)
        self.groupBox_Visual.setObjectName(_fromUtf8("groupBox_Visual"))
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.groupBox_Visual)
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.widget_3dmodel = QtGui.QWidget(self.groupBox_Visual)
        self.widget_3dmodel.setMinimumSize(QtCore.QSize(788, 550))
        self.widget_3dmodel.setObjectName(_fromUtf8("widget_3dmodel"))
        self.verticalLayout_3.addWidget(self.widget_3dmodel)
        self.verticalLayout.addWidget(self.splitter)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1121, 21))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        
    ### also from the QT designer
    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.groupBox_Comm.setTitle(_translate("MainWindow", "COMMUNICATION", None))
        self.label_29.setText(_translate("MainWindow", "Serial Port", None))
        self.refreshBtn_SerialPort.setText(_translate("MainWindow", "Refresh", None))
        self.connectBtn_SerialPort.setText(_translate("MainWindow", "Connect", None))
        self.stopBtn_SerialPort.setText(_translate("MainWindow", "Stop", None))
        self.groupBox.setTitle(_translate("MainWindow", "MODE", None))
        self.dispRadioBtn_Mode.setText(_translate("MainWindow", "DISPLAY", None))
        self.ctrlRadioBtn_Mode.setText(_translate("MainWindow", "CONTROL", None))
        self.radioButton_3.setText(_translate("MainWindow", "RESERVED", None))
        self.groupBox_CtrlOpts.setTitle(_translate("MainWindow", "CONTROL OPTIONS", None))
        self.freeplayRadioBtn_ctrlops.setText(_translate("MainWindow", "Free Play", None))
        self.fixedeeflocRadioBtn_ctrlops.setText(_translate("MainWindow", "Fixed EEFLoc", None))
        self.fixedeefangRadioBtn_ctrlops.setText(_translate("MainWindow", "Fixed EEFAng", None))
        self.groupBox_Servos.setTitle(_translate("MainWindow", "SERVOS", None))
        self.label_30.setText(_translate("MainWindow", "Servo5", None))
        self.label_31.setText(_translate("MainWindow", "Servo4", None))
        self.label_32.setText(_translate("MainWindow", "Servo3", None))
        self.drivingRadioBtn_servo3.setText(_translate("MainWindow", "DRIVING", None))
        self.label_33.setText(_translate("MainWindow", "Servo2", None))
        self.drivingRadioBtn_servo2.setText(_translate("MainWindow", "DRIVING", None))
        self.label_34.setText(_translate("MainWindow", "Servo1", None))
        self.drivingRadioBtn_servo1.setText(_translate("MainWindow", "DRIVING", None))
        self.label_35.setText(_translate("MainWindow", "Servo0", None))
        self.groupBox_Visual.setTitle(_translate("MainWindow", "VISUALIZATION", None))


    ### return available ports
    def refresh_ports(self):
        ports_avail = QtCore.QStringList()
        for p in list(serial.tools.list_ports.comports()):
            num_port = 0;

            if "Arduino" in p[1]:    #Connect only to Arduino devices           
                ports_avail.append(str(p))
            else:        
                while num_port <256:
                    if not "Arduino" in p[1]:
                        str_port = "COM" + str(num_port)
                    if not "Arduino" in p[1] and str_port in p[1]:
                        num_port = 256
                        ports_avail.append(str_port)
                                
                    num_port = num_port+1
        return ports_avail
    
    def set_disp_Mode(self):
        self.dispRadioBtn_Mode.setChecked(True)
        self.ctrlRadioBtn_Mode.setEnabled(True)
        self.radioButton_3.setEnabled(False)
        self.drivingRadioBtn_servo1.setEnabled(False)
        self.drivingRadioBtn_servo2.setEnabled(False)
        self.drivingRadioBtn_servo3.setEnabled(False)
        self.freeplayRadioBtn_ctrlops.setEnabled(False)
        self.fixedeeflocRadioBtn_ctrlops.setEnabled(False)
        self.fixedeefangRadioBtn_ctrlops.setEnabled(False)
        self.horizontalSlider_Servo3.setDisabled(False)

    def set_ctrl_Mode(self):
        self.dispRadioBtn_Mode.setChecked(False)
        self.ctrlRadioBtn_Mode.setChecked(True)
        self.radioButton_3.setEnabled(False)
        self.freeplayRadioBtn_ctrlops.setEnabled(True)
        self.fixedeeflocRadioBtn_ctrlops.setEnabled(True)
        self.fixedeefangRadioBtn_ctrlops.setEnabled(True)

    ### wrapup all the changes to the GUI    
    def init_window(self):
        ### load ports
        self.comboBox_SerialPort.addItems(self.refresh_ports())
        self.connectedPort_deviceName = ''
        self.connectedPort_portName = ''

        ### prepare canvas
        model_canvas = qtViewer3d(self.widget_3dmodel)
        model_canvas.InitDriver()
        self.display = model_canvas._display

        ### reset main window
        self.verticalLayout_4 = QtGui.QVBoxLayout(self.widget_3dmodel)
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.model_canvas = model_canvas
        self.model_canvas.setObjectName(_fromUtf8("model_canvas"))
        self.verticalLayout_4.addWidget(self.model_canvas)

        ### set buttons
        self.set_disp_Mode()
        self.drivingRadioBtn_servo1.setChecked(True)

        self.ui_ctrlops = 0
        self.freeplayRadioBtn_ctrlops.setChecked(True)
        

        ### combine buttons
        self.refreshBtn_SerialPort.clicked.connect(self.update_ports)
        self.connectBtn_SerialPort.clicked.connect(self.connect_port)
        self.stopBtn_SerialPort.clicked.connect(self.on_stop)

        self.ui_mode = 0
        self.dispRadioBtn_Mode.toggled.connect(self.update_ui_mode)
        self.ctrlRadioBtn_Mode.toggled.connect(self.update_ui_mode)
        self.freeplayRadioBtn_ctrlops.toggled.connect(self.update_ui_ctrlops)
        self.fixedeeflocRadioBtn_ctrlops.toggled.connect(self.update_ui_ctrlops)
        self.fixedeefangRadioBtn_ctrlops.toggled.connect(self.update_ui_ctrlops)

        ### set link variables
        self.srvOffset = [70, 66, 22, 102, 92, 118]  # manual calibration for now
        self.srvInit = [70, 66, 0, 118, 92, 118]
        self.servo2Modifier = 1.0  # 1.158
        self.ServoInCommand = [False, False, False, False, False, False]

        ### connect lcd with slider
        self.horizontalSlider_Servo0.setMinimum(0)
        self.horizontalSlider_Servo1.setMinimum(0)
        self.horizontalSlider_Servo2.setMinimum(0)
        self.horizontalSlider_Servo3.setMinimum(0)
        self.horizontalSlider_Servo4.setMinimum(0)
        self.horizontalSlider_Servo5.setMinimum(118)

        self.horizontalSlider_Servo0.setMaximum(180)
        self.horizontalSlider_Servo1.setMaximum(180)
        self.horizontalSlider_Servo2.setMaximum(180)
        self.horizontalSlider_Servo3.setMaximum(180)
        self.horizontalSlider_Servo4.setMaximum(180)
        self.horizontalSlider_Servo5.setMaximum(156)

        self.horizontalSlider_Servo0.setValue(self.srvInit[0])
        self.horizontalSlider_Servo1.setValue(self.srvInit[1])
        self.horizontalSlider_Servo2.setValue(self.srvInit[2])
        self.horizontalSlider_Servo3.setValue(self.srvInit[3])
        self.horizontalSlider_Servo4.setValue(self.srvInit[4])
        self.horizontalSlider_Servo5.setValue(self.srvInit[5])

        self.lcdNumber_Servo0.display(self.srvInit[0])
        self.lcdNumber_Servo1.display(self.srvInit[1])
        self.lcdNumber_Servo2.display(self.srvInit[2])
        self.lcdNumber_Servo3.display(self.srvInit[3])
        self.lcdNumber_Servo4.display(self.srvInit[4])
        self.lcdNumber_Servo5.display(self.srvInit[5])

        # Connect value change event to servo movements
        self.horizontalSlider_Servo0.valueChanged.connect(self.update_servo0)
        self.horizontalSlider_Servo1.valueChanged.connect(self.update_servo1)
        self.horizontalSlider_Servo2.valueChanged.connect(self.update_servo2)
        self.horizontalSlider_Servo3.valueChanged.connect(self.update_servo3)
        self.horizontalSlider_Servo4.valueChanged.connect(self.update_servo4)
        self.horizontalSlider_Servo5.valueChanged.connect(self.update_servo5)

        ### data connection
        self.isConnected = False
        self.serialConnector = None
        self.com_data_queue = None
        self.com_error_queue = None
        self.dataBridge = DataBridge()
        self.timer = QTimer()


        ### load models
        stl_reader = StlAPI_Reader()
        
        self.base_shp = TopoDS_Shape()
        self.link1_shp = TopoDS_Shape()
        self.link2_shp = TopoDS_Shape()
        self.link3_shp = TopoDS_Shape()
        self.link4_shp = TopoDS_Shape()
        self.grpBase_shp = TopoDS_Shape()
        self.grpBlade_l_shp = TopoDS_Shape()
        self.grpBlade_r_shp = TopoDS_Shape()
        
        stl_reader.Read(self.base_shp, './models/Atlasbot-base.stl')
        stl_reader.Read(self.link1_shp, './models/Atlasbot-link1.stl')
        stl_reader.Read(self.link2_shp, './models/Atlasbot-link2.stl')
        stl_reader.Read(self.link3_shp, './models/Atlasbot-link3.stl')
        stl_reader.Read(self.link4_shp, './models/Atlasbot-link4.stl')
        stl_reader.Read(self.grpBase_shp, './models/Atlasbot-gripperbase.stl')
        stl_reader.Read(self.grpBlade_l_shp, './models/Atlasbot-gripperblade_l.stl')
        stl_reader.Read(self.grpBlade_r_shp, './models/Atlasbot-gripperblade_r.stl')


        
        ### preset shapes
        self.base_shp_disp = self.display.DisplayShape(self.base_shp, update=True)
        self.link1_shp_disp = self.display.DisplayShape(self.link1_shp, update=True)
        self.link2_shp_disp = self.display.DisplayShape(self.link2_shp, update=True)
        self.link3_shp_disp = self.display.DisplayShape(self.link3_shp, update=True)
        self.link4_shp_disp = self.display.DisplayShape(self.link4_shp, update=True)
        self.grpBase_shp_disp = self.display.DisplayShape(self.grpBase_shp, update=True)
        self.grpBlade_l_shp_disp = self.display.DisplayShape(self.grpBlade_l_shp, update=True)
        self.grpBlade_r_shp_disp = self.display.DisplayShape(self.grpBlade_r_shp, update=True)

        if enableDispEdge[0] == 1:
            self.base_edge_disp = self.display.DisplayShape(self.draw_edges(self.base_shp), update=True)
            self.display.Context.SetColor(self.base_edge_disp, displayEdgeColor)
        if enableDispEdge[1] == 1:
            self.link1_edge_disp = self.display.DisplayShape(self.draw_edges(self.link1_shp), update=True)
            self.display.Context.SetColor(self.link1_edge_disp, displayEdgeColor)
        if enableDispEdge[2] == 1:
            self.link2_edge_disp = self.display.DisplayShape(self.draw_edges(self.link2_shp), update=True)
            self.display.Context.SetColor(self.link2_edge_disp, displayEdgeColor)
        if enableDispEdge[3] == 1:
            self.link3_edge_disp = self.display.DisplayShape(self.draw_edges(self.link3_shp), update=True)
            self.display.Context.SetColor(self.link3_edge_disp, displayEdgeColor)
        if enableDispEdge[4] == 1:
            self.link4_edge_disp = self.display.DisplayShape(self.draw_edges(self.link4_shp), update=True)
            self.display.Context.SetColor(self.link4_edge_disp, displayEdgeColor)
        if enableDispEdge[5] == 1:
            self.grpBase_edge_disp = self.display.DisplayShape(self.draw_edges(self.grpBase_shp), update=True)
            self.display.Context.SetColor(self.grpBase_edge_disp, displayEdgeColor)
        if enableDispEdge[6] == 1:
            self.grpBlade_l_edge_disp = self.display.DisplayShape(self.draw_edges(self.grpBlade_l_shp), update=True)
            self.display.Context.SetColor(self.grpBlade_l_edge_disp, displayEdgeColor)
        if enableDispEdge[6] == 1:
            self.grpBlade_r_edge_disp = self.display.DisplayShape(self.draw_edges(self.grpBlade_r_shp), update=True)
            self.display.Context.SetColor(self.grpBlade_r_edge_disp, displayEdgeColor)
        

        self.display.Context.SetColor(self.base_shp_disp, displayShapeColor)
        self.display.Context.SetColor(self.link1_shp_disp, displayShapeColor)
        self.display.Context.SetColor(self.link2_shp_disp, displayShapeColor)
        self.display.Context.SetColor(self.link3_shp_disp, displayShapeColor)
        self.display.Context.SetColor(self.link4_shp_disp, displayShapeColor)
        self.display.Context.SetColor(self.grpBase_shp_disp, displayShapeColor)
        self.display.Context.SetColor(self.grpBlade_l_shp_disp, displayShapeColor)
        self.display.Context.SetColor(self.grpBlade_r_shp_disp, displayShapeColor)


        self.display.View.SetBgGradientColors(backgroundColor1, backgroundColor2, Aspect_GFM_DIAG1)


        #data copied from v-rep
        self.srv_axis_init_xyz = [[0.086162, -0.0039037, 0.032242], 
                                  [0.096498, -0.021926, 0.064419], 
                                  [0.082146, 0.014949, 0.11586], 
                                  [0.074163, -0.035895, 0.11590], 
                                  [0.072571, -0.052165, 0.11148], 
                                  [0.067154, -0.078290, 0.10921], 
                                  [0.060881, -0.076044, 0.10922]]
        
        #data converted from v-rep eular angles (r-xyz)
        self.srv_axis_init_dir = [[0.0001043375278241644,  0.0003787538898580119,  0.9999999228295826], 
                                    [0.9413940360911001,  -0.337289177900303,  -0.003644075115352252], 
                                    [-0.9413940360911001,  0.337289177900303,  0.003644075115352252], 
                                    [0.9413940360911001,  -0.337289177900303,  -0.003644075115352252], 
                                    [-0.3383437753245146,  -0.9370160917976437,  -0.08674291562672011], 
                                    [0.02937839028355497,  0.07962174116179024,  -0.996392135917738], 
                                    [0.030367550272932226,  0.08202937857574057,  -0.9961671511051239]]

        #define initial axes
        self.ax0 = gp_Ax1(gp_Pnt(self.srv_axis_init_xyz[0][0], 
                                 self.srv_axis_init_xyz[0][1], 
                                 self.srv_axis_init_xyz[0][2]), 
                          gp_Dir(self.srv_axis_init_dir[0][0], 
                                 self.srv_axis_init_dir[0][1], 
                                 self.srv_axis_init_dir[0][2]))
        
        self.ax1_z = gp_Ax1(gp_Pnt(self.srv_axis_init_xyz[1][0], 
                                 self.srv_axis_init_xyz[1][1], 
                                 self.srv_axis_init_xyz[1][2]), 
                          gp_Dir(0.0001043375278241644, 
                                 0.0003787538898580119, 
                                 0.9999999228295826))
        
        self.ax1 = gp_Ax1(gp_Pnt(self.srv_axis_init_xyz[1][0], 
                                 self.srv_axis_init_xyz[1][1], 
                                 self.srv_axis_init_xyz[1][2]), 
                          gp_Dir(self.srv_axis_init_dir[1][0], 
                                 self.srv_axis_init_dir[1][1], 
                                 self.srv_axis_init_dir[1][2]))

        self.ax2_z = gp_Ax1(gp_Pnt(self.srv_axis_init_xyz[2][0], 
                                 self.srv_axis_init_xyz[2][1], 
                                 self.srv_axis_init_xyz[2][2]), 
                          gp_Dir(0.0001043375278241644, 
                                 0.0003787538898580119, 
                                 0.9999999228295826))

        self.ax2 = gp_Ax1(gp_Pnt(self.srv_axis_init_xyz[2][0], 
                                 self.srv_axis_init_xyz[2][1], 
                                 self.srv_axis_init_xyz[2][2]), 
                          gp_Dir(self.srv_axis_init_dir[2][0], 
                                 self.srv_axis_init_dir[2][1], 
                                 self.srv_axis_init_dir[2][2]))
        
        self.ax3 = gp_Ax1(gp_Pnt(self.srv_axis_init_xyz[3][0], 
                                 self.srv_axis_init_xyz[3][1], 
                                 self.srv_axis_init_xyz[3][2]), 
                          gp_Dir(self.srv_axis_init_dir[3][0], 
                                 self.srv_axis_init_dir[3][1], 
                                 self.srv_axis_init_dir[3][2]))

        self.ax4 = gp_Ax1(gp_Pnt(self.srv_axis_init_xyz[4][0], 
                                 self.srv_axis_init_xyz[4][1], 
                                 self.srv_axis_init_xyz[4][2]), 
                          gp_Dir(self.srv_axis_init_dir[4][0], 
                                 self.srv_axis_init_dir[4][1], 
                                 self.srv_axis_init_dir[4][2]))
        
        self.ax5_l = gp_Ax1(gp_Pnt(self.srv_axis_init_xyz[5][0], 
                                   self.srv_axis_init_xyz[5][1], 
                                   self.srv_axis_init_xyz[5][2]), 
                            gp_Dir(self.srv_axis_init_dir[5][0], 
                                   self.srv_axis_init_dir[5][1], 
                                   self.srv_axis_init_dir[5][2]))

        self.ax5_r = gp_Ax1(gp_Pnt(self.srv_axis_init_xyz[6][0], 
                                   self.srv_axis_init_xyz[6][1], 
                                   self.srv_axis_init_xyz[6][2]), 
                            gp_Dir(self.srv_axis_init_dir[6][0], 
                                   self.srv_axis_init_dir[6][1], 
                                   self.srv_axis_init_dir[6][2]))

        
        ### define line model points
        ## target point
        self.trgt_pt_init_xyz  = [0.057463, -0.095332, 0.10793]
        self.trgt_pt_init_dir = [0.3359612227589048, 0.937850533069614, 0.08698525405712565]
        self.trgt_pt = gp_Ax1(gp_Pnt(self.trgt_pt_init_xyz[0], 
                                     self.trgt_pt_init_xyz[1], 
                                     self.trgt_pt_init_xyz[2]), 
                              gp_Dir(self.trgt_pt_init_dir[0], 
                                     self.trgt_pt_init_dir[1], 
                                     self.trgt_pt_init_dir[2]))

        ## model joint0
        self.joint_0_init_xyz  = [0.085063052, -0.017829008, 0.064463264]
        self.joint_0_init_dir = [0.941394036, -0.337289178, -0.003644075]
        self.joint_0 = gp_Ax1(gp_Pnt(self.joint_0_init_xyz[0], 
                                     self.joint_0_init_xyz[1], 
                                     self.joint_0_init_xyz[2]), 
                              gp_Dir(self.joint_0_init_dir[0], 
                                     self.joint_0_init_dir[1], 
                                     self.joint_0_init_dir[2]))

        ## model joint1
        self.joint_1_init_xyz  = [0.095315214, 0.010230643, 0.115809023]
        self.joint_1_init_dir = [0.941394036, -0.337289178, -0.003644075]
        self.joint_1 = gp_Ax1(gp_Pnt(self.joint_1_init_xyz[0], 
                                     self.joint_1_init_xyz[1], 
                                     self.joint_1_init_xyz[2]), 
                              gp_Dir(self.joint_1_init_dir[0], 
                                     self.joint_1_init_dir[1], 
                                     self.joint_1_init_dir[2]))

        ## model joint2
        self.joint_2_init_xyz  = [0.078262978, -0.037363968, 0.115884129]
        self.joint_2_init_dir = [0.941394036, -0.337289178, -0.003644075]
        self.joint_2 = gp_Ax1(gp_Pnt(self.joint_2_init_xyz[0], 
                                     self.joint_2_init_xyz[1], 
                                     self.joint_2_init_xyz[2]), 
                              gp_Dir(self.joint_2_init_dir[0], 
                                     self.joint_2_init_dir[1], 
                                     self.joint_2_init_dir[2]))

    # End of init_window


        
    ### reflect port changes
    def update_ports(self):
        self.comboBox_SerialPort.clear()
        self.comboBox_SerialPort.addItems(self.refresh_ports())

        if(self.connectedPort_deviceName != ''):
            foundOne = False
            for i in range(self.comboBox_SerialPort.count()):
                if self.connectedPort_deviceName == self.comboBox_SerialPort.itemText(i):
                    foundOne = True
                    break
            if foundOne == False:                
                self.on_stop()

    ### update ui mode
    def update_ui_mode(self):
        if(self.isConnected):
            self.on_stop()
        if(self.dispRadioBtn_Mode.isChecked()):
            self.ui_mode = 0
            self.set_disp_Mode()
        elif(self.ctrlRadioBtn_Mode.isChecked()):
            self.ui_mode = 1
            self.set_ctrl_Mode()

        print('mode',  self.ui_mode)

    ### update ui control options
    def update_ui_ctrlops(self):
        if(self.freeplayRadioBtn_ctrlops.isChecked()):
            self.ui_ctrlops = 0            
            self.drivingRadioBtn_servo1.setEnabled(False)
            self.drivingRadioBtn_servo2.setEnabled(False)
            self.drivingRadioBtn_servo3.setEnabled(False)
            self.horizontalSlider_Servo3.setDisabled(False)
            
        elif(self.fixedeeflocRadioBtn_ctrlops.isChecked()):
            self.ui_ctrlops = 1
            self.drivingRadioBtn_servo1.setEnabled(True)
            self.drivingRadioBtn_servo2.setEnabled(True)
            self.drivingRadioBtn_servo3.setEnabled(True)
            self.horizontalSlider_Servo3.setDisabled(False)
            
        elif(self.fixedeefangRadioBtn_ctrlops.isChecked()):
            self.ui_ctrlops = 2
            self.drivingRadioBtn_servo1.setEnabled(False)
            self.drivingRadioBtn_servo2.setEnabled(False)
            self.drivingRadioBtn_servo3.setEnabled(False)
            self.horizontalSlider_Servo3.setDisabled(True)

        print('control option',  self.ui_ctrlops)

    ###connect to the selected port
    def connect_port(self):
        comStr = str(self.comboBox_SerialPort.currentText())
        if self.serialConnector is not None or comStr == '':
            return
        ports_avail = QtCore.QStringList()
        for p in list(serial.tools.list_ports.comports()):
            if comStr == str(p):
                portStr = comStr.split()[0]
                print("connect to port: %s" % (comStr))
                self.data_q = Queue.Queue()
                self.error_q = Queue.Queue()
                self.serialConnector = SerialConnectorThread(
                    self.ui_mode,
                    self.data_q,
                    self.error_q,
                    portStr,
                    115200)
                self.serialConnector.start()

                com_error = get_item_from_queue(self.error_q)
                if com_error is not None:
                    QMessageBox.Critical(self, 'SerialConnectorThread error',
                                         com_error)
                    self.serialConnector = None
                    print(" --- failed")
                    self.connectBtn_SerialPort.setEnabled(True)
                else:
                    self.isConnected = True
                    self.connectedPort_deviceName = comStr
                    self.connectedPort_portName = portStr
                    self.connectBtn_SerialPort.setEnabled(False)
                    print(" --- successed")

                    ##determine ui mode
                    if (self.ui_mode == 0):
                        time.sleep(1)
                        self.serialConnector.write("d")
                        print("display mode")                        
                        self.timer = QTimer()
                        self.timer.timeout.connect(self.on_timer_up)

                        update_freq = 20
                        if update_freq > 0:
                            self.timer.start(1000.0 / update_freq)
                            
                    elif (self.ui_mode == 1):
                        time.sleep(1)
                        self.serialConnector.write("c")
                        self.output_angles()
                        print("control mode")
                break;

    def draw_edges(self,  shape):
        brepmesh_Mesh(shape,  0.8)
        builder = BRep_Builder()
        comp = TopoDS_Compound()
        builder.MakeCompound(comp)
        ex = TopExp_Explorer(shape,  TopAbs_FACE)
        while ex.More():
            face = topods_Face(ex.Current())
            location = TopLoc_Location()
            facing = (BRep_Tool_Triangulation(face,  location)).GetObject()
            tab = facing.Nodes()
            tri = facing.Triangles()
            for i in range(1,  facing.NbTriangles()+1):
                trian = tri.Value(i)
                index1,  index2,  index3 = trian.Get()
                for j in range(1,  4):
                    if j == 1:
                        m = index1
                        n = index2
                    elif j == 2:
                        n = index3
                    elif j == 3:
                        m = index2
                    me = BRepBuilderAPI_MakeEdge(tab.Value(m),  tab.Value(n))
                    if me.IsDone():
                        builder.Add(comp,  me.Edge())
            ex.Next()
        return comp
            
    ### update data on timer firing
    def on_timer_up(self):
        self.read_serial_data()
        self.update_angles()

    ### when the connection breaks    
    def on_stop(self):
        if self.serialConnector is not None:
            self.serialConnector.join(0.01)
            self.serialConnector = None

        self.isConnected = False        
        self.connectedPort_deviceName = ''
        self.connectedPort_portName = ''
        if(self.ui_mode == 0):
            self.timer.stop()
        self.connectBtn_SerialPort.setEnabled(True)
        
    ### read data
    def read_serial_data(self):

        qdata = list(get_all_from_queue(self.data_q))
        if len(qdata) > 0:
            data = dict(servo0=qdata[-1][0], 
                        servo1=qdata[-1][1], 
                        servo2=qdata[-1][2], 
                        servo3=qdata[-1][3], 
                        servo4=qdata[-1][4], 
                        servo5=qdata[-1][5])

            self.dataBridge.add_data(data)
            
    ### update angles on sliders
    def update_angles(self):
        """ Updates the model with new
            data. The livefeed is used to find out whether new
            data was received since the last update. If not,  
            nothing is updated.
        """
        if self.dataBridge.has_new_data:
            data = self.dataBridge.read_data()

            dataSet = list()
            dataSet.append(
                (data['servo0'],  data['servo1'],  data['servo2'], 
                 data['servo3'],  data['servo4'],  data['servo5']))
            
            if len(dataSet) > 10:
                dataSet.pop(0)
            for s in dataSet:                
                try:
                    intServo0 = int(s[0])
                    intServo1 = int(s[1])
                    intServo2 = int(s[2])
                    intServo3 = int(s[3])
                    intServo4 = int(s[4])
                    intServo5 = int(s[5])
                    ## print(intServo0, intServo1, intServo2, 
                    ##       intServo3, intServo4, intServo5)
                    self.horizontalSlider_Servo0.setValue(intServo0)
                    self.horizontalSlider_Servo1.setValue(intServo1)
                    self.horizontalSlider_Servo2.setValue(intServo2)
                    self.horizontalSlider_Servo3.setValue(intServo3)
                    self.horizontalSlider_Servo4.setValue(intServo4)
                    self.horizontalSlider_Servo5.setValue(intServo5)

                    self.rotate_servos(-1)
                except ValueError:
                    pass

    ### update servo angle displays
    def update_servo0(self):
        self.lcdNumber_Servo0.display(self.horizontalSlider_Servo0.value())
        self.rotate_servos(0)


    def update_servo1(self):
        self.lcdNumber_Servo1.display(self.horizontalSlider_Servo1.value())
        self.rotate_servos(1)

    def update_servo2(self):
        self.lcdNumber_Servo2.display(self.horizontalSlider_Servo2.value())
        self.rotate_servos(2)

    def update_servo3(self):
        self.lcdNumber_Servo3.display(self.horizontalSlider_Servo3.value())
        self.rotate_servos(3)

    def update_servo4(self):
        self.lcdNumber_Servo4.display(self.horizontalSlider_Servo4.value())
        self.rotate_servos(4)

    def update_servo5(self):
        self.lcdNumber_Servo5.display(self.horizontalSlider_Servo5.value())
        self.rotate_servos(5)
                
    def rotate_servos(self, idx_srv):
        ## ui state 0 -> display mode, ui state 1 -> control mode, idx_srv == -1 -> initialization
        ## 0 -> disconnected --> free joint movements/slider drag
        ## 0 -> connected --> free joint movements/listen to serial
        ## 1 -> disconnected --> constraint joint movements/no output
        ## 1 -> connected --> constraint joint movements/output to serial
        ## 1 -> ....... ---> move the driving servo ----> compute other servo angles and move them
        ## 1 -> ....... ---> move the slave servos ----> rotate the joint and re-compute/register the new target point location

        if (idx_srv != -1 and self.isConnected and self.ui_mode == 0):
            return

        if((self.ui_mode == 1 and (True in self.ServoInCommand))):
            print(self.ServoInCommand)
            return

        flag_NewTrgtPt = True        
        flag_output = True
        if (self.ui_mode == 0):
            flag_output = False
            angle_nominal0 = self.horizontalSlider_Servo0.value()
            angle_nominal1 = self.horizontalSlider_Servo1.value()
            angle_nominal2 = self.horizontalSlider_Servo2.value()
            angle_nominal3 = self.horizontalSlider_Servo3.value()
            angle_nominal4 = self.horizontalSlider_Servo4.value()
            angle_nominal5 = self.horizontalSlider_Servo5.value()
            
        elif (self.ui_mode == 1 and self.ui_ctrlops == 0):
            angle_nominal0 = self.horizontalSlider_Servo0.value()
            angle_nominal1 = self.horizontalSlider_Servo1.value()
            angle_nominal2 = self.horizontalSlider_Servo2.value()
            angle_nominal3 = self.horizontalSlider_Servo3.value()
            angle_nominal4 = self.horizontalSlider_Servo4.value()
            angle_nominal5 = self.horizontalSlider_Servo5.value()
            time.sleep(0.01)
            
        elif (self.ui_mode == 1 and self.ui_ctrlops == 1):
            angle_nominal0 = self.horizontalSlider_Servo0.value()
            
            angle = (angle_nominal0 - self.srvOffset[0]) / 360.0 * 2 * pi
            aServoTrsf = gp_Trsf()
            aServoTrsf.SetRotation(self.ax0,  angle)
            ax1 = self.ax1.Transformed(aServoTrsf)
            ax1_z = self.ax1_z.Transformed(aServoTrsf)
            
            ## model joint0
            joint_0 = self.joint_0.Transformed(aServoTrsf)        
            print("model joint0:", joint_0.Location().X(), joint_0.Location().Y(), joint_0.Location().Z())            
                

            if(self.drivingRadioBtn_servo1.isChecked()):
                if(idx_srv == 1):
                    flag_NewTrgtPt = False
                    flag_output = False
                    ## servo 1
                    angle_nominal1 = self.horizontalSlider_Servo1.value()

                    angle = (angle_nominal1-self.srvOffset[1])/360.0*2*pi
                    aServoTrsf1 = gp_Trsf()  
                    aServoTrsf1.SetRotation(ax1,  angle)
                    aServoTrsf1.Multiply(aServoTrsf)
                    ax2 = self.ax2.Transformed(aServoTrsf1)
                   
                    self.joint_1_mem = self.joint_1.Transformed(aServoTrsf1)  # 1
                    joint_1 = self.joint_1_mem

                        
                    angle_nominal2, angle_nominal3, flag_output  = self.compute_rotations_1(ax1, ax1_z, joint_0, joint_1)
                    if (self.isConnected):
                        if(flag_output):
                            self.ServoInCommand[idx_srv] = True
                            self.horizontalSlider_Servo2.setValue(angle_nominal2)
                            self.horizontalSlider_Servo3.setValue(angle_nominal3)
                            self.ServoInCommand[idx_srv] = False
                        else:
                            return
                    else:
                        self.ServoInCommand[idx_srv] = True
                        self.horizontalSlider_Servo2.setValue(angle_nominal2)
                        self.horizontalSlider_Servo3.setValue(angle_nominal3)
                        self.ServoInCommand[idx_srv] = False
                else:
                    angle_nominal1 = self.horizontalSlider_Servo1.value()
                    angle_nominal2 = self.horizontalSlider_Servo2.value()
                    angle_nominal3 = self.horizontalSlider_Servo3.value()

                angle_nominal4 = self.horizontalSlider_Servo4.value()
                angle_nominal5 = self.horizontalSlider_Servo5.value()


            elif(self.drivingRadioBtn_servo2.isChecked()):
                if(idx_srv == 2 ):
                    flag_NewTrgtPt = False
                    flag_output = False
                    ## servo 2
                    angle_nominal2 = self.horizontalSlider_Servo2.value()

                    theta = -(angle_nominal2-98.1058681505)/360.0*2*pi
                    

                        
                    angle_nominal1, angle_nominal3, flag_output  = self.compute_rotations_2(ax1, ax1_z, joint_0, self.joint_1, theta)
                    if (self.isConnected):
                        if(flag_output):
                            self.ServoInCommand[idx_srv] = True
                            self.horizontalSlider_Servo1.setValue(angle_nominal1)
                            self.horizontalSlider_Servo3.setValue(angle_nominal3)
                            self.ServoInCommand[idx_srv] = False
                        else:
                            return
                    else:
                        self.ServoInCommand[idx_srv] = True
                        self.horizontalSlider_Servo1.setValue(angle_nominal1)
                        self.horizontalSlider_Servo3.setValue(angle_nominal3)
                        self.ServoInCommand[idx_srv] = False
                else:
                    angle_nominal1 = self.horizontalSlider_Servo1.value()
                    angle_nominal2 = self.horizontalSlider_Servo2.value()
                    angle_nominal3 = self.horizontalSlider_Servo3.value()

                angle_nominal4 = self.horizontalSlider_Servo4.value()
                angle_nominal5 = self.horizontalSlider_Servo5.value()

            elif(self.drivingRadioBtn_servo3.isChecked()):
                if(idx_srv == 3 ):
                    flag_NewTrgtPt = False
                    flag_output = False
                    ## servo 3
                    angle_nominal3 = self.horizontalSlider_Servo3.value()

                    theta = -(angle_nominal3 - 94.5556396436)/360.0*2*pi

                        
                    angle_nominal1, angle_nominal2, flag_output  = self.compute_rotations_3(ax1, ax1_z, joint_0, self.joint_2, theta)
                    if (self.isConnected):
                        if(flag_output):
                            self.ServoInCommand[idx_srv] = True
                            self.horizontalSlider_Servo1.setValue(angle_nominal1)
                            self.horizontalSlider_Servo2.setValue(angle_nominal2)
                            self.ServoInCommand[idx_srv] = False
                        else:
                            return
                    else:
                        self.ServoInCommand[idx_srv] = True
                        self.horizontalSlider_Servo1.setValue(angle_nominal1)
                        self.horizontalSlider_Servo2.setValue(angle_nominal2)
                        self.ServoInCommand[idx_srv] = False
                else:
                    angle_nominal1 = self.horizontalSlider_Servo1.value()
                    angle_nominal2 = self.horizontalSlider_Servo2.value()
                    angle_nominal3 = self.horizontalSlider_Servo3.value()

                angle_nominal4 = self.horizontalSlider_Servo4.value()
                angle_nominal5 = self.horizontalSlider_Servo5.value()
                
        elif (self.ui_mode == 1 and self.ui_ctrlops == 2):
            angle_nominal0 = self.horizontalSlider_Servo0.value()
            angle_nominal1 = self.horizontalSlider_Servo1.value()
            angle_nominal2 = self.horizontalSlider_Servo2.value()
            angle_nominal3 = self.horizontalSlider_Servo3.value()
            angle_nominal4 = self.horizontalSlider_Servo4.value()
            angle_nominal5 = self.horizontalSlider_Servo5.value()            
            
            angle = (angle_nominal0 - self.srvOffset[0]) / 360.0 * 2 * pi
            aServoTrsf = gp_Trsf()
            aServoTrsf.SetRotation(self.ax0,  angle)
            ax1 = self.ax1.Transformed(aServoTrsf)
            ax1_z = self.ax1_z.Transformed(aServoTrsf)
            
            ## model joint0
            joint_0 = self.joint_0.Transformed(aServoTrsf)        
            print("model joint0:", joint_0.Location().X(), joint_0.Location().Y(), joint_0.Location().Z())
                       
            
            if(idx_srv == 1 or idx_srv == 2):
                flag_NewTrgtPt = False
                flag_output = False

                ## servo 1
                angle_diff  = self.angle_nominal1_mem - angle_nominal1
                
                angle_nominal3 = self.angle_nominal3_mem + angle_diff

                ## servo 2
                angle_diff  = self.servo2Modifier * ( self.angle_nominal2_mem - angle_nominal2 )
                angle_nominal3 = angle_nominal3 - angle_diff
             

                flag_output = True

                if (angle_nominal3 > 180.0 or angle_nominal3 < 0.0):
                    flag_output = False
                
                
                if(flag_output): 
                    self.ServoInCommand[idx_srv] = True                
                    self.horizontalSlider_Servo3.setValue(angle_nominal3)
                    self.ServoInCommand[idx_srv] = False
                else:
                    return

        self.rotate_all_servos(idx_srv, flag_NewTrgtPt, angle_nominal0, angle_nominal1, angle_nominal2, angle_nominal3, angle_nominal4, angle_nominal5)

        if (flag_output):
            self.output_angles()


    def rotate_all_servos(self, idx_srv, flag_NewTrgtPt, angle_nominal0, angle_nominal1, angle_nominal2, angle_nominal3, angle_nominal4, angle_nominal5):
        
        ## servo 0
        angle = (angle_nominal0 - self.srvOffset[0]) / 360.0 * 2 * pi
        aServoTrsf = gp_Trsf()
        aServoTrsf.SetRotation(self.ax0,  angle)
        aServoToploc = TopLoc_Location(aServoTrsf)
        self.display.Context.SetLocation(self.link1_shp_disp,  aServoToploc)
        if enableDispEdge[1] == 1:            
            self.display.Context.SetLocation(self.link1_edge_disp,  aServoToploc)

        ax1 = self.ax1.Transformed(aServoTrsf)

        ax1_z = self.ax1_z.Transformed(aServoTrsf)
        
        ## model joint0
        if (not flag_NewTrgtPt):
            joint_0 = self.joint_0.Transformed(aServoTrsf)
        else:
            self.joint_0_mem = self.joint_0.Transformed(aServoTrsf)
            joint_0 = self.joint_0_mem
              
        #print("model joint0:", joint_0.Location().X(), joint_0.Location().Y(), joint_0.Location().Z())

        ## servo 1
        angle = (angle_nominal1-self.srvOffset[1])/360.0*2*pi
        aServoTrsf1 = gp_Trsf()  
        aServoTrsf1.SetRotation(ax1,  angle)
        aServoTrsf1.Multiply(aServoTrsf)

        ax2 = self.ax2.Transformed(aServoTrsf1)

        aServoToploc = TopLoc_Location(aServoTrsf1)
        self.display.Context.SetLocation(self.link2_shp_disp,  aServoToploc)
        if enableDispEdge[2] == 1:
            self.display.Context.SetLocation(self.link2_edge_disp,  aServoToploc)

        ## model joint1
        if (not flag_NewTrgtPt):
            joint_1 = self.joint_1.Transformed(aServoTrsf1)  # 1
        else:
            self.joint_1_mem = self.joint_1.Transformed(aServoTrsf1)  # 1
            joint_1 = self.joint_1_mem
            self.angle_nominal1_mem = angle_nominal1

        #print("model joint1:", joint_1.Location().X(), joint_1.Location().Y(), joint_1.Location().Z())


        ## servo 2
        angle = self.servo2Modifier * ( angle_nominal2 + self.srvOffset[2]) / 360.0 * 2 * pi
        aServoTrsf2 = gp_Trsf()
        aServoTrsf2.SetRotation(ax2, angle)
        aServoTrsf2.Multiply(aServoTrsf1)

        ax3 = self.ax3.Transformed(aServoTrsf2)

        aServoToploc = TopLoc_Location(aServoTrsf2)
        self.display.Context.SetLocation(self.link3_shp_disp, aServoToploc)
        if enableDispEdge[3] == 1:
            self.display.Context.SetLocation(self.link3_edge_disp, aServoToploc)

        ## model joint2
        if (not flag_NewTrgtPt):
            joint_2 = self.joint_2.Transformed(aServoTrsf2)
        else:
            self.joint_2_mem = self.joint_2.Transformed(aServoTrsf2)  # 1
            joint_2 = self.joint_2_mem
            self.angle_nominal2_mem = angle_nominal2


        #print("model joint2:", joint_2.Location().X(), joint_2.Location().Y(), joint_2.Location().Z())


        ## servo 3
        

        angle = (angle_nominal3-self.srvOffset[3])/360.0*2*pi
        aServoTrsf3 = gp_Trsf()  
        aServoTrsf3.SetRotation(ax3,  angle)
        aServoTrsf3.Multiply(aServoTrsf2)

        ax4 = self.ax4.Transformed(aServoTrsf3)

        aServoToploc = TopLoc_Location(aServoTrsf3)
        self.display.Context.SetLocation(self.link4_shp_disp,  aServoToploc)
        if enableDispEdge[4] == 1:
            self.display.Context.SetLocation(self.link4_edge_disp,  aServoToploc)

        

        
        ## servo 4
        angle = (angle_nominal4-self.srvOffset[4])/360.0*2*pi
        aServoTrsf4 = gp_Trsf()  
        aServoTrsf4.SetRotation(ax4,  angle)
        aServoTrsf4.Multiply(aServoTrsf3)

        ax5_l = self.ax5_l.Transformed(aServoTrsf4)
        ax5_r = self.ax5_r.Transformed(aServoTrsf4)



        aServoToploc = TopLoc_Location(aServoTrsf4)

        self.display.Context.SetLocation(self.grpBase_shp_disp,  aServoToploc)
        if enableDispEdge[5] == 1:
            self.display.Context.SetLocation(self.grpBase_edge_disp,  aServoToploc)


        ## target point
        if (not flag_NewTrgtPt):
            trgt_pt = self.trgt_pt.Transformed(aServoTrsf4)
        else:
            self.trgt_pt_mem = self.trgt_pt.Transformed(aServoTrsf4)
            self.angle_nominal3_mem = angle_nominal3
            trgt_pt = self.trgt_pt_mem
        #print("target point:", trgt_pt.Location().X(), trgt_pt.Location().Y(), trgt_pt.Location().Z())

        
        

        ## servo 5
        angle = (angle_nominal5-self.srvOffset[5])/3/360.0*2*pi
        aServoTrsf5_l = gp_Trsf()  
        aServoTrsf5_l.SetRotation(ax5_l,  angle)
        aServoTrsf5_l.Multiply(aServoTrsf4)

        aServoTrsf5_r = gp_Trsf()  
        aServoTrsf5_r.SetRotation(ax5_r,  -angle)
        aServoTrsf5_r.Multiply(aServoTrsf4)

        aServoToploc = TopLoc_Location(aServoTrsf5_l)
        self.display.Context.SetLocation(self.grpBlade_l_shp_disp,  aServoToploc)
        if enableDispEdge[6] == 1:
            self.display.Context.SetLocation(self.grpBlade_l_edge_disp,  aServoToploc)

        aServoToploc = TopLoc_Location(aServoTrsf5_r)
        self.display.Context.SetLocation(self.grpBlade_r_shp_disp,  aServoToploc)
        if enableDispEdge[6] == 1:
            self.display.Context.SetLocation(self.grpBlade_r_edge_disp,  aServoToploc)


        self.display.Context.UpdateCurrentViewer()



    def compute_anglediff(self, x1, y1, z1, ax1, ax1_z):

        x0 = ax1_z.Direction().X()
        y0 = ax1_z.Direction().Y()
        z0 = ax1_z.Direction().Z()

        xn = ax1.Direction().X()
        yn = ax1.Direction().Y()
        zn = ax1.Direction().Z()

        
        x10 = y1 * z0 - z1 * y0
        y10 = z1 * x0 - x1 * z0
        z10 = x1 * y0 - y1 * x0
		
        cos_theta = (x1 * x0 + y1 * y0 + z1 * z0) / ((x1 ** 2 + y1 ** 2 + z1 **2) * (x0 ** 2 + y0 ** 2 + z0 **2))**0.5
        theta = math.acos(cos_theta)
        theta = math.copysign(theta, -(x10 * xn + y10 * yn + z10 * zn)) / pi * 180.0
        
		
        return (theta)
    
    def compute_EndEffectorRot_1(self, ax1, ax1_z, joint_mem, joint, trgt_pt):
        
        
        xtgt = self.trgt_pt_mem.Location().X() - joint_mem.Location().X()
        ytgt = self.trgt_pt_mem.Location().Y() - joint_mem.Location().Y()
        ztgt = self.trgt_pt_mem.Location().Z() - joint_mem.Location().Z()

 
        x1 = trgt_pt.Location().X() - joint.Location().X()
        y1 = trgt_pt.Location().Y() - joint.Location().Y()
        z1 = trgt_pt.Location().Z() - joint.Location().Z()

        
		
        theta0 = self.compute_anglediff(xtgt, ytgt, ztgt, ax1, ax1_z)
		
        theta1 = self.compute_anglediff(x1, y1, z1, ax1, ax1_z)
		
        theta_diff = theta1 - theta0

        return (theta_diff)
		


    def compute_rotations_1(self, ax1, ax1_z, joint_0, joint_1):
        
        l3 = 0.06209830859
        l2 = 0.05055720935
        l1 = 0.05940402141

        xn = ax1.Direction().X()
        yn = ax1.Direction().Y()
        zn = ax1.Direction().Z()

        xtgt = self.trgt_pt_mem.Location().X()
        ytgt = self.trgt_pt_mem.Location().Y()
        ztgt = self.trgt_pt_mem.Location().Z()

        print('xn', xn, yn, zn)

        def equations(p):
            x, y, z = p
            return (

                (x - joint_1.Location().X()) ** 2 + (y - joint_1.Location().Y()) ** 2 + (
                z - joint_1.Location().Z()) ** 2 - l2 ** 2,
                
                (x - xtgt) ** 2 + (y - ytgt) ** 2 + (z - ztgt) ** 2 - l3 ** 2,
                (joint_1.Location().X() - x) * xn + (joint_1.Location().Y() - y) * yn + (
                joint_1.Location().Z() - z) * zn
            )

        x_jt2, y_jt2, z_jt2 = fsolve(equations, (
        self.joint_2_mem.Location().X(), self.joint_2_mem.Location().Y(), self.joint_2_mem.Location().Z()))

        print('xyz:', x_jt2, y_jt2, z_jt2)

        print('target point 0:', self.trgt_pt.Location().X(), self.trgt_pt.Location().Y(), self.trgt_pt.Location().Z())

        x3 = xtgt - x_jt2
        y3 = ytgt - y_jt2
        z3 = ztgt - z_jt2

        print((x3 ** 2 + y3 ** 2 + z3 ** 2) ** 0.5)

        x2 = x_jt2 - joint_1.Location().X()
        y2 = y_jt2 - joint_1.Location().Y()
        z2 = z_jt2 - joint_1.Location().Z()

        print((x2 ** 2 + y2 ** 2 + z2 ** 2) ** 0.5)

        x1 = joint_1.Location().X() - joint_0.Location().X()
        y1 = joint_1.Location().Y() - joint_0.Location().Y()
        z1 = joint_1.Location().Z() - joint_0.Location().Z()

        x0 = ax1_z.Direction().X()
        y0 = ax1_z.Direction().Y()
        z0 = ax1_z.Direction().Z()

        print('x0', x0, y0, z0)

        x32 = y3 * z2 - z3 * y2
        y32 = z3 * x2 - x3 * z2
        z32 = x3 * y2 - y3 * x2

        x21 = y2 * z1 - z2 * y1
        y21 = z2 * x1 - x2 * z1
        z21 = x2 * y1 - y2 * x1

        x10 = y1 * z0 - z1 * y0
        y10 = z1 * x0 - x1 * z0
        z10 = x1 * y0 - y1 * x0

        cos_theta3 = (x3 * x2 + y3 * y2 + z3 * z2) / ((x3 ** 2 + y3 ** 2 + z3 ** 2) * (
        x2 ** 2 + y2 ** 2 + z2 ** 2)) ** 0.5
        theta3 = math.acos(cos_theta3)
        theta3 = math.copysign(theta3, -(x32 * xn + y32 * yn + z32 * zn))
        print(theta3 / math.pi * 180)

        cos_theta2 = (x2 * x1 + y2 * y1 + z2 * z1) / ((x2 ** 2 + y2 ** 2 + z2 ** 2) * (
        x1 ** 2 + y1 ** 2 + z1 ** 2)) ** 0.5
        theta2 = math.acos(cos_theta2)
        theta2 = math.copysign(theta2, -(x21 * xn + y21 * yn + z21 * zn))
        print(theta2 / math.pi * 180)

        cos_theta1 = (x1 * x0 + y1 * y0 + z1 * z0) / ((x1 ** 2 + y1 ** 2 + z1 ** 2) * (
        x0 ** 2 + y0 ** 2 + z0 ** 2)) ** 0.5
        theta1 = math.acos(cos_theta1)
        theta1 = math.copysign(theta1, -(x10 * xn + y10 * yn + z10 * zn))
        print(theta1 / math.pi * 180)

        servo2Angle = -theta2 / pi * 180.0 + 98.1058681505

        servo3Angle = theta3 / pi * 180.0 + 94.5556396436

        flag_output = True

        if (servo2Angle > 180.0 or servo2Angle < 0.0 or servo3Angle > 180.0 or servo3Angle < 0.0):
            flag_output = False

        return (servo2Angle, servo3Angle, flag_output)
    
            
    def compute_rotations_2(self, ax1, ax1_z, joint_0, joint_1, theta2):
        
        l3 = 0.06209830859
        l2 = 0.05055720935
        l1 = 0.05940402141

        xn = ax1.Direction().X()
        yn = ax1.Direction().Y()
        zn = ax1.Direction().Z()

        xtgt = self.trgt_pt_mem.Location().X()
        ytgt = self.trgt_pt_mem.Location().Y()
        ztgt = self.trgt_pt_mem.Location().Z()

        print('xn', xn, yn, zn)

        
        u = self.joint_1_mem.Direction().X()
        v = self.joint_1_mem.Direction().Y()
        w = self.joint_1_mem.Direction().Z()

        def equations(p):
            x, y, z = p
            return (

                (x - joint_0.Location().X()) ** 2 + (y - joint_0.Location().Y()) ** 2 + (
                z - joint_0.Location().Z()) ** 2 - l1 ** 2,
                
                ((x*(v**2 + w**2) - u*(y*v+z*w-u*((l2/l1+1)*(x-joint_0.Location().X())+joint_0.Location().X()) - v*((l2/l1+1)*(y-joint_0.Location().Y())+joint_0.Location().Y()) - w*((l2/l1+1)*(z-joint_0.Location().Z())+joint_0.Location().Z())))*(1-math.cos(theta2))
                 + ((l2/l1+1)*(x-joint_0.Location().X())+joint_0.Location().X())*math.cos(theta2) + (-z*v + y*w - w*((l2/l1+1)*(y-joint_0.Location().Y())+joint_0.Location().Y()) +v*((l2/l1+1)*(z-joint_0.Location().Z())+joint_0.Location().Z()))*math.sin(theta2) - xtgt)**2 +

                ((y*(u**2 + w**2) - v*(x*u+z*w-u*((l2/l1+1)*(x-joint_0.Location().X())+joint_0.Location().X()) - v*((l2/l1+1)*(y-joint_0.Location().Y())+joint_0.Location().Y()) - w*((l2/l1+1)*(z-joint_0.Location().Z())+joint_0.Location().Z())))*(1-math.cos(theta2))
                 + ((l2/l1+1)*(y-joint_0.Location().Y())+joint_0.Location().Y())*math.cos(theta2) + (z*u - x*w + w*((l2/l1+1)*(x-joint_0.Location().X())+joint_0.Location().X()) - u*((l2/l1+1)*(z-joint_0.Location().Z())+joint_0.Location().Z()))*math.sin(theta2) - ytgt)**2 +

                ((z*(u**2 + v**2) - w*(x*u+y*v-u*((l2/l1+1)*(x-joint_0.Location().X())+joint_0.Location().X()) - v*((l2/l1+1)*(y-joint_0.Location().Y())+joint_0.Location().Y()) - w*((l2/l1+1)*(z-joint_0.Location().Z())+joint_0.Location().Z())))*(1-math.cos(theta2))
                 + ((l2/l1+1)*(z-joint_0.Location().Z())+joint_0.Location().Z())*math.cos(theta2) + (-y*u + x*v - v*((l2/l1+1)*(x-joint_0.Location().X())+joint_0.Location().X()) +u*((l2/l1+1)*(y-joint_0.Location().Y())+joint_0.Location().Y()))*math.sin(theta2) - ztgt)**2

                - l3**2,
                
                (joint_0.Location().X() - x) * xn + (joint_0.Location().Y() - y) * yn + (
                joint_0.Location().Z() - z) * zn
            )

        x_jt1, y_jt1, z_jt1 = fsolve(equations, (
        self.joint_1_mem.Location().X(), self.joint_1_mem.Location().Y(), self.joint_1_mem.Location().Z()))

        a = x_jt1
        b = y_jt1
        c = z_jt1

        x_jt2 = (a*(v**2 + w**2)
                 - u*(b*v+c*w-u*((l2/l1+1)*(x_jt1-joint_0.Location().X())+joint_0.Location().X())
                      - v*((l2/l1+1)*(y_jt1-joint_0.Location().Y())+joint_0.Location().Y())
                      - w*((l2/l1+1)*(z_jt1-joint_0.Location().Z())+joint_0.Location().Z())))*(1-math.cos(theta2)) + ((l2/l1+1)*(x_jt1-joint_0.Location().X())+joint_0.Location().X())*math.cos(theta2) + (-c*v + b*w - w*((l2/l1+1)*(y_jt1-joint_0.Location().Y())+joint_0.Location().Y())+v*((l2/l1+1)*(z_jt1-joint_0.Location().Z())+joint_0.Location().Z()))*math.sin(theta2)
                 
        y_jt2 = (b*(u**2 + w**2)
                 - v*(a*u+c*w-u*((l2/l1+1)*(x_jt1-joint_0.Location().X())+joint_0.Location().X())
                      - v*((l2/l1+1)*(y_jt1-joint_0.Location().Y())+joint_0.Location().Y())
                      - w*((l2/l1+1)*(z_jt1-joint_0.Location().Z())+joint_0.Location().Z())))*(1-math.cos(theta2)) + ((l2/l1+1)*(y_jt1-joint_0.Location().Y())+joint_0.Location().Y())*math.cos(theta2) + (c*u - a*w + w*((l2/l1+1)*(x_jt1-joint_0.Location().X())+joint_0.Location().X()) - u*((l2/l1+1)*(z_jt1-joint_0.Location().Z())+joint_0.Location().Z()))*math.sin(theta2)

        
        z_jt2 = (c*(u**2 + v**2)
                 - w*(a*u+b*v-u*((l2/l1+1)*(x_jt1-joint_0.Location().X())+joint_0.Location().X())
                      - v*((l2/l1+1)*(y_jt1-joint_0.Location().Y())+joint_0.Location().Y())
                      - w*((l2/l1+1)*(z_jt1-joint_0.Location().Z())+joint_0.Location().Z())))*(1-math.cos(theta2)) + ((l2/l1+1)*(z_jt1-joint_0.Location().Z())+joint_0.Location().Z())*math.cos(theta2) + (-b*u + a*v - v*((l2/l1+1)*(x_jt1-joint_0.Location().X())+joint_0.Location().X()) +u*((l2/l1+1)*(y_jt1-joint_0.Location().Y())+joint_0.Location().Y()))*math.sin(theta2)


        print('xyz:', x_jt1, y_jt1, z_jt1)

        print('target point 0:', self.trgt_pt.Location().X(), self.trgt_pt.Location().Y(), self.trgt_pt.Location().Z())

        x3 = xtgt - x_jt2
        y3 = ytgt - y_jt2
        z3 = ztgt - z_jt2

        print((x3 ** 2 + y3 ** 2 + z3 ** 2) ** 0.5)

        x2 = x_jt2 - x_jt1
        y2 = y_jt2 - y_jt1
        z2 = z_jt2 - z_jt1

        print((x2 ** 2 + y2 ** 2 + z2 ** 2) ** 0.5)

        x1 = x_jt1 - joint_0.Location().X()
        y1 = y_jt1 - joint_0.Location().Y()
        z1 = z_jt1 - joint_0.Location().Z()

        x0 = ax1_z.Direction().X()
        y0 = ax1_z.Direction().Y()
        z0 = ax1_z.Direction().Z()

        print('x0', x0, y0, z0)

        x32 = y3 * z2 - z3 * y2
        y32 = z3 * x2 - x3 * z2
        z32 = x3 * y2 - y3 * x2

        x21 = y2 * z1 - z2 * y1
        y21 = z2 * x1 - x2 * z1
        z21 = x2 * y1 - y2 * x1

        x10 = y1 * z0 - z1 * y0
        y10 = z1 * x0 - x1 * z0
        z10 = x1 * y0 - y1 * x0

        cos_theta3 = (x3 * x2 + y3 * y2 + z3 * z2) / ((x3 ** 2 + y3 ** 2 + z3 ** 2) * (
        x2 ** 2 + y2 ** 2 + z2 ** 2)) ** 0.5
        theta3 = math.acos(cos_theta3)
        theta3 = math.copysign(theta3, -(x32 * xn + y32 * yn + z32 * zn))
        print(theta3 / math.pi * 180)

        cos_theta2 = (x2 * x1 + y2 * y1 + z2 * z1) / ((x2 ** 2 + y2 ** 2 + z2 ** 2) * (
        x1 ** 2 + y1 ** 2 + z1 ** 2)) ** 0.5
        theta2 = math.acos(cos_theta2)
        theta2 = math.copysign(theta2, -(x21 * xn + y21 * yn + z21 * zn))
        print(theta2 / math.pi * 180)

        cos_theta1 = (x1 * x0 + y1 * y0 + z1 * z0) / ((x1 ** 2 + y1 ** 2 + z1 ** 2) * (
        x0 ** 2 + y0 ** 2 + z0 ** 2)) ** 0.5
        theta1 = math.acos(cos_theta1)
        theta1 = math.copysign(theta1, -(x10 * xn + y10 * yn + z10 * zn))
        print(theta1 / math.pi * 180)

        servo1Angle = theta1 / pi * 180.0 + 96.1684978685

        servo3Angle = theta3 / pi * 180.0 + 94.5556396436

        flag_output = True

        if (servo1Angle > 180.0 or servo1Angle < 0.0 or servo3Angle > 180.0 or servo3Angle < 0.0):
            flag_output = False

        return (servo1Angle, servo3Angle, flag_output)

    

    def compute_rotations_3(self, ax1, ax1_z, joint_0, joint_2, theta3):
        
        l3 = 0.06209830859
        l2 = 0.05055720935
        l1 = 0.05940402141

        xn = ax1.Direction().X()
        yn = ax1.Direction().Y()
        zn = ax1.Direction().Z()

        xtgt = self.trgt_pt_mem.Location().X()
        ytgt = self.trgt_pt_mem.Location().Y()
        ztgt = self.trgt_pt_mem.Location().Z()

        print('xn', xn, yn, zn)

        
        u = self.joint_2_mem.Direction().X()
        v = self.joint_2_mem.Direction().Y()
        w = self.joint_2_mem.Direction().Z()

        def equations(p):
            x, y, z = p
            return (

                (x - xtgt) ** 2 + (y - ytgt) ** 2 + (z - ztgt) ** 2 - l3 ** 2,
                
                ((x*(v**2 + w**2) - u*(y*v+z*w-u*((l2/l3+1)*(x-xtgt)+xtgt) - v*((l2/l3+1)*(y-ytgt)+ytgt) - w*((l2/l3+1)*(z-ztgt)+ztgt)))*(1-math.cos(theta3))
                 + ((l2/l3+1)*(x-xtgt)+xtgt)*math.cos(theta3) + (-z*v + y*w - w*((l2/l3+1)*(y-ytgt)+ytgt) +v*((l2/l3+1)*(z-ztgt)+ztgt))*math.sin(theta3) - joint_0.Location().X())**2 +

                ((y*(u**2 + w**2) - v*(x*u+z*w-u*((l2/l3+1)*(x-xtgt)+xtgt) - v*((l2/l3+1)*(y-ytgt)+ytgt) - w*((l2/l3+1)*(z-ztgt)+ztgt)))*(1-math.cos(theta3))
                 + ((l2/l3+1)*(y-ytgt)+ytgt)*math.cos(theta3) + (z*u - x*w + w*((l2/l3+1)*(x-xtgt)+xtgt) - u*((l2/l3+1)*(z-ztgt)+ztgt))*math.sin(theta3) - joint_0.Location().Y())**2 +

                ((z*(u**2 + v**2) - w*(x*u+y*v-u*((l2/l3+1)*(x-xtgt)+xtgt) - v*((l2/l3+1)*(y-ytgt)+ytgt) - w*((l2/l3+1)*(z-ztgt)+ztgt)))*(1-math.cos(theta3))
                 + ((l2/l3+1)*(z-ztgt)+ztgt)*math.cos(theta3) + (-y*u + x*v - v*((l2/l3+1)*(x-xtgt)+xtgt) +u*((l2/l3+1)*(y-ytgt)+ytgt))*math.sin(theta3) - joint_0.Location().Z())**2

                - l1**2,
                
                (xtgt - x) * xn + (ytgt - y) * yn + (ztgt - z) * zn
            )

        x_jt2, y_jt2, z_jt2 = fsolve(equations, (
        self.joint_2_mem.Location().X(), self.joint_2_mem.Location().Y(), self.joint_2_mem.Location().Z()))

       
        a = x_jt2
        b = y_jt2
        c = z_jt2


        x_jt1 = (a*(v**2 + w**2)
                 - u*(b*v+c*w-u*((l2/l1+1)*(x_jt2-xtgt)+xtgt)
                      - v*((l2/l1+1)*(y_jt2-ytgt)+ytgt)
                      - w*((l2/l1+1)*(z_jt2-ztgt)+ztgt)))*(1-math.cos(theta3)) + ((l2/l1+1)*(x_jt2-xtgt)+xtgt)*math.cos(theta3) + (-c*v + b*w - w*((l2/l1+1)*(y_jt2-ytgt)+ytgt)+v*((l2/l1+1)*(z_jt2-ztgt)+ztgt))*math.sin(theta3)
                 
        y_jt1 = (b*(u**2 + w**2)
                 - v*(a*u+c*w-u*((l2/l1+1)*(x_jt2-xtgt)+xtgt)
                      - v*((l2/l1+1)*(y_jt2-ytgt)+ytgt)
                      - w*((l2/l1+1)*(z_jt2-ztgt)+ztgt)))*(1-math.cos(theta3)) + ((l2/l1+1)*(y_jt2-ytgt)+ytgt)*math.cos(theta3) + (c*u - a*w + w*((l2/l1+1)*(x_jt2-xtgt)+xtgt) - u*((l2/l1+1)*(z_jt2-ztgt)+ztgt))*math.sin(theta3)

        
        z_jt1 = (c*(u**2 + v**2)
                 - w*(a*u+b*v-u*((l2/l1+1)*(x_jt2-xtgt)+xtgt)
                      - v*((l2/l1+1)*(y_jt2-ytgt)+ytgt)
                      - w*((l2/l1+1)*(z_jt2-ztgt)+ztgt)))*(1-math.cos(theta3)) + ((l2/l1+1)*(z_jt2-ztgt)+ztgt)*math.cos(theta3) + (-b*u + a*v - v*((l2/l1+1)*(x_jt2-xtgt)+xtgt) +u*((l2/l1+1)*(y_jt2-ytgt)+ytgt))*math.sin(theta3)


        



        print('xyz:', x_jt1, y_jt1, z_jt1)

        print('target point 0:', self.trgt_pt.Location().X(), self.trgt_pt.Location().Y(), self.trgt_pt.Location().Z())

        x3 = xtgt - x_jt2
        y3 = ytgt - y_jt2
        z3 = ztgt - z_jt2

        print((x3 ** 2 + y3 ** 2 + z3 ** 2) ** 0.5)

        x2 = x_jt2 - x_jt1
        y2 = y_jt2 - y_jt1
        z2 = z_jt2 - z_jt1

        print((x2 ** 2 + y2 ** 2 + z2 ** 2) ** 0.5)

        x1 = x_jt1 - joint_0.Location().X()
        y1 = y_jt1 - joint_0.Location().Y()
        z1 = z_jt1 - joint_0.Location().Z()

        x0 = ax1_z.Direction().X()
        y0 = ax1_z.Direction().Y()
        z0 = ax1_z.Direction().Z()

        print('x0', x0, y0, z0)

        x32 = y3 * z2 - z3 * y2
        y32 = z3 * x2 - x3 * z2
        z32 = x3 * y2 - y3 * x2

        x21 = y2 * z1 - z2 * y1
        y21 = z2 * x1 - x2 * z1
        z21 = x2 * y1 - y2 * x1

        x10 = y1 * z0 - z1 * y0
        y10 = z1 * x0 - x1 * z0
        z10 = x1 * y0 - y1 * x0

        cos_theta3 = (x3 * x2 + y3 * y2 + z3 * z2) / ((x3 ** 2 + y3 ** 2 + z3 ** 2) * (
        x2 ** 2 + y2 ** 2 + z2 ** 2)) ** 0.5
        theta3 = math.acos(cos_theta3)
        theta3 = math.copysign(theta3, -(x32 * xn + y32 * yn + z32 * zn))
        print(theta3 / math.pi * 180)

        cos_theta2 = (x2 * x1 + y2 * y1 + z2 * z1) / ((x2 ** 2 + y2 ** 2 + z2 ** 2) * (
        x1 ** 2 + y1 ** 2 + z1 ** 2)) ** 0.5
        theta2 = math.acos(cos_theta2)
        theta2 = math.copysign(theta2, -(x21 * xn + y21 * yn + z21 * zn))
        print(theta2 / math.pi * 180)

        cos_theta1 = (x1 * x0 + y1 * y0 + z1 * z0) / ((x1 ** 2 + y1 ** 2 + z1 ** 2) * (
        x0 ** 2 + y0 ** 2 + z0 ** 2)) ** 0.5
        theta1 = math.acos(cos_theta1)
        theta1 = math.copysign(theta1, -(x10 * xn + y10 * yn + z10 * zn))
        print(theta1 / math.pi * 180)

        servo1Angle = theta1 / pi * 180.0 + 96.1684978685

        servo2Angle = -theta2 / pi * 180.0 + 98.1058681505

        flag_output = True

        if (servo1Angle > 180.0 or servo1Angle < 0.0 or servo2Angle > 180.0 or servo2Angle < 0.0):
            flag_output = False

        return (servo1Angle, servo2Angle, flag_output)
    
    def compute_angles(self):
        ## compute angles
        x3 = trgt_pt.Location().X()-joint_2.Location().X()
        y3 = trgt_pt.Location().Y()-joint_2.Location().Y()
        z3 = trgt_pt.Location().Z()-joint_2.Location().Z()

        x2 = joint_2.Location().X()-joint_1.Location().X()
        y2 = joint_2.Location().Y()-joint_1.Location().Y()
        z2 = joint_2.Location().Z()-joint_1.Location().Z()

        x1 = joint_1.Location().X()-joint_0.Location().X()
        y1 = joint_1.Location().Y()-joint_0.Location().Y()
        z1 = joint_1.Location().Z()-joint_0.Location().Z()

        x0 =0.0001043375278241644
        y0 =0.0003787538898580119
        z0 =0.9999999228295826

        xn = self.ax1.Direction().X()
        yn = self.ax1.Direction().Y()
        zn = self.ax1.Direction().Z()

        x32 = y3*z2-z3*y2
        y32 = z3*x2-x3*z2
        z32 = x3*y2-y3*x2

        x21 = y2*z1-z2*y1
        y21 = z2*x1-x2*z1
        z21 = x2*y1-y2*x1

        x10 = y1*z0-z1*y0
        y10 = z1*x0-x1*z0
        z10 = x1*y0-y1*x0

        cos_theta3 = (x3*x2+y3*y2+z3*z2)/((x3**2+y3**2+z3**2)*(x2**2+y2**2+z2**2))**0.5
        theta3 = math.acos(cos_theta3)
        theta3 = math.copysign(theta3, -(x32*xn+y32*yn+z32*zn))
        print(theta3/math.pi*180)


        cos_theta2 = (x2*x1+y2*y1+z2*z1)/((x2**2+y2**2+z2**2)*(x1**2+y1**2+z1**2))**0.5
        theta2 = math.acos(cos_theta2)
        theta2 = math.copysign(theta2, -(x21*xn+y21*yn+z21*zn))
        print(theta2/math.pi*180)

        cos_theta1 = (x1*x0+y1*y0+z1*z0)/((x1**2+y1**2+z1**2)*(x0**2+y0**2+z0**2))**0.5
        theta1 = math.acos(cos_theta1)
        theta1 = math.copysign(theta1, -(x10*xn+y10*yn+z10*zn))
        print(theta1/math.pi*180)

        l3 = 0.062098309
        l2 = 0.050557209
        l1 = 0.059404022



    def output_angles(self):
        if(self.isConnected):
            data = (str(self.horizontalSlider_Servo0.value())+","+str(self.horizontalSlider_Servo1.value())+","+str(self.horizontalSlider_Servo2.value())+","
                    +str(self.horizontalSlider_Servo3.value())+","+str(self.horizontalSlider_Servo4.value())+","+str(self.horizontalSlider_Servo5.value())+";")

            self.serialConnector.write(data)
            print('data:'+data)


##  end of class Ui_MainWindow

     

        
try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context,  text,  disambig):
        return QtGui.QApplication.translate(context,  text,  disambig,  _encoding)
except AttributeError:
    def _translate(context,  text,  disambig):
        return QtGui.QApplication.translate(context,  text,  disambig)


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    ui.init_window()
    MainWindow.show()
    ui.rotate_servos(-1)
    sys.exit(app.exec_())

