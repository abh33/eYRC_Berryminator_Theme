
from PyQt5 import QtCore, QtGui, QtWidgets
import sys, os
import os
import signal
import sys
import platform
import csv
import json
import cv2
import numpy as np
import string
import random
import base64
from datetime import datetime
from itertools import islice
import cryptocode
import uuid
import traceback
import math
import time
import re
from pyzbar.pyzbar import decode

team_id_val = None
folder_path = None
task = None
pid = None

# QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)


if hasattr(sys, "frozen"):
	# print("executable", sys.executable)
	sys.path.append(os.path.dirname(sys.executable))

# NOTE: Refer https://stackoverflow.com/questions/39885354/pyinstaller-cannot-add-txt-files
def resource_path(relative_path):
	""" Get absolute path to resource, works for dev and for PyInstaller """
	try:
		# PyInstaller creates a temp folder and stores path in _MEIPASS
		base_path = sys._MEIPASS
	except Exception:
		base_path = os.environ.get("_MEIPASS2",os.path.abspath("."))

	return os.path.join(base_path, relative_path)

class EmittingStream(QtCore.QObject):

    textWritten = QtCore.pyqtSignal(str)

    def write(self, text):
        self.textWritten.emit(str(text))


class Ui_Berryminator_Evaluator(object):

    def openwindow(self):
        self.window = QtWidgets.QMainWindow()
        self.ui = Ui_Dialog()
        self.ui.setupUi(self.window)
        Berryminator_Evaluator.hide()
        self.window.show()
        # self.start_process()
        # self.process = QtCore.QProcess()
        # self.process.start("python", ['task_1a_cardinal.py', str(team_id_val), folder_path, task])

    # def start_process(self):
    #     global pid
    #     self.process = QtCore.QProcess()
    #     self.process.readyReadStandardOutput.connect(self.handle_stdout)
    #     self.process.readyReadStandardError.connect(self.handle_stderr)
    #     self.process.stateChanged.connect(self.handle_state)

    #     # os.kill(pid, signal.CTRL_C_EVENT)
    #     self.process.finished.connect(self.process_finished)  # Clean up once complete.

    #     print("Task 4 Task 4 Task 4")
    #     # print(folder_path)
    #     command = 'cmd.exe /C task_4_cardinal.exe ' + str(team_id_val) + ' "' + folder_path + '"'
    #     print(command)
    #     self.process.start(command)
    #     pid = self.process.processId()
    #     print("process pid" + str(pid))

    # def handle_stderr(self):
    #     data = self.process.readAllStandardError()
    #     stderr = bytes(data).decode("utf8")
    #     # self.message(stderr)
    #     print(stderr)

    # def handle_stdout(self):
    #     data = self.process.readAllStandardOutput()
    #     stdout = bytes(data).decode("utf8")
    #     # self.message(stdout)
    #     print(stdout)

    # def handle_state(self, state):
    #     states = {
    #         QtCore.QProcess.NotRunning: 'Not running',
    #         QtCore.QProcess.Starting: 'Starting',
    #         QtCore.QProcess.Running: 'Running',
    #     }
    #     state_name = states[state]
    #     # print(f"State changed: {state_name}")
    #     # self.message(f"State changed: {state_name}")

    # def process_finished(self):
    #     print("Process finished.")
    #     self.process = None


    def setupUi(self, Berryminator_Evaluator):
        Berryminator_Evaluator.setObjectName("Berryminator_Evaluator")
        Berryminator_Evaluator.resize(600, 500)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Berryminator_Evaluator.sizePolicy().hasHeightForWidth())
        Berryminator_Evaluator.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("Calibri")
        Berryminator_Evaluator.setFont(font)
        Berryminator_Evaluator.setAutoFillBackground(False)
        Berryminator_Evaluator.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.centralwidget = QtWidgets.QWidget(Berryminator_Evaluator)
        self.centralwidget.setObjectName("centralwidget")
        self.header_1_logo = QtWidgets.QLabel(self.centralwidget)
        self.header_1_logo.setGeometry(QtCore.QRect(10, 10, 245, 50))
        self.header_1_logo.setStyleSheet("")
        self.header_1_logo.setText("")
        self.header_1_logo.setPixmap(QtGui.QPixmap(resource_path("logo_eyantra.png")))
        self.header_1_logo.setScaledContents(True)
        self.header_1_logo.setObjectName("header_1_logo")
        self.header_2_comp = QtWidgets.QLabel(self.centralwidget)
        self.header_2_comp.setGeometry(QtCore.QRect(350, 10, 240, 50))
        # self.header_2_comp.setFrameShape(QtWidgets.QFrame.Box)
        self.header_2_comp.setText("")
        self.header_2_comp.setPixmap(QtGui.QPixmap(resource_path("robotics_comp.png")))
        self.header_2_comp.setScaledContents(True)
        self.header_2_comp.setObjectName("header_2_comp")
        # font = QtGui.QFont()
        # font.setFamily("HighlandGothicFLF")
        # font.setPixelSize(16)
        # font.setBold(True)
        # font.setWeight(75)
        # self.header_2_comp.setFont(font)
        # self.header_2_comp.setScaledContents(False)
        # self.header_2_comp.setAlignment(QtCore.Qt.AlignCenter)
        # self.header_2_comp.setWordWrap(True)
        # self.header_2_comp.setIndent(0)
        # self.header_2_comp.setObjectName("header_2_comp")
        self.header_3_theme_img = QtWidgets.QLabel(self.centralwidget)
        self.header_3_theme_img.setGeometry(QtCore.QRect(100, 80, 400, 190))
        self.header_3_theme_img.setFrameShape(QtWidgets.QFrame.Box)
        self.header_3_theme_img.setText("")
        self.header_3_theme_img.setPixmap(QtGui.QPixmap(resource_path("berryminator.png")))
        self.header_3_theme_img.setScaledContents(True)
        self.header_3_theme_img.setObjectName("header_3_theme_img")
        self.header_4_theme_name = QtWidgets.QLabel(self.centralwidget)
        self.header_4_theme_name.setGeometry(QtCore.QRect(150, 270, 291, 41))
        self.header_4_theme_name.setText("")
        self.header_4_theme_name.setPixmap(QtGui.QPixmap(resource_path("berryminator_theme.png")))
        self.header_4_theme_name.setScaledContents(True)
        self.header_4_theme_name.setObjectName("header_4_theme_name")
        # font = QtGui.QFont()
        # font.setFamily("HighlandGothicFLF")
        # font.setPixelSize(20)
        # font.setBold(True)
        # font.setWeight(75)
        # self.header_4_theme_name.setFont(font)
        # self.header_4_theme_name.setStyleSheet("color: rgb(255, 0, 0);")
        # self.header_4_theme_name.setScaledContents(False)
        # self.header_4_theme_name.setAlignment(QtCore.Qt.AlignCenter)
        # self.header_4_theme_name.setWordWrap(True)
        # self.header_4_theme_name.setIndent(0)
        # self.header_4_theme_name.setObjectName("header_4_theme_name")
        self.folder_display = QtWidgets.QLabel(self.centralwidget)
        self.folder_display.setGeometry(QtCore.QRect(180, 400, 390, 30))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.folder_display.setFont(font)
        self.folder_display.setFrameShape(QtWidgets.QFrame.Box)
        self.folder_display.setText("")
        self.folder_display.setObjectName("folder_display")
        self.select_folder = QtWidgets.QPushButton(self.centralwidget)
        self.select_folder.setGeometry(QtCore.QRect(20, 400, 140, 30))
        font = QtGui.QFont()
        font.setFamily("HighlandGothicFLF")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.select_folder.setFont(font)
        self.select_folder.setStyleSheet("background-color: rgb(255, 255, 0);\n"
"color: rgb(0, 0, 0);")
        self.select_folder.setObjectName("select_folder")
        self.team_id = QtWidgets.QLineEdit(self.centralwidget)
        self.team_id.setGeometry(QtCore.QRect(285, 360, 150, 30))
        font.setFamily("Calibri")
        font.setPixelSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.team_id.setFont(font)
        self.team_id.setObjectName("team_id")
        self.Enter_team_id = QtWidgets.QLabel(self.centralwidget)
        self.Enter_team_id.setGeometry(QtCore.QRect(165, 360, 120, 30))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(19)
        font.setBold(True)
        font.setWeight(75)
        self.Enter_team_id.setFont(font)
        self.Enter_team_id.setObjectName("Enter_team_id")
        self.start_evaluation = QtWidgets.QPushButton(self.centralwidget)
        self.start_evaluation.setGeometry(QtCore.QRect(220, 440, 160, 40))
        font = QtGui.QFont()
        font.setFamily("HighlandGothicFLF")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.start_evaluation.setFont(font)
        self.start_evaluation.setStyleSheet("background-color: rgb(255, 0, 0);\n"
"color: rgb(255, 255, 255);")
        self.start_evaluation.setObjectName("start_evaluation")
        self.header_4_theme_name_2 = QtWidgets.QLabel(self.centralwidget)
        self.header_4_theme_name_2.setGeometry(QtCore.QRect(150, 310, 291, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(30)
        font.setBold(True)
        font.setWeight(75)
        self.header_4_theme_name_2.setFont(font)
        self.header_4_theme_name_2.setStyleSheet("color: rgb(0, 0, 0);")
        self.header_4_theme_name_2.setScaledContents(False)
        self.header_4_theme_name_2.setAlignment(QtCore.Qt.AlignCenter)
        self.header_4_theme_name_2.setWordWrap(True)
        self.header_4_theme_name_2.setIndent(0)
        self.header_4_theme_name_2.setObjectName("header_4_theme_name_2")
        Berryminator_Evaluator.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(Berryminator_Evaluator)
        self.statusbar.setObjectName("statusbar")
        Berryminator_Evaluator.setStatusBar(self.statusbar)

        self.retranslateUi(Berryminator_Evaluator)
        QtCore.QMetaObject.connectSlotsByName(Berryminator_Evaluator)
        self.select_folder.clicked.connect(self.pick_new)
        self.start_evaluation.clicked.connect(self.start_eval)

    def retranslateUi(self, Berryminator_Evaluator):
        _translate = QtCore.QCoreApplication.translate
        Berryminator_Evaluator.setWindowTitle(_translate("Berryminator_Evaluator", "BM_GUI"))
        # self.header_2_comp.setText(_translate("Berryminator_Evaluator", "Robotics Competition 2021-22"))
        # self.header_4_theme_name.setText(_translate("Berryminator_Evaluator", "Berryminator Theme"))
        # self.folder_display.setText(_translate("Berryminator_Evaluator", "Select Folder"))
        self.select_folder.setText(_translate("Berryminator_Evaluator", "Select Folder"))
        self.Enter_team_id.setText(_translate("Berryminator_Evaluator", "Enter Team Id:"))
        self.start_evaluation.setText(_translate("Berryminator_Evaluator", "Start Evaluation"))
        self.start_evaluation.setShortcut(_translate("Berryminator_Evaluator", "Return"))
        self.header_4_theme_name_2.setText(_translate("Berryminator_Evaluator", "Task 5"))

    def pick_new(self):
                dialog = QtWidgets.QFileDialog()
                folder_path = dialog.getExistingDirectory(None, "Select Folder")
                self.folder_display.setText(folder_path)

    def start_eval(self):
                global team_id_val, folder_path
                value = self.team_id.text()
                folder_path = self.folder_display.text()

                team_id_val = None
                folder_path_valid_flag = None
                team_id_valid_flag = None
                if folder_path == '':
                    folder_path_valid_flag = False
                else:
                    folder_path_valid_flag = True

                try:
                    team_id_val = int(value)
                    team_id_valid_flag = True
                except:
                    team_id_valid_flag = False

                msg_box = QtWidgets.QMessageBox()
                msg_box.setWindowTitle("Error!!")
                msg_box.setIcon(QtWidgets.QMessageBox.Critical)

                try:
                
                    if team_id_valid_flag == False and folder_path_valid_flag == False:
                            msg_box.setText('Valid Team ID and Folder Path not selected!')
                            x = msg_box.exec()
                            # sys.exit()
                    elif team_id_valid_flag == True and folder_path_valid_flag == False:
                            msg_box.setText('Valid Folder Path not selected!')
                            x = msg_box.exec()
                            # sys.exit()
                    elif team_id_valid_flag == False and folder_path_valid_flag == True:
                            msg_box.setText('Valid Team ID not selected!')
                            x = msg_box.exec()
                            # sys.exit()
                    elif team_id_valid_flag == True and folder_path_valid_flag == True:
                            self.openwindow()
                    else:
                            pass             

                    # self.openwindow()
                except:
                    # print("hello")
                    pass

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(770, 540)


        # sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        # sizePolicy.setHorizontalStretch(0)
        # sizePolicy.setVerticalStretch(0)
        # sizePolicy.setHeightForWidth(Dialog.sizePolicy().hasHeightForWidth())
        # Dialog.setSizePolicy(sizePolicy)
        # Dialog.setMinimumSize(QtCore.QSize(770, 540))
        # Dialog.setMaximumSize(QtCore.QSize(770, 540))

        Dialog.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.header1 = QtWidgets.QLabel(Dialog)
        self.header1.setGeometry(QtCore.QRect(10, 10, 245, 50))
        self.header1.setStyleSheet("")
        self.header1.setText("")
        self.header1.setPixmap(QtGui.QPixmap(resource_path("logo_eyantra.png")))
        self.header1.setScaledContents(True)
        self.header1.setObjectName("header1")
        self.header3 = QtWidgets.QLabel(Dialog)
        self.header3.setGeometry(QtCore.QRect(520, 10, 240, 50))
        # self.header_2_comp.setFrameShape(QtWidgets.QFrame.Box)
        self.header3.setText("")
        self.header3.setPixmap(QtGui.QPixmap(resource_path("robotics_comp.png")))
        self.header3.setScaledContents(True)
        self.header3.setObjectName("header3")
        # self.header3 = QtWidgets.QLabel(Dialog)
        # self.header3.setGeometry(QtCore.QRect(520, 10, 240, 50))
        # font = QtGui.QFont()
        # font.setFamily("HighlandGothicFLF")
        # font.setPixelSize(16)
        # font.setBold(True)
        # font.setWeight(75)
        # self.header3.setFont(font)
        # self.header3.setScaledContents(False)
        # self.header3.setAlignment(QtCore.Qt.AlignCenter)
        # self.header3.setWordWrap(True)
        # self.header3.setIndent(0)
        # self.header3.setObjectName("header3")
        self.console_output = QtWidgets.QTextEdit(Dialog)
        self.console_output.setGeometry(QtCore.QRect(330, 240, 420, 230))
        self.console_output.setObjectName("console_output")
        self.label_output_console = QtWidgets.QLabel(Dialog)
        self.label_output_console.setGeometry(QtCore.QRect(330, 200, 151, 30))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_output_console.setFont(font)
        self.label_output_console.setObjectName("label_output_console")
        self.end_process = QtWidgets.QPushButton(Dialog)
        self.end_process.setGeometry(QtCore.QRect(530, 490, 130, 30))
        font = QtGui.QFont()
        font.setFamily("HighlandGothicFLF")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.end_process.setFont(font)
        self.end_process.setStyleSheet("background-color: rgb(255, 255, 0);\n"
"color: rgb(0, 0, 0);")
        self.end_process.setObjectName("end_process")
        self.exit = QtWidgets.QPushButton(Dialog)
        self.exit.setGeometry(QtCore.QRect(680, 490, 70, 30))
        font = QtGui.QFont()
        font.setFamily("HighlandGothicFLF")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.exit.setFont(font)
        self.exit.setStyleSheet("background-color: rgb(255, 255, 0);\n"
"color: rgb(0, 0, 0);")
        self.exit.setObjectName("exit")
        self.label_theme_config = QtWidgets.QLabel(Dialog)
        self.label_theme_config.setGeometry(QtCore.QRect(20, 80, 261, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_theme_config.setFont(font)
        self.label_theme_config.setFrameShape(QtWidgets.QFrame.Box)
        self.label_theme_config.setAlignment(QtCore.Qt.AlignCenter)
        self.label_theme_config.setObjectName("label_theme_config")
        self.label_blueberry = QtWidgets.QLabel(Dialog)
        self.label_blueberry.setGeometry(QtCore.QRect(20, 110, 101, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_blueberry.setFont(font)
        self.label_blueberry.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_blueberry.setStyleSheet("")
        self.label_blueberry.setFrameShape(QtWidgets.QFrame.Box)
        self.label_blueberry.setAlignment(QtCore.Qt.AlignCenter)
        self.label_blueberry.setObjectName("label_blueberry")
        self.label_strawberry = QtWidgets.QLabel(Dialog)
        self.label_strawberry.setGeometry(QtCore.QRect(20, 150, 101, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_strawberry.setFont(font)
        self.label_strawberry.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_strawberry.setStyleSheet("")
        self.label_strawberry.setFrameShape(QtWidgets.QFrame.Box)
        self.label_strawberry.setAlignment(QtCore.Qt.AlignCenter)
        self.label_strawberry.setObjectName("label_strawberry")
        self.label_lemon = QtWidgets.QLabel(Dialog)
        self.label_lemon.setGeometry(QtCore.QRect(20, 190, 101, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_lemon.setFont(font)
        self.label_lemon.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_lemon.setStyleSheet("")
        self.label_lemon.setFrameShape(QtWidgets.QFrame.Box)
        self.label_lemon.setAlignment(QtCore.Qt.AlignCenter)
        self.label_lemon.setObjectName("label_lemon")
        self.blueberry1 = QtWidgets.QLabel(Dialog)
        self.blueberry1.setGeometry(QtCore.QRect(120, 110, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.blueberry1.setFont(font)
        self.blueberry1.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.blueberry1.setStyleSheet("")
        self.blueberry1.setFrameShape(QtWidgets.QFrame.Box)
        self.blueberry1.setText("")
        # self.blueberry1.setPixmap(QtGui.QPixmap("blueberry.png"))
        self.blueberry1.setScaledContents(True)
        self.blueberry1.setAlignment(QtCore.Qt.AlignCenter)
        self.blueberry1.setObjectName("blueberry1")
        self.blueberry2 = QtWidgets.QLabel(Dialog)
        self.blueberry2.setGeometry(QtCore.QRect(160, 110, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.blueberry2.setFont(font)
        self.blueberry2.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.blueberry2.setStyleSheet("")
        self.blueberry2.setFrameShape(QtWidgets.QFrame.Box)
        self.blueberry2.setText("")
        # self.blueberry2.setPixmap(QtGui.QPixmap("blueberry.png"))
        self.blueberry2.setScaledContents(True)
        self.blueberry2.setAlignment(QtCore.Qt.AlignCenter)
        self.blueberry2.setObjectName("blueberry2")
        self.blueberry3 = QtWidgets.QLabel(Dialog)
        self.blueberry3.setGeometry(QtCore.QRect(200, 110, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.blueberry3.setFont(font)
        self.blueberry3.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.blueberry3.setStyleSheet("")
        self.blueberry3.setFrameShape(QtWidgets.QFrame.Box)
        self.blueberry3.setText("")
        # self.blueberry3.setPixmap(QtGui.QPixmap("blueberry.png"))
        self.blueberry3.setScaledContents(True)
        self.blueberry3.setAlignment(QtCore.Qt.AlignCenter)
        self.blueberry3.setObjectName("blueberry3")
        self.strawberry1 = QtWidgets.QLabel(Dialog)
        self.strawberry1.setGeometry(QtCore.QRect(120, 150, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.strawberry1.setFont(font)
        self.strawberry1.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.strawberry1.setStyleSheet("")
        self.strawberry1.setFrameShape(QtWidgets.QFrame.Box)
        self.strawberry1.setText("")
        # self.strawberry1.setPixmap(QtGui.QPixmap("strawberry.png"))
        self.strawberry1.setScaledContents(True)
        self.strawberry1.setAlignment(QtCore.Qt.AlignCenter)
        self.strawberry1.setObjectName("strawberry1")
        self.lemon1 = QtWidgets.QLabel(Dialog)
        self.lemon1.setGeometry(QtCore.QRect(120, 190, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.lemon1.setFont(font)
        self.lemon1.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.lemon1.setStyleSheet("")
        self.lemon1.setFrameShape(QtWidgets.QFrame.Box)
        self.lemon1.setText("")
        # self.lemon1.setPixmap(QtGui.QPixmap("lemon.png"))
        self.lemon1.setScaledContents(True)
        self.lemon1.setAlignment(QtCore.Qt.AlignCenter)
        self.lemon1.setObjectName("lemon1")
        self.strawberry2 = QtWidgets.QLabel(Dialog)
        self.strawberry2.setGeometry(QtCore.QRect(160, 150, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.strawberry2.setFont(font)
        self.strawberry2.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.strawberry2.setStyleSheet("")
        self.strawberry2.setFrameShape(QtWidgets.QFrame.Box)
        self.strawberry2.setText("")
        # self.strawberry2.setPixmap(QtGui.QPixmap("strawberry.png"))
        self.strawberry2.setScaledContents(True)
        self.strawberry2.setAlignment(QtCore.Qt.AlignCenter)
        self.strawberry2.setObjectName("strawberry2")
        self.strawberry3 = QtWidgets.QLabel(Dialog)
        self.strawberry3.setGeometry(QtCore.QRect(200, 150, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.strawberry3.setFont(font)
        self.strawberry3.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.strawberry3.setStyleSheet("")
        self.strawberry3.setFrameShape(QtWidgets.QFrame.Box)
        self.strawberry3.setText("")
        # self.strawberry3.setPixmap(QtGui.QPixmap("strawberry.png"))
        self.strawberry3.setScaledContents(True)
        self.strawberry3.setAlignment(QtCore.Qt.AlignCenter)
        self.strawberry3.setObjectName("strawberry3")
        self.lemon2 = QtWidgets.QLabel(Dialog)
        self.lemon2.setGeometry(QtCore.QRect(160, 190, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.lemon2.setFont(font)
        self.lemon2.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.lemon2.setStyleSheet("")
        self.lemon2.setFrameShape(QtWidgets.QFrame.Box)
        self.lemon2.setText("")
        # self.lemon2.setPixmap(QtGui.QPixmap("lemon.png"))
        self.lemon2.setScaledContents(True)
        self.lemon2.setAlignment(QtCore.Qt.AlignCenter)
        self.lemon2.setObjectName("lemon2")
        self.lemon3 = QtWidgets.QLabel(Dialog)
        self.lemon3.setGeometry(QtCore.QRect(200, 190, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.lemon3.setFont(font)
        self.lemon3.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.lemon3.setStyleSheet("")
        self.lemon3.setFrameShape(QtWidgets.QFrame.Box)
        self.lemon3.setText("")
        # self.lemon3.setPixmap(QtGui.QPixmap("lemon.png"))
        self.lemon3.setScaledContents(True)
        self.lemon3.setAlignment(QtCore.Qt.AlignCenter)
        self.lemon3.setObjectName("lemon3")
        self.blueberry_box = QtWidgets.QLabel(Dialog)
        self.blueberry_box.setGeometry(QtCore.QRect(240, 110, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.blueberry_box.setFont(font)
        self.blueberry_box.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.blueberry_box.setStyleSheet("")
        self.blueberry_box.setFrameShape(QtWidgets.QFrame.Box)
        self.blueberry_box.setAlignment(QtCore.Qt.AlignCenter)
        self.blueberry_box.setObjectName("blueberry_box")
        self.strawberry_box = QtWidgets.QLabel(Dialog)
        self.strawberry_box.setGeometry(QtCore.QRect(240, 150, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.strawberry_box.setFont(font)
        self.strawberry_box.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.strawberry_box.setStyleSheet("")
        self.strawberry_box.setFrameShape(QtWidgets.QFrame.Box)
        self.strawberry_box.setAlignment(QtCore.Qt.AlignCenter)
        self.strawberry_box.setObjectName("strawberry_box")
        self.lemon_box = QtWidgets.QLabel(Dialog)
        self.lemon_box.setGeometry(QtCore.QRect(240, 190, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.lemon_box.setFont(font)
        self.lemon_box.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.lemon_box.setStyleSheet("")
        self.lemon_box.setFrameShape(QtWidgets.QFrame.Box)
        self.lemon_box.setAlignment(QtCore.Qt.AlignCenter)
        self.lemon_box.setObjectName("lemon_box")
        self.label_ci = QtWidgets.QLabel(Dialog)
        self.label_ci.setGeometry(QtCore.QRect(20, 320, 211, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_ci.setFont(font)
        self.label_ci.setFrameShape(QtWidgets.QFrame.Box)
        self.label_ci.setAlignment(QtCore.Qt.AlignCenter)
        self.label_ci.setObjectName("label_ci")
        self.label_cp = QtWidgets.QLabel(Dialog)
        self.label_cp.setGeometry(QtCore.QRect(20, 350, 211, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_cp.setFont(font)
        self.label_cp.setFrameShape(QtWidgets.QFrame.Box)
        self.label_cp.setAlignment(QtCore.Qt.AlignCenter)
        self.label_cp.setObjectName("label_cp")
        self.label_cd = QtWidgets.QLabel(Dialog)
        self.label_cd.setGeometry(QtCore.QRect(20, 380, 211, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_cd.setFont(font)
        self.label_cd.setFrameShape(QtWidgets.QFrame.Box)
        self.label_cd.setAlignment(QtCore.Qt.AlignCenter)
        self.label_cd.setObjectName("label_cd")
        self.label_time = QtWidgets.QLabel(Dialog)
        self.label_time.setGeometry(QtCore.QRect(20, 290, 211, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_time.setFont(font)
        self.label_time.setFrameShape(QtWidgets.QFrame.Box)
        self.label_time.setAlignment(QtCore.Qt.AlignCenter)
        self.label_time.setObjectName("label_time")
        self.label_penalty = QtWidgets.QLabel(Dialog)
        self.label_penalty.setGeometry(QtCore.QRect(20, 410, 211, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_penalty.setFont(font)
        self.label_penalty.setFrameShape(QtWidgets.QFrame.Box)
        self.label_penalty.setAlignment(QtCore.Qt.AlignCenter)
        self.label_penalty.setObjectName("label_penalty")
        self.label_bonus = QtWidgets.QLabel(Dialog)
        self.label_bonus.setGeometry(QtCore.QRect(20, 440, 211, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_bonus.setFont(font)
        self.label_bonus.setFrameShape(QtWidgets.QFrame.Box)
        self.label_bonus.setAlignment(QtCore.Qt.AlignCenter)
        self.label_bonus.setObjectName("label_bonus")

        self.label_valid_run = QtWidgets.QLabel(Dialog)
        self.label_valid_run.setGeometry(QtCore.QRect(20, 260, 211, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_valid_run.setFont(font)
        self.label_valid_run.setFrameShape(QtWidgets.QFrame.Box)
        self.label_valid_run.setAlignment(QtCore.Qt.AlignCenter)
        self.label_valid_run.setObjectName("label_valid_run")

        self.valid_run = QtWidgets.QLabel(Dialog)
        self.valid_run.setGeometry(QtCore.QRect(230, 260, 65, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.valid_run.setFont(font)
        self.valid_run.setFrameShape(QtWidgets.QFrame.Box)
        self.valid_run.setAlignment(QtCore.Qt.AlignCenter)
        self.valid_run.setObjectName("valid_run")

        self.num_seconds = QtWidgets.QLabel(Dialog)
        self.num_seconds.setGeometry(QtCore.QRect(230, 290, 65, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.num_seconds.setFont(font)
        self.num_seconds.setFrameShape(QtWidgets.QFrame.Box)
        self.num_seconds.setAlignment(QtCore.Qt.AlignCenter)
        self.num_seconds.setObjectName("num_seconds")
        self.num_ci = QtWidgets.QLabel(Dialog)
        self.num_ci.setGeometry(QtCore.QRect(230, 320, 65, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.num_ci.setFont(font)
        self.num_ci.setFrameShape(QtWidgets.QFrame.Box)
        self.num_ci.setAlignment(QtCore.Qt.AlignCenter)
        self.num_ci.setObjectName("num_ci")
        self.num_cp = QtWidgets.QLabel(Dialog)
        self.num_cp.setGeometry(QtCore.QRect(230, 350, 65, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.num_cp.setFont(font)
        self.num_cp.setFrameShape(QtWidgets.QFrame.Box)
        self.num_cp.setAlignment(QtCore.Qt.AlignCenter)
        self.num_cp.setObjectName("num_cp")
        self.num_cd = QtWidgets.QLabel(Dialog)
        self.num_cd.setGeometry(QtCore.QRect(230, 380, 65, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.num_cd.setFont(font)
        self.num_cd.setFrameShape(QtWidgets.QFrame.Box)
        self.num_cd.setAlignment(QtCore.Qt.AlignCenter)
        self.num_cd.setObjectName("num_cd")
        self.num_penalty = QtWidgets.QLabel(Dialog)
        self.num_penalty.setGeometry(QtCore.QRect(230, 410, 65, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.num_penalty.setFont(font)
        self.num_penalty.setFrameShape(QtWidgets.QFrame.Box)
        self.num_penalty.setAlignment(QtCore.Qt.AlignCenter)
        self.num_penalty.setObjectName("num_penalty")
        self.bonus = QtWidgets.QLabel(Dialog)
        self.bonus.setGeometry(QtCore.QRect(230, 440, 65, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.bonus.setFont(font)
        self.bonus.setFrameShape(QtWidgets.QFrame.Box)
        self.bonus.setAlignment(QtCore.Qt.AlignCenter)
        self.bonus.setObjectName("bonus")
        self.label_score = QtWidgets.QLabel(Dialog)
        self.label_score.setGeometry(QtCore.QRect(160, 490, 161, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(28)
        font.setBold(True)
        font.setWeight(75)
        self.label_score.setFont(font)
        self.label_score.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_score.setAlignment(QtCore.Qt.AlignCenter)
        self.label_score.setObjectName("label_score")
        self.final_score = QtWidgets.QLabel(Dialog)
        self.final_score.setGeometry(QtCore.QRect(320, 490, 91, 31))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(28)
        font.setBold(True)
        font.setWeight(75)
        self.final_score.setFont(font)
        self.final_score.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.final_score.setAlignment(QtCore.Qt.AlignCenter)
        self.final_score.setObjectName("final_score")
        self.header2 = QtWidgets.QLabel(Dialog)
        self.header2.setGeometry(QtCore.QRect(330, 10, 130, 40))
        font = QtGui.QFont()
        font.setFamily("Cambria")
        font.setPixelSize(30)
        font.setBold(True)
        font.setUnderline(False)
        font.setWeight(75)
        font.setStrikeOut(False)
        self.header2.setFont(font)
        self.header2.setStyleSheet("color: rgb(0, 0, 0);")
        self.header2.setScaledContents(False)
        self.header2.setAlignment(QtCore.Qt.AlignCenter)
        self.header2.setWordWrap(True)
        self.header2.setIndent(0)
        self.header2.setObjectName("header2")
        self.teamid = QtWidgets.QLabel(Dialog)
        self.teamid.setGeometry(QtCore.QRect(550, 85, 80, 30))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(24)
        font.setBold(True)
        font.setWeight(75)
        self.teamid.setFont(font)
        self.teamid.setStyleSheet("color: rgb(255, 0, 0);")
        self.teamid.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.teamid.setAlignment(QtCore.Qt.AlignCenter)
        self.teamid.setObjectName("teamid")
        self.label_college_name = QtWidgets.QLabel(Dialog)
        self.label_college_name.setGeometry(QtCore.QRect(290, 130, 130, 25))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_college_name.setFont(font)
        self.label_college_name.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_college_name.setAlignment(QtCore.Qt.AlignCenter)
        self.label_college_name.setWordWrap(True)
        self.label_college_name.setObjectName("label_college_name")
        self.college_name = QtWidgets.QLabel(Dialog)
        self.college_name.setGeometry(QtCore.QRect(450, 130, 280, 70))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.college_name.setFont(font)
        self.college_name.setStyleSheet("color: rgb(255, 0, 0);")
        self.college_name.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.college_name.setTextFormat(QtCore.Qt.AutoText)
        self.college_name.setAlignment(QtCore.Qt.AlignHCenter)
        self.college_name.setWordWrap(True)
        self.college_name.setObjectName("college_name")
        self.label_teamid = QtWidgets.QLabel(Dialog)
        self.label_teamid.setGeometry(QtCore.QRect(290, 90, 130, 20))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        font.setPixelSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_teamid.setFont(font)
        self.label_teamid.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_teamid.setAlignment(QtCore.Qt.AlignCenter)
        self.label_teamid.setWordWrap(True)
        self.label_teamid.setObjectName("label_teamid")

        sys.stdout = EmittingStream(textWritten=self.output_terminal_written)
        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)
        self.exit.clicked.connect(self.exit_btn)
        self.end_process.clicked.connect(self.end_process_btn)

        global theme_config
        theme_config = json.load(open(resource_path("Theme_Config.json")))
        self.show_theme_config(theme_config)
        self.show_team_details(team_id_val)
        # self.show_evaluation_parameters(folder_path + "/theme_implementation_result.txt")
        self.start_process()



    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "BM_GUI"))
        # self.header3.setText(_translate("Dialog", "Robotics Competition 2021-22"))
        self.label_output_console.setText(_translate("Dialog", "Output Console"))
        self.end_process.setText(_translate("Dialog", "End Process"))
        self.exit.setText(_translate("Dialog", "Exit"))
        self.label_theme_config.setText(_translate("Dialog", "Theme Configuration"))
        self.label_blueberry.setText(_translate("Dialog", "Blueberry"))
        self.label_strawberry.setText(_translate("Dialog", "Strawberry"))
        self.label_lemon.setText(_translate("Dialog", "Lemon"))
        self.blueberry_box.setText(_translate("Dialog", "CB1"))
        self.strawberry_box.setText(_translate("Dialog", "CB2"))
        self.lemon_box.setText(_translate("Dialog", "CB1"))
        self.label_ci.setText(_translate("Dialog", "Correct Identification (CI)"))
        self.label_cp.setText(_translate("Dialog", "Correct Pluck (CP)"))
        self.label_cd.setText(_translate("Dialog", "Correct Deposition (CD)"))
        self.label_time.setText(_translate("Dialog", "Time (in sec) (T)"))
        self.label_penalty.setText(_translate("Dialog", "Penalties (P)"))
        self.label_bonus.setText(_translate("Dialog", "Bonus (B)"))
        self.label_valid_run.setText(_translate("Dialog", "Valid Run"))
        self.valid_run.setText(_translate("Dialog", "N/A"))
        self.num_seconds.setText(_translate("Dialog", "0"))
        self.num_ci.setText(_translate("Dialog", "0"))
        self.num_cp.setText(_translate("Dialog", "0"))
        self.num_cd.setText(_translate("Dialog", "0"))
        self.num_penalty.setText(_translate("Dialog", "0"))
        self.bonus.setText(_translate("Dialog", "0"))
        self.label_score.setText(_translate("Dialog", "Total Score:"))
        self.final_score.setText(_translate("Dialog", "0000.00"))
        self.header2.setText(_translate("Dialog", "TASK 5"))
        self.teamid.setText(_translate("Dialog", "0000"))
        self.label_college_name.setText(_translate("Dialog", "College Name:"))
        self.college_name.setText(_translate("Dialog", "Pimpri Chinchwad Education Trust\'s Pimpri Chinchwad College of Engineering"))
        self.label_teamid.setText(_translate("Dialog", "Team ID:"))

    def start_process(self):
        global pid
        self.process = QtCore.QProcess()
        self.process.readyReadStandardOutput.connect(self.handle_stdout)
        self.process.readyReadStandardError.connect(self.handle_stderr)
        self.process.stateChanged.connect(self.handle_state)

        # os.kill(pid, signal.CTRL_C_EVENT)
        self.process.finished.connect(self.process_finished)  # Clean up once complete.

        # self.process.start("python", ["test.py"])

        # print("Task 5 Task 5 Task 5")
        # print(folder_path)
        command = 'cmd.exe /C task_5_cardinal.exe ' + str(team_id_val) + ' "' + folder_path + '"'
        # print(command)
        self.process.start(command)
        pid = self.process.processId()
        # print("process pid" + str(pid))

    def handle_stderr(self):
        data = self.process.readAllStandardError()
        stderr = bytes(data).decode("utf8")
        # self.message(stderr)
        print(stderr)

    def handle_stdout(self):
        data = self.process.readAllStandardOutput()
        stdout = bytes(data).decode("utf8")
        # self.message(stdout)
        print(stdout)

    def handle_state(self, state):
        states = {
            QtCore.QProcess.NotRunning: 'Not running',
            QtCore.QProcess.Starting: 'Starting',
            QtCore.QProcess.Running: 'Running',
        }
        state_name = states[state]
        # print(f"State changed: {state_name}")
        # self.message(f"State changed: {state_name}")

    def process_finished(self):
        print("Process finished.")
        self.process = None
        self.show_evaluation_parameters(folder_path + "/theme_implementation_result.txt")

    def show_theme_config(self, theme_config):
        blueberry = theme_config["B"]
        strawberry = theme_config["S"]
        lemon = theme_config["L"]
        blueberry_num, blueberry_collection_box = int(blueberry[0]), blueberry[2:]
        strawberry_num, strawberry_collection_box = int(strawberry[0]), strawberry[2:]
        lemon_num, lemon_collection_box = int(lemon[0]), lemon[2:]

        self.blueberry_box.setText(blueberry_collection_box)
        self.strawberry_box.setText(strawberry_collection_box)
        self.lemon_box.setText(lemon_collection_box)


        if blueberry_num == 1:
            self.blueberry1.setPixmap(QtGui.QPixmap(resource_path("blueberry.png")))
        elif blueberry_num == 2:
            self.blueberry1.setPixmap(QtGui.QPixmap(resource_path("blueberry.png")))
            self.blueberry2.setPixmap(QtGui.QPixmap(resource_path("blueberry.png")))
        elif blueberry_num == 3:
            self.blueberry1.setPixmap(QtGui.QPixmap(resource_path("blueberry.png")))
            self.blueberry2.setPixmap(QtGui.QPixmap(resource_path("blueberry.png")))
            self.blueberry3.setPixmap(QtGui.QPixmap(resource_path("blueberry.png")))
        else:
            pass

        if strawberry_num == 1:
            self.strawberry1.setPixmap(QtGui.QPixmap(resource_path("strawberry.png")))
        elif strawberry_num == 2:
            self.strawberry1.setPixmap(QtGui.QPixmap(resource_path("strawberry.png")))
            self.strawberry2.setPixmap(QtGui.QPixmap(resource_path("strawberry.png")))
        elif strawberry_num == 3:
            self.strawberry1.setPixmap(QtGui.QPixmap(resource_path("strawberry.png")))
            self.strawberry2.setPixmap(QtGui.QPixmap(resource_path("strawberry.png")))
            self.strawberry3.setPixmap(QtGui.QPixmap(resource_path("strawberry.png")))
        else:
            pass

        if lemon_num == 1:
            self.lemon1.setPixmap(QtGui.QPixmap(resource_path("lemon.png")))
        elif lemon_num == 2:
            self.lemon1.setPixmap(QtGui.QPixmap(resource_path("lemon.png")))
            self.lemon2.setPixmap(QtGui.QPixmap(resource_path("lemon.png")))
        elif lemon_num == 3:
            self.lemon1.setPixmap(QtGui.QPixmap(resource_path("lemon.png")))
            self.lemon2.setPixmap(QtGui.QPixmap(resource_path("lemon.png")))
            self.lemon3.setPixmap(QtGui.QPixmap(resource_path("lemon.png")))
        else:
            pass

    def show_team_details(self, team_id):
        team_details_csv = open(resource_path("team_bm.csv"))
        dict_reader = list(csv.DictReader(team_details_csv))
        team_dict = {}
        for i in dict_reader:
            teamid = i["Team ID"]
            college_name = i["College Name"]
            team_dict[teamid] = college_name

        self.teamid.setText(str(team_id))
        self.college_name.setText(team_dict[str(team_id)])

    # def show_evaluation_parameters(self, result_filename):
    #     team_id, dt, ci, cp, cd, collisions, bonus, end_simulation_time, init_real_time = self.decode_result_file(result_filename)
    #     # end_simulation_time = round(float(end_simulation_time),2)
    #     # self.num_seconds.setText(str(end_simulation_time))
    #     # self.num_ci.setText(str(len(ci)))
    #     # self.num_cp.setText(str(len(cp)))
    #     # self.num_cd.setText(str(len(cd)))
    #     # self.num_penalty.setText(str(len(collisions)))
    #     # self.bonus.setText(bonus)
    #     # bonus_val = 0

    #     # if bonus == 'Y':
    #     #     bonus_val = 200
    #     # else:
    #     #     bonus_val = 0

    #     # total_score = (600 - float(end_simulation_time)) + len(ci)*10 + len(cp)*50 + len(cd)*50 - len(collisions)*30 + bonus_val
    #     total_score = round(total_score, 2)
    #     self.final_score.setText(str(total_score))

    def output_terminal_written(self, text):
        self.console_output.append(text)

    def exit_btn(self):
        sys.exit()

    def end_process_btn(self):
        os.kill(pid, signal.CTRL_C_EVENT)

    def show_evaluation_parameters(self, filename):
        f = open(filename, "r")
        content = f.readlines()

        # Extracting team no and score
        i = 1
        for x in content:
            if   i == 1:
                team_id = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            elif i == 2:
                dt = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            elif i == 3:
                platform = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            elif i == 4:
                mac = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            elif i == 5:
                T = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            elif i == 6:
                CI = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            elif i == 7:
                CP = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            elif i == 8:
                CD = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            elif i == 9:
                P = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            elif i == 10:
                B = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            elif i == 11:
                score = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 12:
            #     final_ci_list = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 13:
            #     final_cp_list = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 14:
            #     final_cd_list = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 15:
            #     final_cb1_drops = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 16:
            #     final_cb2_drops = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")

            # elif i == 17:
            #     raw_ci_list = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 18:
            #     raw_cp_list = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 19:
            #     raw_dropped_in_cb1_list = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 20:
            #     raw_dropped_in_cb2_list = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 21:
            #     raw_collisions_list = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 22:
            #     mass_of_arm = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 23:
            #     path = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")



            # elif i == 24:
            #     eval_rtf_python = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 25:
            #     end_simulation_time = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 26:
            #     init_real_time = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 27:
            #     end_real_time = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 28:
            #     rtf_python = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 29:
            #     eval_all_dynamics = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 30:
            #     eval_no_of_joints = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 31:
            #     no_of_joints = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 32:
            #     torque = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 33:
            #     force = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 34:
            #     individual_values = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 35:
            #     dynamically_not_enabled_list = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            elif i == 36:
                valid_run_flag = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 37:
            #     time_to_substract = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")    
            # elif i == 38:
            #     output_list_child = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")
            # elif i == 39:
            #     output_list_custom = cryptocode.decrypt(x, "q$yh#N?g65CB!pk2")

            i += 1

        # print("Team no                         : ", team_id)
        # print("Date and Time                   : ", dt)
        # print("Platform                        : ", platform)
        # print("Mac                             : ", mac)
        # print("VALID Run?                      : ", valid_run_flag)
        # print("T                               : ", T)
        # print("CI                              : ", CI)
        # print("CP                              : ", CP)
        # print("CD                              : ", CD)
        # print("P                               : ", P)
        # print("B                               : ", B)
        # print("score                           : ", score)
        # print("Final CI List                   : ", final_ci_list)
        # print("Final CP List                   : ", final_cp_list)
        # print("Final CD List                   : ", final_cd_list)
        # print("Final CB1 Drops                 : ", final_cb1_drops)
        # print("Final CB2 Drops                 : ", final_cb2_drops)
        # print()
        # print("Raw CI List                     : ", raw_ci_list)
        # print("Raw CP List                     : ", raw_cp_list)
        # print("Raw Dropped in CB1 List         : ", raw_dropped_in_cb1_list)
        # print("Raw Dropped in CB2 List         : ", raw_dropped_in_cb2_list)
        # print("Collisions                      : ", raw_collisions_list)
        # print("Mass of Arm                     : ", mass_of_arm)
        # print("Path                            : ", path)
        # print("eval rtf                        : ", eval_rtf_python)
        # print("Total simulation time           : ", end_simulation_time)
        # print("Time to substract               : ", time_to_substract)
        # print("Init real time                  : ", init_real_time)
        # print("End real time                   : ", end_real_time)
        # print("RTF from Python                 : ", rtf_python)
        # print("Eval all dynamics               : ", eval_all_dynamics)
        # print("Eval no of joints               : ", eval_no_of_joints)
        # print("No. of joints                   : ", no_of_joints)
        # print("Torque                          : ", torque)
        # print("Force                           : ", force)
        # print("Indivdual Values                : ", individual_values)
        # print("Dyn not enabled list            : ", dynamically_not_enabled_list)
        # print("output_list_child               : ", output_list_child)
        # print("output_list_custom              : ", output_list_custom)

        T = str(round(float(T),2))
        score = str(round(float(score),2))

        self.valid_run.setText(valid_run_flag)
        self.num_seconds.setText(T)
        self.num_ci.setText(CI)
        self.num_cp.setText(CP)
        self.num_cd.setText(CD)
        self.num_penalty.setText(P)
        self.bonus.setText(B)
        self.final_score.setText(score)

    def calculate_bonus(self, CI, CP, CD, Collisions, end_simulation_time):
        blueberry = theme_config["B"][0]
        strawberry = theme_config["S"][0]
        lemon = theme_config["L"][0]
        blueberry_num, blueberry_collection_box = int(blueberry[0]), blueberry[2:]
        strawberry_num, strawberry_collection_box = int(strawberry[0]), strawberry[2:]
        lemon_num, lemon_collection_box = int(lemon[0]), lemon[2:]

        if len(Collisions) > 0:
            return 'N'
        if float(end_simulation_time) >= 600:
            return 'N'

        ci_flag = None
        cp_flag = None
        cd_flag = None

        if CI.count("Blueberry") == blueberry_num and  CI.count("Strawberry") == strawberry_num and CI.count("Lemon") == lemon_num:
            ci_flag = True
        else:
            ci_flag = False

        if CP.count("Blueberry") == blueberry_num and  CP.count("Strawberry") == strawberry_num and CP.count("Lemon") == lemon_num:
            cp_flag = True
        else:
            cp_flag = False

        if CD.count("Blueberry") == blueberry_num and  CD.count("Strawberry") == strawberry_num and CD.count("Lemon") == lemon_num:
            cd_flag = True
        else:
            cd_flag = False

        if ci_flag and cp_flag and cd_flag:
            return 'Y'
        else:
            return 'N'



if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Berryminator_Evaluator = QtWidgets.QMainWindow()
    ui = Ui_Berryminator_Evaluator()
    ui.setupUi(Berryminator_Evaluator)
    Berryminator_Evaluator.show()
    sys.exit(app.exec_())