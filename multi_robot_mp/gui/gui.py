from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtOpenGL import QGLWidget, QGLFormat, QGL
from PyQt5.QtWidgets import (QApplication, QGraphicsView,
                             QGraphicsPixmapItem, QGraphicsScene, QGraphicsPolygonItem,
                             QGraphicsEllipseItem, QGraphicsLineItem, QOpenGLWidget)
from PyQt5.QtGui import QPainter, QPixmap, QPolygonF, QPen
from PyQt5.QtCore import (QObject, QPointF, QPoint, QRectF,
                          QPropertyAnimation, pyqtProperty, QSequentialAnimationGroup,
                          QParallelAnimationGroup, QPauseAnimation, Qt)

from gui.RPolygon import RPolygon
from gui.RDisc import RDisc
from gui.RSegment import RSegment

class MainWindowPlus(QtWidgets.QMainWindow):
  def __init__(self, gui):
    super().__init__()
    self.gui = gui
  # def resizeEvent(self, event):
    # x_ratio = self.width()/self.gui.width
    # y_ratio = self.height()/self.gui.height
    # self.gui.graphicsView.resetTransform()
    # self.gui.graphicsView.scale(x_ratio, y_ratio)
    # self.gui.graphicsView.scale(self.gui.zoom, self.gui.zoom)
  def keyPressEvent(self, event):
    if event.key() == QtCore.Qt.Key_Plus:
      self.gui.zoom /= 0.9
    if event.key() == QtCore.Qt.Key_Minus:
      self.gui.zoom *= 0.9
    self.gui.redraw()



class GUI(object):
  width = 1600
  height = 1000
  zoom = 50.0
  base_line_width = 1

  def __init__(self):
    MainWindow = MainWindowPlus(self)
    self.setupUi(MainWindow)

  def setupUi(self, MainWindow):
    self.MainWindow = MainWindow
    self.sequence = QSequentialAnimationGroup()
    self.sequence.finished.connect(self.animation_finished)
    self.scene = QGraphicsScene()

    MainWindow.setObjectName("MainWindow")
    #MainWindow.setWindowIcon(QtGui.QIcon("icon.png"))
    MainWindow.resize(self.width, self.height)

#qt designer generated code
    self.centralwidget = QtWidgets.QWidget(MainWindow)
    self.centralwidget.setObjectName("centralwidget")
    self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
    self.gridLayout.setObjectName("gridLayout")
    self.graphicsView = QtWidgets.QGraphicsView(self.centralwidget)
    self.graphicsView.setEnabled(True)
    sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
    sizePolicy.setHorizontalStretch(1)
    sizePolicy.setVerticalStretch(0)
    sizePolicy.setHeightForWidth(self.graphicsView.sizePolicy().hasHeightForWidth())
    self.graphicsView.setSizePolicy(sizePolicy)
    self.graphicsView.setObjectName("graphicsView")
    self.gridLayout.addWidget(self.graphicsView, 3, 1, 1, 1)
    self.gridLayout_2 = QtWidgets.QGridLayout()
    self.gridLayout_2.setObjectName("gridLayout_2")
    self.pushButton_1 = QtWidgets.QPushButton(self.centralwidget)
    self.pushButton_1.setObjectName("pushButton_1")
    self.gridLayout_2.addWidget(self.pushButton_1, 3, 0, 1, 1)
    self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
    self.pushButton_2.setObjectName("pushButton_2")
    self.gridLayout_2.addWidget(self.pushButton_2, 5, 0, 1, 1)
    self.pushButton_4 = QtWidgets.QPushButton(self.centralwidget)
    self.pushButton_4.setObjectName("pushButton_4")
    self.gridLayout_2.addWidget(self.pushButton_4, 9, 0, 1, 1)
    self.lineEdit_3 = QtWidgets.QLineEdit(self.centralwidget)
    self.lineEdit_3.setObjectName("lineEdit_3")
    self.gridLayout_2.addWidget(self.lineEdit_3, 6, 0, 1, 1)
    self.pushButton_0 = QtWidgets.QPushButton(self.centralwidget)
    self.pushButton_0.setObjectName("pushButton_0")
    self.gridLayout_2.addWidget(self.pushButton_0, 1, 0, 1, 1)
    self.lineEdit_1 = QtWidgets.QLineEdit(self.centralwidget)
    self.lineEdit_1.setObjectName("lineEdit_1")
    self.gridLayout_2.addWidget(self.lineEdit_1, 2, 0, 1, 1)
    self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
    self.pushButton_3.setObjectName("pushButton_3")
    self.gridLayout_2.addWidget(self.pushButton_3, 7, 0, 1, 1)
    self.pushButton_6 = QtWidgets.QPushButton(self.centralwidget)
    self.pushButton_6.setObjectName("pushButton_6")
    self.gridLayout_2.addWidget(self.pushButton_6, 14, 0, 1, 1)
    self.pushButton_7 = QtWidgets.QPushButton(self.centralwidget)
    self.pushButton_7.setObjectName("pushButton_7")
    self.gridLayout_2.addWidget(self.pushButton_7, 19, 0, 1, 1)
    self.lineEdit_4 = QtWidgets.QLineEdit(self.centralwidget)
    self.lineEdit_4.setObjectName("lineEdit_4")
    self.gridLayout_2.addWidget(self.lineEdit_4, 8, 0, 1, 1)
    self.lineEdit_0 = QtWidgets.QLineEdit(self.centralwidget)
    self.lineEdit_0.setObjectName("lineEdit_0")
    self.gridLayout_2.addWidget(self.lineEdit_0, 0, 0, 1, 1)
    self.lineEdit_2 = QtWidgets.QLineEdit(self.centralwidget)
    self.lineEdit_2.setObjectName("lineEdit_2")
    self.gridLayout_2.addWidget(self.lineEdit_2, 4, 0, 1, 1)
    self.pushButton_5 = QtWidgets.QPushButton(self.centralwidget)
    self.pushButton_5.setObjectName("pushButton_5")
    self.gridLayout_2.addWidget(self.pushButton_5, 11, 0, 1, 1)
    self.lineEdit_7 = QtWidgets.QLineEdit(self.centralwidget)
    self.lineEdit_7.setObjectName("lineEdit_7")
    self.gridLayout_2.addWidget(self.lineEdit_7, 17, 0, 1, 1)
    self.lineEdit_5 = QtWidgets.QLineEdit(self.centralwidget)
    self.lineEdit_5.setObjectName("lineEdit_5")
    self.gridLayout_2.addWidget(self.lineEdit_5, 10, 0, 1, 1)
    self.lineEdit_6 = QtWidgets.QLineEdit(self.centralwidget)
    self.lineEdit_6.setObjectName("lineEdit_6")
    self.gridLayout_2.addWidget(self.lineEdit_6, 12, 0, 1, 1)
    self.gridLayout.addLayout(self.gridLayout_2, 3, 0, 1, 1)
    MainWindow.setCentralWidget(self.centralwidget)
    self.statusbar = QtWidgets.QStatusBar(MainWindow)
    self.statusbar.setObjectName("statusbar")
    MainWindow.setStatusBar(self.statusbar)

    self.retranslateUi(MainWindow)
    QtCore.QMetaObject.connectSlotsByName(MainWindow)

#end of eq designer generated code

    self.lineEdits = []
    self.lineEdits.append(self.lineEdit_0)
    self.lineEdits.append(self.lineEdit_1)
    self.lineEdits.append(self.lineEdit_2)
    self.lineEdits.append(self.lineEdit_3)
    self.lineEdits.append(self.lineEdit_4)
    self.lineEdits.append(self.lineEdit_5)
    self.lineEdits.append(self.lineEdit_6)
    self.lineEdits.append(self.lineEdit_7)
    self.pushButtons = []
    self.pushButtons.append(self.pushButton_0)
    self.pushButtons.append(self.pushButton_1)
    self.pushButtons.append(self.pushButton_2)
    self.pushButtons.append(self.pushButton_3)
    self.pushButtons.append(self.pushButton_4)
    self.pushButtons.append(self.pushButton_5)
    self.pushButtons.append(self.pushButton_6)
    self.pushButtons.append(self.pushButton_7)
    self.graphicsView.setScene(self.scene)
    self.graphicsView.setSceneRect(0, 0, 0, 0)
    self.graphicsView.setRenderHints(QPainter.Antialiasing)
    # self.graphicsView.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
    self.graphicsView.setViewport(QGLWidget(QGLFormat(QGL.SampleBuffers)))
    self.graphicsView.scale(self.zoom, -self.zoom)
    self.graphicsView.setDragMode(1)

  def retranslateUi(self, MainWindow):
    _translate = QtCore.QCoreApplication.translate
    MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
    self.pushButton_1.setText(_translate("MainWindow", "PushButton"))
    self.pushButton_5.setText(_translate("MainWindow", "PushButton"))
    self.pushButton_7.setText(_translate("MainWindow", "PushButton"))
    self.pushButton_3.setText(_translate("MainWindow", "PushButton"))
    self.pushButton_0.setText(_translate("MainWindow", "PushButton"))
    self.pushButton_4.setText(_translate("MainWindow", "PushButton"))
    self.pushButton_2.setText(_translate("MainWindow", "PushButton"))
    self.pushButton_6.setText(_translate("MainWindow", "PushButton"))

  def add_disc(self, r, x, y, fill_color=QtCore.Qt.black):
    d = RDisc(r, x, y, fill_color, line_width = self.base_line_width / self.zoom)
    self.scene.addItem(d.disc)
    return d

  def add_polygon(self, points, fill_color=QtCore.Qt.black):
    p = RPolygon(points, fill_color, line_width = self.base_line_width / self.zoom)
    self.scene.addItem(p.polygon)
    return p

  def add_segment(self, x1, y1, x2, y2, line_color=QtCore.Qt.black):
    s = RSegment(x1, y1, x2, y2, line_color, line_width = self.base_line_width / self.zoom)
    self.scene.addItem(s.line)
    return s

  def linear_translation_animation(self, obj, ix, iy, x, y, duration = 1000):
    anim = QPropertyAnimation(obj, b'pos')
    anim.setDuration(duration)
    anim.setStartValue(QPointF(ix, iy))
    anim.setEndValue(QPointF(x, y))
    return anim

  def translation_animation(self, obj, func, duration = 1000):
    anim = QPropertyAnimation(obj, b'pos')
    anim.setDuration(duration)
    anim.setStartValue(QPointF(func(0)[0], func(0)[1]))
    anim.setEndValue(QPointF(func(1)[0], func(1)[1]))
    vals = [p / 100 for p in range(0, 101)]
    for i in vals:
      anim.setKeyValueAt(i, (QPointF(func(i)[0], func(i)[1])))
    return anim

  def visibility_animation(self, obj, visible):
    anim = QPropertyAnimation(obj, b'visible')
    anim.setDuration(0)
    if(visible): anim.setEndValue(1)
    else: anim.setEndValue(0)
    return anim

  def pause_animation(self, duration = 1000):
    anim = QPauseAnimation(duration)
    return anim

  def parallel_animation(self, *animations):
    group = QParallelAnimationGroup()
    for anim in animations:
      group.addAnimation(anim)
    return group

  def queue_animation(self, *animations):
    for anim in animations:
      self.sequence.addAnimation(anim)

  def play_queue(self):
    self.sequence.start()

  def clear_queue(self):
    self.sequence.clear()

  def clear_scene(self):
    self.scene.clear()

  def redraw(self):
    line_width = self.base_line_width / self.zoom
    for item in self.graphicsView.items():
      pen = item.pen()
      pen.setWidthF(line_width)
      item.setPen(pen)
    self.graphicsView.resetTransform()
    self.graphicsView.scale(self.zoom, -self.zoom)

  def animation_finished(self):
    print("Finished playing animation")

  def set_field(self, i, s):
    self.lineEdits[i].setText(s)

  def get_field(self, i):
    return self.lineEdits[i].text()

  def set_logic(self, i, logic):
    self.pushButtons[i].clicked.connect(logic)

  def set_button_text(self, i, s):
    self.pushButtons[i].setText(s)

  def set_program_name(self, s):
    self.MainWindow.setWindowTitle(s)