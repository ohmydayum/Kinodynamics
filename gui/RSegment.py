from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtOpenGL import QGLWidget, QGLFormat, QGL
from PyQt5.QtWidgets import (QApplication, QGraphicsView,
                             QGraphicsPixmapItem, QGraphicsScene, QGraphicsPolygonItem,
                             QGraphicsEllipseItem, QGraphicsLineItem, QOpenGLWidget)
from PyQt5.QtGui import QPainter, QPixmap, QPolygonF, QPen
from PyQt5.QtCore import (QObject, QPointF, QPoint, QRectF,
                          QPropertyAnimation, pyqtProperty, QSequentialAnimationGroup,
                          QParallelAnimationGroup, QPauseAnimation, Qt)

class RSegment(QObject):
  def __init__(self, x1, y1, x2, y2, color, line_width):
    self._x1 = x1
    self._y1 = y1
    self._x2 = x2
    self._y2 = y2
    self._pos = QPointF(x1, y1)
    super().__init__()
    self.line = QGraphicsLineItem()
    self.line.setLine(x1, y1, x2, y2)
    pen = QPen()
    pen.setWidthF(line_width)
    pen.setColor(color)
    self.line.setPen(pen)

  def x(self):
    return self._pos.x()

  def y(self):
    return self._pos.y()

  @pyqtProperty(QPointF)
  def pos(self):
    return self._pos

  @pos.setter
  def pos(self, value):
    delta_x = value.x() - self._pos.x()
    delta_y = value.y() - self._pos.y()
    self._x1 = self._x1 + delta_x
    self._y1 = self._y1 + delta_y
    self._x2 = self._x2 + delta_x
    self._y2 = self._y2 + delta_y
    self.line.setLine(self._x1, self._y1, self._x2, self._y2)
    self._pos = value