from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtOpenGL import QGLWidget, QGLFormat, QGL
from PyQt5.QtWidgets import (QApplication, QGraphicsView,
                             QGraphicsPixmapItem, QGraphicsScene, QGraphicsPolygonItem,
                             QGraphicsEllipseItem, QGraphicsLineItem, QOpenGLWidget)
from PyQt5.QtGui import QPainter, QPixmap, QPolygonF, QPen
from PyQt5.QtCore import (QObject, QPointF, QPoint, QRectF,
                          QPropertyAnimation, pyqtProperty, QSequentialAnimationGroup,
                          QParallelAnimationGroup, QPauseAnimation, Qt)

class RPolygon(QObject):
  def __init__(self, points, color, line_width):
    self._points = [QPointF(p[0],p[1]) for p in points]
    self._pos = self._points[0]
    super().__init__()
    self.polygon = QGraphicsPolygonItem()
    self.polygon.setPolygon(QPolygonF(self._points))
    self.polygon.setBrush(QtGui.QBrush(color))
    pen = QPen()
    pen.setWidthF(line_width)
    self.polygon.setPen(pen)
    self._visible = 1
    #print(self._points)

  def x(self):
    return self._pos.x()
  def y(self):
    return self._pos.y()
  def points(self):
    return self._points

  @pyqtProperty(QPointF)
  def pos(self):
    return self._pos

  @pos.setter
  def pos(self, value):
    delta_x = value.x() - self._pos.x()
    delta_y = value.y() - self._pos.y()
    self._points = [QPointF(p.x()+delta_x, p.y() + delta_y) for p in self._points]
    self.polygon.setPolygon(QPolygonF(self._points))
    self._pos = value

  @pyqtProperty(int)
  def visible(self):
    return self._visible

  @visible.setter
  def visible(self, value):
    if(value > 0): self.polygon.show()
    else: self.polygon.hide()
    self._visible = value