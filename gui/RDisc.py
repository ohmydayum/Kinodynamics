from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtOpenGL import QGLWidget, QGLFormat, QGL
from PyQt5.QtWidgets import (QApplication, QGraphicsView,
                             QGraphicsPixmapItem, QGraphicsScene, QGraphicsPolygonItem,
                             QGraphicsEllipseItem, QGraphicsLineItem, QOpenGLWidget)
from PyQt5.QtGui import QPainter, QPixmap, QPolygonF, QPen
from PyQt5.QtCore import (QObject, QPointF, QPoint, QRectF,
                          QPropertyAnimation, pyqtProperty, QSequentialAnimationGroup,
                          QParallelAnimationGroup, QPauseAnimation, Qt)

class RDisc(QObject):
  def __init__(self, r, x, y, color, line_width):
    self._radius = r
    self._pos = QPointF(x, y)
    super().__init__()
    self.rect = QRectF(x-r,y-r,2*r,2*r)
    self.disc = QGraphicsEllipseItem()
    self.disc.setRect(self.rect)
    #self.disc.setPos(QPointF(x-r, y-r))
    #self.rect.moveTo(QPointF(x-r, y-r))
    self.disc.setBrush(QtGui.QBrush(color))
    pen = QPen()
    pen.setWidthF(line_width)
    self.disc.setPen(pen)

  def x(self):
    return self._pos.x()
  def y(self):
    return self._pos.y()

  @pyqtProperty(QPointF)
  def pos(self):
    return self._pos

  @pos.setter
  def pos(self, value):
    self.rect = QRectF(value.x()-self._radius,value.y()-self._radius,2*self._radius,2*self._radius)
    self.disc.setRect(self.rect)
    self._pos = value