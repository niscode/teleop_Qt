# -*- config:utf-8 -*-

import sys
import os

import random

from threading import Thread
from datetime import datetime, timedelta
from PySide2.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout, QGraphicsScene, QGraphicsView, QWidget, QLabel, QTextEdit, QTextBrowser, QLineEdit, QSpacerItem, QSizePolicy, QMessageBox)
from PySide2.QtCore import QTimer, QSize, Qt
from PySide2.QtGui import QPixmap, QImage, QPainter, QBrush

import WebSocketCapf as cwebsock
import json


class MapView(QGraphicsView) :
    def __init__(self, parent=None) :
        super(MapView, self).__init__(parent=parent)
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.backgroundImage = None
        self.markerPoint = [0, 0]

    def setImage(self, bgImage) :
        self.backgroundImage = bgImage
        sz = self.backgroundImage.size()
        self.markerPoint = [sz.width() // 2, sz.height() // 2]

    def setMarkerPoint(self, x, y) :
        self.markerPoint = [x, y]
        self.update()
    
    def paintEvent(self,event) :
        super().paintEvent(event)
        self.repaint()
    
    def repaint(self) :
        if self.backgroundImage :
            tmpimg = QImage(self.backgroundImage)
            painter = QPainter() #tmpimg)
            painter.begin(tmpimg)
            painter.setPen(Qt.red)
            brush = QBrush(Qt.red, bs=Qt.SolidPattern)
            painter.setBrush(brush)
            painter.translate(self.markerPoint[0], self.markerPoint[1])
            painter.drawEllipse(-10, -10, 20, 20)
            painter.end()
            painter = None
            pixmap = QPixmap.fromImage(tmpimg)
            tmpimg = None
            #pixmap = pixmap.scaled(sz.width(), sz.height())
            self.scene.clear()
            self.scene.addPixmap(pixmap)
            self.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

class mainView(QMainWindow) :
    def __init__(self) :
        super().__init__()
        self.mapImage = MapView()

        layout = QVBoxLayout()
        layout.addWidget(self.mapImage)
        layout.addWidget(QPushButton('TEST'))

        mainWidget = QWidget()
        mainWidget.setLayout(layout)
        self.setCentralWidget(mainWidget)
        self.show()

        self.loginurl = 'https://ignis2.ca-platform.org/api/login'
        self.websockurl = 'wss://ignis2-websocket.ca-platform.org'
        self.accountid = 'OP002SA'
        self.accountpswd = 'OP002SA'
        # self.accountid = 'CA001'
        # self.accountpswd = 'CA001'
        self.capfWebSocket = cwebsock.WebSocketCapf(self.loginurl, self.accountid, self.accountpswd, self.websockurl)
        self.capfWebSocket.setMessageCallback(self.recvWebMessageCB)
        if not self.capfWebSocket.connect():
            QMessageBox.information(
                None, "Error", "WebSocket Connect Error : " + self.websockurl, QMessageBox.Ok)
            self.close()
            sys.exit()

    def recvWebMessageCB(self, message) :
        jsoncmd = json.loads(message)
        if 'cmd' in jsoncmd.keys() and 'param' in jsoncmd.keys() :
            command = jsoncmd['cmd']
            params  = json.loads(jsoncmd['param'])
            if command == 'position' :
                if 'x' in params.keys() and 'y' in params.keys() :
                    x, y = params['x'], params['y']
                    self.mapImage.setMarkerPoint(x,y)

    def setImage(self, img) :
        self.mapImage.setImage(img)
        self.mapImage.repaint()

if __name__ == '__main__' :
    app = QApplication(sys.argv)
    image = QImage('map_tokyo9f.svg')
    window = mainView()
    window.setImage(image)
    app.exec_()


