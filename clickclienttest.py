# -*- config:utf-8 -*-

import sys
import os

from PySide2.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout, QGraphicsScene, QGraphicsView, QWidget, QLabel, QTextEdit, QTextBrowser, QLineEdit, QSpacerItem, QSizePolicy, QMessageBox)
from PySide2.QtCore import Qt
from PySide2.QtGui import QPixmap, QImage
import WebSocketCapf as cwebsock
import json

class MapView(QGraphicsView):
    def __init__(self, parent=None) :
        super(MapView, self).__init__(parent=parent)
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.backgroundImage = None
        self.sendInputCB = None
        self.markerPoint = [0, 0]

    def setImage(self, bgImage) :
        self.backgroundImage = bgImage

    def setSendInputCB(self, func) :
        self.sendInputCB = func

    def mousePressEvent(self, event):
        qp = self.mapToScene(event.x(), event.y())
        tx, ty = qp.x(), qp.y()
        if not self.sendInputCB is None:
            self.sendInputCB(tx, ty)

    def paintEvent(self,event) :
        super().paintEvent(event)
        self.repaint()

    def repaint(self) :
        if self.backgroundImage :
            pixmap = QPixmap.fromImage(self.backgroundImage)
            self.scene.clear()
            self.scene.addPixmap(pixmap)
            self.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

class mainView(QMainWindow) :
    def __init__(self) :
        super().__init__()

        self.loginurl = 'https://atr-dev01.ca-platform.org/api/login'
        self.websockurl = 'wss://atr-dev01-websocket.ca-platform.org'
        self.accountid = 'CA003'
        self.accountpswd = 'CA003'
        self.capfWebSocket = cwebsock.WebSocketCapf(
            self.loginurl, self.accountid, self.accountpswd, self.websockurl)
        #self.capfWebSocket.setMessageCallback(self.recvWebMessageCB)
        if not self.capfWebSocket.connect():
            QMessageBox.information(
                None, "Error", "WebSocket Connect Error : " + self.websockurl, QMessageBox.Ok)
            self.close()
            sys.exit()

        self.mapImage = MapView()
        self.mapImage.setSendInputCB(self.sendInput)

        layout = QVBoxLayout()
        layout.addWidget(self.mapImage)
        #layout.addWidget(QPushButton('TEST'))

        mainWidget = QWidget()
        mainWidget.setLayout(layout)
        self.setCentralWidget(mainWidget)
        self.show()

    def setImage(self, img) :
        self.mapImage.setImage(img)
        self.mapImage.repaint()

    def sendInput(self, x, y) :
        jsonparam = {'x' : x, 'y' : y}
        jsoncmd = {'cmd' : 'position', 'param' : json.dumps(jsonparam)}
        #jsonmsg = {'targets' : 'OP002SA', 'message' : json.dumps(jsoncmd)}
        jsonmsg = {'targets' : 'CA001', 'message' : json.dumps(jsoncmd)}
        self.capfWebSocket.send(json.dumps(jsonmsg))

if __name__ == '__main__' :
    app = QApplication(sys.argv)
    image = QImage('sample.png')
    window = mainView()
    window.setImage(image)
    app.exec_()
