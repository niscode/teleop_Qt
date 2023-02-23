# -*- config:utf-8 -*-

import sys
import os

from PySide2.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout, QGraphicsScene, QGraphicsView, QWidget, QLabel, QTextEdit, QTextBrowser, QLineEdit, QSpacerItem, QSizePolicy, QMessageBox, QSlider, QGridLayout)
from PySide2.QtCore import Qt
from PySide2.QtGui import QPixmap, QImage, QPainter, QBrush, QFont
import WebSocketCapf as cwebsock
import json

TX = TY = 0.0

class MapView(QGraphicsView):
    def __init__(self, parent=None):
        super(MapView, self).__init__(parent=parent)
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.backgroundImage = None
        self.sendInputCB = None
        self.markerPoint = [0, 0]

    def setImage(self, bgImage):
        self.backgroundImage = bgImage
        sz = self.backgroundImage.size()
        self.markerPoint = [sz.width() // 2, sz.height() // 2]

    ## クリックした座標を取得
    def setSendInputCB(self, func):
        self.sendInputCB = func

    def mousePressEvent(self, event):
        global TX, TY
        qp = self.mapToScene(event.x(), event.y())
        TX, TY = qp.x(), qp.y()
        print(TX,TY)
        if not self.sendInputCB is None:
            self.sendInputCB(TX, TY)
            self.setMarkerPoint(TX, TY)

    ### マーカーを描画
    def setMarkerPoint(self, x, y):
        self.markerPoint = [x, y]
        self.update()

    def paintEvent(self,event) :
        super().paintEvent(event)
        self.repaint()

    def repaint(self):
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
            pixmap = QPixmap.fromImage(self.backgroundImage)
            self.scene.clear()
            self.scene.addPixmap(pixmap)
            self.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)


class UIView(QGraphicsView):
    def __init__(self, parent=None) :
        super(UIView, self).__init__(parent=parent)
        self.setWindowTitle("コントロールパネル")
        self.font = QFont("Arial", 20)
        self.sendSliderCB = None

        # マニュアル移動ボタン
        self.left_button =  QPushButton(" ◁ LEFT ")
        self.left_button.setFont(self.font)
        self.up_button =    QPushButton(" ▲ GO   ")
        self.up_button.setFont(self.font)
        self.down_button =  QPushButton(" ▼ BACK ")
        self.down_button.setFont(self.font)
        self.right_button = QPushButton(" ▷ RIGHT")
        self.right_button.setFont(self.font)

        # シリンダー昇降ボタン
        self.cylinder_slider = QSlider(Qt.Vertical)
        self.cylinder_slider.setTickPosition(QSlider.TicksBelow)
        self.cylinder_slider.setTickInterval(20)
        self.cylinder_slider.setMinimum(2)
        self.cylinder_slider.setMaximum(292)
        self.cylinder_slider.valueChanged.connect(self.changedValue)

        self.cylinder_label = QLabel("0")
        self.cylinder_label.setFont(self.font)
        self.cylinder_label.setAlignment(Qt.AlignCenter | Qt.AlignBottom)

        self.cylinder_button = QPushButton("UPDATE")
        self.cylinder_button.setFont(self.font)
        self.cylinder_button.clicked.connect(self.update_height)

        # ログ表示ボックス
        self.log_box = QLabel("LOG here ...")
        self.label_style = """QLabel {
            font: Arial;
            color: #383838;                /* 文字色 */
            font-size: 20px;               /* 文字サイズ */
            background-color:#e6e4ca;
            border-radius:4px;
            align-center;
        }"""
        self.log_box.setStyleSheet(self.label_style)
        self.log_box.setAlignment(Qt.AlignTop)
        
        layout = QGridLayout()
        layout.addWidget(self.left_button, 0, 0)
        layout.addWidget(self.up_button, 0, 1)
        layout.addWidget(self.down_button, 0, 2)
        layout.addWidget(self.right_button, 0, 3)

        layout.addWidget(self.cylinder_label, 1, 0)
        layout.addWidget(self.cylinder_slider, 1, 1, 2, 1)
        layout.addWidget(self.cylinder_button, 2, 0)

        layout.addWidget(self.log_box, 1, 2, 2, 3)
        self.setLayout(layout)

    ### スライダの情報を取得しサーバへ送信
    def setSendSliderCB(self, func):
        self.sendSliderCB = func

    def changedValue(self):
        size = self.cylinder_slider.value()
        self.cylinder_label.setText(str(size))

    def update_height(self):
        z = self.cylinder_slider.value()
        if not self.sendSliderCB is None:
            self.sendSliderCB(z)
            print("[Send params]   cylinder_pos: " + str(z))


class mainView(QMainWindow) :
    def __init__(self) :
        super().__init__()
        self.setWindowTitle("水道橋Map")
        self.resize(1260, 640)

        self.loginurl = 'https://ignis2.ca-platform.org/api/login'
        self.websockurl = 'wss://ignis2-websocket.ca-platform.org'
        self.accountid = 'CA003'
        self.accountpswd = 'CA003'
        self.capfWebSocket = cwebsock.WebSocketCapf(
            self.loginurl, self.accountid, self.accountpswd, self.websockurl)
        self.capfWebSocket.setMessageCallback(self.recvWebMessageCB)    # サーバ(CA)からROSの情報を取得
        if not self.capfWebSocket.connect():
            QMessageBox.information(
                None, "Error", "WebSocket Connect Error : " + self.websockurl, QMessageBox.Ok)
            self.close()
            sys.exit()

        self.mapImage = MapView()
        self.mapImage.setSendInputCB(self.sendInput)
        self.mapImage.setMarkerPoint(TX, TY)

        self.uiImage = UIView()
        self.uiImage.setSendSliderCB(self.sendSlider)

        # サイズポリシーを取得 （サイズ比を Map:UI=2:1 に）
        self.sizePolicy_map = self.mapImage.sizePolicy()
        self.sizePolicy_ui = self.uiImage.sizePolicy()
        # ストレッチするサイズ比をセット
        self.sizePolicy_map.setHorizontalStretch(2)
        self.sizePolicy_ui.setHorizontalStretch(1)
        # サイズポリシーをセット
        self.mapImage.setSizePolicy(self.sizePolicy_map)
        self.uiImage.setSizePolicy(self.sizePolicy_ui)

        layout = QHBoxLayout()   # 水平方向にUIを並べる
        layout.addWidget(self.mapImage)
        layout.addWidget(self.uiImage)

        mainWidget = QWidget()
        mainWidget.setLayout(layout)
        self.setCentralWidget(mainWidget)
        self.show()

    ## サーバ(CA)からROSの情報を取得
    def recvWebMessageCB(self, message) :
        jsoncmd = json.loads(message)
        if 'mpos_x' in jsoncmd.keys():
            print("jsoncmd['mpos_x']")


    def setImage(self, img) :
        self.mapImage.setImage(img)
        self.mapImage.repaint()

    def sendInput(self, x, y) :
        jsonparam = {'x' : x, 'y' : y}
        jsoncmd = {'cmd' : 'position', 'param' : json.dumps(jsonparam)}
        jsonmsg = {'targets' : 'OP002SA', 'message' : json.dumps(jsoncmd)}
        # jsonmsg = {'targets' : 'CA001', 'message' : json.dumps(jsoncmd)}
        self.capfWebSocket.send(json.dumps(jsonmsg))

    def sendSlider(self, z) :
        jsonparam = {'z' : z}
        jsoncmd = {'cmd' : 'position', 'param' : json.dumps(jsonparam)}
        jsonmsg = {'targets' : 'OP002SA', 'message' : json.dumps(jsoncmd)}
        # jsonmsg = {'targets' : 'CA001', 'message' : json.dumps(jsoncmd)}
        self.capfWebSocket.send(json.dumps(jsonmsg))

if __name__ == '__main__' :
    app = QApplication(sys.argv)
    image = QImage('map_tokyo9f.png')
    window = mainView()
    window.setImage(image)
    app.exec_()
