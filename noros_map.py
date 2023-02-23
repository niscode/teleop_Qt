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

# import rospy
# from geometry_msgs.msg import Twist, Point
# from visualization_msgs.msg import Marker

POS_X = POS_Y = POS_Z = ORI_Z = ORI_W = VOLTAGE = 0.0
MPOS_X = []
MPOS_Y = []
MORI_Z = []
MORI_W = []

def pose_callback(msg_pose):
    global POS_Z
    POS_Z = msg_pose.z

def cylinder_move(goal_h):
    # 現在地点と目標地点を比較して上下のどちらに動くか決める
    rospy.wait_for_message("cylinder_pos", Point, timeout=None)
    current_h = POS_Z
    vel_h = Twist()
    print("[ROS][cylinder]   Current_height: " + str(POS_Z) + " / Goal_height: " + str(goal_h))
    if current_h < goal_h :
        vel_h.linear.z = 0.1
        print("[ROS][cylinder]   上へ参ります。")
    
    if current_h > goal_h :
        vel_h.linear.z = -0.1
        print("[ROS][cylinder]   下へ参ります。")

    while not rospy.is_shutdown():
        ## 02_目標地点を与えてそこまで動かす
        if (goal_h-10 < POS_Z <= goal_h):
            vel_h.linear.z = 0
            pub.publish(vel_h)
            print("[ROS][cylinder]   End_height: " + str(POS_Z))
            break
        pub.publish(vel_h)
        rate.sleep()

## 目標位置となるmarkerを取得
def marker_callback(msg_marker):
    global MPOS_X, MPOS_Y, MORI_Z, MORI_W
    MPOS_X.append(msg_marker.pose.position.x)
    MPOS_Y.append(msg_marker.pose.position.y)
    MORI_Z.append(msg_marker.pose.orientation.z)
    MORI_W.append(msg_marker.pose.orientation.w)
    # print("[ ]" + str(msg_marker.id))

def spinOnce(sub, topic, type):
    rospy.wait_for_message(topic, type, timeout=None)
    sub.unregister();   # 1度だけ呼び出してからsubscriberを停止する

## 電圧値を取得
def voltage_callback(msg_voltage):
    global VOLTAGE
    VOLTAGE = msg_voltage.data



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
        self.resize(800, 640)
        self.mapImage = MapView()
        self.sendMarker = None

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
        self.capfWebSocket.setMessageCallback(self.recvWebMessageCB)    # クライアント(OP)からROSの情報を取得
        if not self.capfWebSocket.connect():
            QMessageBox.information(
                None, "Error", "WebSocket Connect Error : " + self.websockurl, QMessageBox.Ok)
            self.close()
            sys.exit()

    # クライアント(OP)からROSの情報を取得
    def recvWebMessageCB(self, message) :
        jsoncmd = json.loads(message)
        if 'cmd' in jsoncmd.keys() and 'param' in jsoncmd.keys() :
            command = jsoncmd['cmd']
            params  = json.loads(jsoncmd['param'])
            if command == 'position' :
                if 'x' in params.keys() and 'y' in params.keys() :
                    x, y = params['x'], params['y']
                    print(x, y)
                    self.mapImage.setMarkerPoint(x,y)
                if 'z' in params.keys() :
                    z = params['z']
                    cylinder_move(z)

    def setImage(self, img):
        self.mapImage.setImage(img)
        self.mapImage.repaint()

    # 最初に1度だけ送信するROSの情報   waypointの位置
    def sendMarker():
        print("[ROS][waypoint]   Markers_pos published.")
        spinOnce(sub_waypoint, "waypoint", Marker)
        ## 辞書型に変換
        marker_dict = {
            "mpos_x": list(dict.fromkeys(MPOS_X)),
            "mpos_y": list(dict.fromkeys(MPOS_Y)),
            "mori_z": list(dict.fromkeys(MORI_Z)),
            "mori_w": list(dict.fromkeys(MORI_W))
        }
        marker_json = json.dumps(marker_dict)
        print("[ROS][JSON Marker_pos] " + str(marker_json))
        jsonmsg = {'targets' : 'CA003', 'message' : marker_json}
        # jsonmsg = {'targets' : 'CA001', 'message' : json.dumps(jsoncmd)}
        self.capfWebSocket.send(json.dumps(jsonmsg))


if __name__ == '__main__' :
    ## ROS設定
    # rospy.init_node('capf')
    # pub = rospy.Publisher('rover_twist', Twist, queue_size=10)
    # sub_cylinderPos = rospy.Subscriber("cylinder_pos", Point, pose_callback)
    # sub_waypoint = rospy.Subscriber("waypoint", Marker, marker_callback)
    # rate = rospy.Rate(10)

    ## GUI設定
    app = QApplication(sys.argv)
    image = QImage('map_tokyo9f.png')
    window = mainView()
    # window.sendMarker   # waypointの位置を送信
    window.setImage(image)
    app.exec_()