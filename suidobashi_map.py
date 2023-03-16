# -*- config:utf-8 -*-

import sys
import os

import random

from threading import Thread
from PySide2.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout, QGraphicsScene, QGraphicsView, QWidget, QLabel, QTextEdit, QTextBrowser, QLineEdit, QSpacerItem, QSizePolicy, QMessageBox)
from PySide2 import QtCore
from PySide2.QtCore import QTimer, QSize, Qt
from PySide2.QtGui import QPixmap, QImage, QPainter, QBrush

import WebSocketCapf as cwebsock
import json

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point, Pose
from visualization_msgs.msg import Marker
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# myID = 'CA003'
myID = 'CA004'
targetID = 'OP002SA'

POS_X = POS_Y = POS_Z = ORI_Z = ORI_W = VOLTAGE = C_POS = 0.0
isCylinder = False
MPOS_X = []
MPOS_Y = []
MORI_Z = []
MORI_W = []
WAYPOINT_DICT = {}

def cylinder_move(c_pos):
    global isCylinder
    # 現在地点と目標地点を比較して上下のどちらに動くか決める
    rospy.wait_for_message("cylinder_pos", Point, timeout=None)
    current_h = POS_Z
    vel_h = Twist()
    print("[ROS][cylinder]   Current_height: " + str(C_POS) + " / Ideal_height: " + str(c_pos))
    if current_h < c_pos :
        vel_h.linear.z = 0.1
        print("[ROS][cylinder]   上へ参ります。")
    
    if current_h > c_pos :
        vel_h.linear.z = -0.1
        print("[ROS][cylinder]   下へ参ります。")

    while not rospy.is_shutdown():
        ## 02_目標地点を与えてそこまで動かす
        if (c_pos-10 < POS_Z <= c_pos):
            vel_h.linear.z = 0
            pub.publish(vel_h)
            print("[ROS][cylinder]   End_height: " + str(C_POS))
            isCylinder = True
            break
        print(';;;;;;;; now adjusting cylinder ;;;;;;;;')
        pub.publish(vel_h)
        rate.sleep()

def goal_pose(num): 
    client.cancel_goal()
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = WAYPOINT_DICT['mpos_x'][num]
    goal_pose.target_pose.pose.position.y = WAYPOINT_DICT['mpos_y'][num]
    goal_pose.target_pose.pose.orientation.z = WAYPOINT_DICT['mori_z'][num]
    goal_pose.target_pose.pose.orientation.w = WAYPOINT_DICT['mori_w'][num]
    result = client.send_goal(goal_pose)
    print("///// ナビ目的地 x ///// " + str(goal_pose.target_pose.pose.position.x))
    print(MPOS_X)
    if result:
        print(result)
        rospy.loginfo("Goal execution done!")
    
    return goal_pose



# Waypointを取得
def marker_callback(msg_marker):
    global MPOS_X, MPOS_Y, MORI_Z, MORI_W
    MPOS_X.append(msg_marker.pose.position.x)
    MPOS_Y.append(msg_marker.pose.position.y)
    MORI_Z.append(msg_marker.pose.orientation.z)
    MORI_W.append(msg_marker.pose.orientation.w)

def spinOnce(sub, topic, type):
    rospy.wait_for_message(topic, type, timeout=None)
    sub.unregister();   # 1度だけ呼び出してからsubscriberを停止する

# 電圧値を取得
def voltage_callback(msg_voltage):
    global VOLTAGE
    VOLTAGE = msg_voltage.data

# 自己位置を取得
def pose_callback(msg_pose):
    global POS_X, POS_Y, ORI_Z, ORI_W
    POS_X = msg_pose.position.x
    POS_Y = msg_pose.position.y
    ORI_Z = msg_pose.orientation.z
    ORI_W = msg_pose.orientation.w

# シリンダー高さ
def point_callback(msg_point):
    global POS_Z
    POS_Z = msg_point.z

class MapView(QGraphicsView):
    def __init__(self, parent=None) :
        super(MapView, self).__init__(parent=parent)
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.backgroundImage = None
        self.markerPoint = [0, 0]

    def setImage(self, bgImage):
        self.backgroundImage = bgImage
        sz = self.backgroundImage.size()
        self.markerPoint = [sz.width() // 2, sz.height() // 2]

    def setMarkerPoint(self, x, y):
        self.markerPoint = [x, y]
        self.update()
    
    def paintEvent(self,event):
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
            #pixmap = pixmap.scaled(sz.width(), sz.height())
            self.scene.clear()
            self.scene.addPixmap(pixmap)
            self.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)


class mainView(QMainWindow):
    def __init__(self):
        super().__init__()
        self.mapImage = MapView()
        self.resize(800,600)

        # waypointを送るボタン
        self.sendButton = QPushButton('Waypointを送る')
        self.sendButton.clicked.connect(self.sendWaypoints)

        layout = QVBoxLayout()
        layout.addWidget(self.mapImage)
        layout.addWidget(self.sendButton)

        mainWidget = QWidget()
        mainWidget.setLayout(layout)
        self.setCentralWidget(mainWidget)
        self.show()

        self.messageBuffer = []

        self.loginurl = 'https://atr-dev02.ca-platform.org/api/login'
        self.websockurl = 'wss://atr-dev02-websocket.ca-platform.org'
        # self.loginurl = 'https://atr-dev01.ca-platform.org/api/login'
        # self.websockurl = 'wss://atr-dev01-websocket.ca-platform.org'
        self.accountid = myID
        self.accountpswd = myID
        self.capfWebSocket = cwebsock.WebSocketCapf(self.loginurl, self.accountid, self.accountpswd, self.websockurl)
        self.capfWebSocket.setMessageCallback(self.recvWebMessageCB)    # クライアント(OP)からROSの情報を取得
        if not self.capfWebSocket.connect():
            QMessageBox.information(
                None, "Error", "WebSocket Connect Error : " + self.websockurl, QMessageBox.Ok)
            self.close()
            sys.exit()

        self.sendWaypoints()   ## 起動時に一度だけwaypointの位置をクライアント側へ送信

        self.ROSthread = ROSProcess(self)
        self.ROSthread.signal.connect(self.sendInfo)    ## 別スレッドで毎秒 電圧値と自己位置をクライアント側へ送信
        self.ROSthread.start()

        self.timer = QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.messageSession)
        self.timer.start()


    ### クライアント(OP)からROSの情報を取得
    def recvWebMessageCB(self, message) :
        self.messageBuffer.append(message)

    def messageSession(self) :
        if len(self.messageBuffer) > 0 :
            message = self.messageBuffer.pop(0)
            self.doMessage(message)

    def doMessage(self, message) :
        jsoncmd = json.loads(message)
        if 'request' in jsoncmd.keys() :
            command = jsoncmd['request']
            params  = json.loads(jsoncmd['param'])
            if command == 'other':
                if 'msg' in params.keys() :
                    request = params['msg']
                    print(request)
                    if request == 'waypoint':
                        self.sendWaypoints()

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
                    self.cylinderThread = Thread(target=cylinder_move(z))
                    self.cylinderThread.start()
                # 目的地のインデックスを取得
                if 'goal_num' in params.keys() :
                    goal_num = params['goal_num']
                    print ('\033[32m' + str(goal_num+1) + ' に向けてナビゲーションを開始します。' + '\033[0m')
                    self.sendWaypoints()
                    goal_pose(goal_num)


    def setImage(self, img):
        self.mapImage.setImage(img)
        self.mapImage.repaint()

    # waypointの位置   最初に1度だけ送信する
    def sendWaypoints(self):
        global WAYPOINT_DICT
        # print("[ROS][waypoint]   Waypoints_pos published.")
        spinOnce(sub_waypoint, "waypoint", Marker)
        ## 辞書型に変換
        WAYPOINT_DICT = {
            "mpos_x": list(dict.fromkeys(MPOS_X)),
            "mpos_y": list(dict.fromkeys(MPOS_Y)),
            "mori_z": list(dict.fromkeys(MORI_Z)),
            "mori_w": list(dict.fromkeys(MORI_W))
        }
        waypoint_json = json.dumps(WAYPOINT_DICT)
        print("[ROS][JSON Waypoint_pos] " + str(waypoint_json))
        jsonmsg = {'targets' : targetID, 'message' : waypoint_json}
        sendRos = self.capfWebSocket.send(json.dumps(jsonmsg))
        return sendRos

    # 電圧値と自己位置   常に送信する
    def sendInfo(self):
        sub_voltage = rospy.Subscriber("voltage", Float32, voltage_callback)
        sub_robotPose = rospy.Subscriber("robot_pose", Pose, pose_callback)
        sub_cylinderPos = rospy.Subscriber("cylinder_pos", Point, point_callback)
        rate.sleep()

        # spinOnce(sub_voltage, "voltage", Float32)
        # spinOnce(sub_robotPose, "robot_pose", Pose)
        # spinOnce(sub_cylinderPos, "cylinder_pos", Point)

        info_dict = {
            "voltage": VOLTAGE,
            "pos_x": POS_X,
            "pos_y": POS_Y,
            "pos_z": POS_Z,
            "ori_z": ORI_Z,
            "ori_w": ORI_W
        }
        info_json = json.dumps(info_dict)
        jsonmsg = {'targets' : targetID, 'message' : info_json}
        print("[ROS][JSON info] " + str(info_json))
        sendRos = self.capfWebSocket.send(json.dumps(jsonmsg))
        return sendRos

    # シリンダーの動作完了を通知
    def sendCylinderInfo(self):
        if isCylinder:
            cylinder_dict = {"cylinder_move": "done"}
            info_json = json.dumps(cylinder_dict)
            jsonmsg = {'targets' : targetID, 'message' : info_json}
            print("[ROS][JSON info] " + str(info_json))
            sendRos = self.capfWebSocket.send(json.dumps(jsonmsg))
            isCylinder = False
            return sendRos


class ROSProcess(QtCore.QThread):
    signal = QtCore.Signal(int)
    def run(self):
        n = 0
        while not rospy.is_shutdown():
            n += 1
            self.signal.emit(n)
            rospy.sleep(1.0)    # 1秒おきにTelecoの現在の姿勢を送信するスレッド
            # rate.sleep()


if __name__ == '__main__' :
    ## ROS設定
    rospy.init_node('capf')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)    #サーバ名，型ともに利用するサーバと一致させる
    client.wait_for_server()    #サーバーの応答を待つ

    pub = rospy.Publisher('rover_twist', Twist, queue_size=10)
    sub_waypoint = rospy.Subscriber("waypoint", Marker, marker_callback)
    rate = rospy.Rate(10)

    ## GUI設定
    app = QApplication(sys.argv)
    image = QImage('map_atr3f.png')
    window = mainView()
    window.setImage(image)
    app.exec_()