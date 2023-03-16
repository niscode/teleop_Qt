# -*- config:utf-8 -*-


import sys
import time
from threading import Thread
import requests
import websocket
import json


class WebSocketCapf() :
    def __init__(self, loginurl, id, passwd, websockurl) :
        self.loginurl = loginurl
        self.id = id
        self.passwd = passwd
        self.websockurl = websockurl
        self.token = None
        self.wss = None
        self.wssThread = None
        self.messageCallback = None
        self.connected = False

    def isactive(self) :
        return self.connected

    def getUrl(self) :
        return self.websockurl

    def getLoginurl(self) :
        return self.loginurl

    def getLoginToken(self) :
        return self.token

    def setMessageCallback(self, func) :
        self.messageCallback = func

    def on_messageCB(self, message) :
        if message == 'something' : return
        if not self.messageCallback is None :
            self.messageCallback(message)

    def on_errorCB(self, err):
        pass
        print("Error :", err)

    def on_openCB(self):
        self.connected = True
        pass
        #print("opening")

    def on_closeCB(self):
        self.connected = False
        #print("close")

    def send(self, message) :
        #print("SEND :", message)
        #print(self.wss.sock.connected)
        if not self.isactive() :
            print("reconnect")
            self.connect()
        if self.isactive():
            self.wss.send(message)
        else :
            print('cannnot send')

    def connect(self) :
        closed = False
        def on_message(ws, message) :
            self.on_messageCB(message)
        
        def on_error(ws, error) :
            self.on_errorCB(error)
        
        def on_open(ws) :
            self.on_openCB()

        def on_close(ws, status_code, close_msg) :
            closed = True
            #print(status_code, close_msg)
            self.on_closeCB()

        result = False
        payload = {"name": self.id, "password": self.passwd}

        #login
        self.token = None
        #print(self.loginurl, payload)
        try :
            r = requests.post(self.loginurl, params=payload)
            #print(r)
        except :
            print('Error : ' + self.loginurl)
            return False

        if r.status_code != 200:
            print('Error : ' + self.loginurl + " response : " + str(r.status_code))
            return False

        if r.text is None :
            return False
        jdic = json.loads(r.text)
        if "authorisation" in jdic.keys():
            authorisation = jdic["authorisation"]
            if authorisation is None :
                return False
            if "token" in authorisation.keys():
                self.token = authorisation["token"]
                if self.token is None :
                    return False
                
                dummyws = websocket.WebSocket()
                try:
                    dummyws.connect(self.websockurl + '?token=' + self.token)
                except :
                    print("Error : " + self.websockurl + "  connect Error")
                    return False
                dummyws.close()
                
                self.wss = websocket.WebSocketApp(self.websockurl + '?token=' + self.token,
                                                  on_message = on_message,
                                                  on_error = on_error,
                                                  on_open = on_open,
                                                  on_close = on_close  )
                
                self.wssThread = Thread(target = self.wss.run_forever)
                self.wssThread.daemon = True
                self.wssThread.start()
                while not self.connected and not closed:
                    time.sleep(0.01)

                #print('closed =', closed)
                if not closed :
                    result = True
    
        return result


# test
if __name__ == '__main__' :
    def cb(message):
        print('sutekina ', message)


    #websocket.enableTrace(True)
    wssock = WebSocketCapf('https://atr-dev02.ca-platform.org/api/login',
                           'CA001', 'CA001', 'wss://atr-dev02-websocket.ca-platform.org')
    wssock.setMessageCallback(cb)
    if not wssock.connect() :
        print('cannot connect websocket')
        sys.exit(0)
    #wssock.send('test'.encode())

    while True :
        time.sleep(1)

