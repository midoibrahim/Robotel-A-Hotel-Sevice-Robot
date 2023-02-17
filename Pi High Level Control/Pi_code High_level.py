#!/usr/bin/env python3
import serial
import time
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
from http.server import BaseHTTPRequestHandler,HTTPServer
import smtplib, ssl
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

Request=None
port = 465  # For SSL
smtp_server = "smtp.gmail.com"
sender_email = "robotel.mailservice@gmail.com"  # Enter your address
receiver_email = "ahmedbogy2016@gmail.com"  # Enter receiver address
password = "Robotelraspi"
roomnum=0
move=False
#--- Define Tag
id_to_find  = 0
marker_size  = 14 #- [cm]

def send_mail(n):
    global text
    message = MIMEMultipart("alternative")
    message["Subject"] = "Robotel Confirmation"
    message["From"] = sender_email
    message["To"] = receiver_email

    head="""
    Room number: """
    # Create the plain-text and HTML version of your message
    text = """
    Your request will be deleievered soon...
    Hope you enjoy your stay :)

    You Requested:
    """
    if n==1:
        text=""" Your request arrived and is waiting on your door"""
    else:
        text=head+roomnum+text+Request
    print(text)

    # Turn these into plain/html MIMEText objects
    part1 = MIMEText(text, "plain")


    # Add HTML/plain-text parts to MIMEMultipart message
    # The email client will try to render the last part first
    message.attach(part1)

    #Create secure connection with server and send email
    context = ssl.create_default_context()
    with smtplib.SMTP_SSL("smtp.gmail.com", 465, context=context) as server:
         server.login(sender_email, password)
         server.sendmail(sender_email, receiver_email, message.as_string())
    return 1

class RequestHandler_httpd(BaseHTTPRequestHandler):
    def do_GET(self):
        global Request
        global roomnum
        global receiver_email
        global id_to_find
        Request=self.requestline
        Request=Request[5:int(len(Request)-9)]
       # print(Request)
        Request=Request.replace('%20',' ')
        roomnums=[int(s) for s in Request.split() if s.isdigit()]
        roomnum=str(roomnums[0])
        Request=Request[len(roomnum)+1:]
        print(Request)
        print(roomnum)
        receiver_email=Request.split('-',1)[0]
        Request=Request.split('-',1)[1]
        print("receiver_email")
        print(receiver_email)
        send_mail(0)
        messagetosend=bytes(text,"utf")
        self.send_response(200)
        self.send_header('Content-Type','text/[plain')
        self.send_header('Content-Length',len(messagetosend))
        self.end_headers()
        self.wfile.write(messagetosend)
        return

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

def vision(roomnum):
        id_to_find  = int(roomnum)
        ser.flushInput()
        ser.write(b't')
        print("t")
        time.sleep(3)
        ser.flushOutput()
        ser.write(str(id_to_find).encode('utf-8'))
        print(id_to_find)
        time.sleep(2)

        global move
        #--- 180 deg rotation matrix around the x axis
        R_flip  = np.zeros((3,3), dtype=np.float32)
        R_flip[0,0] = 1.0
        R_flip[1,1] =-1.0
        R_flip[2,2] =-1.0

        #--- Define the aruco dictionary
        aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters  = aruco.DetectorParameters_create()
        global cap
        #--- Capture the videocamera (this may also be a video or a picture)
        cap = cv2.VideoCapture(0)
        #-- Set the camera size as the one it was calibrated with
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        cap.set( cv2.CAP_PROP_FPS, 30 )
        pitch_camera=15;
        while True:
                ret, frame = cap.read()
                global succ
                #-- Convert in gray scale
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
                #-- Find all the aruco markers in the image
                corners, ids, rejected = aruco.detectMarkers(image=gray,dictionary=aruco_dict, parameters=parameters)
                #-- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners,ids)
                cv2.imshow('Vision',frame)
                cv2.waitKey(1)
                if move:
                    for i in range(20):
                        cap.grab()
                    if(ser.in_waiting>0):
                        line = ser.readline().decode('utf-8').rstrip()
                        print("i recieved")
                        print(line)
                        ser.flushOutput()
                        if(line == 'a'):
                            send_mail(1)
                        elif(line == 'w'):
                            id_to_find=0
                            move=False
                            succ=False


                if ids is not None and id_to_find in ids and not move:
                    id_corner=np.where(ids==id_to_find)[0][0]
                    #print(id_corner)
                    ser.write(b's')
                    time.sleep(0.5)
                    ret = aruco.estimatePoseSingleMarkers(corners[id_corner], marker_size, camera_matrix, camera_distortion)


                    #-- Unpack the output, get only the first
                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

                    #-- Obtain the rotation matrix tag->camera
                    R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
                    R_tc    = R_ct.T

                    #-- Now get Position and attitude f the camera respect to the marker
                    pos_camera = -R_tc*np.matrix(tvec).T

                    str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])


                     #-- Get the attitude of the camera respect to the frame
                    roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                    str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),math.degrees(yaw_camera))
                    #print(str_attitude)
                    distancez=int(math.sqrt(pos_camera[2]*pos_camera[2]))

                    if id_to_find==0 and not move:
                        if(abs(math.degrees(pitch_camera)) > 8):
                            if(math.degrees(pitch_camera) > 8):
                                for i in range(int(abs(math.degrees(pitch_camera)/8))):
                                    ser.write(b'l')
                                    print("l");
                                    time.sleep(0.5)
                            elif( math.degrees(pitch_camera) < -8):
                                for i in range(int(abs(math.degrees(pitch_camera)/8))):
                                    ser.write(b'r')
                                    print("r");
                                    time.sleep(0.5)
                        else:
                            ser.flushInput()
                            ser.write(str(distancez-20).encode('utf-8'))
                            time.sleep(3)
                            ser.flushOutput()
                            ser.write(b'f')
                            print("f")
                            time.sleep(3)
                            move=True
                    else:
                        if(distancez > 200):
                            if(not move) :
                                ser.flushInput()
                                print(distancez)
                                ser.write(str(distancez).encode('utf-8'))
                                time.sleep(3)
                                move=True
                                ser.flushOutput()
                                ser.write(b'f')
                                print("f")

def server():
        global httpd
        server_address_httpd= ('192.168.43.222',8080)
        httpd = HTTPServer(server_address_httpd,RequestHandler_httpd)
        print('Starting Server')
        while (roomnum == 0):
            httpd.handle_request()

if __name__ == "__main__":
    global ser
    try:
            ser = serial.Serial('/dev/ttyACM0', 115200)
            ser.flush()
            server()
            #--- Get the camera calibration path
            calib_path  = "/home/pi/Desktop/Robotel/Pi-opencv/"
            camera_matrix   = np.loadtxt(calib_path+'cameraMatrix2.txt', delimiter=',')
            camera_distortion   = np.loadtxt(calib_path+'cameraDistortion2.txt', delimiter=',')
            time.sleep(5);
            vision(roomnum)
            #while True :
            #  ser.write(b'f')
            # print('f')
            #time.sleep(1)
            #if ser.in_waiting > 0:
                #  line = ser.readline().decode('utf-8').rstrip()
                #  print(line)

    except KeyboardInterrupt:
        ser.write(b's')
        time.sleep(1)
        #cap.release()
        cv2.destroyAllWindows()
        httpd.server_close()