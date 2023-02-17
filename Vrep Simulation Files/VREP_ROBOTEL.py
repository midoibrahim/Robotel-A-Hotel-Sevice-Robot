import cv2
from cv2 import aruco
import math
import decimal
import sim
import sys
import time
import numpy as np
from http.server import BaseHTTPRequestHandler,HTTPServer
import smtplib, ssl
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 30.0, (512, 512))

Request=None
port = 465  # For SSL
smtp_server = "smtp.gmail.com"
sender_email = "robotel.mailservice@gmail.com"  # Enter your address
receiver_email = "ahmedbogy2016@gmail.com"  # Enter receiver address
password = "Robotelraspi"
roomnum=0

print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 20000, True, True, 5000, 5)  # Connect to CoppeliaSim
if clientID != -1:
    print('Connected to remote API server')
    # Get the object Handles from Vrep scene
    # We will use the handles of the motors and the camera
    errorCode, robotel = sim.simxGetObjectHandle(clientID, 'ROBOTEL', sim.simx_opmode_blocking)
    errorCode, leftMotor = sim.simxGetObjectHandle(clientID, 'Left_Wheel', sim.simx_opmode_blocking)
    errorCode, rightMotor = sim.simxGetObjectHandle(clientID, 'Right_Wheel', sim.simx_opmode_blocking)
    errorCode, cam = sim.simxGetObjectHandle(clientID, 'Camera', sim.simx_opmode_blocking)
    errorCode, compartment = sim.simxGetObjectHandle(clientID, 'Compartment', sim.simx_opmode_blocking)
    errorCode, frontProx = sim.simxGetObjectHandle(clientID, 'Front_Prox', sim.simx_opmode_blocking)
    errorCode, leftProx = sim.simxGetObjectHandle(clientID, 'Left_Prox', sim.simx_opmode_blocking)
    errorCode, rightProx = sim.simxGetObjectHandle(clientID, 'Right_Prox', sim.simx_opmode_blocking)
    errorCode, home = sim.simxGetObjectHandle(clientID, 'home', sim.simx_opmode_blocking)
    errorCode, bill = sim.simxGetObjectHandle(clientID, 'Bill', sim.simx_opmode_blocking)
    # initialize the streaming variables
    errorCode, resolution, image = sim.simxGetVisionSensorImage(clientID, cam, 0, sim.simx_opmode_streaming)
    errorCode, orie = sim.simxGetObjectOrientation(clientID, robotel, -1, sim.simx_opmode_streaming)
    # --- Define the aruco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()

#--- Define Tag
global id_to_find
id_to_find=-1
global door
door="doorJoint"
global doorMotor
doorMotor=0
marker_size= 15 #- [cm]
global state
state="Home"

global displayWindow

#--- Get the camera calibration path
calib_path=  ''
camera_matrix= np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion= np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
font= cv2.FONT_HERSHEY_PLAIN

#--- 180 deg rotation matrix around the x axis
R_flip= np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

check=0

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
#send mail for the guest
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
#initialize server to receive mobile app requests
def server():
    global httpd
    server_address_httpd = ('192.168.1.7', 8080)
    httpd = HTTPServer(server_address_httpd, RequestHandler_httpd)
    print('Starting Server')
    while (roomnum == 0):
        httpd.handle_request()
#requests handler
class RequestHandler_httpd(BaseHTTPRequestHandler):
    def do_GET(self):
        global Request
        global roomnum
        global receiver_email
        global id_to_find
        global displayWindow
        Request=self.requestline
        Request=Request[5:int(len(Request)-9)]
        Request=Request.replace('%20',' ')
        roomnums=[int(s) for s in Request.split() if s.isdigit()]
        roomnum=str(roomnums[0])
        Request=Request[len(roomnum)+1:]
        print(Request)
        print(roomnum)
        id_to_find=int(roomnum)
        receiver_email=Request.split('-',1)[0]
        Request=Request.split('-',1)[1]
        print("receiver_email")
        print(receiver_email)
        req_print="Request From Room number "+str(roomnum)
        errorCode,displayWindow,UI=sim.simxDisplayDialog(clientID,"Request",req_print,sim.sim_dlgstyle_ok,"",None,None,sim.simx_opmode_blocking)
        send_mail(0)
        messagetosend=bytes(text,"utf")
        self.send_response(200)
        self.send_header('Content-Type','text/[plain')
        self.send_header('Content-Length',len(messagetosend))
        self.end_headers()
        self.wfile.write(messagetosend)
        return
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
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
#Get Robotel Orientation
def getRobotelOrientation():
    orientation=0
    errorCode=-1
    while (1):
        errorCode, orie = sim.simxGetObjectOrientation(clientID, robotel, -1, sim.simx_opmode_buffer)
        if errorCode == 0:
            break
    orientation=orie[2]
    if orie[2]<0:
        orientation=math.pi*2+orie[2]
    return orientation
#Rotate function for Robotel
def rotate90(direction,angle=0):
    dir=1
    count=0
    global orie
    if direction=="right":
        dir=-1
    orie=getRobotelOrientation()
    ang = orie
    if angle==0:
        angle = ang+(math.pi/2)*dir
    if angle>math.pi*2:
        angle=angle-math.pi*2
    elif angle<0:
        angle=math.pi*2+angle
    while(1):
        orie=getRobotelOrientation()
        rfactor=5
        leftVel = -(rfactor * abs(angle - orie))*dir
        rightVel = (rfactor * abs(angle - orie))*dir
        errorCode= sim.simxSetJointTargetVelocity(clientID, leftMotor, leftVel, sim.simx_opmode_oneshot)
        errorCode= sim.simxSetJointTargetVelocity(clientID, rightMotor, rightVel, sim.simx_opmode_oneshot)
        if (np.absolute(orie-angle) < 0.001):
            leftVel = 0
            rightVel = 0
            count+=1
            errorCode= sim.simxSetJointTargetVelocity(clientID, leftMotor, 0, sim.simx_opmode_oneshot)
            errorCode= sim.simxSetJointTargetVelocity(clientID, rightMotor, 0, sim.simx_opmode_oneshot)
            if count>1000:
                return
#Set Motors Speed
def setMotorsVelocity(vleft,vright):
    sim.simxSetJointTargetVelocity(clientID, leftMotor, vleft, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, rightMotor, vright, sim.simx_opmode_blocking)
#Get Ultrasonic Reading
def getUltrasoincReading(sensor):
    detect = sim.simxReadProximitySensor(clientID, sensor, sim.simx_opmode_blocking)
    if detect[1] == True:
        obj_distance = math.sqrt(pow(detect[2][0], 2) + pow(detect[2][1],2))
        return detect[1],obj_distance
    else:
        return detect[1],0
#Open room door
def openDoor():
    global doorMotor
    while (1):
        errorCode = sim.simxSetJointTargetPosition(clientID, doorMotor, (-math.pi/2), sim.simx_opmode_blocking)
        if errorCode == 0:
            break
#close room door
def closeDoor():
    global doorMotor
    while (1):
        errorCode = sim.simxSetJointTargetPosition(clientID, doorMotor, 0, sim.simx_opmode_blocking)
        if errorCode == 0:
            break
#Open compartment door
def openCompartment():
    while(1):
        errorCode=sim.simxSetJointTargetPosition(clientID, compartment, 58 * math.pi / 180, sim.simx_opmode_blocking)
        if errorCode==0:
            break
    time.sleep(28)
#close compartment door
def closeCompartment():
    while(1):
        errorCode=sim.simxSetJointTargetPosition(clientID, compartment, 0 * math.pi / 180, sim.simx_opmode_blocking)
        if errorCode == 0:
            break
    time.sleep(28)
#Deliver the package
def deliverPackage():
    global id_to_find
    global state
    state="Package Delivered"
    if (id_to_find % 2) == 0:
        time.sleep(1.5)
        rotate90("right")
        errorCode, displayWindow, UI = sim.simxDisplayDialog(clientID, "Status", "Place your card to get Your Package",sim.sim_dlgstyle_ok, "", None, None,sim.simx_opmode_blocking)
        time.sleep(2)
        openDoor()
        openCompartment()
        print(state)
        errorCode,displayWindow,UI=sim.simxDisplayDialog(clientID,"Status",state,sim.sim_dlgstyle_ok,"",None,None,sim.simx_opmode_blocking)
        sim.simxAddStatusbarMessage(clientID, state, sim.simx_opmode_blocking)
        closeDoor()
        closeCompartment()
        rotate90("right",-math.pi / 2)
    else:
        time.sleep(1.5)
        rotate90("left")
        errorCode, displayWindow, UI = sim.simxDisplayDialog(clientID, "Status", "Place your card to get Your Package",sim.sim_dlgstyle_ok, "", None, None,sim.simx_opmode_blocking)
        time.sleep(2)
        openDoor()
        openCompartment()
        print(state)
        errorCode,displayWindow,UI=sim.simxDisplayDialog(clientID,"Status",state,sim.sim_dlgstyle_ok,"",None,None,sim.simx_opmode_blocking)
        time.sleep(5)
        sim.simxAddStatusbarMessage(clientID, state, sim.simx_opmode_blocking)
        closeDoor()
        closeCompartment()
        rotate90("left",-math.pi / 2)
    id_to_find=0
    state="Heading Home"
    print(state)
    sim.simxAddStatusbarMessage(clientID, state, sim.simx_opmode_blocking)


def getRelativePosition(J):
    errorCode, id = sim.simxGetObjectHandle(clientID, J, sim.simx_opmode_blocking)
    position = sim.simxGetObjectPosition(clientID, cam, id, sim.simx_opmode_blocking)
    print(position[1])
    return (position[1][0]*100), (position[1][1]*100), (position[1][2]*100)

while(1):
    errorCode,check=sim.simxGetObjectPosition(clientID,robotel,home,sim.simx_opmode_blocking)
    check=abs(check[2])
    if errorCode==0:
        break


#Loop for interaction with the Vrep simulation
while (1):
    if state=="Home":
        print("Please Enter The Room Number")
        sim.simxAddStatusbarMessage(clientID, "Please Enter The Room Number", sim.simx_opmode_blocking)
        server()
        print(id_to_find)
        if id_to_find>0:
            state = "Delivering Package"
            door=door+str(id_to_find)
            #print(door)
            errorCode, doorMotor = sim.simxGetObjectHandle(clientID, door, sim.simx_opmode_blocking)
            #print(doorMotor)
            setMotorsVelocity(6,6)
            print(state)
            sim.simxAddStatusbarMessage(clientID, state, sim.simx_opmode_blocking)
         else:
             print("Incorrect Value")
             sim.simxAddStatusbarMessage(clientID,"Incorrect Value",sim.simx_opmode_blocking)
    if(state!="Home"):
        # Get the image from vision sensor
        errorCode, resolution, image = sim.simxGetVisionSensorImage(clientID, cam, 0, sim.simx_opmode_buffer)
        # The image is sent as 1D array we will need to reshape it to HxWx3 for RGB images
        # First we convert it to a numpy array
        # sim.simxAuxiliaryConsoleShow(clientID,displayWindow,0,sim.simx_opmode_blocking)
        img = np.array(image, dtype=np.uint8)
        # Check if the image is read successfully
        if resolution:
            # Reshape the numpy array
            img = img.reshape([resolution[0], resolution[1], 3])
            # openCV deals with images as BGR by default and the sent image is in RGB
            # Here we convert it to BGR
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            img = cv2.flip(img, 0)
            #cv2.imshow('frame', img)

            # -- Convert in gray scale
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)  # -- remember, OpenCV stores color images in Blue, Green, Red
            # -- Find all the aruco markers in the image
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
            # Drawing rectangle around marker, Specifying its ID
            img_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)
            if state != "Heading Home":
                detect,obj_distance=getUltrasoincReading(frontProx)
                while detect==True and obj_distance<0.3:
                    detect, obj_distance = getUltrasoincReading(frontProx)
                    setMotorsVelocity(0,0)
                    time.sleep(15)
                    errorCode=sim.simxRemoveModel(clientID,bill,sim.simx_opmode_blocking)
                    setMotorsVelocity(0,0)
                    if detect==0:
                        setMotorsVelocity(6, 6)
            if np.all(ids is not None):
                if id_to_find in ids:
                    for i in range(0, len(ids)):
                        if ids[i]==id_to_find:
                            # -- ret = [rvec, tvec, ?]
                            # -- array of rotation and position of each marker in camera frame
                            # -- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
                            # -- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
                            ret = aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, camera_distortion)
                            # -- Unpack the output, get only the first
                            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
                            frame=img_markers
                            aruco.drawAxis(frame,  camera_matrix, camera_distortion, rvec, tvec, marker_size/2)  # Draw Axis
                            # -- Print the tag position in camera frame
                            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f" % (tvec[0], tvec[1], tvec[2])
                            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                            # #-- Obtain the rotation matrix tag->camera
                            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                            R_tc = R_ct.T
                            # -- Now get Position and attitude f the camera respect to the marker
                            pos_camera = -R_tc * np.matrix(tvec).T
                            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (
                            pos_camera[0], pos_camera[1], pos_camera[2])
                            cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                            # -- Get the attitude of the camera respect to the frame
                            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)
                            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                            math.degrees(roll_camera), math.degrees(pitch_camera), math.degrees(yaw_camera))
                            cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                            distance = pos_camera[2]
                            out.write(img_markers)
                            errorCode,currentDis=sim.simxGetObjectPosition(clientID,robotel,home,sim.simx_opmode_blocking)
                            currentDis=abs(currentDis[2])
                            if currentDis<check:
                                if id_to_find==0:
                                    state="Heading Home"
                                    corner=corners[0][0]
                                    # Get its x and y position
                                    xpos = (corner[0][0]+corner[1][0])/2
                                    ypos = (corner[1][1]+corner[2][1])/2
                                    # Adjust the base speed
                                    rVel=8
                                    vel = 30
                                    rfactor = 0.5
                                    # Calculate the motors' speed given the position of the sphere
                                    leftVel = (-(ypos / resolution[1] - 0.5) * vel) + (rfactor * (0.5 - xpos / resolution[0]) * rVel)
                                    rightVel = (-(ypos / resolution[1] - 0.5) * vel) - (rfactor * (0.5 - xpos / resolution[0]) * rVel)
                                    if abs(leftVel)>1.5:
                                        setMotorsVelocity(-leftVel,-rightVel)
                                #print(distance)
                                if state == "Heading Home":
                                    if distance < 70 and distance>65:
                                        setMotorsVelocity(0, 0)
                                        state = "Home"
                                        print(state)
                                        sim.simxAddStatusbarMessage(clientID, state, sim.simx_opmode_blocking)
                                        rotate90("right", math.pi / 2)
                            ID='ID'+str(id_to_find)
                            xr,yr,zr=getRelativePosition(ID)
                            if (pos_camera[2]<(zr+10) and pos_camera[2]>(zr-10)):
                                cv2.destroyAllWindows()
                                if state!="Heading Home":
                                    if distance<150:
                                        setMotorsVelocity(0,0)
                                        state="Arrived"
                                        print(state)
                                        sim.simxAddStatusbarMessage(clientID, state, sim.simx_opmode_blocking)
                                        deliverPackage()
                                        setMotorsVelocity(6,6)
                else:
                    setMotorsVelocity(6,6)
            # Showing the image with marker identified
            #cv2.imshow('Aruco', img_markers)
            #sz = np.size(ids)
        c = cv2.waitKey(1)
        if c == 27:
            break
