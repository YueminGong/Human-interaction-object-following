import numpy as np
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import cv2
import os
import time
import kinect.const as const
#import const

import shelve

class KinectV2(object):

    def __init__(self):
        self.kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color |
                                                 PyKinectV2.FrameSourceTypes_BodyIndex |
                                                 PyKinectV2.FrameSourceTypes_Depth |
                                                 PyKinectV2.FrameSourceTypes_Body) #**

        #load calibration results
        self.rgbCamera = shelve.open(const.rgbCameraIntrinsic)
        self.rgb_Fx = self.rgbCamera['camera_matrix'][0,0]
        self.rgb_Fy = self.rgbCamera['camera_matrix'][1,1]
        self.rgb_Cx = self.rgbCamera['camera_matrix'][0,2]
        self.rgb_Cy = self.rgbCamera['camera_matrix'][1,2]

        self.irCamera = shelve.open(const.irCameraIntrinsic)
        self.depth_Fx = self.irCamera['camera_matrix'][0,0]
        self.depth_Fy = self.irCamera['camera_matrix'][1,1]
        self.depth_Cx = self.irCamera['camera_matrix'][0,2]
        self.depth_Cy = self.irCamera['camera_matrix'][1,2]

        self.stereoCamera = shelve.open(const.rgbToIR)
        # distanceFile = shelve.open(const.distanceErrorFunction)
        # self.depthErrorFunction = dict(zip(distanceFile['x'], distanceFile['y']))

        self.numberOfPicture = 0

    def close(self):
        self.kinect.close()

    def load_handposition(self,frame):
        # get frames from kinect
        body_frame = self.kinect.get_last_body_frame()
        hand_position = None  # Initialize with a default value in case the hand joint is not found
        Wrist_position = None
        if body_frame is not None:
            for i in range(0, self.kinect.max_body_count):
                body = body_frame.bodies[i]
                if not body.is_tracked:
                    continue

                # Get the left hand joint position
                joints = body.joints
                #joints_points = self.kinect.body_joints_to_color_space(joints)

                hand_joint = joints[PyKinectV2.JointType_HandRight]
                hand_position = (hand_joint.Position.x, hand_joint.Position.y, hand_joint.Position.z)
                Wrist_joint = joints[PyKinectV2.JointType_WristRight]
                Wrist_position = (Wrist_joint.Position.x, Wrist_joint.Position.y, Wrist_joint.Position.z)

                # draw the bond
                #self.draw_body(frame, joints_points, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_WristRight)


                break  # Exit the loop once the hand joint is found (if you only want the first tracked hand)

        return hand_position,Wrist_position

    def windows(self,hand_position):
        fx = self.depth_Fx
        fy = self.depth_Fy
        cx = self.depth_Cx
        cy = self.depth_Cy
        x = hand_position[0]
        y = -1*hand_position[1]
        z = hand_position[2]
        print('x:',x)
        z = z
        u = (x * fx)/ z + cx
        v = (y * fy) / z + cy
        result = [u, v]

        return result

    # draw body bone
    def draw_body_bone(self, frame, joints, joint0, joint1):
        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked):
            print("no joints!")
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good
        start = (joints[joint0].x, joints[joint0].y)
        end = (joints[joint1].x, joints[joint1].y)

        try:
            # Draw a red line between point1 and point2
            color = (0, 0, 255)  # Red color (BGR format)
            thickness = 2
            point0 = start
            point1 = end
            cv2.line(frame, point0, point1, color, thickness)
        except:  # need to catch it due to possible invalid positions (with inf)
            pass

    #draw body
    def draw_body(self, frame, joints, joint0, joint1):
        self.draw_body_bone(frame, joints, joint0, joint1)



    def takePicture(self):
        #get frames from Kinect
        colorFrame = self.kinect.get_last_color_frame()
        depthFrame = self.kinect.get_last_depth_frame()
        bodyIndexFrame = self.kinect.get_last_body_index_frame()


        #reshape to 2-D space
        colorFrame = colorFrame.reshape((const.rgb_image_size[0],const.rgb_image_size[1],4))
        depthFrame = depthFrame.reshape(const.ir_image_size)
        bodyIndexFrame = bodyIndexFrame.reshape(const.ir_image_size)

        #compensate  lens distortion
        colorFrame = cv2.undistort(colorFrame,self.rgbCamera['camera_matrix'], self.rgbCamera['dist_coefs'])
        depthFrame = cv2.undistort(depthFrame,self.irCamera['camera_matrix'], self.irCamera['dist_coefs'])
        bodyIndexFrame = cv2.undistort(bodyIndexFrame,self.irCamera['camera_matrix'], self.irCamera['dist_coefs'])

        distanceToBody = np.zeros(const.ir_image_size,np.uint16)
        distanceToBody[bodyIndexFrame != const.indexForBackground] = depthFrame[bodyIndexFrame != const.indexForBackground]



        # #compensate systematic errors of Kinect depth sensor
        # for i in range(0,distanceToBody[:,0].__len__()):
        #     for j in range(0,distanceToBody[0,:].__len__()):
        #         if distanceToBody[i,j] not in self.depthErrorFunction:
        #             print(str(distanceToBody[i,j]) + "is not in range of the depth error function!")
        #         else:
        #             distanceToBody[i,j] = distanceToBody[i,j] - self.depthErrorFunction[distanceToBody[i,j]]

        #combine depth, RGB and bodyIndexFrame
        combinedImage = self.__align__(colorFrame,distanceToBody)

        #self.__saveData__(colorFrame, depthFrame.reshape(const.ir_image_size), combinedImage, distanceToBody.reshape(const.ir_image_size))
        #self.numberOfPicture = self.numberOfPicture + 1

        return combinedImage,depthFrame # I add depthFrame here

    def newExperiment(self, folderName=[]):
        if not folderName:
            folderName = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
        self.folderName = const.pictureFolder + folderName + '/'
        if not os.path.exists(self.folderName):
            os.makedirs(self.folderName)
        self.numberOfPicture = 1


    def __saveData__(self,colorFrame,depthArray, combinedImage, distanceToBody):
        cv2.imwrite(self.folderName + str(self.numberOfPicture) + '_rgb.png',colorFrame)
        cv2.imwrite(self.folderName + str(self.numberOfPicture) + '_dep.png', depthArray)
        cv2.imwrite(self.folderName + str(self.numberOfPicture) + '_combined.png',combinedImage)
        depthArray.tofile(self.folderName + str(self.numberOfPicture) + '_depth.dat')
        distanceToBody.tofile(self.folderName +str(self.numberOfPicture) + '_distanceToBody.dat')

    #colorize depth frame
    def __align__(self,colorFrame,depthFrame):
        combinedImage = np.zeros((const.ir_image_size[0],const.ir_image_size[1],3))
        depthFrame = depthFrame/1000 #from mm to meters

        # From Depth Map to Point Cloud
        #book "Hacking the Kinect" by Jeff Kramer P. 130, http://gen.lib.rus.ec/book/index.php?md5=55a9155e10b1d5bb92811f69cb15f127
        worldCoordinates = np.zeros((np.prod(const.ir_image_size),3))
        i = 0
        for depthX in range(1,depthFrame.shape[1]):
            for depthY in range(1, depthFrame.shape[0]):
                z = depthFrame[depthY,depthX]
                if (z > 0):
                    worldCoordinates[i,0] = z*(depthX-self.depth_Cx)/self.depth_Fx #x
                    worldCoordinates[i,1] = z*(depthY-self.depth_Cy)/self.depth_Fy #y
                    worldCoordinates[i,2] = z #z
                i = i + 1

        #Projecting onto the Color Image Plane, Hacking the Kinect, P. 132
        worldCoordinates = np.dot(worldCoordinates, self.stereoCamera['R'].T) + self.stereoCamera['T'].T
        rgbX = np.round(worldCoordinates[:,0]*self.rgb_Fx/worldCoordinates[:,2]+self.rgb_Cx)
        rgbY = np.round(worldCoordinates[:,1]*self.rgb_Fy/worldCoordinates[:,2]+self.rgb_Cy)

        #colorize depth image
        i = 0
        for depthX in range(1, depthFrame.shape[1]):
            for depthY in range(1, depthFrame.shape[0]):
                if (
                        (rgbX[i] >= 0)
                        and (rgbX[i] < const.rgb_image_size[1])
                        and (rgbY[i] >= 0)
                        and (rgbY[i] < const.rgb_image_size[0])
                ):
                    # Cast rgbX[i] and rgbY[i] to integers
                    x_index = int(rgbX[i])
                    y_index = int(rgbY[i])

                    combinedImage[depthY, depthX, 0] = colorFrame[y_index, x_index][0]
                    combinedImage[depthY, depthX, 1] = colorFrame[y_index, x_index][1]
                    combinedImage[depthY, depthX, 2] = colorFrame[y_index, x_index][2]
                i = i + 1

        combinedImage = np.uint8(combinedImage)

        return combinedImage





