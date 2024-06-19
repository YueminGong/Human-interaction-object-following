import numpy as np
import cv2
from kinect.KinectV2 import KinectV2
#from KinectV2 import KinectV2

from pykinect2 import PyKinectV2
import keyboard
import scipy.io
import time
from threading import Thread

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Initialize the Kinect sensor
kinect = KinectV2()
kinect.newExperiment('hand_tracking_test')

global rela_p, p_offset, p_offset2, new_meas
rela_p = np.zeros((3,1))
p_offset2 = np.array([[-100-180], [60-30], [-106+135]])
p_offset = np.array([[200+100+30-5], [33+13-50-5], [-160-142-80+50]]) # + towards me, + towards left, - down (this moves the origin to that direction, but the robot to the opposite direction when tracking)
new_meas = False


def start():
    Thread(target=main, args=()).start()

def draw_body_bone(frame, joints, joint0, joint1):
    joint0State = joints[joint0].TrackingState;
    joint1State = joints[joint1].TrackingState;

    # both joints are not tracked
    if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked):
        print("no joints!")
        return

    # both joints are not *really* tracked
    if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
        return

    try:
        # Draw a red line between point1 and point2
        color = (0, 0, 255)  # Red color (BGR format)
        thickness = 2
        point0 = joint0
        point1 = joint1
        cv2.line(frame, point0, point1, color, thickness)
    except:  # need to catch it due to possible invalid positions (with inf)
        pass

def position_transform():
    #global p_offset2
    import math
    import numpy as np
    theta = -1*math.pi/6
    cs = math.cos(theta)
    si = math.sin(theta)
    Rca = np.matrix([[-1, 0, 0], [0, 0, -1], [0, 1, 0]])* np.matrix([[cs, 0, si], [0, 1, 0], [-si, 0, cs]])
    #print(Rca.shape)

    PborgC = np.array([[372], [-2], [717.5]])
    #PborgA = p_offset2
    #PcorgA = PborgA - Rca * PborgC
    PcorgA = - Rca * PborgC
    #ssssssprint(PcorgA)

    return PcorgA,Rca

def real_position_A_system(Rca,PcorgA,Prp_orgC):
    global p_offset
    [x,y,z] = Prp_orgC
    Prp_orgC = np.matrix([[x],[y],[z]]) * 1000
    Prp_orgA = PcorgA + Rca * Prp_orgC

    #rotation
    #Prp_orgA = np.matrix([[1, 0, 0], [0, -1, 0], [0, 0, -1]]) * Prp_orgA
    #print('world position:',Prp_orgA)

    #Prp_orgC = np.array([0.1182299363989337, 0.03303833475580709, 0.671]) * 1000

    Prp_orgA = np.transpose(Prp_orgA - p_offset)
    print('hand position in robot workspace',Prp_orgA)
    return Prp_orgA

def reverse_stero(x,y,z):
    fx = 328.66756973
    fy = 328.84503878
    cx = 245.08890333
    cy = 200.808506

    z = z * 100
    u = x * fx / z + cx
    v = y * fy / z + cy
    result = [u, v]

    return result

def readPos():
    global new_meas, rela_p
    print('Reading position')
    if new_meas:
        new_meas = False
        return rela_p
    else:
        return None
    
def readPos2():
    global rela_p
    #print('Reading position', rela_p)
    return rela_p

 


def main():###### points
    global rela_p, p_offset, new_meas
    try:

        #plt.ion()
        #fig = plt.figure()
        #ax = fig.add_subplot(111, projection='3d')

        pts = np.zeros((0,3))

        save_p = p_offset
        num = 1
        # Initialize variables to track the time for each sampling operation
        start_time_1 = None
        while True:
            #tic = time.perf_counter()

            # You can also capture images and use them in your hand tracking algorithm
            combined_image, depth_frame = kinect.takePicture()


            # Capture the real-time hand position
            hand_position, Wrist_position = kinect.load_handposition(combined_image)


            if hand_position is not None and Wrist_position is not None:
                # Print the hand position (replace this with your hand tracking logic)
                print("Hand Position:", hand_position)
                # print('shape',hand_position[0]*1000)
                # resulthand = reverse_stero(hand_position[0], hand_position[1], hand_position[2])
                # resultwrist = reverse_stero(Wrist_position[0], Wrist_position[1], Wrist_position[2])
                # color = (0, 0, 255)  # Red color (BGR format)
                # thickness = 2
                # point0 = resulthand
                # point1 = resultwrist
                # print('point1:',point1)
                rehand = kinect.windows(hand_position)
                rewrist = kinect.windows(Wrist_position)

                #print(rehand)
                #print(rewrist)
                color = (0, 0, 255)  # Red color (BGR format)
                thickness = 2
                cv2.line(combined_image, (int(rehand[0]), int(rehand[1])), (int(rewrist[0]), int(rewrist[1])), color, thickness)
                #cv2.line(combined_image, point0, point1, color, thickness)
            #
            # if wrist_position is not None:
            #     # Print the hand position (replace this with your hand tracking logic)
            #     print("Wrist Position:", wrist_position)

            # if depth_frame is not None:
            #         body = depth_frame.bodies
            #         if not body.is_tracked:
            #             continue
            #
            #         joints = body.joints
            #         # convert joint coordinates to color space
            #         draw_body_bone(combined_image, joints, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft)

            '''global a
            a = 1

            if keyboard.is_pressed('s'):
                a = 2
                if start_time_1 is None:
                    start_time_1 = time.time()  # Record the start time for sampling operation 1
            else:
                if start_time_1 is not None:
                    end_time_1 = time.time()  # Record the end time for sampling operation 1
                    time_cost_1 = end_time_1 - start_time_1
                    start_time_1 = None
                    print(f"Operation 1: Time cost = {time_cost_1:.2f} seconds")
            '''

            # elif keyboard.is_pressed('T'):
            #     a = 1
            #     if start_time_2 is None:
            #         start_time_2 = time.time()  # Record the start time for sampling operation 2

            # Show the real-time position of the center point
            if Wrist_position is not None:
                real_p = np.array([Wrist_position[0],Wrist_position[1],Wrist_position[2]])
                print('real time position of the wrist is ', real_p)

                # transform the point to the steropoint
                PcorgA, Rca = position_transform()
                Prp_orgC = real_p
                Prp_orgA = real_position_A_system(Rca, PcorgA, Prp_orgC)

                # realative position
                rela_p = Prp_orgA.T
                rela_p[2] = - rela_p[2]
                #print(rela_p)
                new_meas = True
                '''
                pts = np.vstack((pts,Prp_orgA))
                print('Points',pts)
                print('x_pt',pts[:,0])
                # Plot figure
                ax.cla()
                #ax.plot3D(pts[:,0].squeeze(), -pts[:,1].squeeze(), -pts[:,2].squeeze())
                ax.scatter3D(pts[:,0].squeeze(), -pts[:,1].squeeze(), pts[:,2].squeeze(), c=range(pts.shape[0]),cmap='jet')
                ax.scatter3D(0,0,0,c='0',marker='*',s=100)
                plt.draw()
                plt.pause(0.02)
                '''
                

                # save_p = np.zeros((60, 3))
                # save_p = np.zeros((1,3))
                # for i in range(60):
                #     if not any(np.all(save_p == real_p, axis=1)):
                #         save_p = np.vstack((save_p, real_p))
                #         i += 1
                #         print('shape is :',save_p.shape)
                #     else:
                #         i = i

                # save_p = np.zeros((0, 3))  # Initialize an empty array to store the points

                '''if not any(np.all(save_p == real_p, axis=1)):
                    save_p = np.vstack((save_p, rela_p))
                    num += 1
                    print('Data point {} saved. Shape is: {}'.format(num + 1, save_p.shape))
                else:
                    print('Data point {} is a duplicate, skipping...'.format(num + 1))
                '''

                # if start_time_1 is not None:
                #     end_time_1 = time.time()  # Record the end time for sampling operation 1
                #     time_cost_1 = end_time_1 - start_time_1
                #     start_time_1 = None

                # print(time_cost_1)
                # save_p = np.zeros((60,3))
                # for i in range(60):
                #     if a == 2:
                #         save_p[i,:] = real_p

                # print('world corrdinate of A :', Prp_orgA)

            # save the point data into excel
            # Save real_p to a .mat file
            '''data_to_save = {'save_p': save_p}
            file_path = 'C:/Users/adria/OneDrive - Danmarks Tekniske Universitet/PHD/kinect/data/hand_data.mat'
            scipy.io.savemat(file_path, data_to_save)
            '''

            # Use the combined_image and depth_frame data in your "handtrack" program
            # Update your hand tracking algorithm or visualization with the captured data
            # ...

            # Display the combined image (for visualization purposes)
            cv2.imshow("Combined Image", combined_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                #plt.close(fig)
                break

            #toc = time.perf_counter()

            #print('Time: ', toc-tic)

    finally:
        # Close the Kinect sensor when done
        kinect.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

