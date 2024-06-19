import numpy as np
import time
import serial
from ctrlFncs import *
from setSupPy import setSup
from trackerSetup import sensorRead
from kinect import handTracker
import matplotlib
import matplotlib.pyplot as plt
import keyboard


# Initialize Kinect
handTracker.start()


# get the serial ready
ser = serial.Serial('COM5')
ser.baudrate = 19200
ser.set_buffer_size(1000)
ser.bytesize = serial.EIGHTBITS
ser.stopbits = 1


# Start at the home position (x,y,z)=(0,0,0)
homePos = np.array([500, 500, 500]).reshape(-1,1)
print('waiting to start')
time.sleep(1)
setSup(0, 0, 0, 0, 0, 0, 0, 0, 500, 500, 500, 500, 500, 500, ser)

print('waiting to settle')
time.sleep(4)

# Sampling setup
sample = sensorRead()  # Measure the robot position
sample = np.array(sample).reshape(-1,1)
t_prev = sample[0]  # Initialize the sampling time
chi_home = sample[1:4]  # Sample the (x,y,z) coordinates of the home positions
u_hat = np.array([0, 0, 0]).reshape(-1,1)  # Initialize input estimator to the home position inputs


# Estimator setup
Gamma = np.array([[5, 0, 0],
                  [0, 5, 0],
                  [0, 0, 10]])

# Gamma = np.array([[2000, 0, 0],
#                   [0, 2000, 0],
#                   [0, 0, 1000]])

#Gamma = np.array([[20, 0, 0],
#                  [0, 20, 0],
#                  [0, 0, 10]]) * 7.5

# STSMC setup
k1_STSMC = np.array([[9, 0, 0],
                     [0, 9, 0],
                     [0, 0, 10]]) * 0.5

k2_STSMC = np.array([[9, 0, 0],
                     [0, 9, 0],
                     [0, 0, 10]]) * 0.5

# Boundary layer in mm units
Phi = np.array([3, 3, 3]).reshape(-1,1) * 1.5

# Specify how many iterations to run the system for
iterations = 2000


# Initialize the integration part of the SSTSMC
int_e = 0
v_a_prior = np.array([0, 0, 0]).reshape(-1,1)  # SSTSMC

v_prev = np.array([0, 0, 0]).reshape(-1,1)
chi_prev = np.array([0, 0, 0]).reshape(-1,1)

# Plot variables

'''
# Initialize empty arrays to gather data for plotting
chi_values = []
tau_values = []
u_hat_values = []
v_values = []
e_values = []
int_e_values = []
t_values = []
u_hat_d_values = []
r_values = []
dt_values = []
v_hat_values = []

Delta_log = np.zeros((3, iterations))
v_dot_log = np.zeros((3, iterations))
dh_dx_dot_log = np.zeros((3, iterations))

dh_du_inv_log = np.zeros(iterations)
'''
pts_log = np.zeros((0,3))
ref_log = np.zeros((0,3))
chi_log = np.zeros((0,3))
uhat_log = np.zeros((0,3))
uhd_log = np.zeros((0,3))
u_log = np.zeros((0,3))
v_log = np.zeros((0,3))
t_log = np.zeros((0,1))


p = np.zeros((3, 1))
r = np.zeros((3, 1))
p_prev = np.zeros((3, 1))
a = 0.95

r_sphere = 120
h_max = -60
r_xy = 60

t_log = np.vstack((t_log,0))

while True:

    tic = time.perf_counter()

    # Circle reference signal
    '''speed = -0.01
    angle = speed * i
    radius = 20
    height = -30
    r = np.array([1 + radius * np.cos(angle), -1 + radius * np.sin(angle), height]).reshape(-1,1)
    r_d = np.array([-radius * speed * np.sin(angle), radius * speed * np.cos(angle), 0]).reshape(-1,1)
    '''

    '''p_new = handTracker.readPos()
    if p_new is not None:
        p_prev = p
        p = p_new

        print('New postition',p)'''


    p = handTracker.readPos2()

    p = p.reshape(-1,1)

    r_prev = r
    #i_interp = (i % multiple) + 1
    #r = p * (i_interp / multiple) + p_prev * (1 - i_interp / multiple)

    r = a*r + (1-a)*p

    z_projection = r[:2]
    module = np.linalg.norm(r)
    if module > r_xy:
        z_projection = z_projection / module * r_xy
    else:
        z_projection = z_projection
    hz = r[2, 0]
    if hz <= h_max:
        hz = h_max
    elif hz > h_max and z_projection[0] ** 2 + z_projection[1] ** 2 + (hz + r_sphere) ** 2 <= r_sphere ** 2:
        hz = hz
    else:
        hz = np.sqrt(r_sphere ** 2 - z_projection[0] ** 2 - z_projection[1] ** 2) - r_sphere

    projection_p = np.vstack((z_projection, hz))
    r = projection_p

    r_d = (r - r_prev) / 0.02


    # Sample the robot position
    sample = sensorRead()  # You need to get this sample appropriately
    sample = np.array(sample).reshape(-1,1)
    t = sample[0]  # Get current time value
    chi = sample[1:4] - chi_home  # (x,y,z) position
    x, y, z = chi

    dt = t - t_prev

    # Ramp up the reference signal
    '''r = r * min(i / 500, 1)
    r_d = r_d * min(i / 500, 1)
    '''

    # SSTSMC Controller
    e = chi - r  # Tracking error
    v, v_a = fnc_STSMC(chi, r, Phi, k1_STSMC, k2_STSMC, v_a_prior, dt)
    v_a_prior = v_a

    v = v + r_d - fnc_f(chi)


    # Input estimator
    tau = tau_fnc(v, u_hat, chi, Gamma)
    u_hat_d = proj(u_hat, tau)
    u_hat = u_hat + u_hat_d * dt

    # Input estimator - sliding mode observer
    #u_hat_d, dh_du_inv = tau_sld(v, u_hat, chi, Gamma)
    #u_hat = u_hat + u_hat_d * dt

    # Uncertainty estimator for sliding mode observer
    #Delta, v_dot, dh_dx_dot = uncert_bound(v, v_prev, chi, chi_prev, u_hat, dt)
    v_prev = v

    if (np.max(u_hat + 500) > 500 or np.min(u_hat + 500) < 350):
        print("U out of bounds")

    # Limit u_hat to avoid NaN
    u_hat[u_hat>500] = 500
    u_hat[u_hat<-650] = -650

    # Send control signal to robot
    u = u_hat + 500
    u[u>500] = 500
    u[u<350] = 350

    # You need to define the servoPos function and dev appropriately
    setSup(0, 0, 0, 0, 0, 0, 0, 0, int(u[0].item()), int(u[1].item()), int(u[2].item()), 500, 500, 500, ser)

    # Monitor how closely v_hat follows v by calculating h(chi, uhat)
    v_hat = B1 @ u_hat + fnc_f2(u_hat, chi) + fnc_g(u_hat)

    # Save this time value to calculate dt in the next iteration
    t_prev = t

    tlapse = time.perf_counter() - tic
    t_left = 0.02 - tlapse
    if t_left > 0:
        time.sleep(t_left)
    else:
        print('slow')

    treal = time.perf_counter() - tic

    pts_log = np.vstack((pts_log, p.reshape(1,-1)))
    ref_log = np.vstack((ref_log, r.reshape(1,-1)))
    chi_log = np.vstack((chi_log, chi.reshape(1,-1)))
    uhat_log = np.vstack((uhat_log, u_hat.reshape(1,-1)))
    u_log = np.vstack((u_log, u.reshape(1,-1)))
    t_log = np.vstack((t_log, t_log[-1]+treal))

    try:  # used try so that if user pressed other than the given key error will not be shown
        if keyboard.is_pressed('x'):  # if key 'q' is pressed 
            print('You Pressed A Key!')
            break  # finishing the loop
    except:
        pass 


fig1 = plt.figure('3d')
ax1 = fig1.add_subplot(projection='3d')
#ax1.scatter(pts_log[:,0].A1,pts_log[:,1].A1,pts_log[:,2].A1,c='b')
ax1.scatter(ref_log[:,0].A1,ref_log[:,1].A1,ref_log[:,2].A1,c='r')
ax1.scatter(chi_log[:,0].squeeze(),chi_log[:,1].squeeze(),chi_log[:,2].squeeze(),c='k')


fig2, ax2 = plt.subplots(3,1)

print('pts_log.shape',pts_log.shape)
print('pts_log[:,0].squeeze().shape',pts_log[:,0].A1.shape)
for i in range(3):
    p_i = pts_log[:,i].A1
    r_i = ref_log[:,i].A1
    uh_i = uhat_log[:,i].A1
    u_i = u_log[:,i].A1

    ax2[i].plot(t_log[:-1].squeeze(),p_i*10, marker = 'o')
    ax2[i].plot(t_log[:-1].squeeze(),r_i*10, marker = 'o')
    ax2[i].plot(t_log[:-1].squeeze(),uh_i, marker = 'o')
    ax2[i].plot(t_log[:-1].squeeze(),u_i, marker = 'o')
    ax2[i].grid(which='both')


plt.show()

print('THE END')
