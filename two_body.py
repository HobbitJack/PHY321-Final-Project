import numpy as np
from math import *
import matplotlib.pyplot as plt
Grav_const = 6.6743e-11
planets = {"Sun": {"mass": 1.988e30,"vx": 9.305E+00,"vy": -1.283E+01,"x": -1.068E+09,"y": -4.177E+08},
            "Mercury": {"mass": 3.301e23,"vx": 3.666E+04,"vy": -1.230E+04,"x": -2.212E+10,"y": -6.682E+10},
            "Venus": {"mass": 4.867e24,"vx": 8.985E+02,"vy": -3.517E+04,"x": -1.086E+11,"y": -3.784E+09},
            "Earth": {"mass": 5.972e24,"vx": -2.983E+04,"vy": -5.224E+03,"x": -2.628e+10,"y": 1.445e+11},
            "Mars": {"mass": 6.417e23,"vx": 1.304E+03,"vy": 2.628E+04,"x": 2.069E+11,"y": -3.561E+09},
            "Jupiter": {"mass": 1.898e27,"vx": -7.893E+03,"vy": 1.115E+04,"x": 5.978E+11,"y": 4.387E+11},
            "Saturn": {"mass": 5.683e26,"vx": -7.420E+03,"vy": 6.726E+03,"x": 9.576E+11,"y": 9.821E+11},
            "Uranus": {"mass": 8.681e25,"vx": 4.647E+03,"vy": 4.614E+03,"x": 2.158E+12,"y": -2.055E+12},
            "Neptune": {"mass": 1.024e26,"vx": 4.475E+03,"vy": 3.064E+03,"x": 2.514E+12,"y": -3.739E+12}}
def orbit():
    dt,final_time = 86400, 31536000 #seconds
    n = int(ceil(final_time/dt)) #number of steps
    Mass_sun = planets["Sun"]["mass"]
    verlet_x, verlet_y, verlet_vx, verlet_vy = np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n) #initializing position and velocity arrays for Verlet
    verlet_vx[0], verlet_vy[0], verlet_x[0], verlet_y[0] = planets["Earth"]["vx"], planets["Earth"]["vy"], planets["Earth"]["x"], planets["Earth"]["y"]  #initial velocities and positions for Verlet
    for i in range(n-1):
        r = np.sqrt(verlet_x[i]**2 + verlet_y[i]**2)
        verlet_ax = -Grav_const * Mass_sun / (r**3) * verlet_x[i]
        verlet_ay = -Grav_const * Mass_sun / (r**3) * verlet_y[i]

        # Update positions
        verlet_x[i+1] = verlet_x[i] + verlet_vx[i] * dt + 0.5 * verlet_ax * dt**2
        verlet_y[i+1] = verlet_y[i] + verlet_vy[i] * dt + 0.5 * verlet_ay * dt**2

        # Calculate new acceleration at the new position
        r_new = np.sqrt(verlet_x[i+1]**2 + verlet_y[i+1]**2)
        verlet_ax_new = -Grav_const * Mass_sun / (r_new**3) * verlet_x[i+1]
        verlet_ay_new = -Grav_const * Mass_sun / (r_new**3) * verlet_y[i+1]

        # Update velocities
        verlet_vx[i+1] = verlet_vx[i] + 0.5 * (verlet_ax + verlet_ax_new) * dt
        verlet_vy[i+1] = verlet_vy[i] + 0.5 * (verlet_ay + verlet_ay_new) * dt

    plt.figure(figsize=(8,8))
    plt.plot(verlet_x,verlet_y,zorder=1,color='blue',linewidth = 1) #plot orbit
    plt.scatter(0,0, c = 'yellow',label ='Sun', s=1000) #plot sun 
    plt.scatter(verlet_x[-1],verlet_y[-1], c='dodgerblue',zorder=3,label='Earth', s=100) #plot planet position at time end
    plt.legend()
    plt.title('Orbit of Earth')
    plt.xlabel('x Position(m)')
    plt.ylabel('y Position(m)')
    plt.tight_layout()