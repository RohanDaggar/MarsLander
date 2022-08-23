# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
import math

def runMassSpring():
    # mass, spring constant, initial position and velocity
    m = 1
    k = 1
    x = 0
    v = 1

    # simulation time, timestep and time
    t_max = 1000
    dt = 1 #! inital 0.1 is too small, values increase 
    t_array = np.arange(0, t_max, dt)

    # initialise empty lists to record trajectories
    x_list = []
    v_list = []

    # Verlet integration
    for t in t_array:
        
        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)
        
        # This is for the first operation of the verlet to utilise initial velocity
        if t == 0:
            a = x
            x = x + v*dt + 0.5 * dt**2 * -k * x
            v = (x-a)/dt
        # else calculate new position and velocity
        else:
            a=x
            x = 2 * x - x_list[-2] + dt**2 * -k * x / m
            v = (x-a)/dt
        

    # convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
    x_array = np.array(x_list)
    v_array = np.array(v_list)

    # plot the position-time graph
    plt.figure(1)
    plt.clf()
    plt.xlabel('time (s)')
    plt.grid()
    plt.plot(t_array, x_array, label='x (m)')
    plt.plot(t_array, v_array, label='v (m/s)')
    plt.legend()
    plt.show()

def runThreeD():
    scenario = '0'
    while scenario not in ['1','2','3','4']:
        print("Pick scenario:")
        print("1. Straight down descent ")
        print("2. Circular orbit")
        print("3. Elliptical orbit")
        print("4. Hyperbolic escape")
        scenario = input("> ")
    
    # Gravitational constant, mass of planet (mars), mass of moving body, radius of planet (mars)
    G = 6.6743e-11
    M = 6.42e23
    m = 1
    marsRad = 3389500
    
    # Initial numpy arrays to store position, velocity, max time and time step
    if scenario == '1': 
        r = np.array([[100000+marsRad],[0],[0]])
        v = np.array([[0],[0],[0]])
        t_max = 500
        dt = 0.1
    elif scenario == '2':
        orbitalRad = 1000000 +marsRad
        circOrbVel = math.sqrt(G*M/(orbitalRad))
        orbitalTime = 2 * math.pi * math.sqrt(orbitalRad**3/(G*M))
        r = np.array([[orbitalRad],[0],[0]])
        v = np.array([[0],[circOrbVel],[0]])
        t_max = 2*orbitalTime
        dt = orbitalTime/10000
    elif scenario == '3':
        r = np.array([[1000000+marsRad],[0],[0]])
        v = np.array([[1300],[3400],[0]])
        t_max = 20000
        dt = 2
    elif scenario == '4':
        r = np.array([[1000000+marsRad],[1000000+marsRad],[0]])
        v = np.array([[-2000],[3500],[0]])
        t_max = 20000
        dt = 5
    else:
        raise RuntimeError
    
    # time array created 
    t_array = np.arange(0, t_max, dt)
    #initalise np arrays (not sure how to not put 0s there so its removed later)
    position = np.array([[0],[0],[0]])
    velocity = np.array([[0],[0],[0]])
    #array for the radius of mars to see in the plots
    marsmarsRadius = np.full((3,int(t_max/dt)), marsRad)
    
    # Initial calculation for the first step
    
    Force = - G * M * m * r / np.linalg.norm(r)**3
    next_position = position + dt*v + 0.5 * dt * dt * Force/m
    next_velocity = (next_position-position)/dt
    
    position = np.hstack((position,next_position))
    velocity = np.hstack((velocity,next_velocity))
    
    
    # Verlet integration
    for t in t_array:
        
        # append current state to trajectories
        position = np.hstack((position,r))
        velocity = np.hstack((velocity,v))
        
        # calculate new position and velocity
        Force = - G * M * m * r / np.linalg.norm(r)**3
        r = r + dt * v
        v = v + dt * Force / m
        
        
    #this step is needed to remove the initial 0s created
    position = np.array([np.delete(position[0],0), np.delete(position[1],0), np.delete(position[2],0)])
    velocity = np.array([np.delete(velocity[0],0), np.delete(velocity[1],0), np.delete(velocity[2],0)])
    
    # plot the position-time graph according to the scenario
    if scenario == '1':
        plt.figure("position")
        plt.clf()
        plt.xlabel('time (s)')
        plt.grid()
        plt.plot(t_array, position[0], label='position (m)')
        plt.plot(t_array, marsmarsRadius[0], label='mars surface')
        plt.legend()
        
        plt.figure("velocity")
        plt.clf()
        plt.xlabel('time (s)')
        plt.ylabel('velocity (m/s)')
        plt.grid()
        plt.plot(t_array, velocity[0])
        plt.show()
    else:
        plt.figure("position")
        plt.clf()
        plt.grid()
        plt.xlim(-3*marsRad,3*marsRad)
        plt.ylim(-3*marsRad,3*marsRad)
        plt.xlabel("x axis")
        plt.ylabel("y axis")
        plt.plot(position[0], position[1], label='orbit')
        Mars = plt.Circle((0, 0), marsRad, color='orange')
        plt.gca().add_patch(Mars)
        plt.show()


if __name__ == "__main__":
    runThreeD()
