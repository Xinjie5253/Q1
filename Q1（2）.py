import numpy as np # this one is to do manipulate arrays 
import scipy as sp
import scipy.integrate as spi
import matplotlib.pyplot as plt  # this one is to produce nice graphs and plots


class Car:

    def __init__(self, 
             length=2.3,
             velocity=1,
             x_pos_init=0, y_pos_init=0.3, pose_init=np.deg2rad(5)):
        self.__length = length
        self.__velocity = velocity
        self.__x = x_pos_init
        self.__y = y_pos_init
        self.__pose = pose_init

    def y(self):
        return self.__y

    def move(self, steering_angle, dt):
        # This method computes the position and orientation (pose) 
        # of the car after time `dt` starting from its current 
        # position and orientation by solving an IVP
        #
        def bicycle_model(_t, z):
            # Define system dynamics [given in (2.1a-c)]
            # [v*cos(θ), v*sin(θ), (v/L)*tan(u)]
            # z = (x, y, θ)
            theta=z[2]
            return [self.__velocity*np.cos(theta),
                    self.__velocity*np.sin(theta),
                    self.__velocity*np.tan(steering_angle)/self.__length]

        sol = spi.solve_ivp(bicycle_model, 
                            [0, dt],
                            [self.__x, self.__y, self.__pose], 
                            t_eval=np.linspace(0, dt, 2))
        new_state = sol.y[:, -1]
        self.__x = new_state[0]
        self.__y = new_state[1]
        self.__pose = new_state[2]

    def velocity(self):
        return self.__velocity
   
    def state(self):
        return [self.__x, self.__y, self.__pose]



henry = Car(length=2.3, velocity=5)
# time interval and dt configuration
a = 0
b = 2
n_points = 5000
# build arrays to store data 
x_tra =np.zeros((n_points+1,))
y_tra=np.zeros((n_points+1,))
h = (b-a)/n_points

for i in range(n_points+1):
   
    henry.move(steering_angle=np.deg2rad(-2), dt=h)# set steering angle
    z=henry.state()
    # trajectory
    x_tra[i]=z[0]
    y_tra[i]=z[1]
    
plt.rcParams['font.size'] = '14'
plt.xlabel('x')
plt.ylabel('y')

plt.plot(x_tra,y_tra)
plt.legend((''))
plt.show()
