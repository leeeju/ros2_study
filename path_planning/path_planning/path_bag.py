#!/usr/bin/env python3
import bisect
import rosbag
import math
import numpy as np
import matplotlib.pyplot as plt
import os


class bcolors:
    NC   = '\x1b[0m'
    BOLD = "\x1b[1m"
    RED  = '\x1b[31m'
    GRN  = '\x1b[32m'
    CYN  = '\x1b[36m'
    REDB = '\x1b[41m'
    GRNB = '\x1b[44m'
    YELLOW = '\x1b[93m'

class Spline:
    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        self.a = [iy for iy in y]

        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def __search_index(self, x):
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0

        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h):
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B

    def calcd(self, t):
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

class Spline2D:
    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_yaw(self, s):
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw

def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw = [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))

    return rx, ry, ryaw

def no_continuous(x, y):
    xx, yy = [], []

    for i, j in zip(x, y):
        if (xx[-1:] == [i]) & (yy[-1:] == [j]): continue
        xx.append(i)
        yy.append(j)

    return xx, yy

def extract_topics(bag_file, idx, length):
    gps_x, gps_y = [], []
    map_x, map_y = [], [] 

    bag = rosbag.Bag(bag_file)
    file_name = (bag_file.split("/"))[-1][:-4]
    progress = 0

    print("\n" + bcolors.BOLD + "========================================\n" + bcolors.NC)
    print(bcolors.BOLD + bcolors.REDB + "Progress : " + str(idx+1) + " / " + str(length) + bcolors.NC, end="\n\n")
    print(bcolors.BOLD + bcolors.RED + "current file : " + file_name + " (" + str(round(bag.size / (1<<30), 2)) + "GB)" + bcolors.NC, end="\n\n")

    for topic, msg, t in bag.read_messages(topics = "/status"):
        utm_position = [msg.relative_pose.translation.x,msg.relative_pose.translation.y]
        gps_x.append(utm_position[0])
        gps_y.append(utm_position[1])
        progress = (t.to_sec() - bag.get_start_time()) / (bag.get_end_time() - bag.get_start_time()) * 100

        for i in range (0, 25):
            if int(progress/4) == i:
                print(bcolors.BOLD + bcolors.YELLOW + ">>>" + bcolors.NC, end='')    
            else:
                print(bcolors.YELLOW + "=" + bcolors.NC, end='')
        print("   " + bcolors.YELLOW + str(round(progress, 2)) + "%" + bcolors.NC, end='\r')
    
    print("", end="\n\n")
    print(bcolors.BOLD + "========================================" + bcolors.NC)

    return gps_x, gps_y

def save_path(directory, sensor_name, x = [], y = [], yaw = []):
    f_x   = open(directory + "/" + sensor_name + "_x.txt",   'w')
    f_y   = open(directory + "/" + sensor_name + "_y.txt",   'w')
    f_yaw = open(directory + "/" + sensor_name + "_yaw.txt", 'w')

    for i in range(0, len(x)-1):
        f_x.write(str(x[i]) + '\n')
        f_y.write(str(y[i]) + '\n')
        f_yaw.write(str(yaw[i]) + '\n')

    f_x.write(str(x[-1]))
    f_y.write(str(y[-1]))
    f_yaw.write(str(yaw[-1]))

    f_x.close()
    f_y.close()
    f_yaw.close()

def load_path(directory, sensor_name):
    sx = list(map(float, open(directory + "/" + sensor_name + "_x.txt").read().split("\n")[:-1]))
    sy = list(map(float, open(directory + "/" + sensor_name + "_y.txt").read().split("\n")[:-1]))
    syaw = list(map(float, open(directory + "/" + sensor_name + "_yaw.txt").read().split("\n")[:-1]))

    return sx, sy, syaw

if __name__ == "__main__":
    rosbag_file_list = [
        "/home/jiwon/ros_bag_file/status_32.bag"
    ]

    file_length = len(rosbag_file_list)

    for idx, rosbag_file in enumerate(rosbag_file_list):
        directory = rosbag_file[:-4]

        if not os.path.exists(directory):
            os.makedirs(directory)

        gps_x, gps_y = extract_topics(rosbag_file, idx, file_length)

        gps_x, gps_y = no_continuous(gps_x, gps_y)

        n = 3000
        gps_xx = np.array([gps_x[i * n:(i + 1) * n] for i in range((len(gps_x) + n - 1) // n)])
        gps_yy = np.array([gps_y[i * n:(i + 1) * n] for i in range((len(gps_y) + n - 1) // n)])

        xx = []
        yy = []
        yawyaw = []

        for i in range(len(gps_xx)):
            print(i)
            cx, cy, cyaw = calc_spline_course(gps_xx[i], gps_yy[i], ds=0.1)
            xx.extend(cx)
            yy.extend(cy)
            yawyaw.extend(cyaw)

        # gps_x, gps_y, gps_yaw = calc_spline_course(gps_x, gps_y, ds=0.1)
        
        save_path(directory, "gps", xx, yy, yawyaw)

        plt.figure(figsize=(20,10), dpi=100)

        plt.subplot(121)
        plt.plot(xx, yy, 'b-')
        plt.xlabel("latitude")
        plt.ylabel("longitude")
        plt.legend(['gps route'], fontsize = 15)

        plt.subplot(122)
        plt.plot(gps_x, gps_y, 'r-')
        plt.xlabel("x")
        plt.ylabel("y")
        plt.legend(['map route'], fontsize = 15)

        plt.suptitle(directory.split("/")[-1], fontsize = 30)
        plt.savefig(directory + "/" + directory.split("/")[-1])

        plt.pause(3)
        plt.close()
        plt.figure()
        plt.plot(xx, yy, 'r*-')
        plt.grid()
        plt.show()