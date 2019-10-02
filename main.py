import cv2
import cv2.aruco as aruco
from oculusTracking import CV1Sensor
from controlCar import CarController
from util import *
import numpy as np
import time
from threading import Thread
import json
import itertools
import argparse
from enum import IntEnum, auto
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import glob



# dimensions and positions of markers on box clockwise from the front-right corner (meters)
size_x, size_y, size_z = 0.2345 / 2, 0.2455 / 2, 0.120
marker_geometry = np.array([[size_x, size_y, size_z], [size_x, -size_y, size_z], [-size_x, -size_y, size_z], [-size_x, size_y, size_z], [0, size_y, size_z]])            



# class to hold a CV1Sensor connection and its calibration and orientation information
class CalibratedSensor:
    def __init__(self, index=0, num_transfers=10):      
        self.sensor = CV1Sensor(index=index, numTransfers=num_transfers)
        
        try:
            # make sure the calibration data for the sensor exists
            with open(f'calibration/{self.sensor.serialNumber()}/data.json', 'r') as infile:
                data = json.load(infile)
                self.matrix = np.array(data['matrix'], dtype=np.float)
                self.dist = np.array(data['dist'], dtype=np.float)
        except FileNotFoundError:
            raise Exception(f'Calibration for sensor {self.sensor.serialNumber} not found!')

        try:
            # load orientation data if the file exists for this camera
            with open(f'calibration/{self.sensor.serialNumber()}/orientation.json', 'r') as infile:
                data = json.load(infile)
                self.oriented = True
                self.position = np.array(data['position'], dtype=np.float)
                self.forward = np.array(data['forward'], dtype=np.float)
                self.right = np.array(data['right'], dtype=np.float)
                self.down = np.array(data['down'], dtype=np.float)
        except FileNotFoundError:
            # fall back on zeros if the orientation file does not exist
            self.oriented = False
            self.position = np.zeros(3, dtype=np.float)
            self.forward = np.zeros(3, dtype=np.float)
            self.right = np.zeros(3, dtype=np.float)
            self.down = np.zeros(3, dtype=np.float)

        # number of orientation samples when calibrating to store the average 
        self.orient_n = 0


    # begin capture from the Oculus sensor
    def start(self):
        self.sensor.start()


    # adjust capture settings for a good tracking-capable image
    def setup(self):
        self.sensor.sensor.setCoarseExposureTime(600)
        self.sensor.sensor.setGain(128)


    # return the latest frame from the Oculus sensor if possible
    def latest_frame(self):
        if self.sensor.hasNewFrame():
            # I had some setup issue in the UVC interface that resulted in incorrect pixel timings in and misaligned rows
            # I tried many things, but cutting off the top of the image was the easiest
            return self.sensor.latestFrame()[150:, :]
        return None


    # transform a 3D vector/point in world space to camera space
    def world_pt_in_cam(self, p):
        v = p - self.position
        return np.array([np.dot(v, self.right), np.dot(v, self.down), np.dot(v, self.forward)])


    # transform a 3D vector/point in camera space to world space
    def cam_pt_in_world(self, p, offset=True):
        wp = np.array([self.right, self.down, self.forward]).T @ p
        if offset:
            wp += self.position
        return wp


    # save the orientation information to a JSON file for reading later
    def save_orientation(self):
        data = {}
        data['position'] = self.position.tolist()
        data['forward'] = self.forward.tolist()
        data['right'] = self.right.tolist()
        data['down'] = self.down.tolist()
        with open(f'calibration/{self.sensor.serialNumber()}/orientation.json', 'w') as outfile:
            json.dump(data, outfile)



class ControlState(IntEnum):
    WAITING = auto()
    TRACKING_BALL = auto()
    BALL_LANDED = auto()



class Application:
    def __init__(self):
        np.set_printoptions(suppress=True)

        parser = argparse.ArgumentParser()
        parser.add_argument('-orientsensors', action='store_true', help='Include this argument to calibrate the sensor positions in space.')
        parser.add_argument('-t', type=float, default=5, help='Amount of time (s) to calibrate sensor positions.')
        self.args = parser.parse_args()

        self.running = False
        self.capture_thread = Thread(target=self.capture_thread_func)   
        self.sensors = [CalibratedSensor(index=i, num_transfers=7) for i in range(2)]
        self.cam_frames = [np.zeros((810, 1280, 3), dtype=np.uint8) for i in range(2)]
        self.cam_vecs = [None for i in range(2)]
        self.car_indices = [None for i in range(2)]
        self.ball_vecs = [None for i in range(2)]
        self.car_pts = None
        self.ball_pt = None
        self.ball_pt_history = []
        self.ball_first_t = 0
        self.control_state = ControlState.BALL_LANDED
        self.state_did_change = False
        self.car_transform = None
        self.car_controller = CarController(port=glob.glob('/dev/tty.usb*')[0])
        self.car_controller.start()

    
    # main function for separate capture thread
    def capture_thread_func(self):
        print('starting capture thread')

        t0 = time.time()
        while self.running:

            # calibration timer
            if self.args.orientsensors:
                t = time.time() - t0
                if t >= self.args.t:
                    self.running = False
                    break
                # print(f'[Calibrating] {self.args.t - t:.2f}s remaining')

            # frame capture and tracking code
            for i, sensor in enumerate(self.sensors):
                frame_gray = sensor.latest_frame()
                if frame_gray is None:
                    continue

                frame = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR)

                # mask out non-reflective parts of the frame
                inv_mask = frame_gray < 200
                frame_gray[inv_mask] = 0
                frame_gray[~inv_mask] = 255
                
                # smooth the blobs
                frame_gray = cv2.erode(frame_gray, np.ones((2,2), np.uint8), iterations=1)
                frame_gray = cv2.dilate(frame_gray, np.ones((2,2), np.uint8), iterations=2)

                # detect tracking markers on frame
                contours, _ = cv2.findContours(frame_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                pts_original = np.array([[*p[0], p[1]] for p in [cv2.minEnclosingCircle(c) for c in contours]])
                n_pts = pts_original.shape[0]

                if n_pts > 0:
                    pts_undist = pts_original.copy()
                    pts_undist[:, 0:2] = cv2.undistortPoints(pts_undist[:, 0:2].reshape(1,-1,2), sensor.matrix, sensor.dist).reshape(-1, 2)
                    
                    
                    if self.args.orientsensors:
                        # detect car position and solve for orientation data for each sensor
                        result = self.car_detector(pts_undist)
                        if result is not None:                            
                            # PnP to get camera orientation relative to marker geometry of car
                            status, rvec, tvec = cv2.solvePnP(marker_geometry, pts_original[result, 0:2], sensor.matrix, sensor.dist)
                            aruco.drawAxis(frame, sensor.matrix, sensor.dist, rvec, tvec, 0.25)

                            # compute position and principal axis of camera in world space
                            rmat, _ = cv2.Rodrigues(rvec)
                            inv_rmat = np.linalg.inv(rmat)
                            pos = inv_rmat.dot(-tvec).reshape(-1)
                            forward = inv_rmat.dot(([0,0,1]-tvec.T).T).reshape(-1) - pos
                            right = inv_rmat.dot(([1,0,0]-tvec.T).T).reshape(-1) - pos
                            up = inv_rmat.dot(([0,-1,0]-tvec.T).T).reshape(-1) - pos
                            
                            # summation for averaging the calibration data
                            sensor.position += pos
                            sensor.forward += forward
                            sensor.right += right
                            sensor.up += up
                            sensor.orient_n += 1
                        else:
                            # show detected points if the car cannot be identified
                            for p in pts_original:
                                cv2.circle(frame, (int(p[0]), int(p[1])), int(p[2]) + 5, (0, 0, 255), 2)

                    else:
                        self.cam_vecs[i] = np.hstack((pts_undist[:, 0:2], np.ones((pts_undist.shape[0], 1))))
                        
                        # show detected points
                        for p in pts_original:
                            cv2.circle(frame, (int(p[0]), int(p[1])), int(p[2]) + 5, (0, 0, 255), 2)

                        # find the set of points that form the car in the frame
                        result = self.car_detector(pts_undist)
                        self.car_indices[i] = result
                        self.ball_vecs[i] = None
                        
                        if result is not None:
                            # cv2.polylines(frame, [pts_original[result, 0:2].astype(np.int)], False, (0, 255, 0), 2)

                            # draw the coordinate frame of the car if it has been calculated by the tracking function
                            if self.car_transform is not None:
                                rmat, t = self.car_transform
                                ps = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], np.float) * 0.4
                                ps = np.apply_along_axis(lambda p: p @ rmat + t, -1, ps)
                                ps = np.apply_along_axis(sensor.world_pt_in_cam, -1, ps)
                                ps = cv2.projectPoints(ps, np.zeros(shape=3), np.zeros(shape=3), sensor.matrix, None)[0].reshape(-1, 2).astype(np.int)
                                cv2.line(frame, tuple(ps[0].tolist()), tuple(ps[1].tolist()), (0, 0, 255), 2)
                                cv2.line(frame, tuple(ps[0].tolist()), tuple(ps[2].tolist()), (0, 255, 0), 2)
                                cv2.line(frame, tuple(ps[0].tolist()), tuple(ps[3].tolist()), (255, 0, 0), 2)

                            # if self.car_pts is not None:
                            #     ps = np.apply_along_axis(sensor.world_pt_in_cam, -1, self.car_pts)
                            #     ps = cv2.projectPoints(ps, np.zeros(shape=3), np.zeros(shape=3), sensor.matrix, sensor.dist)[0].reshape(-1, 2)
                            #     for p in ps:
                            #         cv2.circle(frame, tuple(p.astype(np.int).tolist()), 10, (0, 255, 255), 2)

                            if self.ball_pt is not None:
                                p = sensor.world_pt_in_cam(self.ball_pt)
                                p = cv2.projectPoints(p.reshape(1,3), np.zeros(shape=3), np.zeros(shape=3), sensor.matrix, sensor.dist)[0].reshape(2)
                                cv2.circle(frame, tuple(p.astype(np.int).tolist()), 15, (255, 0, 255), 2)

                            # the ball will be the 6th point in the image, but if at some angle...
                            # calculate an average ball vector per camera based on all of the unassigned vectors
                            if n_pts > 5:
                                i_ball = list(set(range(n_pts)) - set(result))
                                self.ball_vecs[i] = np.mean(self.cam_vecs[i][i_ball], axis=0)

                # save frame for display by main thread
                self.cam_frames[1-i] = frame

            # run main tracking code if not launched with -orientsensors
            if not self.args.orientsensors:
                self.track_scene()


    # main tracking code to find find 3D points of car and ball
    def track_scene(self):
        # ensure car has been found
        if self.car_indices[0] is None or self.car_indices[1] is None:
            self.car_transform = None
            self.car_controller.vel_x = 0
            self.car_controller.vel_y = 0
            self.car_controller.write()
            return
        
        # abstraction to calculate the nearest point between two camera vectors based on their indices
        def calc_nearest(a, b):
            return nearest_point_between_skews(self.sensors[0].position, self.sensors[1].position, self.sensors[0].cam_pt_in_world(self.cam_vecs[0][a], offset=False), self.sensors[1].cam_pt_in_world(self.cam_vecs[1][b], offset=False))
        
        # calculate 3D points of car and transform in space
        self.car_pts = np.array([calc_nearest(self.car_indices[0][i], self.car_indices[1][i]) for i in range(5)])
        self.car_transform = calc_transform(marker_geometry, self.car_pts)

        # make sure ball has been found
        if self.ball_vecs[0] is None or self.ball_vecs[1] is None or self.control_state == ControlState.BALL_LANDED:
            self.ball_pt = None
            return

        self.ball_pt = nearest_point_between_skews(self.sensors[0].position, self.sensors[1].position, self.sensors[0].cam_pt_in_world(self.ball_vecs[0], offset=False), self.sensors[1].cam_pt_in_world(self.ball_vecs[1], offset=False))
        
        if self.ball_pt[2] > 0.1:
            ball_t = time.time()
            if self.control_state == ControlState.WAITING:
                self.control_state = ControlState.TRACKING_BALL
                self.ball_first_t = ball_t
                print(self.control_state.name)
            ball_flight_t = ball_t - self.ball_first_t
            self.ball_pt_history.append((ball_flight_t, *self.ball_pt))
            self.estimate_ball_path()
        else:
            if self.control_state == ControlState.TRACKING_BALL:
                self.control_state = ControlState.BALL_LANDED
                print(self.control_state.name)
                print(self.ball_pt_history[-1])

                self.ball_pt_history = []


    # called by the scene tracking function to model the path of the ball, estimate its landing position, and move the car to catch it
    def estimate_ball_path(self):
        # only start estimating when enough data
        if len(self.ball_pt_history) < 10:
            return
        data = np.array(self.ball_pt_history)

        # kinematic equation from physics: x = x0 + (dx0)(t)
        def model_linear(t, x0, dx):
            return x0 + dx * t
        
        # build on previous model, but with acceleration due to graivty
        def model_with_gravity(t, x0, dx0):
            return x0 + dx0 * t - 0.5 * 9.81 * np.power(t,2)

        # find velocity and height values to model the ball's path
        (x0, dx), _ = curve_fit(model_linear, data[:,0], data[:,1])
        (y0, dy), _ = curve_fit(model_linear, data[:,0], data[:,2])
        (z0, dz0), _ = curve_fit(model_with_gravity, data[:,0], data[:,3])

        # estimate landing position based on when the ball will land
        estimate_t_land = (-dz0 - np.sqrt(dz0 ** 2 + 2 * 9.81 * (z0 - 0.250))) / -9.81
        t_until_land = estimate_t_land - (time.time() - self.ball_first_t)
        estimate = np.array([model_linear(estimate_t_land, x0, dx), model_linear(estimate_t_land, y0, dy), 0])

        # find forward and right vectors for controlling the movement of the mecanum wheels
        rmat, t = self.car_transform
        forward = np.array([0, 1, 0]) @ rmat
        right = np.array([1, 0, 0]) @ rmat
        delta = estimate - t

        # set the velocity of the car along X and Y axis to move fast toward the estimated landing position
        self.car_controller.vel_x = np.dot(delta, right) * 6
        self.car_controller.vel_y = np.dot(delta, forward) * 6
        self.car_controller.write()
        print(f'[{t_until_land:.3f}s] | X: {estimate[0]:.3f}m, Y: {estimate[1]:.3f}m | DX: {delta[0]:.3f}m, DY: {delta[1]:.3f}m | Controls: [{self.car_controller.vel_x:.3f}, {self.car_controller.vel_y:.3f}]')


    # find best set of 5 markers to represent the car
    def car_detector(self, pts):
        # find 3 colinear points that are roughly the same distance from the center point (for the front 3 markers)
        n = pts.shape[0]
        best_set = None
        best_v = 0
        for p in itertools.combinations(range(n), 3):
            a, b, c = p
            lens = [np.linalg.norm(pts[a, 0:2] - pts[b, 0:2]), np.linalg.norm(pts[a, 0:2] - pts[c, 0:2]), np.linalg.norm(pts[b, 0:2] - pts[c, 0:2])]
            front1, middle, front2 = [(a, c, b), (a, b, c), (b, a, c)][np.argmax(lens)]
            v1 = pts[front1, 0:2] - pts[middle, 0:2]
            v2 = pts[front2, 0:2] - pts[middle, 0:2]
            l1 = np.linalg.norm(v1)
            l2 = np.linalg.norm(v2)
            linearity = np.abs(np.dot(v1, v2) / l1 / l2)
            ratio = l1 / l2
            if ratio > 1:
                ratio = l2 / l1
            score = linearity + ratio
            if linearity > 0.97 and ratio >= 0.6 and score > best_v:
                best_set = front1, middle, front2
                best_v = score
        
        # return none if could not find points to represent front 3 markers
        if best_set is None:
            return None
        front1, middle, front2 = best_set

        # remaining markers should be the back two
        ni = list(range(n))
        for i in best_set:
            ni.remove(i)
        if len(ni) < 2:
            # if there are not 2 more points, the car cannot be found
            return None
        elif len(ni) == 2:
            # if there are 2 more points, they will be the back points
            back1, back2 = ni
        else:
            # if there are more than 2 points, find the best pair that fits the shape of the car
            target_v = pts[front1, 0:2] - pts[front2, 0:2]
            target_d = np.linalg.norm(target_v)
            best_set = None
            best_v = 0
            for p in itertools.combinations(ni, 2):
                a, b = p
                v = pts[a, 0:2] - pts[b, 0:2]
                d = np.linalg.norm(v)
                linearity = np.abs(np.dot(v, target_v) / d / target_d)
                ratio = d / target_d
                if ratio > 1:
                    ratio = target_d / d
                score = linearity + ratio
                if linearity > 0.97 and ratio >= 0.9 and score > best_v:
                    best_set = p
                    best_v = score
            if best_set is None:
                return None
            back1, back2 = best_set

        # find right vector direction to decide which points are on what side of the car
        mid = np.mean(pts[[front1, back1, back2, front2, middle], 0:2], axis=0)
        v_back = pts[middle, 0:2] - mid 
        v_right = np.array([-v_back[1], v_back[0]])
        v_right *= 1 / np.linalg.norm(v_right)

        # assing left and right points based on dot products with the right vector direction
        # dot product will be positive when angle between vectors in < 90deg
        v_front1 = pts[front1, 0:2] - pts[middle, 0:2]

        if np.dot(v_front1, v_right) / np.linalg.norm(v_front1) > 0:
            p_fr, p_fl = front1, front2
        else:
            p_fr, p_fl = front2, front1

        v_back1 = pts[back1, 0:2] - pts[middle, 0:2]
        if np.dot(v_back1, v_right) / np.linalg.norm(v_back1) > 0:
            p_br, p_bl = back1, back2
        else:
            p_br, p_bl = back2, back1

        # returns indices of markers clockwise from the front-right
        p_fm = middle
        return (p_fr, p_br, p_bl, p_fl, p_fm)


    # starts the sensor capture and begin tracking the space
    def start(self):
        if not self.args.orientsensors:
            for sensor in self.sensors:
                if not sensor.oriented:
                    print('ERROR: sensors not oriented yet!')
                    print('run with the -orientsensors flag')
                    return

        print('starting sensors')
        for sensor in self.sensors:
            sensor.start()

        time.sleep(1)
        print('adjusting capture settings')
        for sensor in self.sensors:
            sensor.setup()

        # start the capture and tracking thread
        self.running = True
        self.capture_thread.start()

        # display results of the tracking system in the main thread with UI
        while self.running:
            try:
                # stack the camera frames horizontally and display in one window
                stack_frame = np.hstack(tuple(self.cam_frames))
                cv2.imshow('Capture', stack_frame)
                key = cv2.waitKey(1)

                # press spacebar before throwing to tell the system to wait for the ball to appear
                if key == 32:
                    print('waiting for throw')
                    self.control_state = ControlState.WAITING


                # if self.state_did_change:
                    # self.state_did_change = False
                    # if self.control_state == ControlState.BALL_LANDED:
                        # df = pd.DataFrame(self.ball_pt_history, columns=['t', 'x', 'y', 'z'])
                        
                        # plt.title('X/Y trend')
                        # plt.xlim(-2, 2)
                        # plt.ylim(-2, 2)
                        # plt.plot(df['x'], df['y'])
                        # plt.show()

                        # plt.title('Z/t trend')
                        # # plt.ylim(-2, 2)
                        # plt.plot(df['t'], df['z'])
                        # plt.show()

                        # self.ball_pt_history = []

            except KeyboardInterrupt:
                break
        self.running = False


        # save averaged calibration data to a file for each sensor
        if self.args.orientsensors:
            for sensor in self.sensors:
                scalar = 1 / sensor.orient_n
                sensor.position *= scalar
                sensor.forward *= scalar
                sensor.right *= scalar
                sensor.down *= scalar

                print(f'Sensor {sensor.sensor.serialNumber()}:')
                print(f'  Position: {sensor.position}\n  Forward: {sensor.forward}\n  Right: {sensor.right}\n  Down: {sensor.down}')
                print()
                sensor.save_orientation()




# main function to start at launch
def main():
    app = Application()
    app.start()

if __name__ == '__main__':
    main()



