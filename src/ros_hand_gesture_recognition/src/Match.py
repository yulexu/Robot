#!/usr/bin/python3

import rospy
from std_msgs.msg import Header, String, Int32
from geometry_msgs.msg import Pose
from ros_hand_gesture_recognition.msg import HandGesture  # 替换为实际的包名和消息类型
from rfid.msg import rfid_msg
from darknet_ros_msgs.msg import DetectionPoseList
import numpy as np
import time
from scipy.interpolate import interp1d
import os
import json
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean

def interpolate_data(time_data, data, target_times):
    # filter invalide timestamp 
    valid_indices = np.where(~np.isnan(time_data) & ~np.isnan(data) & (np.diff(time_data, prepend=time_data[0]) > 0))[0]
    time_data = np.array(time_data)[valid_indices]
    data = np.array(data)[valid_indices]
    
    # if filtered data insufficient to interpolate, return original data 
    if len(time_data) < 2 or len(data) < 2:
        return np.interp(target_times, time_data, data).tolist()

    f = interp1d(time_data, data, kind='linear', fill_value="extrapolate")
    return f(target_times).tolist()


class YourClass:
    def __init__(self):
        self.sort_tracks = {}
        self.active_sort_ids = {}

        self.Matched_Gesture_ID = None
        self.phasor_data = {}  
        self.track_duration = 4     # time duration for match sort and RFID
        self.wavelength = 0.162924707 * 2  
        self.sampling_interval = 0  # sampling interval, 0 for sample all
        self.last_sample_time = None  
        self.current_gesture = None  
        self.gesture_start_time = None

        rospy.Subscriber("/gesture/hand_sign", HandGesture, self.GestureCallback)
        rospy.Subscriber('/rfid_message', rfid_msg, self.callback_1)
        rospy.Subscriber("/sort_track/Detection_pose",DetectionPoseList, self.SortCallback)

        self.tag_publisher = rospy.Publisher('/matched_epc', String, queue_size=10)
        self.tracking_id_publisher = rospy.Publisher('/sort/tracking_ID',Int32, queue_size=10)
        self.start_time = None
        self.sort_track_buffer = []
        self.epc_track_buffer = {}


    def SortCallback(self, data):
        # record all sort id position 
        self.active_sort_ids = {}
        # rospy.loginfo("SortCallback called")
        threshold = 0.5
        for pose in data.detectionposelist:
            sort_id = pose.id
            sort_position = (pose.x, pose.y, pose.z)
            timestamp = rospy.get_time()  
            # ids in current frame
            self.active_sort_ids[sort_id] = (timestamp, sort_position)

            if sort_id not in self.sort_tracks:
                self.sort_tracks[sort_id] = [(timestamp, sort_position)]
            else:
                last_position = self.sort_tracks[sort_id][-1][1]
                if all(abs(sort_position[i] - last_position[i]) <= threshold for i in range(3)):
                    self.sort_tracks[sort_id].append((timestamp, sort_position))
            # rospy.loginfo(f"Updated sort_tracks: {self.sort_tracks}")
        if self.start_time is not None:
            # rospy.loginfo("Begin casching sort info")
            if self.Matched_Gesture_ID and self.Matched_Gesture_ID in self.sort_tracks:
                    current_position = self.sort_tracks[self.Matched_Gesture_ID][-1]
                    if len(self.sort_track_buffer) == 0 or self.sort_track_buffer[-1] != current_position:
                        self.sort_track_buffer.append(current_position)
                    # rospy.logwarn(f"sort buffer casching:{current_position}")
                    # eg.[WARN] [1721718983.269797]: sort buffer casching:(1721718979.8205092, (1.428, -0.08382490158450806, -0.2025187192458945))



    def GestureCallback(self, data):
        # find sort id with the min distance between the last sort id pos and gesture pose
        # only when [new]! Stop gesture detected
        # rospy.logwarn("GestureCallback called")
        try:
            gesture_name = data.gesture_name
            gesture_position = (data.pose.position.x, data.pose.position.y)

            if gesture_name == "Stop" and self.current_gesture != "Stop":
                self.current_gesture = "Stop"
                self.gesture_start_time = time.time()   # record gesture start time
                closest_sort_id = None
                min_distance = float('inf')
                for sort_id, (timestamp, position) in self.active_sort_ids.items():
                    # last_position = positions[-1][1]
                    gesture_transformed = (gesture_position[0], gesture_position[1])  # 保持手势坐标不变
                    sort_transformed = (-position[1], position[2])  # 转换 sort 坐标 different coordinate
                    distance = self.calculate_distance_2d(gesture_transformed, sort_transformed)
                    # rospy.loginfo(f"sort_id: {sort_id}, distance: {distance}")
                    if distance < min_distance:
                        min_distance = distance
                        closest_sort_id = sort_id

                # record match rfid start time

                if closest_sort_id is not None:
                    rospy.loginfo(f"The closest sort_id to the 'Stop' gesture is: {closest_sort_id}")
                    self.Matched_Gesture_ID = closest_sort_id
                    self.start_time = time.time()  
                    self.sort_track_buffer = []
                    self.epc_track_buffer = {}
                    self.last_sample_time = self.start_time
                    self.phasor_data = {}
                    # publish tracking id
                    self.tracking_id_publisher.publish(closest_sort_id)  

            elif gesture_name != "Stop":
                self.current_gesture = None
                self.gesture_start_time = None
        except Exception as e:
            rospy.logerr(f"Error in GestureCallback: {e}")

    def callback_1(self, msg):
        # caching the sort pose(after matching with gesture) and epc pose during time:self.track_duration
        # find the matched epc
        # rospy.loginfo("callback_1 called")
        try:
            if self.start_time is None:
                return
            # run when gesture start time is not None
            current_time = time.time()
            if current_time - self.start_time > self.track_duration:
                self.start_time = None
                matched_epc = self.find_best_matching_epc()
                if matched_epc:
                    rospy.logwarn(f"Matched EPC: {matched_epc}")
                    # publish matched EPC
                    self.tag_publisher.publish(matched_epc)
                else:
                    rospy.logerr("find_best_matching_epc returns NONE")
                return

            # caching epc info every interval
            if self.last_sample_time is None or current_time - self.last_sample_time >= self.sampling_interval:

                epc = msg.epc
                ant_idx = msg.ant - 1
                # if ant_idx != 1:  # 只使用天线2
                #     return
                
                self.last_sample_time = current_time

                phase = msg.phase / 180 * np.pi  # Convert to radians

                if epc not in self.phasor_data:
                    self.phasor_data[epc] = {
                        'phasor_unwrapped': [None] * 4,
                        'begin_phasor': [None] * 4,
                        'phasor_old': [None] * 4,
                        'phasor_current': [None] * 4
                    }

                phasor_data_epc = self.phasor_data[epc]


                # Phase unwrapping
                if phasor_data_epc['phasor_unwrapped'][ant_idx] is None:
                    phasor_data_epc['begin_phasor'][ant_idx] = phase
                    phasor_data_epc['phasor_unwrapped'][ant_idx] = phase  # No calibration
                    phasor_data_epc['phasor_old'][ant_idx] = phase
                else:
                    phasor_data_epc['phasor_current'][ant_idx] = phase
                    df_ph = phasor_data_epc['phasor_current'][ant_idx] - phasor_data_epc['phasor_old'][ant_idx]
                    df_options = [df_ph - 2 * np.pi, df_ph, df_ph + 2 * np.pi]
                    df_min = min(df_options, key=abs)
                    phasor_data_epc['phasor_unwrapped'][ant_idx] += df_min
                    phasor_data_epc['phasor_old'][ant_idx] = phasor_data_epc['phasor_current'][ant_idx]

                    relative_distance = -phasor_data_epc['phasor_unwrapped'][ant_idx] * self.wavelength / (4 * np.pi)
                    # if epc == 'E280-1160-6000-0216-D2A7-FD55':
                    #     rospy.logwarn(f"rfid phase distance of {epc}:{relative_distance}")
                    timestamp = rospy.get_time()  
                    if epc not in self.epc_track_buffer:
                        self.epc_track_buffer[epc] = []
                    self.epc_track_buffer[epc].append((ant_idx,timestamp, relative_distance))
                    # if epc == 'E280-1160-6000-0216-D2A7-FD55':
                    #     rospy.logwarn(f"epc_buffer of {epc}:{self.epc_track_buffer[epc][-1]}")

                    # 打印相对距离数据
                    # rospy.loginfo(f"EPC: {epc}, Relative Distance: {relative_distance}")

                    # if self.Matched_Gesture_ID and self.Matched_Gesture_ID in self.sort_tracks:
                    #     current_position = self.sort_tracks[self.Matched_Gesture_ID][-1]
                    #     # if len(self.sort_track_buffer) == 0 or self.sort_track_buffer[-1] != current_position:
                    #     self.sort_track_buffer.append(current_position)

                    # per interval 打印轨迹数据
                    # rospy.loginfo(f"Sort ID: {self.Matched_Gesture_ID}, Position: {current_position}")
        except Exception as e:
            rospy.logerr(f"Error in callback_1: {e}")

    def find_best_matching_epc(self):
        
        rospy.loginfo("find_best_matching_epc called")

        try:
            if not self.sort_track_buffer:
                rospy.logerr("[self.sort_track_buffer] is None!")
                return None
            if not self.epc_track_buffer:
                rospy.logerr("[self.epc_track_buffer] is None!")
                return None

            rospy.logerr(f"length of sort buffer{len(self.sort_track_buffer)}")
            data_dir = "/home/linzp/catkin_op/src/ros_hand_gesture_recognition/src/data"

            #1. Compute relative trajectory of sort
            sort_relative_trajectories = []

            ant_positions = [
            np.array([0.375, 0]),  # Antenna 1
            np.array([0.125, 0]),  # Antenna 2
            np.array([-0.125, 0]),  # Antenna 3
            np.array([-0.375, 0])   # Antenna 4
            ]
            for ant_index in range(4):
                ant_pos=ant_positions[ant_index]
                pos_0 = self.sort_track_buffer[0][1]
                initial_distance = np.sqrt((-pos_0[1] - ant_pos[0])**2 + (pos_0[0] - ant_pos[1])**2)
                sort_positions = [pos for _, pos in self.sort_track_buffer]
                sort_relative_trajectory = [(
                    np.sqrt(
                        (-pos[1] - ant_pos[0])**2 +
                        (pos[0] - ant_pos[1])**2
                    ) - initial_distance) for pos in sort_positions
                ]
                sort_relative_trajectories.append(sort_relative_trajectory)
                # record
                with open(os.path.join(data_dir, f'Ant{ant_index+1}_sort_relative_trajectory.json'), 'w') as f:
                    json.dump(sort_relative_trajectory, f)

                rospy.logwarn("Sort Times and Relative Trajectory:")
                sort_times = [t for t, _ in self.sort_track_buffer]
                for time, trajectory in zip(sort_times, sort_relative_trajectory):
                    rospy.loginfo(f"Time: {time}, Ant:{ant_index}, Relative Trajectory: {trajectory}")            
            
            best_match_epc = None
            min_distance_sum = float('inf')

            if not os.path.exists(data_dir):
                os.makedirs(data_dir)
            # record data
            # with open(os.path.join(data_dir, f'epc_buffer_{epc}.json'), 'w') as f:
            #     json.dump(relative_distances, f)
            


            #2. Compute relative trajectory of RFID

            for epc, data in self.epc_track_buffer.items():
                epc_times = {ant: [] for ant in range(4)}
                epc_distances = {ant: [] for ant in range(4)}

                for ant_idx, t, d in data:
                    epc_times[ant_idx].append(t)
                    epc_distances[ant_idx].append(d)

                # Check if we have enough data for each antenna
                valid_antennas = [ant for ant, times in epc_times.items() if len(times) >= 20]

                if len(valid_antennas) < 4:
                    rospy.logwarn(f"EPC: {epc} does not have enough data points for all antennas. Skipping this EPC.")
                    continue

                distance_sum = 0

                for ant in range(4):
                    if ant in valid_antennas:
                        sort_relative_trajectory = np.array(sort_relative_trajectories[ant])
                        relative_distances = np.array(epc_distances[ant])
                        sort_relative_trajectory = sort_relative_trajectory.flatten()
                        relative_distances = relative_distances.flatten()

                        if sort_relative_trajectory.ndim != 1 or relative_distances.ndim != 1:
                            rospy.logerr("One of the input vectors is not 1-D.")
                            continue
                        
                        distance, path = fastdtw(sort_relative_trajectory, relative_distances)
                        # rospy.logwarn(f"DTW distance of {epc} for Antenna {ant+1}: {distance}")

                        distance_sum += distance
                        # record
                        with open(os.path.join(data_dir, f'Ant{ant+1}_epc_buffer_{epc}.json'), 'w') as f:
                            json.dump(relative_distances.tolist(), f)
                    else:
                        rospy.logwarn(f"EPC: {epc} does not have valid data for Antenna {ant+1}. Using default high distance.")
                        distance_sum += float('inf')

                rospy.logwarn(f"Total DTW distance of {epc}: {distance_sum}")

                
                if distance_sum < min_distance_sum:
                    min_distance_sum = distance_sum
                    best_match_epc = epc

            if best_match_epc is not None:
                rospy.logerr(f"Best Match EPC: {best_match_epc}, Total Distance Sum: {min_distance_sum}")
            else:
                rospy.logerr("No matching EPC found.")
            # Clear buffer for next match
            self.sort_track_buffer = []
            self.epc_track_buffer = {}

            return best_match_epc
        except Exception as e:
            rospy.logerr(f"Error in find_best_matching_epc: {e}")
            return None

    def calculate_distance_2d(self, pos1, pos2):
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

if __name__ == "__main__":
    rospy.init_node('Gesture_Tag_Match')
    your_class_instance = YourClass()
    rospy.spin()
