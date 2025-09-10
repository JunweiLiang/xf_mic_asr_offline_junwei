#!/usr/bin/env python3
# coding=utf-8
# @Author: Aiden
# @Date: 2022/12/14
# modified by Junwei 09/09/2025
# 环形麦克风阵列Python SDK
import re
import time
import json
import rospy
import serial
import signal
from std_srvs.srv import Trigger
from std_msgs.msg import Int32, Bool
from xf_mic_asr_offline.srv import SetString

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

import threading
from datetime import datetime
from unitree_sdk2py.utils.crc import CRC

angle_topic_name = "rt/speaker_angle"
# 50 Hz publishing


class CircleMic:
    def __init__(self, port):
        self.serialHandle = serial.Serial(port, 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=0.02)

        # 2. ADD THIS LINE: Pause the script to give the hardware time to boot up
        time.sleep(1.5)

        self.running = True
        self.key = r"{\"content.*?aiui_event\"}"
        self.key_type = r"{\"code.*?\"}"

    # 麦克风阵列切换
    def switch_mic(self, mic="mic6_circle"):
        # mic：麦克风阵列类型，mic4：线性4麦，mic6：线性6麦， mic6_circle：环形6麦
        param ={
            "type": "switch_mic",
            "content": {
                "mic": "mic6_circle"
            }
        }
        param['content']['mic'] = mic 
        header = [0xA5, 0x01, 0x05]  
        res = self.send(header, param)
        if res is not None:
            pattern = re.compile(self.key_type)
            m = re.search(pattern, str(res))
            if m is not None:
                m = m.group(0)
                if m is not None:
                    return m
        
        return False

    # 获取版本信息
    def get_setting(self):
        param ={
            "type": "version"
        }
        
        header = [0xA5, 0x01, 0x05]  
        res = self.send(header, param)
        if res is not None:
            pattern = re.compile(self.key_type)
            m = re.search(pattern, str(res))
            if m is not None:
                m = m.group(0)
                if m is not None:
                    return m
        
        return False

    # 唤醒词更换（浅定制）
    def set_wakeup_word(self, str_pinyin='xiao3 bai2 xiao3 bai2'):
        # 参数为中文拼音
        # 更多参数请参考https://aiui.xfyun.cn/doc/aiui/3_access_service/access_hardware/r818/protocol.html
        param ={
            "type": "wakeup_keywords",
            "content": {
                "keyword": 'xiao3 bai2 xiao3 bai2',
                "threshold": "900", # 阈值越高越不敏感，越需要发音清晰，不容易误触发；#默认900
            }
        }
        
        param['content']['keyword'] = str_pinyin
        header = [0xA5, 0x01, 0x05]  
        return self.send(header, param)

    # 数据串口发送
    def send(self, header, args):
        result = None
        while True:
            count = self.serialHandle.inWaiting()
            if count != 0:
                recv_data = self.serialHandle.readall()
                if recv_data[0] == 0xa5 and recv_data[1] == 0x01 and (recv_data[2] == 0xff or recv_data[2] == 0x01):  # 握手确认
                    self.serialHandle.write([0xa5, 0x01, 0xff, 0x04, 0x00, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0xb2])  # 握手请求确认
                    packet = header 

                    data = bytes(json.dumps(args), encoding="utf8")

                    length = len(data)
                    low_length = int(length & 0xFF)
                    high_length = int(length >> 8)
                    
                    packet.extend([low_length, high_length])
                    packet.extend([0x00, 0x00])
                    
                    packet.extend(data)

                    # OLD LINE:
                    # checksum = 255 - (sum(packet) % 256) + 1

                    # NEW, SAFER LINE:
                    checksum = (256 - (sum(packet) & 0xFF)) & 0xFF

                    packet.append(checksum)
                    
                    self.serialHandle.write(packet)  # 发送主控消息
                    break
            else:
                self.serialHandle.write([0xa5, 0x01, 0x01, 0x04, 0x00, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0xb0])
                time.sleep(0.1)

        while True:
            count = self.serialHandle.inWaiting()
            if count != 0:
                recv_data = self.serialHandle.readall()
                if recv_data[0] == 0xa5 and recv_data[1] == 0x01 and (recv_data[2] == 0xff or recv_data[2] == 0x01):  # 收到设备消息
                    result = recv_data
                    self.serialHandle.write([0xa5, 0x01, 0xff, 0x04, 0x00, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0xb2])  # 确认消息
                    break
                    
        return result

    #def get_awake_result(self, flag_pub=None, angle_pub=None):
    def get_awake_result(self, node_obj):
        while self.running:
            count = self.serialHandle.inWaiting()
            if count != 0:
                recv_data = self.serialHandle.readall()

                if b'content' in recv_data:
                    pattern = re.compile(self.key)
                    m = re.search(pattern, str(recv_data).replace('\\', ''))
                    if m is not None:
                        m = m.group(0).replace('"{"', '{"').replace('}"', '}')
                        if m is not None:
                            full_data = json.loads(m)
                            angle = int(full_data['content']['info']['ivw']['angle'])

                            #if flag_pub is not None and angle_pub is not None:
                            #    flag_pub.publish(True)
                            #    angle_pub.publish(angle)
                            #    print('>>>>>唤醒角度为: %s\n' % angle)
                            with node_obj.ctrl_lock:
                                node_obj.current_angle_data["angle"] = angle
                                node_obj.current_angle_data["timestamp"] = time.time()
                            print_with_time("recv_data：%s" % full_data)
                            print_with_time("唤醒角度：%s" % angle)
            else:
                self.serialHandle.write([0xa5, 0x01, 0x01, 0x04, 0x00, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0xb0])
                time.sleep(0.2)
    
    def stop(self):
        self.running = False

def print_with_time(*args, **kwargs):
    timestamp = datetime.now().strftime("[%Y-%m-%d %H:%M:%S]")
    print(timestamp, *args, **kwargs)

class AwakeNode:
    def __init__(self, name):
        rospy.init_node(name)
        signal.signal(signal.SIGINT, self.shutdown)

        mic_type = rospy.get_param('~mic_type', 'mic6_circle')
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        awake_word = rospy.get_param('~awake_word', 'xiao3 bai2 xiao3 bai2')
        network_interface = rospy.get_param('~network_interface', 'eth0')

        # 3. Wait for the physical hardware to boot up.
        rospy.loginfo("Waiting for microphone hardware to initialize...")
        time.sleep(3)

        # 4. Now, connect to the serial port.
        try:
            self.mic = CircleMic(port)
            rospy.loginfo("Successfully connected to microphone.")
        except serial.serialutil.SerialException as e:
            rospy.logerr(f"Failed to connect to microphone on port {port}: {e}")
            rospy.signal_shutdown("Could not initialize microphone hardware.")
            return

        # we do not need these service
        #rospy.Service('~set_mic_type', SetString, self.set_mic_type_srv)
        #rospy.Service('~get_setting', Trigger, self.get_setting_srv)
        #rospy.Service('~set_wakeup_word', SetString, self.set_wakeup_word_srv)

        # Junwei: we do not publish to ROS now
        #self.awake_angle_pub = rospy.Publisher('~awake_angle', Int32, queue_size=1)
        #self.awake_flag_pub = rospy.Publisher('~awake_flag', Bool, queue_size=1)

        print_with_time(self.mic.switch_mic(mic_type))
        print_with_time("设置唤醒词结果：")
        print_with_time(self.mic.set_wakeup_word(awake_word))

        print_with_time('>>>>>Wake up word: %s' % awake_word)

        ChannelFactoryInitialize(0, network_interface)

        #  added by Junwei. Publish the awake angle to a DDS topic
        # the content will be a json string
        self.angle_publisher = ChannelPublisher(angle_topic_name, String_)
        self.angle_publisher.Init()
        self.control_dt = 1/50.0

        # initial data
        # timestamp is when the robot recognize a person and its angle
        self.msg = String_("")
        self.current_angle_data = {"angle": 0, "timestamp": time.time()}
        # publish the angle with timestamp constantly
        self.publish_thread = threading.Thread(target=self._publish_angle)
        self.ctrl_lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        # this runs forever, update the angle  data when needed
        self.mic.get_awake_result(self)

    def _publish_angle(self):
        while True:
            start_time = time.time()

            with self.ctrl_lock:
                angle_data = self.current_angle_data

            self.msg.data = json.dumps(angle_data)
            self.angle_publisher.Write(self.msg)

            current_time = time.time()
            all_t_elapsed = current_time - start_time
            sleep_time = max(0, (self.control_dt - all_t_elapsed))
            time.sleep(sleep_time)

       
    def shutdown(self, signum, frame):
        self.mic.stop()
        rospy.loginfo('shutdown')

if __name__ == '__main__':
    AwakeNode('mic')
