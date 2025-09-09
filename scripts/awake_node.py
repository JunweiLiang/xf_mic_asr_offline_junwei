#!/usr/bin/env python3
# coding=utf-8
# @Author: Aiden
# @Date: 2022/12/14
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

class CircleMic:
    def __init__(self, port):
        self.serialHandle = serial.Serial(port, 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=0.02)
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
    def set_wakeup_word(self, str_pinyin="xiao3 huan4 xiao3 huan4"):
        # 参数为中文拼音
        # 更多参数请参考https://aiui.xfyun.cn/doc/aiui/3_access_service/access_hardware/r818/protocol.html
        param ={
            "type": "wakeup_keywords",
            "content": {
                "keyword": "xiao3 huan4 xiao3 huan4",
                "threshold": "500", 
            }
        }
        
        param['content']['keyword'] = str_pinyin
        header = [0xA5, 0x01, 0x05]  
        self.send(header, param)

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

                    checksum = 255 - (sum(packet) % 256) + 1
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

    def get_awake_result(self, flag_pub=None, angle_pub=None):
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
                            angle = int(json.loads(m)['content']['info']['ivw']['angle'])
                            if flag_pub is not None and angle_pub is not None:
                                flag_pub.publish(True)
                                angle_pub.publish(angle)
                                print('>>>>>唤醒角度为: %s\n' % angle)
            else:
                self.serialHandle.write([0xa5, 0x01, 0x01, 0x04, 0x00, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0xb0])
                time.sleep(0.2)
    
    def stop(self):
        self.running = False

class AwakeNode:
    def __init__(self, name):
        rospy.init_node(name)

        signal.signal(signal.SIGINT, self.shutdown)

        mic_type = rospy.get_param('~mic_type', 'mic6_circle')
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        awake_word = rospy.get_param('~awake_word', 'xiao3 huan4 xiao3 huan4')
        self.mic = CircleMic(port)
        rospy.Service('~set_mic_type', SetString, self.set_mic_type_srv)
        rospy.Service('~get_setting', Trigger, self.get_setting_srv)
        rospy.Service('~set_wakeup_word', SetString, self.set_wakeup_word_srv)

        self.awake_angle_pub = rospy.Publisher('~awake_angle', Int32, queue_size=1)
        self.awake_flag_pub = rospy.Publisher('~awake_flag', Bool, queue_size=1)
        self.mic.switch_mic(mic_type)
        self.mic.set_wakeup_word(awake_word)
        print('>>>>>Wake up word: %s' % awake_word)
        self.mic.get_awake_result(self.awake_flag_pub, self.awake_angle_pub)

    def set_mic_type_srv(self, msg):
        res = self.mic.switch_mic(msg.data) 
        return [True, str(res)]

    def get_setting_srv(self, msg):
        res = self.mic.get_setting()
        return [True, str(res)]

    def set_wakeup_word_srv(self, msg):
        self.mic.set_wakeup_word(msg.data)
        return [True, 'success']
       
    def shutdown(self, signum, frame):
        self.mic.stop()
        rospy.loginfo('shutdown')

if __name__ == '__main__':
    AwakeNode('mic')
