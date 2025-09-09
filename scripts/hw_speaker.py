#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/12/15
# @author:aiden
import json
import rospy
from std_msgs.msg import String, Int32
from xf_mic_asr_offline import voice_play

language = 'Chinese'
def play(name):
    voice_play.play(name, language=language)

def words_callback(msg):
    words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
    if language == 'Chinese':
        words = words.replace(' ', '')
    print('words:', words)
    if words is not None and words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)', '失败10次(Fail-10-times']:
        if words == '过来':
            play('come')
        if words == '回来':
            play('come')
        if words == '前进':
            play('come')
        if words == '后退':
            play('come')
        if words == '打开空调':
            play('come')
        if words == '打开电视':
            play('come')
        if words == '关闭空调':
            play('come')
        if words == '关闭电视':
            play('come')
        if words == '导航到前台':
            play('come')
        print('>>>>>>do something\n')
    elif words == '唤醒成功(wake-up-success)':
        play('awake')
    elif words == '休眠(Sleep)':
        pass

def angle_callback(msg):
    print('angle:', msg.data)

rospy.init_node('hw_speaker', anonymous=True)
rospy.Subscriber('/call_recognition/voice_words', String, words_callback)
rospy.Subscriber('/awake_node/awake_angle', Int32, angle_callback)
while True:
    try:
        if rospy.get_param('/xf_asr_offline_node/init_finish'):
            break
    except:
        rospy.sleep(0.1)

play('running')

rospy.loginfo('唤醒口令: 小幻小幻(Wake up word: hello hiwonder)')
rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
rospy.loginfo('控制指令: 左转 右转 前进 后退 漂移 过来(Voice command: turn left/turn right/go forward/go backward/drift/come here)')

try:
    rospy.spin()
except Exception as e:
    rospy.logerr(str(e))
    rospy.loginfo("Shutting down")
