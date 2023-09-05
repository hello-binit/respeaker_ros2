#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

import speech_recognition as SR

from actionlib_msgs.msg import GoalStatus
from audio_common_msgs.msg import AudioData
from sound_play_msgs.action import SoundRequest as SoundRequestAction
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class SpeechToText(Node):
    def __init__(self):
        super().__init__("speech_to_text")
        
        # format of input audio data
        self.sample_rate = self.declare_parameter("sample_rate", 16000).value
        self.sample_width = self.declare_parameter("sample_width", 2).value

        # language of STT service
        self.language = self.declare_parameter("language", "en-US").value

        # ignore voice input while the robot is speaking
        self.self_cancellation = self.declare_parameter("self_cancellation", True).value

        # time to assume as SPEAKING after tts service is finished
        self.tts_tolerance = Duration(seconds=self.declare_parameter("tts_tolerance", 1.0).value)

        self.recognizer = SR.Recognizer()

        self.tts_action = None
        self.last_tts = None
        self.is_canceling = False
        if self.self_cancellation:
            self.tts_action = ActionClient(self, SoundRequestAction, "sound_play")
            
            if self.tts_action.wait_for_server(timeout_sec=5.0):
                self.tts_timer = self.create_timer(0.1, self.tts_timer_cb)
            else:
                self.get_logger().error("action sound_play is not initialized.")
                self.tts_action = None

        self.pub_speech = self.create_publisher(SpeechRecognitionCandidates, "speech_to_text", 1)
        self.sub_audio = self.create_subscription(AudioData, "speech_audio", self.audio_cb, 1)

    def tts_timer_cb(self):
        stamp = self.get_clock().now()
        active = False
        # for st in self.tts_action.action_client.last_status_msg.status_list:
        #     if st.status == GoalStatus.ACTIVE:
        #         active = True
        #         break
        # if active:
        #     if not self.is_canceling:
        #         self.get_logger().debug("START CANCELLATION")
        #         self.is_canceling = True
        #         self.last_tts = None
        # elif self.is_canceling:
        #     if self.last_tts is None:
        #         self.last_tts = stamp
        #     if stamp - self.last_tts > self.tts_tolerance:
        #         self.get_logger().debug("END CANCELLATION")
        #         self.is_canceling = False

    def audio_cb(self, msg):
        # if self.is_canceling:
        #     self.get_logger().info("Speech is cancelled")
        #     return
        data = SR.AudioData(bytes(msg.data), self.sample_rate, self.sample_width)

        try:
            self.get_logger().info("Waiting for result %d" % len(data.get_raw_data()))
            result, confidence = self.recognizer.recognize_google(
                data, language=self.language, show_all=False, with_confidence=True)

            msg = SpeechRecognitionCandidates(transcript=[result], confidence=[confidence])
            self.pub_speech.publish(msg)
        except SR.UnknownValueError as e:
            self.get_logger().error("Failed to recognize: %s" % str(e))
            self.get_logger().info("value error")
        except SR.RequestError as e:
            self.get_logger().error("Failed to recognize: %s" % str(e))
            self.get_logger().info("request error")


def main():
    rclpy.init()
    stt = SpeechToText()
    rclpy.spin(stt)

if __name__ == '__main__':
    main()