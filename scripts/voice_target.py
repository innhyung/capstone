#!/usr/bin/env python3
"""
음성 명령 → 타겟 객체 → ROS2 토픽 publish.

사용 예:
    "리모컨 찾아줘"  →  TTS "네 리모컨을 찾을게요"  →  /target_object 에 "remote" publish
    "마우스 어디 있어"  →  TTS "네 마우스를 찾을게요"  →  "mouse" publish
"""
import os
import subprocess
import tempfile
import speech_recognition as sr
from gtts import gTTS

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# 한국어 → COCO class (영어) 매핑
# COCO 80 classes 중 실내에서 흔히 찾을 만한 것들
KO_TO_COCO = {
    "리모컨": "remote", "리모콘": "remote",
    "마우스": "mouse",
    "키보드": "keyboard",
    "노트북": "laptop",
    "휴대폰": "cell phone", "핸드폰": "cell phone", "스마트폰": "cell phone",
    "책": "book",
    "컵": "cup",
    "병": "bottle", "물병": "bottle",
    "가방": "backpack", "백팩": "backpack",
    "핸드백": "handbag",
    "우산": "umbrella",
    "시계": "clock",
    "가위": "scissors",
    "칫솔": "toothbrush",
    "사과": "apple", "바나나": "banana", "오렌지": "orange",
    "케이크": "cake", "도넛": "donut", "피자": "pizza", "샌드위치": "sandwich",
    "포크": "fork", "칼": "knife", "숟가락": "spoon", "그릇": "bowl",
    "와인잔": "wine glass",
    "TV": "tv", "텔레비전": "tv",
    "의자": "chair",
    "쇼파": "couch", "소파": "couch",
    "침대": "bed",
    "식탁": "dining table", "테이블": "dining table",
    "변기": "toilet",
    "화병": "vase", "꽃병": "vase",
    "사람": "person",
    "고양이": "cat", "강아지": "dog", "개": "dog",
}


def particle_eul_reul(word: str) -> str:
    """한국어 단어의 받침 유무로 '을' / '를' 결정."""
    if not word:
        return "를"
    last = word[-1]
    code = ord(last) - 0xAC00
    if not 0 <= code <= 11171:
        return "를"
    has_batchim = (code % 28) != 0
    return "을" if has_batchim else "를"


def speak(text: str):
    """gTTS로 한국어 음성 생성 후 ffplay로 재생 (block)."""
    with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as f:
        path = f.name
    try:
        gTTS(text=text, lang="ko").save(path)
        subprocess.run(
            ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", path],
            check=False,
        )
    finally:
        try:
            os.remove(path)
        except OSError:
            pass


def extract_target(text: str):
    """발화 안의 한국어 키워드 → (한국어 단어, COCO 클래스명) 또는 None."""
    for ko, en in KO_TO_COCO.items():
        if ko in text:
            return ko, en
    return None


class VoiceTargetNode(Node):
    def __init__(self):
        super().__init__("voice_target_node")
        self.publisher_ = self.create_publisher(String, "/target_object", 10)
        self.recognizer = sr.Recognizer()
        # 말 중간에 잠깐 멈춰도 문장이 끊기지 않도록 길게
        self.recognizer.pause_threshold = 1.5
        self.recognizer.non_speaking_duration = 0.8
        with sr.Microphone() as source:
            self.get_logger().info("주변 소음 측정 중... (1초)")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
        self.get_logger().info("준비 완료. ('그만' 이라고 말하면 종료)")
        speak("어떤 물건을 찾고 싶으신가요")

    def listen_once(self):
        try:
            with sr.Microphone() as source:
                self.get_logger().info("듣는 중...")
                # timeout: 말 시작까지 최대 대기 시간 / phrase_time_limit: 한 문장 최대 길이
                audio = self.recognizer.listen(source, timeout=10, phrase_time_limit=15)
        except sr.WaitTimeoutError:
            self.get_logger().warn("아무 소리도 안 들렸어요.")
            return None

        try:
            text = self.recognizer.recognize_google(audio, language="ko-KR")
            self.get_logger().info(f"입력: '{text}'")
            return text
        except sr.UnknownValueError:
            self.get_logger().warn("못 알아들었어요.")
            speak("다시 말씀해주세요")
            return None
        except sr.RequestError as e:
            self.get_logger().error(f"Google STT 연결 실패: {e}")
            return None

    def run(self):
        EXIT_KEYWORDS = ("그만", "종료", "끝")
        while rclpy.ok():
            text = self.listen_once()
            if text is None:
                continue

            # 종료 명령은 객체 매칭보다 먼저 체크
            if any(kw in text for kw in EXIT_KEYWORDS):
                self.get_logger().info("종료 명령 인식.")
                speak("네 물건찾기를 종료할게요")
                break

            hit = extract_target(text)
            if hit is None:
                self.get_logger().warn("매핑되는 객체 없음.")
                speak("어떤 물건을 찾을지 모르겠어요")
                continue

            ko_word, coco_class = hit
            response = f"네 {ko_word}{particle_eul_reul(ko_word)} 찾을게요"
            self.get_logger().info(f"→ publish '{coco_class}'  ({response})")
            speak(response)

            msg = String()
            msg.data = coco_class
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceTargetNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
