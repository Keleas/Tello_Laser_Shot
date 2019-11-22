import cv2
import numpy as np
from drone.cv_system import SystemCV
from drone.base_controls import CommandController


class TestController(CommandController):
    """ Моделирование работы автопилота """
    def __init__(self):
        super(TestController, self).__init__()

        self.cv_system = SystemCV()

    def run(self):
        """ Основной цикл запуска тестировщика """
        video_stream = cv2.VideoCapture(0)
        num_frames = 0

        while True:

            ret, frame = video_stream.read()
            self.drone_info.frame = frame.copy()
            num_frames = num_frames + 1 if num_frames < 1000 else 0

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

            self.cv_system.find_target(self.drone_info.frame)

            self.display_frame_upd(pos_y=450)

        cv2.destroyAllWindows()
        video_stream.release()