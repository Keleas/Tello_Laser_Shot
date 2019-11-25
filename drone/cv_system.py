import cv2
import numpy as np

from drone.tools import LaserController
from drone.base_controls import DroneCommand


def is_true_object(**args):
    """
    Подтверждение, что цель имеет заданный цвет
    :param args: dict: список эмпирик, которые описывают объект
    :return bool: True - это заданная цель, False - все остальное
    """

    """
    To-Do: алгоритм поиска цели по заданным эмпирикам
    """
    # Ваш код ...
    pass


class SystemCV(object):
    """ Система технического зрения для обработки видеоизображений """

    def __init__(self, **args):
        # методы класса для передачи команд дрону
        self.move_cmd = DroneCommand()
        # методы класса управления лазерной указкой

        if args['laser_on']:
            self.laser = LaserController()

        # размеры исходного изображения
        self.W = None  # ширина кадра
        self.H = None  # высота кадра
        self.h_shift = None  # нормировка на фактический центр обзора дрона
        self.h_laser = None  # нормировка на центр пятна лазера на изображении

        # флаг - цель об обнаружении цели
        self.is_target_detected = False

        # параметры цели
        self.target_color_params =  {'mode': 'target',
                                     # эмпирики для цели
                                     }
        # параметры индикатора поражения цели
        self.detector_color_params = {'mode': 'detector',
                                      # эмпирики для индикатора поражения цели
                                      }

    def find_target(self, frame):
        """
        Поиск цели по геомтерическим соотношениям и цвету
        :param frame: np.array: избражение из видеопотока
        :return: np.array: массив потенциальных целей
        """
        # инициализация размеров изображения
        if not self.H and not self.W:
            self.H, self.W, _ = frame.shape

        """
        To-Do: алгоритм поиска заданной цели
        """
        # Ваш код ...

        # изначально цель не найдена
        self.is_target_detected = False

        potential_targets = np.array([])
        
        # На вход мы получаем видео с БПЛА

        img_bnw = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
        img = cv2.GaussianBlur(img_bnw, (5, 5), 0)
        bilF = cv2.bilateralFilter(img, d=2, sigmaColor=250, sigmaSpace=250)

        edges = cv2.Canny(bilF, 90, 360, 5)

        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        contours_new = []
        h, w = frame.shape[:2]

        for cnt in contours:
            if cv2.contourArea(cnt) > 1000 and cv2.contourArea(cnt) < h * w - 100000:
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                peri_app = cv2.arcLength(approx, True)

                if 7 < len(approx) < 10 and peri_app / peri < 0.97 and peri_app / peri > 0.89:

                    mask = np.ones_like(img_bnw.copy())
                    cv2.drawContours(mask, [cnt], -1, 255, -1)

                    out = np.ones_like(frame)
                    out[mask == 255] = frame[mask == 255]

                    x, y, w, h = cv2.boundingRect(cnt)
                    roi = out[y:y + h, x:x + w]
                    roi_hsv = cv2.cvtColor(roi.copy(), cv2.COLOR_BGR2HSV)
                    roi_hsv_mask = cv2.inRange(roi_hsv, (0, 0, 0), (255, 10, 10))
                    s = np.sum(cv2.bitwise_not(roi_hsv_mask))
                    k = roi_hsv_mask.shape[0] * roi_hsv_mask.shape[1]

                    percent = (s / 255) / k
                    
                    blue_roi = roi_hsv.copy()
                    blue_roi = cv2.inRange(blue_roi, (70, 100, 100), (110, 255, 255))

                    sum_blue = np.sum(blue_roi)

                    if percent > 0.65 and sum_blue < 10:
                        potential_targets.append((h, w, x + w / 2, y + h / 2)) # potential_targets = [высота, ширина, x, y - центр мишени]
                        contours_new.append(cnt)
                        
                        self.is_target_detected = True


        return potential_targets

    def keep_priority_target(self, targets, mode='nearest'):
        """
        Стратегия выбора оптимальной цели
        :param targets: массив целей
        :param mode: стратегия выбора оптимальной цели ['nearest']
        :return: int - индекс приоритетной цели
        """
        target_idx = None  # индекс приоритетной цели в targets

        """
        To-Do: реализовать стратегии выбора оптимальной цели для поражения из нескольких
        """
        # выбор статегии определения оптимальной цели
        if mode == 'nearest':
            target_idx = self.nearest_priority_strategy(targets)

        else:
            # Ваш код ...
            target_idx = None
            pass

        # обновить состояние обнаружения цели
        self.is_target_detected = True

        return target_idx

    def nearest_priority_strategy(self, targets):
        """
        Стратегия выбора оптимальной цели по кртачайшему смещению X, Y до цели
        :param targets:
        :return: int: индекс приоритетной цели в массиве targets
        """
        priority_idx = 0  # индекс приоритетной цели в массеве targets

        """
        To-Do: расчет ближайшей цели
        """
        # Ваш код ...
        pass

        return priority_idx

    def get_movement_params(self, target):
        """
        Пересчет положения цели в команду движения
        :param target: опционаьная цель
        :return: DroneCommand(): команды управления
        """

        """
        To-Do: расчет смещения и команд управления для наведения на цель
        """
        # Ваш код ...
        pass

        """
        To-Do: условия для условного выстрела в цели 
        """
        # Ваш код ...
        pass


positions = {
    'hand_pose': (15, 40),
    'fps': (15, 20),
    'null_pos': (200, 200),
}
if __name__ == '__main__':
    """ Тестирование отдельных классов """
    args = {'laser_on': False,
            'test_var': 10}
    cv_module = SystemCV(**args)
    video_stream = cv2.VideoCapture(0)

    num_frames = 0
    while True:
        ret, frame = video_stream.read()
        frameRet = frame.copy()
        num_frames = num_frames + 1 if num_frames < 1000 else 0
        timer = cv2.getTickCount()

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

        cv_module.find_target(frameRet)

        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        cv2.putText(frameRet, 'FPS : ' + str(int(fps)), positions['fps'], cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (255, 255, 255), 1)
        cv2.imshow('Main frame', frameRet)

    cv2.destroyAllWindows()
    video_stream.release()
