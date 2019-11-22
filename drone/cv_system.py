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
