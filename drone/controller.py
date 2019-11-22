import cv2
import time
import numpy as np

from drone.tools import RouteCreator, Autopilot
from drone.base_controls import CommandController, DroneCommand
from drone.djitellopy.tello import Tello
from drone.cv_system import SystemCV


class DroneController(CommandController):
    """ Система управления всеми модулями дрона """

    def __init__(self):
        # инициализция класса управления
        super(DroneController, self).__init__()

        def _init_autopilot():
            _route_creator = RouteCreator()
            standard_route = _route_creator.left_right_route()  # маршрут поиска целей
            autopilot = Autopilot(standard_route)
            return autopilot

        self.autopilot = _init_autopilot()  # система автопилота - класс Autopilot

        # методы взаимодействия с дроном
        self.drone = Tello()
        # методы взаимодействия с системой технического зрения
        self.cv_system = SystemCV()

        # Остальные методы класса системы управления
        # Ваш код ...

    def upd_flight_params(self):
        """
        Обновить текущее состояние параметров полета дрона
        Изменяется значение метода self.drone_info
        :return: bool: True - все прошло успешно, False - в другом случае
        """
        self.drone_info.flight_params['battery'] = self.drone.get_battery()
        # self.drone_info.flight_params['speed'] = self.drone.get_speed()
        # self.drone_info.flight_params['height'] = self.drone.get_height()
        self.drone_info.flight_params['attitude'] = self.drone.get_attitude()
        self.drone_info.flight_params['barometer'] = self.drone.get_barometer()
        # self.drone_info.flight_params['TOF_distance'] = self.drone.get_distance_tof()
        # self.drone_info.flight_params['flight_time'] = self.drone.get_flight_time()

        return True

    # def upd_frame_read(self):
    #     """
    #     Вернуть текущее изображение из видеопотока
    #     :return: np.array: текущее изображение из видеопотока
    #     """
    #     frame = self.drone.get_frame_read().frame
    #     return frame

    def upd_flight_velocity_state(self):
        """
        Отправить сигналы управления в дрон
        :return: bool: True - все прошло успешно, False - в противном случае
        """
        if not self.is_autopilot and self.send_cmd_control:
            # передать сигналы управления в дрон
            self.drone.send_rc_control(**self.autopilot_cmd.move_cmd_format)

            # сохранить историю команд управления
            self.cmd_logger = np.vstack((self.cmd_logger, list(self.autopilot_cmd.move_cmd_format.values())))
            return True

        if self.is_autopilot and self.send_cmd_control:
            # маштабирование скорости по заданому ограничению на значения сигналов управления
            for key in self.autopilot_cmd.move_cmd_format:
                if key != 'yaw_velocity':
                    self.autopilot_cmd.move_cmd_format[key] *= self.default_states['speed'] * 2 / 10  # нормировка
                self.autopilot_cmd.move_cmd_format[key] = int(self.autopilot_cmd.move_cmd_format[key])

            # передать сигналы управления в дрон
            self.drone.send_rc_control(**self.autopilot_cmd.move_cmd_format)

            # сохранить историю команд управления
            self.cmd_logger = np.vstack((self.cmd_logger, list(self.autopilot_cmd.move_cmd_format.values())))
            self.autopilot_cmd = DroneCommand()  # обнуление команды
            return True

    # def send_drone_info(self):
    #     """ Отправить информацию с дрона """
    #     return self.drone_info

    def video_loop(self):
        """ Основной цикл для обработки видеопотока и формирования поведения дрона """
        should_stop = False  # флаг остановки
        frame_counter = 0  # счетчиков кадров видеопотока
        FPS = 30  # предполагаемая частота кадров
        self.frame_reader = self.drone.get_frame_read()  # thread для чтения видеопотока
        self.drone_info.frame = self.frame_reader.frame  # инициализация первого изображения из видепотока

        # цикл упавления
        while not should_stop:
            try:
                # обновить состояние дрона
                self.upd_flight_velocity_state()
                # задержка для отклика приемника сигналов управления
                time.sleep(1 / FPS)

                # преобразовать текущий сигнал видеопотока в изображение
                self.drone_info.frame = self.frame_reader.frame
                if self.drone_info.frame is None or self.drone_info.frame.size == 0:
                    continue

                # обновить счетчик кадров
                frame_counter += 1
                frame_counter = 0 if frame_counter % 1002 == 0 else frame_counter

                # получить команду с ручного управления
                key = cv2.waitKey(1)
                self.key_parser(key, self.drone)
                if key == 27:
                    should_stop = True

                # управление через автопилот
                if self.is_autopilot:
                    self.autopilot_loop()

                # обновление отображаемой информации
                if frame_counter % FPS == 0:
                    self.upd_flight_params()
                self.display_frame_upd()

            except IndexError:
                print('something goes wrong...')
                self.drone.land()
                should_stop = True

            except Exception:
                self.drone.background_frame_read.stop()
                cv2.destroyAllWindows()
                self.drone.land()
                should_stop = True

        # очистка буфферов
        print('[INFO] Battery lvl:', self.drone.get_battery())
        self.drone.background_frame_read.stop()
        cv2.destroyAllWindows()
        self.drone.end()

    def autopilot_loop(self):
        """ Центральный цикл для запуска всего интерфейса """
        # найти цель в текущем self.drone_info,frame
        self.cv_system.find_target(self.drone_info.frame)

        # навестись на цель, если она найдена
        if self.cv_system.is_target_detected:
            """
            To-Do: алгоритм машрута двжиения к найденной цели
            """
            pass

        # запустить алгоритм поиска цели
        else:
            """
            To-Do: алгортм маршрута поиска потенциальной цели
            """
            # Ваш код
            pass


