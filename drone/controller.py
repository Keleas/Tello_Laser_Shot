import cv2
import time
import numpy as np

from drone.tools import RouteCreator, Autopilot
from drone.base_controls import CommandController, DroneCommand
from drone.djitellopy.tello import Tello
from drone.cv_system import SystemCV
from drone.tools import SignalTransformer


class DroneController(CommandController):
    """ Система управления всеми модулями дрона для автопилота"""

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
        self.is_autopilot = False
        # методы взаимодействия с системой технического зрения
        cv_args = {'laser_on': True,
                   }
        self.cv_system = SystemCV(**cv_args)

        self.frame_stops = 0
        self.should_stop = False
        self.stop = 0
        self.count = 0
        self.handsome = 0
        self.rip = 0

        # Остальные методы класса системы управления
        # Ваш код ...

    def upd_flight_params(self):
        """
        Обновить текущее состояние параметров полета дрона
        Изменяется значение метода self.drone_info
        :return: bool: True - все прошло успешно, False - в другом случае
        """
        # self.drone_info.flight_params['battery'] = self.drone.get_battery()
        # self.drone_info.flight_params['speed'] = self.drone.get_speed()
        # self.drone_info.flight_params['height'] = self.drone.get_height()
        # self.drone_info.flight_params['attitude'] = self.drone.get_attitude()
        # self.drone_info.flight_params['barometer'] = self.drone.get_barometer()
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
                # if key != 'yaw_velocity':
                #     self.autopilot_cmd.move_cmd_format[key] *= self.default_states['speed'] * 2 / 10  # нормировка
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
        while not self.should_stop:
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
                    self.should_stop = True
                # управление через автопилот
                if self.is_autopilot:
                    self.autopilot_loop()

                # обновление отображаемой информации
                if frame_counter % FPS == 0:
                    self.upd_flight_params()
                self.display_frame_upd()

            except IndexError:
                print('index wrong...')
                self.drone.land()
                self.should_stop = True

            # except Exception:
            #     self.drone.background_frame_read.stop()
            #     cv2.destroyAllWindows()
            #     self.drone.land()
            #     should_stop = True

        # очистка буфферов
        print('[INFO] Battery lvl:', self.drone.get_battery())
        self.drone.background_frame_read.stop()
        cv2.destroyAllWindows()
        self.drone.end()

    def autopilot_loop(self):
        """ Центральный цикл для запуска всего интерфейса """
        # найти цель в текущем self.drone_info,frame
        target = self.cv_system.find_target(self.drone_info.frame)
        # навестись на цель, если она найдена
        """
            'left_right_velocity': 0,  # x
            'forward_backward_velocity': 0,  # y
            'up_down_velocity': 0,  # z
            'yaw_velocity': 0,  # yaw
            1280×720 
        """

        h_frame, w_frame, _ = self.drone_info.frame.shape  # размеры картинки

        if self.stop == 0:  # флаг для старта
            # self.drone.takeoff()

            self.send_cmd_control = True
            self.stop = 1  # режим полета

        if self.cv_system.is_target_detected:
            self.frame_stops += 1
            # potential_targets = [высота, ширина, x, y - центр мишени]
            h = int(target[0][0])
            w = int(target[0][1])
            x = int(target[0][2])
            y = int(target[0][3])
            alpha = np.arccos(min(h, w)/max(h, w))*360/np.pi
            yaw_direction = 1

            default_speed = 8
            default_epsilon = 30  # max(h, w)/30
            default_distance = 40

            fire = True
            transformer = SignalTransformer()
            distance_to_target = transformer.fit(h, w)  # фактическое расстоняие до объекта

            # print(x, y, 'delta:', x - w_frame/2, y//2 - h_frame//2)
            cv2.circle(self.drone_info.frame, (x, y), 2, (0, 0, 255), -1)  # отрисовать центр изображения
            cv2.circle(self.drone_info.frame, (x, y), default_epsilon, (0, 255, 160), 2)  # отрисовать центр изображения
            cv2.line(self.drone_info.frame, (w_frame//2, h_frame//2), (x, y), (0, 0, 255), 2)
            if distance_to_target - default_distance > 40:
                self.autopilot_cmd.move_cmd_format['forward_backward_velocity'] = default_speed
                # fire = False
                pass
            elif default_distance - distance_to_target > 40:
                # self.autopilot_cmd.move_cmd_format['forward_backward_velocity'] = - default_speed
                # fire = False
                pass
            if x - w_frame//2 > default_epsilon:  # движение налево
                # self.autopilot_cmd.move_cmd_format['left_right_velocity'] = 50*(640-x)/640
                # self.autopilot_cmd.move_cmd_format['left_right_velocity'] = default_speed
                fire = False
            elif w_frame//2 - x > default_epsilon:  # двжиение направо
                # self.autopilot_cmd.move_cmd_format['left_right_velocity'] = -100*(640-x)/640
                self.autopilot_cmd.move_cmd_format['left_right_velocity'] = - default_speed
                fire = False
            if y - h_frame//2 > default_epsilon:  # движение вверх
                # self.autopilot_cmd.move_cmd_format['up_down_velocity'] = -100*(640-x)/640
                self.autopilot_cmd.move_cmd_format['up_down_velocity'] = - default_speed
                fire = False
            elif h_frame//2 - y > default_epsilon:  # двжиние вниз
                # self.autopilot_cmd.move_cmd_format['up_down_velocity'] = 100*(640-x)/640
                self.autopilot_cmd.move_cmd_format['up_down_velocity'] = default_speed
                fire = False

            print(self.autopilot_cmd.move_cmd_format)  # debug mode

            if fire:
                print('fire on')  # debug mode
                self.cv_system.laser.fire('FireC', puls_dur=150)
                cv2.imwrite('fire.jpg', self.drone_info.frame)

        # запустить алгоритм поиска цели
        elif self.frame_stops > 10:
            """
            To-Do: алгортм маршрута поиска потенциальной цели
            """
            # self.frame_stops = 0
            # cmd_signal = 0
            # cmd2_signal = 0
            # if self.handsome <= 100:
            #     cmd_signal = 15
            #     self.handsome += 1
            # elif self.handsome == 101:
            #     cmd_signal = -15
            #     self.handsome += 1
            # elif self.handsome == 102 and (self.rip >= 0 and self.rip <= 100):
            #     cmd2_signal = 10
            #     self.rip += 1
            # elif (self.handsome > 102 and self.handsome < 202) or self.rip == 101:
            #     cmd_signal = -15
            #     self.rip = 0
            #     self.handsome += 1
            # elif self.handsome == 202:
            #     cmd_signal = 15
            #     self.handsome += 1
            # elif self.handsome == 203 and (self.rip >= 0 and self.rip <= 100):
            #     cmd2_signal = 10
            #     self.rip += 1
            # elif (self.handsome > 203 and self.handsome < 303) or self.rip == 101:
            #     cmd_signal = 15
            #     self.rip = 0
            #     self.handsome += 1
            # elif self.handsome == 303:
            #     cmd_signal = -15
            #     self.handsome += 1
            # elif self.handsome == 304 and (self.rip >= 0 and self.rip <= 100):
            #     cmd2_signal = 10
            #     self.rip += 1
            # elif (self.handsome > 304 and self.handsome < 404) or self.rip == 101:
            #     cmd_signal = -15
            #     self.rip = 0
            # elif self.handsome == 404:
            #     cmd_signal = 15
            #     self.handsome += 1
            #
            # self.autopilot_cmd.move_cmd_format['left_right_velocity'] = cmd_signal
            # self.autopilot_cmd.move_cmd_format['forward_backward_velocity'] = 0
            # self.autopilot_cmd.move_cmd_format['yaw_velocity'] = 0
            # self.autopilot_cmd.move_cmd_format['up_down_velocity'] = cmd2_signal

            pass

        if self.cv_system.counter_finder == 2 or self.handsome == 405:
            import time
            print("YAYYAYAYAYAYAYYAYAYAYAYAY")
            time.sleep(1)
            # self.drone.land()  # debug
            # self.should_stop = True



