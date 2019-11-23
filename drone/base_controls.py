import cv2
import numpy as np


class DroneCommand(object):
    """
    Класс для передачи команд управления дроном Tello Edu в заданных
    для него форматах передачи информации.
    """
    def __init__(self):
        # изменения коордиант xyz прямолинейным движением и угла рыскания
        self.move_cmd_format = {
            'left_right_velocity': 0,  # x
            'forward_backward_velocity': 0,  # y
            'up_down_velocity': 0,  # z
            'yaw_velocity': 0,  # yaw
        }

    def __call__(self, *args, **kwargs):
        if len(args[0]) != 4:
            raise Exception(f'Invalid number of movement params, received: {len(args[0])}, expected: 4')

        else:
            for key, val in zip(self.move_cmd_format.keys(), args[0]):
                self.move_cmd_format[key] = val
            return self


class DroneInfo(object):
    """
    Класс для передачи информации, полученной с дрона Tello Edu.
    """
    def __init__(self):
        # доступные состояния для дрона
        self.flight_params = {
            'battery': None,
            'speed': None,
            'height': None,
            'attitude': None,
            'barometer': None,
            'TOF_distance': None,
            'flight_time': None,
        }
        self.frame = None  # изображения из видеопотока
        self.flight_mode = None  # режим управления дрона (4 или 1 канальный)


class CommandController(object):
    """ Система ручного управления всеми модулями ЛА """

    def __init__(self, **args):
        self.cmd_logger = np.zeros((1, 4))   # логгер движения ЛА

        self.send_cmd_control = False  # флаг - контроль управления

        # состояния скорости
        self.default_states = {
            'starting_speed': 10,
            'speed': 3,  # нормировка значений сигналов управления [1, ..., 5]
        }

        # API для автопилота
        self.autopilot_cmd = DroneCommand()
        self.drone_info = DroneInfo()

        # контроль передачи режимов управления
        self.drone_info.flight_mode = '4 canal control'
        self.send_rc_control = True  # флаг - передача команд через 4 канала управления
        self.is_autopilot = False  # флаг - автоплиот

        # видеопоток с ЛА
        self.frame_reader = None

    def key_parser(self, key, drone):
        """ Обработчик команд, поступающих с клавиатуры """

        # Press T to take off
        if key == ord('t'):
            print("Taking Off")
            drone.takeoff()
            self.send_cmd_control = True

        # Press L to land
        if key == ord('l'):
            print("Landing")
            drone.land()
            self.send_cmd_control = False

        # Если есть права для управления оператором
        if self.drone_info.flight_mode == '1 canal control' and self.send_cmd_control:
            cmd_signal = int(self.default_states['starting_speed'] * self.default_states['speed'])

            if key == ord('w'):
                drone.move_forward(cmd_signal)
            elif key == ord('s'):
                drone.move_back(cmd_signal)

            if key == ord('c'):
                drone.move_right(cmd_signal)
            elif key == ord('z'):
                drone.move_left(cmd_signal)

            if key == ord('e'):
                drone.move_up(cmd_signal)
            elif key == ord('q'):
                drone.move_down(cmd_signal)

            if key == ord('d'):
                drone.rotate_clockwise(cmd_signal)
            elif key == ord('a'):
                drone.rotate_counter_clockwise(cmd_signal)

        # Press 1 to set distance to 1
        if key == ord('1'):
            self.default_states['speed'] = 1

        # Press 2 to set distance to 2
        if key == ord('2'):
            self.default_states['speed'] = 2

        # Press 3 to set distance to 3
        if key == ord('3'):
            self.default_states['speed'] = 3

        # Press 4 to set distance to 4
        if key == ord('4'):
            self.default_states['speed'] = 4

        # Press 5 to set distance to 5
        if key == ord('5'):
            self.default_states['speed'] = 5

        # Press m for canal control override
        if key == ord('m'):
            if self.drone_info.flight_mode == '1 canal control':
                self.drone_info.flight_mode = '4 canal control'
            else:
                self.drone_info.flight_mode = '1 canal control'

        # Press p for controls override
        # if key == ord('p'):
        #     if not self.is_autopilot:
        #         self.is_autopilot = True
        #         # self.autopilot.reload()
        #     else:
        #         self.is_autopilot = False

        if self.drone_info.flight_mode == '4 canal control' and self.send_cmd_control:
            cmd_signal = int(self.default_states['starting_speed'] * self.default_states['speed']) * 2

            # S & W to fly forward & back
            if key == ord('w'):
                self.autopilot_cmd.move_cmd_format['forward_backward_velocity'] = cmd_signal
            elif key == ord('s'):
                self.autopilot_cmd.move_cmd_format['forward_backward_velocity'] = - cmd_signal
            else:
                self.autopilot_cmd.move_cmd_format['forward_backward_velocity'] = 0

            # a & d to pan left & right
            if key == ord('d'):
                self.autopilot_cmd.move_cmd_format['yaw_velocity'] = cmd_signal
            elif key == ord('a'):
                self.autopilot_cmd.move_cmd_format['yaw_velocity'] = - cmd_signal
            else:
                self.autopilot_cmd.move_cmd_format['yaw_velocity'] = 0

            # Q & E to fly up & down
            if key == ord('e'):
                self.autopilot_cmd.move_cmd_format['up_down_velocity'] = cmd_signal
            elif key == ord('q'):
                self.autopilot_cmd.move_cmd_format['up_down_velocity'] = - cmd_signal
            else:
                self.autopilot_cmd.move_cmd_format['up_down_velocity'] = 0

            # c & z to fly left & right
            if key == ord('c'):
                self.autopilot_cmd.move_cmd_format['left_right_velocity'] = cmd_signal
            elif key == ord('z'):
                self.autopilot_cmd.move_cmd_format['left_right_velocity'] = - cmd_signal
            else:
                self.autopilot_cmd.move_cmd_format['left_right_velocity'] = 0

        return True

    def display_frame_upd(self, pos_y=700, delta_y=20):
        """
        Отображение информации полета и состояния дрона
        :param pos_y: положение отображаемой информации на кадре по высоте
        :param delta_y: расстояние между строками информации
        :return: bool: True - все прошло успешно, False - в противном случае
        """
        display_color = (255, 255, 255)
        H, W, _ = self.drone_info.frame.shape

        # отображение информации на кадре
        for key in self.drone_info.flight_params:
            if self.drone_info.flight_params[key]:
                cv2.putText(self.drone_info.frame, f"{key}: {self.drone_info.flight_params[key]}", (8, pos_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, display_color, 1)
                pos_y -= delta_y

        additional_info = [self.is_autopilot, self.drone_info.flight_mode, self.default_states['speed']]
        for i, info in enumerate(additional_info):
            cv2.putText(self.drone_info.frame, f"Autopilot: {info}", (8, pos_y - i * delta_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, display_color, 1)

        cv2.circle(self.drone_info.frame, (W // 2, H // 2), 3, (255, 0,), -1)  # отрисовать центр изображения

        cv2.imshow('Main frame', self.drone_info.frame)

        return True

    def reload_controller(self):
        self.autopilot_cmd = DroneCommand()  # обнуление команды
