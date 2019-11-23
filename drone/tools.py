import socket
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3

from drone.base_controls import DroneCommand

plt.rcParams['animation.html'] = 'html5'


class SignalTransformer(object):
    """
    Класс для преобразования сигналов управления в физические величины и наоборот
    """
    def __init__(self):
        pass

    def signal_to_real(self, signal):
        """
        Преобразовать сигнал в физические величины
        :param signal: сигнал управления типа
                    [left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity]
        :return: физические величины типа
                    [x, y, z, angle]
        """
        signal[3] = signal[3] * 2 / 100  # нормировка ручная [DEBUG]
        return signal

    def real_to_signal(self, real):
        """
        Преобразовать физические велечины в сигнал
        :param real: физические величины типа
                    [x, y, z, angle]
        :return: сигнал управления типа
                    [left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity]
        """
        return real

    def fit(self):
        """ Расчитать регрессию по определению дальности до объекта """
        pass


class RouteCreator(object):
    """
    Класс для формированя заданных маршрутов движения дрона
    Маршрут из себя представляет
    route = [time, [left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity]]

    Допустимые значения:
        left_right_velocity: -100~100 <--> (left/right)
        forward_backward_velocity: -100~100 <--> (backward/forward)
        up_down_velocity: -100~100 <--> (down/up)
        yaw_velocity: -100~100 <--> (yaw - left/right)

    """
    def __init__(self):
        self.FPS = 30  # скорость работы алгоритмов управления

    def circle_route(self):
        """ Траектория движения по эллипсоидной траектории """
        time_on_line = 100  # время исполнения траектории
        time = self.FPS * time_on_line
        route = np.zeros((time, 4))

        # debug values
        route[:, 3] = -100  # yaw
        route[:, 1] = 10  # forward/backward
        # route[:, 0] = 30  # left/right
        # route[:, 2] = 20  # up/down

        return route

    def left_right_route(self):
        """ Траектория движения влево-вправо """
        time_on_line = 5  # время исполнения траектории
        time = self.FPS * time_on_line
        route = np.zeros((time, 4))

        # влево-вправо-влево  -- возвращается в исходную точку
        route[:time // 4, 0] = 50
        route[time // 4:time // 4 * 3, 0] = -50
        route[time // 4 * 3:, 0] = 50

        return route


class NavigationSystem(object):
    """
    Класс для визуализации заданных траекторий движения ЛА
    """
    def __init__(self, route):
        # траектория движения
        self.route = route
        self.route[:, :3] *= 1e-3
        self.route_coords = None

        # параметры отображения графиков
        self.figsize = (12, 8)

        # габариты комнаты (в метрах)
        self.room_x = 2.
        self.room_y = 5.
        self.room_z = 3.

        self.transformer = SignalTransformer()

        self.control_points = np.array([[1, 0, 2],
                                        [0, 0, 2],
                                        [0, 0, 2.2],
                                        [0.3, 1.6, 2.3],
                                        [0, 1.4, 2.5],
                                        ])

    def recalculate_route(self, zero_point):
        """
        Пересчет задания для дрона (time, [move_cmd_format]) в координаты (time, [x, y, z])
        :param zero_point: [x, y, z] - начальная точка старта маршрута
        :return: bool: True - все прошло успешно, False - в другом случае
        """
        # матрица поворота
        def rotate_matrix(alpha):
            return np.array([[np.cos(alpha), -np.sin(alpha)],
                             [np.sin(alpha), np.cos(alpha)]])

        real_route = np.array([self.transformer.signal_to_real(point) for point in self.route])  # пересчет сигналов
        real_coords = np.zeros((self.route.shape[0] + 1, 3))  # координаты полодения ЛА
        real_coords[0, :] = zero_point  # точка старта

        cur_yaw = 0
        for i in range(1, real_route.shape[0]):
            point = real_route[i]
            _x, _y, z, yaw = point
            yaw = yaw / 180 * np.pi
            cur_yaw += yaw % (2*np.pi)
            x, y = np.dot(rotate_matrix(cur_yaw), np.array([_x, _y]).T)
            real_coords[i] = np.array([x, y, z]) + real_coords[i-1]

        self.route_coords = real_coords

        return True

    def plot_route(self):
        """ Построение заданной траектории движения в симуляторе """
        # инициалзиация параметров графика
        zero_point = np.array([0, 0, 0.8])
        self.control_points = np.vstack((zero_point, self.control_points))
        self.recalculate_route(zero_point)
        fig = plt.figure(figsize=self.figsize)
        ax = p3.Axes3D(fig, autoscale_on=False)
        Nfrm = self.route_coords.shape[0]
        fps = 60

        xc, yc, zc = self.control_points.T
        x = np.linspace(1, 4, 11)
        y = np.linspace(4, 7, 22)
        z = np.linspace(7, 9, 33)
        data = np.meshgrid(x, y, z, indexing='ij', sparse=True)

        xc = np.linspace(0, 1, 3)

        cube_points = np.array([[self.room_x//2, self.room_y//2, 0],
                                [self.room_x//2, -self.room_y//2, 0],
                                [-self.room_x//2, self.room_y//2, 0],
                                [-self.room_x//2, -self.room_y//2, 0],
                                [self.room_x//2, self.room_y//2, self.room_z],
                                [self.room_x//2, -self.room_y//2, self.room_z],
                                [-self.room_x//2, self.room_y//2, self.room_z],
                                [-self.room_x//2, -self.room_y//2, self.room_z],
                                ])

        def update(idx):
            ax.clear()  # очистить карту - отображение текущей точки движения
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            cmap = 'Blues_r'
            alpha = 0.1
            # габариты комнаты
            X, Y = np.meshgrid([-self.room_x//2, self.room_x//2], [-self.room_y//2, self.room_y//2])
            ax.plot_surface(X, Y, np.ones(4).reshape(2, 2) * 0, alpha=alpha, cmap=cmap)
            ax.plot_surface(X, Y, np.ones(4).reshape(2, 2) * self.room_z, alpha=alpha, cmap=cmap)
            X, Y = np.meshgrid([-self.room_x // 2, self.room_x // 2], [0, self.room_z])
            ax.plot_surface(X, np.ones(4).reshape(2, 2) * self.room_y//2, Y, alpha=alpha, cmap=cmap)
            ax.plot_surface(X, np.ones(4).reshape(2, 2) * (-self.room_y//2), Y, alpha=alpha, cmap=cmap)
            X, Y = np.meshgrid([-self.room_y // 2, self.room_y // 2], [0, self.room_z])
            ax.plot_surface(np.ones(4).reshape(2, 2) * self.room_x//2, X, Y, alpha=alpha, cmap=cmap)
            ax.plot_surface(np.ones(4).reshape(2, 2) * (-self.room_x//2), X, Y, alpha=alpha, cmap=cmap)
            ax.scatter3D(cube_points[:, 0], cube_points[:, 1], cube_points[:, 2], cmap=cmap)

            # xc, yc, zc = self.control_points.T
            # ax.scatter(xc, yc, zc, linewidth=1., color='red')
            # ax.plot(self.control_points[:, 0], self.control_points[:, 1], self.control_points[:, 2], linewidth=1.,
            #         color='red')

            # ограничения по карте
            ax.set_xlim3d(-self.room_x//2 - 0.8, self.room_x//2 + 0.8)
            ax.set_ylim3d(-self.room_y//2 - 0.8, self.room_y//2 + 0.8)
            ax.set_zlim3d(0, self.room_z + 0.8)

            # коориднаты x, y, z - по текущему индексу времени
            x = self.route_coords[:idx].T[0]
            y = self.route_coords[:idx].T[1]
            z = self.route_coords[:idx].T[2]
            # последняя точка траектории
            if idx != 0:
                ax.scatter(x[idx-1], y[idx-1], z[idx-1], linewidth=1., color='red')
            # вся траектория
            ax.plot(x, y, z, linewidth=0.8, c='black', linestyle='--')

        ani = animation.FuncAnimation(fig, update, Nfrm, interval=1000 / fps)
        plt.show()

        return True


class Autopilot(object):
    """
    Класс для автопилотирования ЛА
    """
    def __init__(self, route):
        self.time = 0  # текущее состояние времени полета
        self.standard_route = route  # заданный маршрут автопилота [time, [move_cmd_format]
        self.cmd_format = DroneCommand()

    def update(self):
        """ Обновить состояние автопилота """
        self.time += 1
        # зациклить исполнение маршрута автопилота
        if self.time == self.standard_route.shape[0]:
            self.time = 0

            new_route = self.standard_route
            new_route[:2] = self.standard_route[-2:]
            self.standard_route = new_route

    def reload(self):
        """ Перезапустить систему автопилота """
        self.time = 0

    def send_cmd(self):
        """
        Вернуть команду действий
        :return: сигналы управленя типа move_cmd_format
        """
        cmd = self.standard_route[self.time]
        self.cmd_format = self.cmd_format(cmd)
        return self.cmd_format


class LaserController(object):
    """
    Класс управления лазером
    """
    def __init__(self):
        UDP_PORT = 54321
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.connect(('192.168.200.1', UDP_PORT))

    def fire(self, cmd_type, puls_dur=10, delay_dur=10):
        """

        :param cmd_type: str: тип стрельбы (Fire, FireC, FireP)
            Fire - тестовая команда для проверки подключения к модулю
            FireC - непрерывный импульс с заданной длительностью puls_dur
            FireP - периодический импульс с заданной длительностью puls_dur и задержжкой delay_dur
        :param puls_dur: int [0, ..., 9999]: длительность импульса в милисекундах
        :param delay_dur: int [0, ..., 99]: задержка между импульсами в милисекундах
        :return: bool: True - все прошло успешно, False - в другом случае
        """
        cmd = None  # команда управления
        # конвертирование команды под заданный формат
        if 0 < len(str(puls_dur)) < 5:
            puls_dur_str = '0' * (4 - len(str(puls_dur))) + str(puls_dur)
        else:
            raise TypeError('puls_dur не может принимать такое значение')
        if 0 < len(str(delay_dur)) < 3:
            delay_dur_str = '0' * (2 - len(str(delay_dur))) + str(delay_dur)
        else:
            raise TypeError('delay_dur не может принимать такое значение')

        # сформировать команду управления
        if cmd_type == 'Fire':
            cmd = 'Fire'
        elif cmd_type == 'FireC':
            cmd = 'FireC' + puls_dur_str
        elif cmd_type == 'FireP':
            cmd = 'FireP' + puls_dur_str + delay_dur_str
        else:
            raise TypeError('cmd_type не может принимать такие значения')

        # отправить команду
        if cmd:
            self.port.send(cmd.encode())
            return True
        else:
            return False


if __name__ == '__main__':
    """ Тестирование отдельных классов """
    # test_cmd = DroneCommand()
    # test_cmd([0, 0, 0, 0])
    # print(test_cmd.move_cmd_format.values())
    # print(*test_cmd.move_cmd_format)

    # route_creator = RouteCreator()
    # test_route = route_creator.circle_route()
    #
    # navigation = NavigationSystem(test_route)
    # navigation.plot_route()

    import time
    laser = LaserController()
    while True:
        time.sleep(2)
        laser.fire('Fire', puls_dur=1000, delay_dur=10)


