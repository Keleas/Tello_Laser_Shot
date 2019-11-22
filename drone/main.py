from drone.controller import DroneController
from drone.virtual_drone import TestController
from drone.tools import NavigationSystem


class FrontEnd(object):
    """
    Основной цикл взаимодействия модулей автономной системы управления дрона Tello Edu.

    Управление всеми командами осуществялется путем активации атрибутов класса CommandController.
    Обработка оптического потока, приходящегося с дрона, происходит путем активации атрибутов класса SystemCV.

    Взаимодействие двух модулей всей системы производиться за счет передачи команды DroneCommand и
    информации DroneInfo.
    """
    def __init__(self):
        # управляющий модуль ЛА
        self.command_controller = DroneController()
        self.virt_drone = TestController()

        self.display_time_upd = 30

    def check_connect(self):
        """
        Проверить соединение с ЛА
        :return: bool: True в случае успеха, False в противном случае
        """
        try:
            self.command_controller.drone.connect()
            self.command_controller.drone.streamoff()
            self.command_controller.drone.streamon()
            return True

        except:
            return False

    def run(self):
        """ Главынй цикл программы управления """

        # self.autopilot_tester.run()
        if self.check_connect():
            self.command_controller.video_loop()

        else:
            # запуск виртуального беспилотника на локалке
            # камера подключается от пк, управление имитируется в графике
            self.virt_drone.run()

        return True


def main():
    frontend = FrontEnd()
    frontend.run()  # запуск системы управления ЛА

    return True


if __name__ == '__main__':
    main()
