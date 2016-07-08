import os
import sys
import csv
import time
import glob
import os.path
import itertools

import PyQt5
import serial

from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIntValidator

from PyQt5.QtWidgets import (
    QLCDNumber,
    QLabel,
    QComboBox,
    QCheckBox,
    QPushButton,
    QFileDialog,
    QLineEdit,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QApplication,
    QFrame,
    QDialog
)

from bitstring import BitArray

PACKET_LENGTH = 32
LCD_DIGIT_COUNT = 6
DELAY_CNT = 15

MOVE_DONE = b'M'
ZERO_ALL = b'X'

PACKET_HEADER = '0x55'
ACCEL_PREFIX = '0x51'
VEL_PREFIX = '0x52'
ANGLE_PREFIX = '0x53'

DEFAULT_PORT = 'COM14'
IMU_BAUD = '115200'
PLATFORM_BAUD = '9600'
BAUDRATES = (
    '110',
    '300',
    '600',
    '1200',
    '2400',
    '4800',
    '9600',
    '14400',
    '19200',
    '28800',
    '38400',
    '56000',
    '57600',
    '115200'
)

STYLE = """
QLCDNumber {
    background: black;
    color:green;
}

QLCDNumber:flat {
    border: none;
}

QLabel {
    color: black;
    font: bold
}

Margin {
    font: bold 60px;
    qproperty-alignment: AlignCenter;
}

Header {
    font: 20px; 
    qproperty-alignment: 'AlignBottom | AlignCenter';
}

ConnectButton {
    font: bold 14px;
    padding-top: 0px;
    padding-bottom: 0px;
    padding-right: 2px;
    padding-left: 2px;
}

GoButton {
    font: bold 14px;
    padding-top: 0px;
    padding-bottom: 0px;
    padding-right: 2px;
    padding-left: 2px;
}

ConnectButton:checked {
    color: red
}

BrowseButton {
    font: 14px;
    padding-top: 2px;
    padding-bottom: 2px;
    padding-right: 4px;
    padding-left: 4px;
}

AngleButton {
    font: bold 14px;
    padding-top: 1px;
    padding-bottom: 1px;
    padding-right: 3px;
    padding-left: 3px;
}
"""


def get_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        import winreg

        path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
        ports = []

        for i in itertools.count():
            try:
                param, value, _ = winreg.EnumValue(key, i)
                if 'BthModem' not in param:
                    ports.append(value)
            except EnvironmentError:
                break
        return ports
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


# noinspection PyArgumentList
class ImuSignal(QtCore.QObject):
    angle_x = QtCore.pyqtSignal(str)
    angle_y = QtCore.pyqtSignal(str)
    angle_z = QtCore.pyqtSignal(str)

    accel_x = QtCore.pyqtSignal(str)
    accel_y = QtCore.pyqtSignal(str)
    accel_z = QtCore.pyqtSignal(str)

    vel_x = QtCore.pyqtSignal(str)
    vel_y = QtCore.pyqtSignal(str)
    vel_z = QtCore.pyqtSignal(str)


# noinspection PyArgumentList
class PlatformSignal(QtCore.QObject):
    move_done = QtCore.pyqtSignal(bool)


class ImuReadThread(QtCore.QThread):
    def __init__(self):
        super().__init__()

        self.signal = ImuSignal()

        self.imu_data = {
            'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 0.0, 'accel_t': 0.0,
            'vel_x': 0.0, 'vel_y': 0.0, 'vel_z': 0.0, 'vel_t': 0.0,
            'angle_x': 0.0, 'angle_y': 0.0, 'angle_z': 0.0, 'angle_t': 0.0
        }

        self.start_angle_x = 0
        self.start_angle_y = 0
        self.start_angle_z = 0

        self.record_state = False

    def open_port(self, port, baudrate):
        """

        :param port:
        :param baudrate:
        """
        self.ser = serial.Serial(port, baudrate)

    def create_file(self, path):
        """

        :param path:
        """
        ext = '.csv'
        fname = os.path.join(
            path,
            time.strftime('%Y%m%d%H%M%S') + ext
        )
        self.fobject = open(fname, 'w', newline='')

        fieldnames = sorted(self.imu_data.keys())
        self.file_writer = csv.DictWriter(self.fobject, fieldnames=fieldnames)
        self.file_writer.writeheader()

        self.record_state = True

    def read_ser_data(self):
        """

        :return:
        """
        ser_data = BitArray()

        while True:
            packet_header = BitArray(self.ser.read())
            if packet_header == PACKET_HEADER:
                ser_data += packet_header
                break

        ser_data += BitArray(self.ser.read(PACKET_LENGTH))
        return ser_data

    def decode_imu_data(self, ser_data):
        """

        :param ser_data:
        """
        chunks = ser_data.unpack('bytes:11, bytes:11, bytes:11')

        for chunk in chunks:
            _, prefix, *bs = BitArray(chunk).unpack(
                'bytes:1, bytes:1, intle:16, intle:16, intle:16, intle:16'
            )
            if BitArray(prefix) == ACCEL_PREFIX:
                self.imu_data['accel_x'] = bs[0] / 32768 * 16
                self.imu_data['accel_y'] = bs[1] / 32768 * 16
                self.imu_data['accel_z'] = bs[2] / 32768 * 16
                self.imu_data['accel_t'] = bs[3] / 340 + 36.25
            elif BitArray(prefix) == VEL_PREFIX:
                self.imu_data['vel_x'] = bs[0] / 32768 * 2000
                self.imu_data['vel_y'] = bs[1] / 32768 * 2000
                self.imu_data['vel_z'] = bs[2] / 32768 * 2000
                self.imu_data['vel_t'] = bs[3] / 340 + 36.25
            elif BitArray(prefix) == ANGLE_PREFIX:
                self.imu_data['angle_x'] = bs[0] / 32768 * 180
                self.imu_data['angle_y'] = bs[1] / 32768 * 180
                self.imu_data['angle_z'] = bs[2] / 32768 * 180
                self.imu_data['angle_t'] = bs[3] / 340 + 36.25
            else:
                pass  # TODO: error handling here

    def set_relative_angle(self):
        """

        """
        self.start_angle_x = self.imu_data['angle_x']
        self.start_angle_y = self.imu_data['angle_y']
        self.start_angle_z = self.imu_data['angle_z']

    def set_absolute_angle(self):
        """

        """
        self.start_angle_x = 0
        self.start_angle_y = 0
        self.start_angle_z = 0

    def run(self):
        """

        """
        delay_cnt = DELAY_CNT

        while True:
            self.decode_imu_data(self.read_ser_data())

            if self.record_state:
                self.file_writer.writerow(self.imu_data)

            if delay_cnt:
                delay_cnt -= 1
            else:
                delay_cnt = DELAY_CNT

                self.signal.angle_x.emit(
                    '{:=6.1f}'.format(
                        self.imu_data['angle_x'] - self.start_angle_x
                    )
                )
                self.signal.angle_y.emit(
                    '{:=6.1f}'.format(
                        self.imu_data['angle_y'] - self.start_angle_y
                    )
                )
                self.signal.angle_z.emit(
                    '{:=6.1f}'.format(
                        self.imu_data['angle_z'] - self.start_angle_z
                    )
                )
                self.signal.accel_x.emit(
                    '{:=6.2f}'.format(self.imu_data['accel_x'])
                )
                self.signal.accel_y.emit(
                    '{:=6.2f}'.format(self.imu_data['accel_y'])
                )
                self.signal.accel_z.emit(
                    '{:=6.2f}'.format(self.imu_data['accel_z'])
                )
                self.signal.vel_x.emit(
                    '{:=5.0f}'.format(self.imu_data['vel_x'])
                )
                self.signal.vel_y.emit(
                    '{:=5.0f}'.format(self.imu_data['vel_y'])
                )
                self.signal.vel_z.emit(
                    '{:=5.0f}'.format(self.imu_data['vel_z'])
                )


class PlatformThread(QtCore.QThread):
    def __init__(self):
        super().__init__()

        self.signal = PlatformSignal()

    def open_port(self, port, baudrate):
        """

        :param port:
        :param baudrate:
        """
        self.ser = serial.Serial(port, baudrate)

    def read_ser_data(self):
        """

        :return:
        """
        ser_data = self.ser.read()

        return ser_data

    def write_ser_data(self, data):
        """

        :param data:
        """
        self.ser.write(data)

    def run(self):
        """

        """
        while True:
            answer = self.read_ser_data()
            if answer == MOVE_DONE:
                self.signal.move_done.emit(True)
            elif answer == ZERO_ALL:
                print('All coordinates are zero.')


class Margin(QLabel):
    def __init__(self, txt):
        super().__init__(txt)


class Header(QLabel):
    def __init__(self, txt):
        super().__init__(txt)


class ConnectButton(QPushButton):
    def __init__(self, txt):
        super().__init__(txt)


class BrowseButton(QPushButton):
    def __init__(self, txt):
        super().__init__(txt)


class AngleButton(QPushButton):
    def __init__(self, txt):
        super().__init__(txt)


class GoButton(QPushButton):
    def __init__(self, txt):
        super().__init__(txt)


class Interface(QDialog):
    # noinspection PyUnresolvedReferences
    def __init__(self):
        super().__init__(
            None,
            QtCore.Qt.Window |
            QtCore.Qt.WindowTitleHint |
            QtCore.Qt.WindowCloseButtonHint
        )

        self.imu_connection_state = False
        self.platform_connection_state = False
        #        self.ports = (DEFAULT_PORT,)
        self.ports = get_ports()

        self.imu_thread = ImuReadThread()
        self.imu_thread.finished.connect(self.close_imu_port)

        self.platform_thread = PlatformThread()
        self.platform_thread.finished.connect(self.close_platform_port)

        self.initUI()

    def connect_imu(self):
        """

        :return:
        """
        port = self.imu_ports_list.currentText()
        baudrate = int(self.imu_baud_list.currentText())

        if not port:
            return 1  # TODO: custom error handling here

        try:
            if not self.imu_connection_state:
                self.imu_thread.open_port(port, baudrate)
                self.record_box.setEnabled(False)

                if self.record_box.isChecked():
                    self.imu_thread.create_file(self.file_path.text())

                self.imu_thread.start()
                self.imu_thread.set_absolute_angle()
                self.imu_connection_state = True
            else:
                self.imu_thread.terminate()
                self.imu_connection_state = False
                self.record_box.setEnabled(True)
                self.clear_lcds()
        except serial.SerialException as se:
            self.imu_connect_button.setChecked(False)
            print(se.args)

    def connect_platform(self):
        """

        :return:
        """
        port = self.platform_ports_list.currentText()
        baudrate = int(self.platform_baud_list.currentText())

        if not port:
            return 1  # TODO: custom error handling here

        try:
            if not self.platform_connection_state:
                self.platform_thread.open_port(port, baudrate)
                self.platform_thread.start()
                self.platform_go_button.setEnabled(True)
                self.platform_zero_button.setEnabled(True)
                self.platform_connection_state = True
            else:
                self.platform_thread.terminate()
                self.platform_go_button.setEnabled(False)
                self.platform_zero_button.setEnabled(False)
                self.platform_connection_state = False
        except serial.SerialException as se:
            self.platform_connect_button.setChecked(False)
            print(se.args)

    def close_imu_port(self):
        """

        """
        self.imu_thread.ser.flush()
        self.imu_thread.ser.close()

        if self.record_box.isChecked():
            self.imu_thread.fobject.flush()
            self.imu_thread.fobject.close()

    def close_platform_port(self):
        """

        """
        self.platform_thread.ser.flush()
        self.platform_thread.ser.close()

    def create_lcd(self, signal):
        """

        :param signal:
        :return:
        """
        lcd = QLCDNumber(self)

        lcd.setDigitCount(LCD_DIGIT_COUNT)
        lcd.setSegmentStyle(QLCDNumber.Flat)
        signal.connect(lcd.display)

        return lcd

    def clear_lcds(self):
        """

        """
        for lcd in self.findChildren(QLCDNumber):
            lcd.display(0)

    def show_select_dir_dialog(self):
        """

        """
        # noinspection PyCallByClass,PyTypeChecker,PyArgumentList
        path = QFileDialog.getExistingDirectory(
            self,
            'Выберите директорию',
            os.getcwd(),
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks
        )
        if path:
            self.file_path.setText(str(path))

    def send_coords(self):
        message = '^MOVE,{},{},{},{}$'

        if not self.sync_box.isChecked():
            coord_1 = self.rod_1.text()
            coord_2 = self.rod_2.text()
            coord_3 = self.rod_3.text()
            coord_4 = self.rod_4.text()
        else:
            coord_1 = self.rod_1.text()
            coord_2 = '-' + self.rod_1.text()
            coord_3 = self.rod_3.text()
            coord_4 = '-' + self.rod_3.text()

        message = message.format(coord_1, coord_2, coord_3, coord_4)

        self.platform_thread.write_ser_data(message.encode())
        self.platform_go_button.setEnabled(False)

    def send_zero_all(self):
        self.platform_thread.write_ser_data(b'^ZERO$')
        self.rod_1.setText('0')
        self.rod_2.setText('0')
        self.rod_3.setText('0')
        self.rod_4.setText('0')

    @staticmethod
    def create_vline():
        """

        :return:
        """
        vline = QFrame()
        vline.setFrameShape(QFrame.VLine)
        vline.setFrameShadow(QFrame.Sunken)

        return vline

    @staticmethod
    def create_hline():
        """

        :return:
        """
        hline = QFrame()
        hline.setFrameShape(QFrame.HLine)
        hline.setFrameShadow(QFrame.Sunken)

        return hline

    def disable_roads(self):
        """

        """
        if self.sync_box.isChecked():
            self.rod_2.setEnabled(False)
            self.rod_4.setEnabled(False)
            self.rod_2.setText('')
            self.rod_4.setText('')
            self.label_2.setText('')
            self.label_4.setText('')
        else:
            self.rod_2.setEnabled(True)
            self.rod_4.setEnabled(True)
            self.label_2.setText('0')
            self.label_4.setText('0')
            self.rod_2.setText('0')
            self.rod_4.setText('0')

    # noinspection PyUnresolvedReferences
    def initUI(self):

        # _____________________________IMU MENU________________________________

        """

        """
        imu_menu = QHBoxLayout()

        imu_menu.addWidget(QLabel('Д'))
        imu_menu.addWidget(self.create_vline())

        rel_angle_button = AngleButton('0')
        rel_angle_button.setToolTip('Относительные значения угла')
        # noinspection PyUnresolvedReferences
        rel_angle_button.clicked.connect(self.imu_thread.set_relative_angle)
        imu_menu.addWidget(rel_angle_button)

        abs_angle_button = AngleButton('A')
        abs_angle_button.setToolTip('Абсолютные значения угла')
        # noinspection PyUnresolvedReferences
        abs_angle_button.clicked.connect(self.imu_thread.set_absolute_angle)
        imu_menu.addWidget(abs_angle_button)

        imu_menu.addWidget(self.create_vline())

        self.imu_connect_button = ConnectButton('\U0001F50C')
        self.imu_connect_button.setToolTip('Подключить/отключить')
        self.imu_connect_button.setCheckable(True)
        if not self.ports:
            self.imu_connect_button.setEnabled(False)
        # noinspection PyUnresolvedReferences
        self.imu_connect_button.clicked.connect(self.connect_imu)
        imu_menu.addWidget(self.imu_connect_button)

        imu_menu.addWidget(QLabel('Порт:'))

        self.imu_ports_list = QComboBox(self)
        for port in self.ports:
            self.imu_ports_list.addItem(port)
        imu_menu.addWidget(self.imu_ports_list)

        imu_menu.addWidget(QLabel('Скорость:'))

        self.imu_baud_list = QComboBox(self)
        for baudrate in BAUDRATES:
            self.imu_baud_list.addItem(baudrate)
        self.imu_baud_list.setCurrentIndex(BAUDRATES.index(IMU_BAUD))
        imu_menu.addWidget(self.imu_baud_list)

        imu_menu.addWidget(self.create_vline())

        imu_menu.addWidget(QLabel('Запись:'))

        self.record_box = QCheckBox()
        imu_menu.addWidget(self.record_box)

        imu_menu.addWidget(QLabel('Путь:'))

        self.file_path = QLineEdit(os.getcwd())
        self.file_path.setReadOnly(True)
        imu_menu.addWidget(self.file_path)

        select_path_button = BrowseButton('...')
        select_path_button.setToolTip('Выбрать директорию')
        # noinspection PyUnresolvedReferences
        select_path_button.clicked.connect(self.show_select_dir_dialog)
        imu_menu.addWidget(select_path_button)

        # __________________________PLATFORM MENU______________________________

        platform_menu = QHBoxLayout()

        platform_menu.addWidget(QLabel('П'))

        platform_menu.addWidget(self.create_vline())

        self.platform_connect_button = ConnectButton('\U0001F50C')
        self.platform_connect_button.setToolTip('Подключить/отключить')
        self.platform_connect_button.setCheckable(True)
        if not self.ports:
            self.platform_connect_button.setEnabled(False)
        # noinspection PyUnresolvedReferences
        self.platform_connect_button.clicked.connect(self.connect_platform)
        platform_menu.addWidget(self.platform_connect_button)

        platform_menu.addWidget(QLabel('Порт:'))

        self.platform_ports_list = QComboBox(self)
        for port in self.ports:
            self.platform_ports_list.addItem(port)

        platform_menu.addWidget(self.platform_ports_list)

        platform_menu.addWidget(QLabel('Скорость:'))

        self.platform_baud_list = QComboBox(self)
        for baudrate in BAUDRATES:
            self.platform_baud_list.addItem(baudrate)

        self.platform_baud_list.setCurrentIndex(BAUDRATES.index(PLATFORM_BAUD))
        platform_menu.addWidget(self.platform_baud_list)

        platform_menu.addWidget(self.create_vline())

        self.platform_zero_button = AngleButton('0')  # FIXME: Add custom class
        self.platform_zero_button.setToolTip('Обнулить координаты платформы')
        self.platform_zero_button.setEnabled(False)
        # noinspection PyUnresolvedReferences
        self.platform_zero_button.clicked.connect(self.send_zero_all)
        platform_menu.addWidget(self.platform_zero_button)

        self.platform_go_button = GoButton('\U000025BA')
        self.platform_go_button.setToolTip('Поехали')
        self.platform_go_button.setEnabled(False)
        # noinspection PyUnresolvedReferences
        self.platform_thread.signal.move_done.connect(
            self.platform_go_button.setEnabled
        )
        # noinspection PyUnresolvedReferences
        self.platform_go_button.clicked.connect(self.send_coords)
        platform_menu.addWidget(self.platform_go_button)

        steps_validator = QIntValidator(-99999, 99999)
        steps_in_mm_validator = QIntValidator(1, 99999)

        self.steps_in_mm = QLineEdit('300')
        self.steps_in_mm.setAlignment(Qt.AlignRight)
        self.steps_in_mm.setValidator(steps_in_mm_validator)

        self.label_1 = QLabel('0.000')
        self.label_1.setFixedWidth(60)
        self.label_1.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.label_2 = QLabel('0.000')
        self.label_2.setFixedWidth(60)
        self.label_2.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.label_3 = QLabel('0.000')
        self.label_3.setFixedWidth(60)
        self.label_3.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.label_4 = QLabel('0.000')
        self.label_4.setFixedWidth(60)
        self.label_4.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        platform_menu.addWidget(QLabel('X'))
        self.rod_1 = QLineEdit('0')
        self.rod_1.setAlignment(Qt.AlignRight)
        self.rod_1.setValidator(steps_validator)
        self.rod_1.textChanged.connect(
            lambda text: self.convert_steps_to_mm(text, self.label_1)
        )
        platform_menu.addWidget(self.rod_1)

        platform_menu.addWidget(QLabel('Y'))
        self.rod_2 = QLineEdit('0')
        self.rod_2.setAlignment(Qt.AlignRight)
        self.rod_2.setValidator(steps_validator)
        self.rod_2.textChanged.connect(
            lambda text: self.convert_steps_to_mm(text, self.label_2)
        )
        platform_menu.addWidget(self.rod_2)

        platform_menu.addWidget(QLabel('Z'))
        self.rod_3 = QLineEdit('0')
        self.rod_3.setAlignment(Qt.AlignRight)
        self.rod_3.setValidator(steps_validator)
        self.rod_3.textChanged.connect(
            lambda text: self.convert_steps_to_mm(text, self.label_3)
        )
        platform_menu.addWidget(self.rod_3)

        platform_menu.addWidget(QLabel('A'))
        self.rod_4 = QLineEdit('0')
        self.rod_4.setAlignment(Qt.AlignRight)
        self.rod_4.setValidator(steps_validator)
        self.rod_4.textChanged.connect(
            lambda text: self.convert_steps_to_mm(text, self.label_4)
        )
        platform_menu.addWidget(self.rod_4)

        platform_menu.addWidget(self.create_vline())

        platform_menu.addWidget(QLabel('Синхр.:'))

        self.sync_box = QCheckBox()
        # noinspection PyUnresolvedReferences
        self.sync_box.stateChanged.connect(self.disable_roads)
        platform_menu.addWidget(self.sync_box)

        # ___________________________INFO______________________________________

        info = QHBoxLayout()
        info.addWidget(QLabel('Шагов в мм:'))
        info.addWidget(self.steps_in_mm)
        info.addWidget(QLabel('Смещение по осям:'))
        info.addWidget(QLabel('X = '))
        info.addWidget(self.label_1)
        info.addWidget(QLabel('мм;'))
        info.addWidget(QLabel('Y = '))
        info.addWidget(self.label_2)
        info.addWidget(QLabel('мм;'))
        info.addWidget(QLabel('Z = '))
        info.addWidget(self.label_3)
        info.addWidget(QLabel('мм;'))
        info.addWidget(QLabel('A = '))
        info.addWidget(self.label_4)
        info.addWidget(QLabel('мм;'))

        # ___________________________VALUES TABLE______________________________

        table = QGridLayout()

        table.addWidget(Margin('X'), 1, 0)
        table.addWidget(Margin('Y'), 2, 0)
        table.addWidget(Margin('Z'), 3, 0)

        table.addWidget(Header('Угол (гр.)'), 0, 1)
        table.addWidget(Header('Ускорение (g)'), 0, 2)
        table.addWidget(Header('Скорость (гр./сек.)'), 0, 3)

        table.addWidget(self.create_lcd(self.imu_thread.signal.angle_x), 1, 1)
        table.addWidget(self.create_lcd(self.imu_thread.signal.angle_y), 2, 1)
        table.addWidget(self.create_lcd(self.imu_thread.signal.angle_z), 3, 1)

        table.addWidget(self.create_lcd(self.imu_thread.signal.accel_x), 1, 2)
        table.addWidget(self.create_lcd(self.imu_thread.signal.accel_y), 2, 2)
        table.addWidget(self.create_lcd(self.imu_thread.signal.accel_z), 3, 2)

        table.addWidget(self.create_lcd(self.imu_thread.signal.vel_x), 1, 3)
        table.addWidget(self.create_lcd(self.imu_thread.signal.vel_y), 2, 3)
        table.addWidget(self.create_lcd(self.imu_thread.signal.vel_z), 3, 3)

        table.setColumnStretch(0, 2)
        table.setColumnStretch(1, 5)
        table.setColumnStretch(2, 5)
        table.setColumnStretch(3, 5)

        table.setRowStretch(0, 1)
        table.setRowStretch(1, 10)
        table.setRowStretch(2, 10)
        table.setRowStretch(3, 10)

        # ___________________________LAYOUT____________________________________

        layout = QVBoxLayout()
        layout.addLayout(imu_menu)
        layout.addWidget(self.create_hline())
        layout.addLayout(platform_menu)
        layout.addWidget(self.create_hline())
        layout.addLayout(info)
        layout.addWidget(self.create_hline())
        layout.addLayout(table)

        self.setLayout(layout)

        self.setFixedSize(750, 400)
        self.setWindowTitle('MPU6050')
        self.show()

    def convert_steps_to_mm(self, text, label):
        try:
            value = int(text)
            steps_in_mm = int(self.steps_in_mm.text())
        except ValueError:
            value = steps_in_mm = 0

        if steps_in_mm > 0:
            value /= steps_in_mm
            label.setText('{:5.3f}'.format(value))
        else:
            label.setText('0.000')


# noinspection PyCallByClass,PyArgumentList
def main():
    pyqt = os.path.dirname(PyQt5.__file__)
    # noinspection PyTypeChecker
    QApplication.addLibraryPath(os.path.join(pyqt, 'plugins'))

    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)

    ex = Interface()
    print(ex)

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
