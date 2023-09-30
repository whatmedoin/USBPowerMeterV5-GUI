import sys
import serial
import serial.tools.list_ports
from serial.tools.list_ports_common import ListPortInfo
from PyQt5 import uic, QtWidgets
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from time import sleep
from threading import Thread, Lock
from queue import Queue
from typing import Optional
from os import path
# import traceback

class UpdateInterval:
    """An enum containing constant refresh rates

    :param SLOW: Slow update speed
    :type SLOW: float
    :param MEDIUM: Medium update speed
    :type MEDIUM: float
    :param FAST: Fast update speed
    :type FAST: float
    """
    SLOW = 1.0
    MEDIUM = 0.5
    FAST = 0.1

class Model:
    """A class containing the data model (and the logic) for the application
    
    :param _serial_connection: The serial connection to the power meter
    :type _serial_connection: serial.Serial
    :param data_update_interval: The refresh rate of the data
    :type data_update_interval: float
    :param _min_max_lock: A lock for the min/max values
    :type _min_max_lock: threading.Lock
    :param _current_power_lock: A lock for the current power values
    :type _current_power_lock: threading.Lock
    :param max_power_dbm: The maximum power in dBm
    :type max_power_dbm: float
    :param min_power_dbm: The minimum power in dBm
    :type min_power_dbm: float
    :param max_power: The maximum power in the current unit
    :type max_power: float
    :param min_power: The minimum power in the current unit
    :type min_power: float
    :param max_power_unit: The unit of the maximum power
    :type max_power_unit: str
    :param min_power_unit: The unit of the minimum power
    :type min_power_unit: str
    """
    def __init__(self):
        """Constructor method
        """
        self._serial_connection = None
        self.captured_values = []
        self._captured_values_unit = "mW"
        self.data_update_interval = UpdateInterval.MEDIUM
        self._min_max_lock = Lock()
        self._current_power_lock = Lock()
        self.reset_min_max()
        self._reset_current_power()

    # Basic getters and setters

    def get_com_ports(self) -> list[ListPortInfo]:
        """Returns a list of available serial ports. THIS FUNCTION IS CALLED BY AN EXTERNAL THREAD
        
        :return: A list of available serial ports
        :rtype: list
        """
        return serial.tools.list_ports.comports()
    
    def is_connected(self) -> bool:
        """Returns whether the power meter is connected or not

        :return: Whether the serial connection to power meter is open or not
        :rtype: bool
        """
        return self._serial_connection is not None and self._serial_connection.is_open
    
    def get_min_max_power(self) -> tuple[tuple[float, float, str], tuple[float, float, str]]:
        """Returns minimal and maximal observed power. THIS FUNCTION MAY BE CALLED BY AN EXTERNAL THREAD

        :return: The minimum and maximum observed power (in dBm, in W or mW or uW, unit of the second return value)
        :rtype: tuple
        """
        # Prevent min/max values from being updated while reading
        with self._min_max_lock:
            return (self.min_power_dbm, self.min_power, self.min_power_unit,), (self.max_power_dbm, self.max_power, self.max_power_unit,)
        
    def get_current_power(self) -> tuple[float, float, str]:
        """Returns the current power

        :return: The current power (in dBm, in W or mW or uW, unit of the second return value)
        :rtype: tuple
        """
        # Prevent current values from being updated while reading
        with self._current_power_lock:
            return self.current_power_dbm, self.current_power, self.current_power_unit
        
    def set_data_update_interval(self, interval: float):
        """Sets the refresh rate of the serial data

        :param interval: The new refresh rate
        :type interval: float
        """
        self.data_update_interval = interval

    def get_data_update_interval(self) -> float:
        """Returns the refresh rate of the serial data. THIS FUNCTION MAY BE CALLED BY AN EXTERNAL THREAD

        :return: The refresh rate of the serial data
        :rtype: float
        """
        return self.data_update_interval
    
    def get_capture_count(self) -> int:
        """Returns the number of captured values

        :return: The number of captured values
        :rtype: int
        """
        return len(self.captured_values)
    
    # Basic logic
    
    def _reset_current_power(self):
        """Reset the current power values"""
        with self._current_power_lock:
            self.current_power_dbm = 0.0
            self.current_power = 0.0
            self.current_power_unit = "uW"

    def reset_min_max(self):
        """Reset the minimum and maximum observed power values"""
        with self._min_max_lock:
            self.max_power_dbm = -70.0
            self.min_power_dbm = 70.0
            self.max_power = 0.0
            self.min_power = 0.0
            self.max_power_unit = "uW"
            self.min_power_unit = "uW"

    def add_capture(self, current: tuple[float, float, str], min: tuple[float, float, str], max: tuple[float, float, str]) -> int:
        """Adds a new row to the captured values table

        :param current: The current power data (in dBm, in W or mW or uW, unit of the second value)
        :type current: tuple
        :param min: The minimum power data (in dBm, in W or mW or uW, unit of the second value)
        :type min: tuple
        :param max: The maximum power data (in dBm, in W or mW or uW, unit of the second value)
        :type max: tuple
        :return: The index of the new row
        :rtype: int
        """
        current_power_dbm, current_power, current_power_unit = current
        min_power_dbm, min_power, min_power_unit = min
        max_power_dbm, max_power, max_power_unit = max
        current_multiply_factor = self._get_power_multiply_factor(current_power_unit, self._captured_values_unit)
        min_multiply_factor = self._get_power_multiply_factor(min_power_unit, self._captured_values_unit)
        max_multiply_factor = self._get_power_multiply_factor(max_power_unit, self._captured_values_unit)
        current_power *= current_multiply_factor
        min_power *= min_multiply_factor
        max_power *= max_multiply_factor
        self.captured_values.append(
            [
                [current_power_dbm, current_power, self._captured_values_unit], 
                [min_power_dbm, min_power, self._captured_values_unit], 
                [max_power_dbm, max_power, self._captured_values_unit]
            ]
        )
        return len(self.captured_values) - 1

    def get_capture(self, index: int) -> Optional[list[list[float, float, str], list[float, float, str], list[float, float, str]]]:
        """Get the captured values at a given index

        :param index: The index of the captured values
        :type index: int
        :return: All power data at the given index in table's unit
        :rtype: list
        """
        if index < 0 or index >= len(self.captured_values):
            return None
        return self.captured_values[index]
    
    def clear_captured(self):
        """Clears the captured values list
        """
        self.captured_values = []
    
    def connect(self, port: str) -> bool:
        """Tries to connect to the power meter
        
        :param port: The serial port to connect to
        :type port: str
        :return: Whether the connection was successful or not
        :rtype: bool
        """
        try:
            self._serial_connection = serial.Serial(port, 460800)
        except:
            return False
        return True
    
    def disconnect(self) -> bool:
        """Disconnects from the power meter. THIS FUNCTION MAY BE CALLED BY AN EXTERNAL THREAD

        :return: Whether the disconnection was successful or not
        :rtype: bool
        """
        try:
            if self._serial_connection is not None:
                self._serial_connection.close()
                self._serial_connection = None
        except:
            return False
        return True
    
    def request_settings(self):
        """Requests the current configuration (frequency, attenuation) from the power meter"""
        # Increase update speed to get the configuration faster
        self.set_data_update_interval(UpdateInterval.FAST)
        # Values may not be received if serial input buffer is full
        self._serial_connection.reset_input_buffer()
        self._serial_connection.write(b"Read\r\n")
        self._serial_connection.flush()

    def set_settings(self, frequency: int, attenuation: float):
        """Sets the configuration of the power meter. THIS FUNCTION IS SOMETIMES CALLED BY AN EXTERNAL THREAD.
        
        :param frequency: The frequency in MHz
        :type frequency: int
        :param attenuation: The attenuation in dBm
        :type attenuation: float
        """
        self._serial_connection.write(f"A{frequency:04d}+{attenuation:.2f}\r\n".encode("utf-8"))
        self._serial_connection.flush()

    def get_next_captured_power_unit(self) -> str:
        """Returns the next power unit for captured values (uW -> mW -> W -> uW)

        :param current_unit: The current power unit
        :type current_unit: str
        :return: The next power unit
        :rtype: str
        """
        if self._captured_values_unit == "uW":
            return "mW"
        elif self._captured_values_unit == "mW":
            return "W"
        elif self._captured_values_unit == "W":
            return "uW"
        return "uW"
    
    def _get_power_multiply_factor(self, prev_unit: str, new_unit: str) -> float:
        """Returns the multiply factor for converting between power units

        :param prev_unit: The previous power unit
        :type prev_unit: str
        :param new_unit: The new power unit
        :type new_unit: str
        :return: The multiply factor
        :rtype: float
        """
        multiply_factor = 1.0
        if new_unit == "uW":
            multiply_factor = 1000.0
        elif new_unit == "mW":
            multiply_factor = 1.0
        elif new_unit == "W":
            multiply_factor = 0.001

        if prev_unit == "uW":
            multiply_factor *= 0.001
        elif prev_unit == "mW":
            multiply_factor *= 1.0
        elif prev_unit == "W":
            multiply_factor *= 1000.0
        return multiply_factor

    def update_captured_power_unit(self, new_unit: str):
        """Updates the power unit of captured values

        :param prev_unit: The previous power unit
        :type prev_unit: str
        :param new_unit: The new power unit
        :type new_unit: str
        """
        multiply_factor = self._get_power_multiply_factor(self._captured_values_unit, new_unit)
        # Captures
        for i, value_list in enumerate(self.captured_values):
            # Current, min, max
            for j in range(len(value_list)):
                # Convert raw power to new unit
                self.captured_values[i][j][1] = self.captured_values[i][j][1] * multiply_factor
                self.captured_values[i][j][2] = new_unit
        self._captured_values_unit = new_unit
    
    def read_and_parse(self) -> tuple[Optional[float|int], Optional[float], Optional[str]]:
        """Reads and parses serial data. THIS FUNCTION IS CALLED BY AN EXTERNAL THREAD
        
        :return: Depending on incoming data, it may be (frequency, attenuation, None) for configuration
            or (power in dBm, power in W, unit of power) for power data or (None, None, None) for invalid data
        :rtype: tuple
        """
        raw_data = self._serial_connection.read_until(b"A").decode("utf-8")
        power_sum = 0.0
        power_dbm_sum = 0.0
        if raw_data[0] == 'a' and len(raw_data) >= 5000:
            valid = 0
            power_unit = None
            power_sign = None
            for i in range(500):
                measurement = raw_data[i * 10 + 1:11 + i * 10]
                if len(measurement) != 10 or not measurement[0] in ['-', '+'] or not measurement[9] in ['u', 'm', 'w'] or not measurement[1:9].isdigit():
                    continue
                valid += 1
                if power_unit is None:
                    if measurement[9] == 'u':
                        power_unit = "uW"
                    elif measurement[9] == 'm':
                        power_unit = "mW"
                    elif measurement[9] == 'w':
                        power_unit = "W"
                if power_sign is None:
                    power_sign = measurement[0]
                power_dbm_sum += float(measurement[1:4]) / 10.0
                power_sum += float(measurement[4:9]) / 100.0
            # Update all current power data simultaneously
            with self._current_power_lock:
                self.current_power_dbm = float(power_sign + str(power_dbm_sum / float(valid)))
                self.current_power = power_sum / float(valid)
                self.current_power_unit = power_unit
            # Min/max values may be resetted anytime, so we need to lock them
            # Current values shouldn't be updated externally at this point, as they are private, so we don't need to lock them
            with self._min_max_lock:
                if self.current_power_dbm > self.max_power_dbm:
                    self.max_power_dbm = self.current_power_dbm
                    self.max_power = self.current_power
                    self.max_power_unit = power_unit
                if self.current_power_dbm < self.min_power_dbm:
                    self.min_power_dbm = self.current_power_dbm
                    self.min_power = self.current_power
                    self.min_power_unit = power_unit
            return self.current_power_dbm, self.current_power, power_unit
        elif raw_data[0] == 'R' and len(raw_data) >= 10:
            separator = "+"
            if "-" in raw_data:
                separator = "-"
            sep_index = raw_data.index(separator)
            frequency = int(raw_data[1:sep_index])
            attenuation_dbm = float(raw_data[sep_index:10])
            return frequency, attenuation_dbm, None
        return None, None, None

class Controller:
    """A class containing the controller logic for the application

    :param _app: The Qt application
    :type _app: QtWidgets.QApplication
    :param _view: The view of the application
    :type _view: View
    :param _model: The model of the application
    :type _model: Model
    :param _com_port_update_stop: Whether the serial port watcher thread should stop
    :type _com_port_update_stop: bool
    :param _com_port_update_thread: The serial port watcher thread
    :type _com_port_update_thread: threading.Thread
    :param _data_processing_stop: Whether the data processing thread should stop
    :type _data_processing_stop: bool
    :param _data_processing_thread: The data processing thread
    :type _data_processing_thread: threading.Thread
    """
    def __init__(self):
        """Constructor method
        """
        self._app = QtWidgets.QApplication(sys.argv)
        self._view = View(self)
        self._model = Model()

        self._com_port_update_stop = False
        self._com_port_update_thread = None
        self._start_com_port_update()

        self._data_processing_stop = False
        self._data_processing_thread = None

    # App logic

    def start_app(self):
        """Starts the application and waits for it to exit"""
        self._view.show_UI()
        self._app.exec()
        self.safe_exit()

    def safe_exit(self):
        """Cleans up before exiting the application"""
        self._stop_com_port_update()
        self._view.stop_periodic_update()
        self._stop_data_processing()
        self._model.disconnect()

    # Handlers

    def connect_btn_handler(self):
        """Tries to connect/disconnect to/from the power meter
        """
        if not self._model.is_connected():
            self._stop_com_port_update()
            port = self._view.get_selected_port()
            self._model.connect(port)
            self._model.request_settings()
            self._start_data_processing()
            self._view.notify(f"Connected to {port}", 3000)
            self._view.enable_capture_section(True)
        else:
            self._stop_data_processing()
            self._model.disconnect()
            self._start_com_port_update()
            if self._model.get_capture_count() == 0:
                self._view.enable_capture_section(False)
        self._view.set_connected(self._model.is_connected())

    def pause_btn_handler(self):
        """Pauses/resumes the data updates
        """
        if self._data_processing_thread is None:
            self._start_data_processing()
            self._view.resume_updates(True)
            self._view.notify("Updates resumed!", 3000)
        else:
            self._stop_data_processing()
            self._view.resume_updates(False)
            self._view.notify("Updates paused")

    def reset_stats_btn_handler(self):
        """Resets min/max power values
        """
        self._model.reset_min_max()
        min_power, max_power = self._model.get_min_max_power()
        self._view.set_power_data((None, None, None), min_power, max_power)

    def reset_captured_btn_handler(self):
        """Resets captured values
        """
        self._model.clear_captured()
        self._view.reset_capture_table()
        if not self._model.is_connected():
            self._view.enable_capture_section(False)

    def capture_btn_handler(self):
        """Captures statistics into a table slot
        """
        current_power = self._model.get_current_power()
        min_power, max_power = self._model.get_min_max_power()
        index = self._model.add_capture(current_power, min_power, max_power)
        current_power_converted, min_power_converted, max_power_converted = self._model.get_capture(index)
        self._view.add_capture(current_power_converted, min_power_converted, max_power_converted)

    def export_btn_handler(self):
        """Exports captured statistics into a file
        """
        pass

    def apply_settings_btn_handler(self):
        """Applies the configuration to the power meter
        """
        frequency, attenuation = self._view.get_settings()
        self._model.set_settings(frequency, attenuation)
        self._view.notify("Configuration applied!", 3000)

    def read_settings_btn_handler(self):
        """Requests the current configuration from the power meter
        """
        self._model.request_settings()

    def update_speed_changed_handler(self, index: int, notify: bool=True):
        """Changes the refresh rate of the data. THIS FUNCTION MAY BE CALLED BY AN EXTERNAL THREAD
        
        :param index: The index of the selected refresh rate
        :type index: int
        :param notify: Whether to notify the user about the change
        :type notify: bool
        """
        if index == 0:
            self._model.set_data_update_interval(UpdateInterval.SLOW)
        elif index == 1:
            self._model.set_data_update_interval(UpdateInterval.MEDIUM)
        elif index == 2:
            self._model.set_data_update_interval(UpdateInterval.FAST)
        if notify:
            self._view.notify(f"Refresh rate set to {self._model.get_data_update_interval()}s", 3000)

    def table_header_clicked_handler(self, index: int):
        """Changes the power unit of captured values
        
        :param index: The index of the clicked header
        :type index: int
        """
        if not index in [1, 3, 5]:
            return
        new_unit = self._model.get_next_captured_power_unit()
        self._view.set_captured_power_unit(new_unit)
        self._model.update_captured_power_unit(new_unit)
        self._view.reset_capture_table()
        for i in range(self._model.get_capture_count()):
            current_power_converted, min_power_converted, max_power_converted = self._model.get_capture(i)
            self._view.add_capture(current_power_converted, min_power_converted, max_power_converted)

    # Thread logic

    def _update_com_ports(self):
        """Periodically checks for available serial ports and updates the UI accordingly
        """
        prev_ports = None
        while not self._com_port_update_stop:
            com_ports = self._model.get_com_ports()
            if prev_ports is None or len(com_ports) != len(prev_ports):
                prev_ports = com_ports
                self._view.add_update(self._view.set_ports_available, [f"{com_port.device}: {com_port.description}" for com_port in com_ports])
                self._view.add_update(self._view.notify, f"{len(com_ports)} serial devices available")
            sleep(0.5)

    def _stop_com_port_update(self):
        """Stops the serial port watcher thread
        """
        if self._com_port_update_thread is not None and self._com_port_update_thread.is_alive():
            self._com_port_update_stop = True
            self._com_port_update_thread.join()
            self._com_port_update_thread = None

    def _start_com_port_update(self):
        """Starts the serial port watcher thread. THIS FUNCTION MAY BE CALLED BY AN EXTERNAL THREAD
        """
        if self._com_port_update_thread is None or not self._com_port_update_thread.is_alive():
            self._com_port_update_stop = False
            self._com_port_update_thread = Thread(target=self._update_com_ports)
            self._com_port_update_thread.start()

    def _process_data(self):
        """Periodically fetches parsed data from the power meter and updates the UI accordingly
        """
        while not self._data_processing_stop:
            data = None
            try:
                data = self._model.read_and_parse()
            except:
                # traceback.print_exc()
                self._model.disconnect()
                self._view.add_update(self._view.set_connected, False)
                self._view.add_update(self._view.notify, "Connection lost!", 3000)
                self._data_processing_stop = True
                self._start_com_port_update()
                continue
            if data[0] is not None:
                if data[2] is not None:
                    # Power data
                    min_power, max_power = self._model.get_min_max_power()
                    self._view.add_update(self._view.set_power_data, data, min_power, max_power)
                else:
                    # Configuration
                    frequency, attenuation, _ = data
                    self._view.add_update(self._view.set_settings, frequency, attenuation)
                    self._view.add_update(self._view.notify, "Configuration fetched!", 3000)
                    # Reset update speed
                    self.update_speed_changed_handler(self._view.get_update_speed_index(), False)
            sleep(self._model.get_data_update_interval())

    def _stop_data_processing(self):
        """Stops the data processing thread
        """
        if self._data_processing_thread is not None and self._data_processing_thread.is_alive():
            self._data_processing_stop = True
            self._data_processing_thread.join()
            self._data_processing_thread = None

    def _start_data_processing(self):
        """Starts the data processing thread
        """
        if self._data_processing_thread is None or not self._data_processing_thread.is_alive():
            self._data_processing_stop = False
            self._data_processing_thread = Thread(target=self._process_data)
            self._data_processing_thread.start()

class UpdateQueueWatcher(QObject):
    """A class for watching a queue for UI updates from external threads and emitting signals to the main thread when an update is available

    :param queue: The queue to watch
    :type queue: Queue
    :param finished: Signal emitted when the thread finishes
    :type finished: pyqtSignal
    :param update_available: Signal emitted when an update is available
    :type update_available: pyqtSignal
    """
    finished = pyqtSignal()
    update_available = pyqtSignal(tuple)

    def __init__(self, queue: Queue):
        """Constructor method
        """
        super(QObject, self).__init__()
        self.queue = queue
        self._stop = False

    def run(self):
        """Watches the queue for updates and emits a signal with update data when available
        """
        # TODO thread will not stop if queue is empty
        while not self._stop:
            self.update_available.emit(self.queue.get())
        self.finished.emit()

    def stop(self):
        self._stop = True

class View:
    """A class containing the view of the application

    :param _controller: The controller of the application
    :type _controller: Controller
    :param _window: The UI window
    :type _window: QtWidgets.QMainWindow
    :param _update_queue: A queue for storing UI updates from external threads
    :type _update_queue: Queue
    :param _update_watcher: A thread for watching the update queue
    :type _update_watcher: UpdateQueueWatcher
    """
    def __init__(self, controller: Controller):
        """Constructor method

        :param controller: The controller of the application
        :type controller: Controller
        """
        self._controller = controller
        self._window = uic.loadUi(path.abspath(path.join(path.dirname(__file__), 'powerMeter.ui')))
        self._update_queue = Queue()
        self._init_UI()
        self._start_update_watcher()

    # UI logic

    def _init_UI(self):
        """Initializes the UI
        """
        self._window.connectBtn.clicked.connect(self._controller.connect_btn_handler)
        self._window.pauseBtn.clicked.connect(self._controller.pause_btn_handler)
        self._window.resetStatsBtn.clicked.connect(self._controller.reset_stats_btn_handler)
        self._window.resetCapturedBtn.clicked.connect(self._controller.reset_captured_btn_handler)
        self._window.captureBtn.clicked.connect(self._controller.capture_btn_handler)
        self._window.exportBtn.clicked.connect(self._controller.export_btn_handler)
        self._window.updateSpeed.currentIndexChanged.connect(self._controller.update_speed_changed_handler)
        self._window.applySettingsBtn.clicked.connect(self._controller.apply_settings_btn_handler)
        self._window.readSettingsBtn.clicked.connect(self._controller.read_settings_btn_handler)
        self._window.capturedValues.horizontalHeader().sectionClicked.connect(self._controller.table_header_clicked_handler)
        # self.window.attenuation.wheelEvent = lambda event: None
        self._window.frequency.wheelEvent = lambda event: None
        self._window.updateSpeed.wheelEvent = lambda event: None
        self._window.comPorts.wheelEvent = lambda event: None

    def show_UI(self):
        """Shows the window
        """
        self._window.show()

    def set_connected(self, connected: bool):
        """Changes the UI according to the connection status

        :param connected: Whether the power meter is connected or not
        :type connected: bool
        """
        self.resume_updates(connected)
        self._window.connectBtn.setText("Disconnect" if connected else "Connect")
        self._window.comPorts.setEnabled(not connected)

        self._window.pauseBtn.setEnabled(connected)
        self._window.resetStatsBtn.setEnabled(connected)
        self._window.updateSpeed.setEnabled(connected)
        self._window.updateSpeedLabel.setEnabled(connected)

        self._window.attenuationLabel.setEnabled(connected)
        self._window.attenuation.setEnabled(connected)
        self._window.frequencyLabel.setEnabled(connected)
        self._window.frequency.setEnabled(connected)
        self._window.applySettingsBtn.setEnabled(connected)

    def set_ports_available(self, ports: list[str]):
        """Displays available serial ports

        :param ports: A list of available serial ports
        :type ports: list
        """
        self._window.comPorts.clear()
        self._window.comPorts.addItems(ports)
        self._window.connectBtn.setEnabled(len(ports) > 0)

    def set_power_data(self, current: tuple[Optional[float], Optional[float], Optional[str]],
                       min: tuple[Optional[float], Optional[float], Optional[str]],
                       max: tuple[Optional[float], Optional[float], Optional[str]]):
        """Displays power data

        :param current: The current power data (in dBm, in W or mW or uW, unit of the second value)
        :type current: tuple
        :param min: The minimum power data (in dBm, in W or mW or uW, unit of the second value)
        :type min: tuple
        :param max: The maximum power data (in dBm, in W or mW or uW, unit of the second value)
        :type max: tuple
        """
        current_dbm, current_w, current_unit = current
        min_dbm, min_w, min_unit = min
        max_dbm, max_w, max_unit = max
        if current_dbm is not None and current_w is not None and current_unit is not None:
            self._window.curDbm.setText(f"{current_dbm:.2f} dBm")
            self._window.curW.setText(f"{current_w:.2f} {current_unit}")
        if min_dbm is not None and min_w is not None and min_unit is not None:
            self._window.minDbm.setText(f"{min_dbm:.2f} dBm")
            self._window.minW.setText(f"{min_w:.2f} {min_unit}")
        if max_dbm is not None and max_w is not None and max_unit is not None:
            self._window.maxDbm.setText(f"{max_dbm:.2f} dBm")
            self._window.maxW.setText(f"{max_w:.2f} {max_unit}")

    def set_settings(self, frequency: int, attenuation: float):
        """Displays the current configuration

        :param frequency: The frequency in MHz
        :type frequency: int
        :param attenuation: The attenuation in dBm
        :type attenuation: float
        """
        self._window.frequency.setValue(frequency)
        self._window.attenuation.setValue(attenuation)

    def resume_updates(self, resume: bool):
        """Resumes/pauses the data updates

        :param resume: Whether to resume or pause the data updates
        :type resume: bool
        """
        self._window.pauseBtn.setText("Pause updates" if resume else "Resume updates")
        self._window.readSettingsBtn.setEnabled(resume)
        self._window.resetStatsBtn.setEnabled(resume)

    def notify(self, text: str, duration: int=0):
        """Sets status bar text (for a specified duration)

        :param text: The text to display
        :type text: str
        :param duration: The duration of the status bar message in milliseconds (0 means infinite)
        :type duration: int
        """
        self._window.statusBar.showMessage(text, duration)

    def get_selected_port(self) -> str:
        """Get the selected serial port

        :return: The selected serial port
        :rtype: str
        """
        return self._window.comPorts.currentText().split(":")[0]
    
    def get_update_speed_index(self) -> int:
        """Get the index of the selected refresh rate

        :return: The index of the selected refresh rate
        :rtype: int
        """
        return self._window.updateSpeed.currentIndex()
    
    def get_settings(self) -> tuple[int, float]:
        """Get the current device configuration

        :return: The current device configuration (frequency, attenuation)
        :rtype: tuple
        """
        return self._window.frequency.value(), self._window.attenuation.value()
    
    def set_captured_power_unit(self, unit: str):
        """Set the power unit of captured values
        """
        for i in [1, 3, 5]:
            header = self._window.capturedValues.horizontalHeaderItem(i)
            header.setText(header.text().split(" ")[0] + " " + unit)

    def add_capture(self, current: list[float, float, str], min: list[float, float, str], max: list[float, float, str]):
        """Adds a new row to the captured values table

        :param current: The current power data (in dBm, in W or mW or uW, unit of the second value)
        :type current: tuple
        :param min: The minimum power data (in dBm, in W or mW or uW, unit of the second value)
        :type min: tuple
        :param max: The maximum power data (in dBm, in W or mW or uW, unit of the second value)
        :type max: tuple
        """
        current_dbm, current_w, _ = current
        min_dbm, min_w, _ = min
        max_dbm, max_w, _ = max
        row = self._window.capturedValues.rowCount()
        self._window.capturedValues.insertRow(row)
        self._window.capturedValues.setItem(row, 0, QtWidgets.QTableWidgetItem(f"{current_dbm:.2f}"))
        self._window.capturedValues.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{current_w:.2f}"))
        self._window.capturedValues.setItem(row, 2, QtWidgets.QTableWidgetItem(f"{min_dbm:.2f}"))
        self._window.capturedValues.setItem(row, 3, QtWidgets.QTableWidgetItem(f"{min_w:.2f}"))
        self._window.capturedValues.setItem(row, 4, QtWidgets.QTableWidgetItem(f"{max_dbm:.2f}"))
        self._window.capturedValues.setItem(row, 5, QtWidgets.QTableWidgetItem(f"{max_w:.2f}"))

    def reset_capture_table(self, is_connected: bool=True):
        """Resets the captured values table
        """
        self._window.capturedValues.setRowCount(0)

    def enable_capture_section(self, enable: bool):
        self._window.captureBtn.setEnabled(enable)
        self._window.exportBtn.setEnabled(enable)
        self._window.capturedValues.setEnabled(enable)
        self._window.resetCapturedBtn.setEnabled(enable)

    # Thread logic

    def _start_update_watcher(self):
        """Starts the update watcher thread
        """
        self._update_watcher_thread = QThread()
        self._update_watcher = UpdateQueueWatcher(self._update_queue)
        self._update_watcher.moveToThread(self._update_watcher_thread)
        self._update_watcher_thread.started.connect(self._update_watcher.run)
        self._update_watcher.finished.connect(self._update_watcher_thread.quit)
        self._update_watcher.finished.connect(self._update_watcher.deleteLater)
        self._update_watcher_thread.finished.connect(self._update_watcher_thread.deleteLater)

        self._update_watcher.update_available.connect(self._update_queue_callback)
        self._update_watcher_thread.start()

    def _update_queue_callback(self, update: tuple[callable, list]):
        """Callback for the update watcher thread. Contains the UI update to execute.
            This function is called by the main thread, so it can safely update the UI.

        :param update: The UI update to execute
        :type update: tuple
        """
        func, args = update
        func(*args)

    def stop_periodic_update(self):
        """Stops the update watcher thread
        """
        self._update_watcher.stop()

    def add_update(self, func: callable, *args: list):
        """Adds a UI update to the queue. THIS FUNCTION IS CALLED BY AN EXTERNAL THREAD

        :param func: The UI update to execute
        :type func: callable
        :param args: The arguments of the UI update
        :type args: list
        """
        self._update_queue.put((func, args,))

if __name__ == "__main__":
    controller = Controller()
    controller.start_app()