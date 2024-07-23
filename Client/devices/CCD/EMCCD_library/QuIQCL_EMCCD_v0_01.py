"""
This is a heritage of HunHuh who left us remaining lots of fineworks

Modification applied by JJH to integrate it with CCD GUI
"""
import os
import time
import threading
import numpy as np
import ctypes as ct

from PyQt5.QtCore import pyqtSignal, QObject
from EMCCD_const import EMCCD_Const
from EMCCD_drv import *
from datetime import datetime

filename = os.path.abspath(__file__)
dirname = os.path.dirname(filename)

class AndorCapabilities(ct.Structure):
    """
    struct AndorCapabilities used for GetCapacities function
    """
    _fields_ = [("ulSize", ct.c_ulong),
                ("ulAcqModes", ct.c_ulong),
                ("ulReadModes", ct.c_ulong),
                ("ulFTReadModes", ct.c_ulong),
                ("ulTriggerModes", ct.c_ulong),
                ("ulCameraType", ct.c_ulong),
                ("ulPixelModes", ct.c_ulong),
                ("ulSetFunctions", ct.c_ulong),
                ("ulGetFunctions", ct.c_ulong),
                ("ulFeatures", ct.c_ulong),
                ("ulPCICard", ct.c_ulong),
                ("ulEMGainCapability", ct.c_ulong)]

    def __init__(self):
        super().__init__()
        self.ulSize = ct.sizeof(AndorCapabilities)

    def print_capabilities(self):
        print("ulSize: {:b}".format(self.ulSize))
        print("ulAcqModes: {:b}".format(self.ulAcqModes))
        print("ulReadModes: {:b}".format(self.ulReadModes))
        print("ulFTReadModes: {:b}".format(self.ulFTReadModes))
        print("ulTriggerModes: {:b}".format(self.ulTriggerModes))
        print("ulCameraType: {:b}".format(self.ulCameraType))
        print("ulPixelModes: {:b}".format(self.ulPixelModes))
        print("ulSetFunctions: {:b}".format(self.ulSetFunctions))
        print("ulGetFunctions: {:b}".format(self.ulGetFunctions))
        print("ulFeatures: {:b}".format(self.ulFeatures))
        print("ulPCICard: {:b}".format(self.ulPCICard))
        print("ulEMGainCapability: {:b}".format(self.ulEMGainCapability))


class EMCCD(EMCCD_Const):
    
    def __init__(self, parent=None, dll_file=None):
        self.parent = parent
        if not dll_file == None:
            self.DLL_FILE = dll_file
        self.dll = ct.WinDLL(self.DLL_FILE)
        self._load_dll(self.DLL_FILE)

        self.buffer_cv = threading.Condition()
        self._trigger_count = 1
        self._temperature = 30
        self._target_temperature = -60
        
        self._avail_acquisition_modes = ['single_scan', 'continuous_scan']
            
        self._acquisition_mode = 'continuous_scan'
        
        self._image_buffer = np.zeros((self.RAW_IMG_HEIGHT, self.RAW_IMG_WIDTH), dtype=np.int32)
        self._buffer_size = 50
        self._buffer_list = []
        self._img_cnt = 0

        self._running_thread = None
        self._cooling_tread = None
        
        self._cool_on = False
        self._cool_status_list = ["stabilized", "cooling", "hot"]
        self._cool_status = "hot"

        self._image_buffer = np.zeros((self.RAW_IMG_HEIGHT, self.RAW_IMG_WIDTH), dtype=np.int32)

        with open(dirname + "/emccd_functions.txt") as f:
            dll_methods = f.read().splitlines()
        
        for method in dll_methods:
            try:
                # used try-except to skip reserved(not yet implemented) functions
                setattr(self, method, self.error_check(getattr(self.dll, method)))
            except AttributeError as e:
                print(method, "not yet implemented (Reserved Function. Check SDK for details).")

        self.open_device()
        

    def _load_dll(self, dll_file=""):
        """Loads the Thor .NET binaries and adds them to the PATH.
        """
        self.dll = ct.WinDLL(dll_file)

        with open("./EMCCD_library/emccd_functions.txt") as f:
            dll_methods = f.read().splitlines()
        
        for method in dll_methods:
            try:
                # used try-except to skip reserved(not yet implemented) functions
                setattr(self, method, self.error_check(getattr(self.dll, method)))
            except AttributeError as e:
                print(method, "not yet implemented (Reserved Function. Check SDK for details).")


    def error_check(self, func):
        def func_wrapper(*args, **kwargs):
            if "ignore_code" in kwargs.keys():
                ignore_code = kwargs["ignore_code"]
                del kwargs["ignore_code"]
            else:
                ignore_code = []

            if "ignore_error" in kwargs.keys():
                ignore_error = kwargs["ignore_error"]
                del kwargs["ignore_error"]
            else:
                ignore_error = False

            code = func(*args, **kwargs)

            if not (code == DRV_SUCCESS or ignore_error or code in ignore_code):
                print('atmcd SDK returned Error: {} [{}]'.format(int_to_drv[code], code))
            return code
        func_wrapper.__name__ = func.__name__
        return func_wrapper
    
    def open_device(self):
        dll_dir = self.DLL_FILE[:self.DLL_FILE.rfind("/")+1]
        code = self.Initialize(dll_dir)
        if code == DRV_SUCCESS:
            print('Device successfully connected. You need to cool the device before you can start imaging')
        else:
            print('Something went wrong with Initialize()')
        self.default_setting()
        
    def default_setting(self):
        andor_capabilities = AndorCapabilities()
        self.GetCapabilities(ct.pointer(andor_capabilities))
        # andor_capabilities.print_capabilities()

        self.SetReadMode(4)  # Image Readout Mode
        self.SetTriggerMode(0)  # use Internal Trigger
        self.SetShutter(1, 1, self.SHU_CLOSINGTIME, self.SHU_OPENINGTIME)  # SetShutter(int typ, int mode, int closingtime, int openingtime) 
                                                                 # mode 0:auto, 1: open, 2: close
        self.SetImage(1, 1, 1, self.RAW_IMG_WIDTH, 1, self.RAW_IMG_HEIGHT)  # Full size

        # get size of circular buffer
        circular_buffer_size = ct.c_long(0)
        self.GetSizeOfCircularBuffer(ct.pointer(circular_buffer_size))
        self.circular_buffer_size = circular_buffer_size.value

        # set acquisition mode
        self.acquisition_mode = "continuous_scan"
        self.SetAcquisitionMode(5)

        min_gain = ct.c_int(0)
        max_gain = ct.c_int(0)
        
        self.GetEMGainRange(ct.pointer(min_gain), ct.pointer(max_gain))
        self._gain_range = [min_gain.value, max_gain.value]

        self._gain = self.DEFAULT_GAIN
        self.gain = self._gain
        
        # self.SetNumberAccumulations(1)
        
        # self._trigger_count = self.DEFAULT_NUMBER_OF_KINETIC_SERIES
        # self.trigger_count = self._trigger_count

        
        self._exposure_time = self.DEFAULT_EXPOSURE
        self.exposure_time = self._exposure_time

    def _get_multiple_acquired_data(self, sleep_time):
        scan_buffer = np.ascontiguousarray(np.zeros(self.RAW_IMG_SIZE, dtype=np.int32))

        status = ct.c_int(DRV_ACQUIRING)
        while status.value == DRV_ACQUIRING:
            self.GetStatus(ct.pointer(status))
            
            while self.WaitForAcquisitionTimeOut(int(sleep_time*1000), ignore_code=[DRV_NO_NEW_DATA]) == DRV_SUCCESS:
                code = self.GetMostRecentImage(scan_buffer.ctypes.data_as(ct.POINTER(ct.c_int32)),
                                               ct.c_ulong(self.RAW_IMG_SIZE),
                                               ignore_code=[DRV_NO_NEW_DATA])
                if code == DRV_SUCCESS:
                    self._image_buffer = scan_buffer.reshape((self.RAW_IMG_WIDTH, self.RAW_IMG_HEIGHT))
                    self._img_cnt += 1
                    self.ccd_image = np.copy(self._image_buffer)
                    if not (len(self._buffer_list) < self._buffer_size):
                        while (len(self._buffer_list) >= self._buffer_size):
                            self._buffer_list.pop(0)
                    self._buffer_list.append(self.ccd_image)

    def get_status(self):
        status = ct.c_int(0)
        self.GetStatus(ct.pointer(status))
        return status.value

    def get_acquisition_timings(self):
        self.exposure = ct.c_float(0.0)
        self.accumulation_cycle_time = ct.c_float(0.0)
        self.kinetic_cycle_time = ct.c_float(0.0)
        code = self.GetAcquisitionTimings(ct.pointer(self.exposure), ct.pointer(self.accumulation_cycle_time), ct.pointer(self.kinetic_cycle_time))


    def run_device(self):
        self._img_cnt = -1
        status = self.get_status()
        if status == DRV_IDLE:
            code = self.StartAcquisition()
            if code == DRV_SUCCESS:               
                # thread start for acquisition
                self.get_acquisition_timings()
                self._running_thread = threading.Thread(target=self._get_multiple_acquired_data, 
                                                       args=([self.kinetic_cycle_time.value]))
                self._running_thread.start()
            else:
                print('Something went wrong with acquisition')
        else:
            print('Could not start acquisition because drive was not IDLE.')

    # def save_acquired_data(self, file_name=None):
    #     if file_name == None:
    #         file_name = "./EMCCD_Acquired_data_{}.tif".format(datetime.now().strftime("%y%m%d_%H%M%S"))
        
    #     counter = 0
    #     while os.path.exists(file_name):
    #         if counter:
    #             file_name = file_name.replace("{}.tif".format(counter-1), "{}.tif".format(counter))
    #         else:
    #             file_name = file_name.replace(".tif", "_0.tif")
    #         counter += 1
    #     self.SaveAsTiff(ct.c_char_p(file_name.encode("utf-8")), 
    #                     ct.c_char_p("".encode("utf-8")), 0, 1)

    def stop_device(self):
        self.AbortAcquisition(ignore_code=[DRV_IDLE])

    def close_device(self):
        if self.acquire_thread is not None:
            self.stop_device()
            self.acquire_thread.join()

        # need to notify to wake up waiting threads
        self.buffer_cv.acquire()
        self.buffer_cv.notify_all()
        self.buffer_cv.release()

        self.CoolerOFF(ignore_error=True)
        self.ShutDown()
#%%
    @property
    def sensor_size(self):
        return [self.RAW_IMG_WIDTH, self.RAW_IMG_HEIGHT]
    
    @property
    def gain(self):
        return self._gain
        
    @gain.setter
    def gain(self, value):
        value = int(value)
        if value > max(self._gain_range):
            value = max(self._gain_range)
        self.SetEMCCDGain(value)
        self._gain = value
        
    @property
    def exposure_time(self):
        return self._exposure_time
    
    @exposure_time.setter
    def exposure_time(self, value):
        """ The exposure time is in ms.
        ThorlabsCCD handles the exposure time in ms.
        To make it consistant, the internal variable is saved in ms.
        """
        self.SetExposureTime(ct.c_float(value/1000))
        self._exposure_time = value
        
    @property
    def acquisition_mode(self):
        return self._acquisition_mode
    
    @acquisition_mode.setter
    def acquisition_mode(self, mode):
        assert isinstance(mode, str), "Acquisition mode should be a string."
        if not mode in self._avail_acquisition_modes:
            raise ValueError ("Acqusition mode should be either 'single_scan' or 'continuous_scan'.")
        self._acquisition_mode = mode
        
        if mode == "single_scan":
            self.SetAcquisitionMode(3) # kinetic series
        else:
            self.SetAcquisitionMode(5) # run till abort
        
    @property
    def trigger_count(self):
        return self._trigger_count
    
    @trigger_count.setter
    def trigger_count(self, count):
        assert isinstance(count, int), "Trigger count should be an int."
        self.SetNumberKinetics(count)
        self._buffer_size = count
        self._trigger_count = count
   
    @property
    def temperature(self):
        return self._temperature

    @temperature.setter
    def temperature(self, temp):
        self.SetTemperature(temp)
        self._temperature = temp
        
    def get_temperature(self):
        pnt_temp = ct.c_int(0)
        self.GetTemperature(ct.byref(pnt_temp),
                            ignore_code=[DRV_TEMP_OFF,
                                         DRV_TEMP_NOT_STABILIZED,
                                         DRV_TEMP_STABILIZED,
                                         DRV_TEMP_NOT_REACHED,
                                         DRV_TEMP_DRIFT,
                                         DRV_ACQUIRING])
        self._temperature = pnt_temp.value
        return self._temperature
        
    def cooler_on(self, temp):
        self.temperature = temp
        self.CoolerON()
        self._cool_on = True
        self._cool_status = "cooling"
        self._cooling_thread = threading.Thread(target=self._get_cooling_status)
        self._cooling_thread.start()
        
        
    def cooler_off(self):
        self._cool_on = False
        self._cooling_thread.join()
        self.CoolerOFF()
        self._cool_status = "hot"

    def _get_cooling_status(self):
        while self._cool_on:
            temp = self.get_temperature()
            if temp < self._target_temperature + 2 and \
               temp > self._target_temperature - 2:
               self._cool_status = "stabilized"
            else:
               self._cool_status = "cooling"
               time.sleep(3)
            
#%%
    @property
    def ccd_image(self):
        return self._image_buffer

    @ccd_image.setter
    def ccd_image(self, image):
        self.buffer_cv.acquire()
        self._image_buffer = image
        self.buffer_cv.notify_all()
        self.buffer_cv.release()