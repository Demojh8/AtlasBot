import Queue
import threading
import time

import serial


class SerialConnectorThread(threading.Thread):
    """ A thread for monitoring a COM port. The COM port is 
        opened when the thread is started.
    
        data_q:
            Queue for received data. Items in the queue are
            (data, timestamp) pairs, where data is a binary 
            string representing the received data, and timestamp
            is the time elapsed from the thread's start (in 
            seconds).
        
        error_q:
            Queue for error messages. In particular, if the 
            serial port fails to open for some reason, an error
            is placed into this queue.
        
        port:
            The COM port to open. Must be recognized by the 
            system.
        
        port_baud/stopbits/parity: 
            Serial communication parameters
        
        port_timeout:
            The timeout used for reading the COM port. If this
            value is low, the thread will return data in finer
            grained chunks, with more accurate timestamps, but
            it will also consume more CPU.
    """
    def __init__(   self, ui_mode,
                    data_q, error_q, 
                    port_name,
                    port_baud,
                    port_stopbits=serial.STOPBITS_ONE,
                    port_parity=serial.PARITY_NONE,
                    port_timeout=1):
        threading.Thread.__init__(self)

        self.ui_mode = ui_mode
        self.serial_port = None
        self.serial_arg = dict( port=port_name,
                                baudrate=port_baud,
                                stopbits=port_stopbits,
                                parity=port_parity,
                                timeout=port_timeout)

        self.data_q = data_q
        self.error_q = error_q
        
        self.alive = threading.Event()
        self.alive.set()
        
    def run(self):
        try:
            if self.serial_port: 
                self.serial_port.close()
            self.serial_port = serial.Serial(**self.serial_arg)
            
        except serial.SerialException, e:
            self.error_q.put(e.message)
            return
        

        if(self.ui_mode == 0):
             
            while self.alive.isSet():
                # Reading 1 byte, followed by whatever is left in the
                # read buffer, as suggested by the developer of 
                # PySerial.
                # 
                data = self.serial_port.readline()

                dataList = data.split()
                
                if len(dataList) == 6:
                    self.data_q.put((dataList[0], dataList[1], dataList[2], dataList[3], dataList[4], dataList[5]))

            # clean up
            if self.serial_port:
                self.serial_port.close()

        elif(self.ui_mode == 1):
            pass

    def write(self, data):
        self.serial_port.write(data)
            
        

    def join(self, timeout=None):
        self.alive.clear()
        threading.Thread.join(self, timeout)
