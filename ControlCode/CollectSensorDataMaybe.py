"""

This file collects data from both SensONE and 3D printed sensor at the same time.

For each timestamp, the wrench is collected from the SensONE (in its own frame)
and the 8 raw sensor values are collected from the 3D printed sensor (s_0 to s_7).

The results are stored in a csv file, each row containing the next values:
< Timestamp, Fx, Fy, Fz, Mx, My, Mz, s0, s1, s2, s3, s4, s5, s6, s7 >

"""
import math
import sys
import struct
import time
import collections

import pysoem
import ctypes
import struct

import csv
import os

import serial


class MinimalExample:

    BOTA_VENDOR_ID = 0xB07A
    BOTA_PRODUCT_CODE = 0x00000001
    SINC_LENGTH = 256
    # The time step is set according to the sinc filter size
    time_step = 1.0

    def __init__(self):
        self._master = pysoem.Master()
        SlaveSet = collections.namedtuple(
            'SlaveSet', 'slave_name product_code config_func')
        self._expected_slave_mapping = {0: SlaveSet('BFT-MEDS-ECAT-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)}

    def bota_sensor_setup(self, slave_pos):
        print("bota_sensor_setup")
        slave = self._master.slaves[slave_pos]

        ## Set sensor configuration
        # calibration matrix active
        slave.sdo_write(0x8010, 1, bytes(ctypes.c_uint8(1)))
        # temperature compensation
        slave.sdo_write(0x8010, 2, bytes(ctypes.c_uint8(0)))
        # IMU active
        slave.sdo_write(0x8010, 3, bytes(ctypes.c_uint8(1)))

        ## Set force torque filter
        # FIR disable
        slave.sdo_write(0x8006, 2, bytes(ctypes.c_uint8(1)))
        # FAST enable
        slave.sdo_write(0x8006, 3, bytes(ctypes.c_uint8(0)))
        # CHOP enable
        slave.sdo_write(0x8006, 4, bytes(ctypes.c_uint8(0)))
        # Sinc filter size
        slave.sdo_write(0x8006, 1, bytes(ctypes.c_uint16(self.SINC_LENGTH)))

        ## Get sampling rate
        sampling_rate = struct.unpack('h', slave.sdo_read(0x8011, 0))[0]
        print("Sampling rate {}".format(sampling_rate))
        if sampling_rate > 0:
            self.time_step = 1.0/float(sampling_rate)

        print("time step {}".format(self.time_step))

    def run(self):
        self._master.open(r'\Device\NPF_{0337D12E-4925-41B4-96DF-B5A60A03FD7E}')

        # config_init returns the number of slaves found
        if self._master.config_init() > 0:

            print("{} slaves found and configured".format(
                len(self._master.slaves)))

            for i, slave in enumerate(self._master.slaves):

                assert(slave.man == self.BOTA_VENDOR_ID)
                assert(
                        slave.id == self._expected_slave_mapping[i].product_code)
                slave.config_func = self._expected_slave_mapping[i].config_func

            # PREOP_STATE to SAFEOP_STATE request - each slave's config_func is called
            self._master.config_map()

            # wait 50 ms for all slaves to reach SAFE_OP state
            if self._master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
                self._master.read_state()
                for slave in self._master.slaves:
                    if not slave.state == pysoem.SAFEOP_STATE:
                        print('{} did not reach SAFEOP state'.format(slave.name))
                        print('al status code {} ({})'.format(hex(slave.al_status),
                                                              pysoem.al_status_code_to_string(slave.al_status)))
                raise Exception('not all slaves reached SAFEOP state')

            self._master.state = pysoem.OP_STATE
            self._master.write_state()

            self._master.state_check(pysoem.OP_STATE, 50000)
            if self._master.state != pysoem.OP_STATE:
                self._master.read_state()
                for slave in self._master.slaves:
                    if not slave.state == pysoem.OP_STATE:
                        print('{} did not reach OP state'.format(slave.name))
                        print('al status code {} ({})'.format(hex(slave.al_status),
                                                              pysoem.al_status_code_to_string(slave.al_status)))
                raise Exception('not all slaves reached OP state')

            try:
                port = 'COM3'
                baudrate = 115200
                ser = serial.Serial(port, baudrate, parity=serial.PARITY_NONE)
                print("Got the serial port.")

                n_sensors = 8
                overload_lower = 50
                overload_upper = 950

                datapoints = 0
                start_datapoint = 10
                total_datapoints = 1000 + start_datapoint
                start_time = time.time()

                #while True:
                while datapoints < total_datapoints:
                    datapoints += 1

                    # Get timestamp
                    timestamp = (time.time() - start_time)

                    # Get wrench
                    self._master.send_processdata()
                    self._master.receive_processdata(2000)
                    sensor_input_as_bytes = self._master.slaves[0].input
                    Fx = struct.unpack_from('f', sensor_input_as_bytes, 5)[0]
                    Fy = struct.unpack_from('f', sensor_input_as_bytes, 9)[0]
                    Fz = struct.unpack_from('f', sensor_input_as_bytes, 13)[0]
                    Mx = struct.unpack_from('f', sensor_input_as_bytes, 17)[0]
                    My = struct.unpack_from('f', sensor_input_as_bytes, 21)[0]
                    Mz = struct.unpack_from('f', sensor_input_as_bytes, 25)[0]

                    # Get raw sensor values
                    try:
                        data = ser.readline()
                        (str_D, seq_number, error_mask, s0, s1, s2, s3, s4, s5, s6, s7) = \
                            [t(s) for t, s in zip((str, int, int, int, int, int, int, int, int, int, int), data.split())]
                    except ValueError:
                        print("parsing input data failed, data='", data, "'")
                        continue
                    except IndexError:  # probably wrong formatted string...
                        print("could not parse message/data:", data)
                        continue

                    s = [s0, s1, s2, s3, s4, s5, s6, s7]
                    for i in range(n_sensors):
                        if (s[i] < overload_lower) or (s[i] > overload_upper):
                            print(f"Force overload channel {i}")
                            continue

                    # Print line
                    # print('\n')
                    # print(f"Fx: {Fx}, Fy: {Fy}, Fz: {Fz}, Mx: {Mx}, My: {My}, Mz: {Mz},\n"
                    #      f" s0: {s0}, s1: {s1}, s2: {s2}, s3: {s3}, s4: {s4}, s5: {s5}, s6: {s6}, s7: {s7}")
                    # print('\n')
                    if datapoints > start_datapoint:
                        writer.writerow({'Timestamp': timestamp,
                                         'Fx': Fx, 'Fy': Fy, 'Fz': Fz, 'Mx': Mx,'My': My, 'Mz': Mz,
                                         's0': s0, 's1': s1, 's2': s2, 's3': s3, 's4': s4, 's5': s5, 's6': s6, 's7': s7
                                         })

                    csvfile.flush()
                    time.sleep(self.time_step)
                    print(f"Saved {datapoints} datapoints")

            except KeyboardInterrupt:
                # ctrl-C abort handling
                print('Stopped.')
            finally:
                csvfile.close()
                print('The csv file is closed.')

            self._master.state = pysoem.INIT_STATE
            # request INIT state for all slaves
            self._master.write_state()
            self._master.close()
        else:
            print('slaves not found')


if __name__ == '__main__':

    print('get_data')

    # Open CSV file once at the start
    filename = 'data.csv'
    csvfile = open(filename, 'a', newline='')
    fieldnames = ['Timestamp', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz', 's0', 's1', 's2', 's3', 's4', 's5', 's6', 's7']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    # Write header only if file is empty
    if os.path.getsize(filename) == 0:
        writer.writeheader()

    try:
        MinimalExample().run()
    except Exception as expt:
        print("Exception. Something went wrong.")
        sys.exit(1)

