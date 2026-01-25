#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import smbus2
import time
from collections import deque

# -------------------------
# MAX30102 REGISTERS
# -------------------------
REG_INT_STATUS_1 = 0x00
REG_INT_ENABLE_1 = 0x02
REG_FIFO_WR_PTR  = 0x04
REG_OVF_COUNTER  = 0x05
REG_FIFO_RD_PTR  = 0x06
REG_FIFO_DATA    = 0x07
REG_FIFO_CONFIG  = 0x08
REG_MODE_CONFIG  = 0x09
REG_SPO2_CONFIG  = 0x0A
REG_LED1_PA      = 0x0C
REG_LED2_PA      = 0x0D

# Timing
SAMPLE_PERIOD   = 0.02   # 50 Hz
WINDOW_SECONDS  = 10.0   # analysis window


class PulseOxNode(Node):
    def __init__(self):
        super().__init__('pulse_ox_node')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x57)

        self.bus_id = self.get_parameter('i2c_bus').value
        self.addr   = self.get_parameter('i2c_addr').value

        # -------------------------
        # I2C init
        # -------------------------
        self.bus = smbus2.SMBus(self.bus_id)
        self.init_max30102()

        # -------------------------
        # Publishers
        # -------------------------
        self.pub_raw = self.create_publisher(
            Float32MultiArray,
            '/radar/pulseox/raw',
            10
        )

        self.pub_vitals = self.create_publisher(
            Float32MultiArray,
            '/radar/pulseox/vitals',
            10
        )

        # -------------------------
        # Buffers
        # -------------------------
        self.times   = deque()
        self.ir_buf  = deque()
        self.red_buf = deque()

        # -------------------------
        # Timer
        # -------------------------
        self.timer = self.create_timer(SAMPLE_PERIOD, self.read_and_publish)

        self.get_logger().info(
            'PulseOxNode running: '
            '/radar/pulseox/raw and /radar/pulseox/vitals'
        )

    # -------------------------
    # Sensor init
    # -------------------------
    def init_max30102(self):
        # Reset
        self.bus.write_byte_data(self.addr, REG_MODE_CONFIG, 0x40)
        time.sleep(0.1)

        # Clear FIFO
        self.bus.write_byte_data(self.addr, REG_FIFO_WR_PTR, 0x00)
        self.bus.write_byte_data(self.addr, REG_OVF_COUNTER, 0x00)
        self.bus.write_byte_data(self.addr, REG_FIFO_RD_PTR, 0x00)

        # FIFO config: sample averaging = 4
        self.bus.write_byte_data(self.addr, REG_FIFO_CONFIG, 0x0F)

        # SpO2 mode
        self.bus.write_byte_data(self.addr, REG_MODE_CONFIG, 0x03)

        # SpO2 config
        self.bus.write_byte_data(self.addr, REG_SPO2_CONFIG, 0x27)

        # LED currents
        self.bus.write_byte_data(self.addr, REG_LED1_PA, 0x3F)  # RED
        self.bus.write_byte_data(self.addr, REG_LED2_PA, 0x3F)  # IR

        # Enable interrupt
        self.bus.write_byte_data(self.addr, REG_INT_ENABLE_1, 0x40)

        self.get_logger().info('MAX30102 initialized')

    # -------------------------
    # Read sample
    # -------------------------
    def read_sample(self):
        data = self.bus.read_i2c_block_data(self.addr, REG_FIFO_DATA, 6)
        red = ((data[0] << 16) | (data[1] << 8) | data[2]) & 0x3FFFF
        ir  = ((data[3] << 16) | (data[4] << 8) | data[5]) & 0x3FFFF
        return red, ir

    # -------------------------
    # Buffer handling
    # -------------------------
    def update_buffers(self, t, ir, red):
        self.times.append(t)
        self.ir_buf.append(ir)
        self.red_buf.append(red)

        while self.times and (t - self.times[0]) > WINDOW_SECONDS:
            self.times.popleft()
            self.ir_buf.popleft()
            self.red_buf.popleft()

    # -------------------------
    # HR & SpO2 computation
    # -------------------------
    def compute_hr_and_spo2(self):
        n = len(self.ir_buf)
        if n < 30:
            return None, None

        t_list   = list(self.times)
        ir_list  = list(self.ir_buf)
        red_list = list(self.red_buf)

        ir_mean = sum(ir_list) / n
        ir_centered = [x - ir_mean for x in ir_list]

        amp = max(ir_centered) - min(ir_centered)
        if amp < 500:
            return None, None

        threshold = min(ir_centered) + 0.5 * amp

        peak_times = []
        for i in range(1, n - 1):
            if (
                ir_centered[i] > threshold and
                ir_centered[i] > ir_centered[i - 1] and
                ir_centered[i] > ir_centered[i + 1]
            ):
                peak_times.append(t_list[i])

        if len(peak_times) < 2:
            return None, None

        intervals = [
            peak_times[i + 1] - peak_times[i]
            for i in range(len(peak_times) - 1)
            if 0.3 < (peak_times[i + 1] - peak_times[i]) < 2.0
        ]

        if not intervals:
            return None, None

        avg_interval = sum(intervals) / len(intervals)
        hr_bpm = 60.0 / avg_interval

        if hr_bpm < 40 or hr_bpm > 180:
            hr_bpm = None

        red_mean = sum(red_list) / n
        ac_red = (sum((x - red_mean) ** 2 for x in red_list) / n) ** 0.5
        ac_ir  = (sum((x - ir_mean) ** 2 for x in ir_list) / n) ** 0.5

        spo2 = None
        if red_mean > 0 and ir_mean > 0 and ac_ir > 0:
            r = (ac_red / red_mean) / (ac_ir / ir_mean)
            spo2 = 110.0 - 25.0 * r
            if spo2 < 70 or spo2 > 100:
                spo2 = None

        return hr_bpm, spo2

    # -------------------------
    # Main loop
    # -------------------------
    def read_and_publish(self):
        try:
            red, ir = self.read_sample()
            now = time.time()

            self.update_buffers(now, ir, red)

            raw_msg = Float32MultiArray()
            raw_msg.data = [float(red), float(ir)]
            self.pub_raw.publish(raw_msg)

            hr, spo2 = self.compute_hr_and_spo2()
            if hr is not None or spo2 is not None:
                vitals_msg = Float32MultiArray()
                vitals_msg.data = [
                    float(hr) if hr is not None else -1.0,
                    float(spo2) if spo2 is not None else -1.0
                ]
                self.pub_vitals.publish(vitals_msg)

        except Exception as e:
            self.get_logger().error(f'PulseOx error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PulseOxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
