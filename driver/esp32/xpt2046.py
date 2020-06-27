# xpt2046 / tsc2046 / ads7846 touch screen controller driver for LVGL
# Copyright 2020 by Thorsten von Eicken
# Based on a version by Amir Gonnen, Copyright (c) 2019 Littlev Graphics Library, MIT License
#
# This version uses a single SPI transaction to read the touch screen X, Y coordinates and pressure
# values Z1, and Z2. The SPI transaction uses 16 clock cycles per value read and reads each of the
# four values three times, averaging the last two values read. The three reads of each value are
# done in a manner that keeps the output drive on the touch screen constant allowing the voltage
# being read to settle during the first read (which is thrown away). The subsequent two values read
# are then averaged for a little noise filtering. This method is described in the Texas Instruments
# / Burr-Brown "TOUCH SCREEN CONTROLLER TIPS" bulletin. The main disadvantage of this method is that
# it consumes more power than individual single-ended reads.
#
# The "output" of the driver is through the read method, which is called from within
# lv_task_handler. It returns None if the screen is not touched, and screen x-y coordinates if it is
# touched. In order to produce screen x-y coordinates the touch screen range has to be calibrated by
# providing x_min, x_max, y_min, y_max values via the calibrate method. The touch xpt2046 starts out
# in an uncalibrated mode, which is useful to run a calibration application (see tpcal.py in the
# lv_bindings utils directory).

import time
import espidf as esp
import lvesp32
import lvgl as lv

REP = const(3)  # how often to repeat a command, the result is the avg of the last two values
CMD_OFF = const(0x80)
CMD_X = const(0xD1)
CMD_Y = const(0x91)
CMD_Z1 = const(0xB1)
CMD_Z2 = const(0xC1)


class xpt2046:

    CMDS = (CMD_X, CMD_Y, CMD_Z1, CMD_Z2)

    def __init__(
        self, miso=-1, mosi=-1, clk=-1, cs=25, spihost=esp.HSPI_HOST, mhz=5, transpose=False,
        touch_margin=100
    ):
        # Initializations
        self.screen_width = lv.disp_get_hor_res(lv.disp_t.cast(None))
        self.screen_height = lv.disp_get_ver_res(lv.disp_t.cast(None))
        self.miso = miso
        self.mosi = mosi
        self.clk = clk
        self.cs = cs
        self.spihost = spihost
        self.mhz = mhz
        self.cal_x0 = None
        self.transpose = transpose
        self.touch_margin = touch_margin

        self.touch_count = 0
        self.touch_cycles = 0

        self.spi_init()

        # Prepare buffer with commands to send to device
        nc = len(self.CMDS)
        self.buflen = 2 * nc * REP + 1  # +1 for final CMD_OFF
        self.txbuf = esp.heap_caps_malloc(self.buflen, esp.MALLOC_CAP.DMA)
        txbuf = self.txbuf.__dereference__(self.buflen)
        for c in range(nc):
            for r in range(REP):
                txbuf[(c * REP + r) * 2 + 0] = self.CMDS[c]  # first byte is command, second is 0
                txbuf[(c * REP + r) * 2 + 1] = 0
        txbuf[2 * nc * REP] = CMD_OFF
        # print("xpt2046 txbuf:", ["%02x" % v for v in txbuf])
        self.rxbuf = esp.heap_caps_malloc(self.buflen, esp.MALLOC_CAP.DMA)

        indev_drv = lv.indev_drv_t()
        lv.indev_drv_init(indev_drv)
        indev_drv.type = lv.INDEV_TYPE.POINTER
        indev_drv.read_cb = self.read
        lv.indev_drv_register(indev_drv)
        print("xpt2046 touch initialized")

    def calibrate(self, x0, y0, x1, y1):
        self.cal_x0 = x0
        self.cal_y0 = y0
        self.cal_x1 = x1
        self.cal_y1 = y1

    def spi_init(self):
        esp.gpio_pad_select_gpio(self.cs)

        # Initialize the SPI bus, if needed
        if self.miso >= 0 and self.mosi >= 0 and self.clk >= 0:
            esp.gpio_pad_select_gpio(self.miso)
            esp.gpio_pad_select_gpio(self.mosi)
            esp.gpio_pad_select_gpio(self.clk)

            esp.gpio_set_direction(self.miso, esp.GPIO_MODE.INPUT)
            esp.gpio_set_pull_mode(self.miso, esp.GPIO.PULLUP_ONLY)
            esp.gpio_set_direction(self.mosi, esp.GPIO_MODE.OUTPUT)
            esp.gpio_set_direction(self.clk, esp.GPIO_MODE.OUTPUT)

            buscfg = esp.spi_bus_config_t(
                {
                    "miso_io_num": self.miso,
                    "mosi_io_num": self.mosi,
                    "sclk_io_num": self.clk,
                    "quadwp_io_num": -1,
                    "quadhd_io_num": -1,
                    "max_transfer_sz": 128,
                }
            )
            ret = esp.spi_bus_initialize(self.spihost, buscfg, 1)
            if ret != 0:
                raise RuntimeError("Failed initializing SPI bus")

        # Register finalizer callback to deinit SPI.  This gets called on soft reset.
        self.finalizer = lvesp32.cb_finalizer(self.deinit)

        # Attach the xpt2046 to the SPI bus
        devcfg = esp.spi_device_interface_config_t(
            {
                "clock_speed_hz": self.mhz * 1000 * 1000,
                "mode": 0,  # SPI mode 0
                "spics_io_num": self.cs,  # CS pin
                "queue_size": 2,
                "duty_cycle_pos": 128,
            }
        )
        ptr_to_spi = esp.C_Pointer()
        ret = esp.spi_bus_add_device(self.spihost, devcfg, ptr_to_spi)
        if ret != 0:
            raise RuntimeError("Failed adding SPI device")
        self.spi = ptr_to_spi.ptr_val

    trans_result_ptr = esp.C_Pointer()

    # Deinitalize SPI device and bus
    def deinit(self):
        print("Deinitializing XPT2046...")
        if self.spi:
            # Pop all pending transaction results
            ret = 0
            while ret == 0:
                ret = esp.spi_device_get_trans_result(self.spi, self.trans_result_ptr, 1)

            # Remove device
            esp.spi_bus_remove_device(self.spi)

            # Free SPI bus
            esp.spi_bus_free(self.spihost)

    # do_cmds performs the commands in [cmds] in a single transaction. Each command is run in
    # 16 cycles overlapping the reading of one command with the writing of the next, per datasheet.
    # Each command is repeated rep times and the last two results are averaged.
    def do_cmds(self):
        trans = esp.spi_transaction_t(
            {"length": self.buflen * 8, "tx_buffer": self.txbuf, "rx_buffer": self.rxbuf}
        )
        rxbuf = self.rxbuf.__dereference__(self.buflen)
        res = esp.spi_device_polling_transmit(self.spi, trans)
        # print("got:", res, ["%02x"%v for v in rxbuf])
        vals = [(rxbuf[1 + 2 * i] << 5) | (rxbuf[2 + 2 * i] >> 3) for i in range(self.buflen // 2)]
        # print("res:", res, "vals:", vals)
        res = [
            (vals[c * REP + REP - 2] + vals[c * REP + REP - 1]) // 2 for c in range(len(self.CMDS))
        ]
        return res

    # update reads the device and scales the result according to the calibration
    def update(self):
        raw_x, raw_y, z1, z2 = self.do_cmds()
        if z1 < self.touch_margin or z2 > 4192-self.touch_margin:
            return None
        # print("X:", raw_x, "Y:", raw_y, "Z1:", z1, "Z2:", z2)
        if self.transpose:
            raw_y, raw_x = (raw_x, raw_y)
        if self.cal_x0 is None:
            return raw_x, raw_y
        # scale X to screen dimensions
        x = (raw_x - self.cal_x0) * self.screen_width // (self.cal_x1 - self.cal_x0)
        if x < 0:
            x = 0
        elif x >= self.screen_width:
            x = self.screen_width - 1
        # scale Y to screen dimensions
        y = (raw_y - self.cal_y0) * self.screen_height // (self.cal_y1 - self.cal_y0)
        if y < 0:
            y = 0
        elif y >= self.screen_height:
            y = self.screen_height - 1
        return x, y

    start_time_ptr = esp.C_Pointer()
    end_time_ptr = esp.C_Pointer()
    cycles_in_ms = esp.esp_clk_cpu_freq() // 1000

    def read(self, indev_drv, data) -> int:
        esp.get_ccount(self.start_time_ptr)
        coords = self.update()
        esp.get_ccount(self.end_time_ptr)

        if self.end_time_ptr.int_val > self.start_time_ptr.int_val:
            self.touch_cycles += self.end_time_ptr.int_val - self.start_time_ptr.int_val
            self.touch_count += 1

        # print("touch", coords)
        if coords:
            data.point.x, data.point.y = coords
            data.state = lv.INDEV_STATE.PR
            return False
        data.state = lv.INDEV_STATE.REL
        return False

    def stat(self):
        return self.touch_cycles / (self.touch_count * self.cycles_in_ms)
