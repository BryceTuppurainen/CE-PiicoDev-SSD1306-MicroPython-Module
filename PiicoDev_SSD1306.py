# Sends data to the Core Electronics PiicoDev OLED SSD1306 Display
# Ported by Peter Johnston at Core Electronics October 2021
# Original Repos:
# 2021-09-12:
# https://github.com/micropython/micropython/blob/master/drivers/display/ssd1306.py
# 2021-08-01:
# https://github.com/fizban99/microbit_ssd1306/blob/master/ssd1306_text.py
# 2021-07-15:
# https://github.com/adafruit/Adafruit_Python_SSD1306/blob/master/Adafruit_SSD1306/SSD1306.py

# 2021 OCT 14 - Initial release
# 2022 JAN 04 - Remove dependency on PIL module.  Improve compatibility with pbm files.


from math import cos, sin, radians
# NOTE: Pylint Error: <- potential optimization: sleep_ms, I2CBase,
# I2CUnifiedMachine, I2CUnifiedMicroBit, I2CUnifiedLinux, i2c,
# SMBus, i2c_msg, sleep, ceil and I2C unused from wildcard import
from PiicoDev_Unified import *

_SET_CONTRAST = 0x81
_SET_ENTIRE_ON = 0xA4
_SET_NORM_INV = 0xA6
_SET_DISP = 0xAE
_SET_MEM_ADDR = 0x20
_SET_COL_ADDR = 0x21
_SET_PAGE_ADDR = 0x22
_SET_DISP_START_LINE = 0x40
_SET_SEG_REMAP = 0xA0
_SET_MUX_RATIO = 0xA8
_SET_IREF_SELECT = 0xAD
_SET_COM_OUT_DIR = 0xC0
_SET_DISP_OFFSET = 0xD3
_SET_COM_PIN_CFG = 0xDA
_SET_DISP_CLK_DIV = 0xD5
_SET_PRECHARGE = 0xD9
_SET_VCOM_DESEL = 0xDB
_SET_CHARGE_PUMP = 0x8D

BUFFER_START_ADDR = int.from_bytes(b'\x80', 'big')
DEFAULT_ADDR = 0x3C
WIDTH = 128
HEIGHT = 64
# The display is of size 128x64 pixels, this is 'divided' into a collection of 16x8 '8x8 pages'
PAGE_EDGE = 8

FILE_EXCEPTION_WARNING = '\nThere was an unexpected error reading the file\nPlease confirm that the file exists, is valid, and is readable\n'

_SYSNAME = os.uname().sysname

if _SYSNAME == 'microbit':
    from microbit import *
    from utime import sleep_ms
    from ustruct import pack_into
elif _SYSNAME == 'Linux':
    from struct import pack_into
else:
    import framebuf as Framebuf

if _SYSNAME == 'microbit' or _SYSNAME == 'Linux':
    class Framebuf:
        """
        TODO: Framebuf should have a docstring specified here
        """

        class FrameBuffer():

            def __init__(self):
                # ! Note for reviewer: This call to the superclass isn't 'needed'
                # ! however, it is typically best practice as this way independant of version or
                # ! environment (as well as for static analysis) it is clear where and when the
                # ! class is initialised with the properties of super
                super().__init__()
                self.char_cols = {}

            def _set_pos(self, col=0, page=0):
                """
                Framebuffer manipulation, used by Microbit and Linux
                """
                self.write_cmd(0xb0 | page)  # page number
                # take upper and lower value of col * 2
                c1, c2 = col * 2 & 0x0F, col >> 3
                self.write_cmd(0x00 | c1)  # lower start column address
                self.write_cmd(0x10 | c2)  # upper start column address

            def fill(self, c=0):
                if c == 0:
                    for i in range(len(self.buffer)):
                        self.buffer[i] = 0x00
                else:
                    for i in range(len(self.buffer)):
                        self.buffer[i] = 0xFF

            def pixel(self, x, y, c=1):
                x = x & (WIDTH - 1)
                y = y & (HEIGHT - 1)
                page, offset = divmod(y, PAGE_EDGE)
                i = x + page * WIDTH
                if c:
                    b = self.buffer[i] | (1 << offset)
                else:
                    b = self.buffer[i] & ~ (1 << offset)
                pack_into(">B", self.buffer, i, b)
                self._set_pos(x, page)

            def line(self, x1, y1, x2, y2, c=1):
                # author: Bresenham
                steep = abs(y2-y1) > abs(x2-x1)

                if steep:
                    # Swap x/y
                    tmp = x1
                    x1 = y1
                    y1 = tmp

                    tmp = y2
                    y2 = x2
                    x2 = tmp

                if x1 > x2:
                    # Swap start/end
                    tmp = x1
                    x1 = x2
                    x2 = tmp
                    tmp = y1
                    y1 = y2
                    y2 = tmp

                dx = x2 - x1
                dy = abs(y2-y1)

                err = dx/2

                if(y1 < y2):
                    ystep = 1
                else:
                    ystep = -1

                while x1 <= x2:
                    if steep:
                        self.pixel(y1, x1, c)
                    else:
                        self.pixel(x1, y1, c)
                    err -= dy
                    if err < 0:
                        y1 += ystep
                        err += dx
                    x1 += 1

            # From left-most point (x,y) draw a line right of length l
            def hline(self, x, y, l, c=1):
                self.line(x, y, x + l, y, c)

            # From top-most point (x,y) draw a line down of height h
            def vline(self, x, y, h, c=1):
                self.line(x, y, x, y + h, c)

            # Draw the perimeter of a rectangle to the display
            def rect(self, x, y, w, h, c=1):
                self.hline(x, y, w, c)
                self.hline(x, y+h, w, c)
                self.vline(x, y, h, c)
                self.vline(x+w, y, h, c)

            # Draw a solid rectangle to the display
            def fill_rect(self, x, y, w, h, c=1):
                for i in range(y, y + h):
                    self.hline(x, i, w, c)

            def text(self, text, x, y, c=1):
                self.get_char_cols()
                text_columns = []
                for char in text:
                    text_columns.append(
                        self.char_cols[char])
                for cx, column in enumerate(text_columns):
                    for cy in range(PAGE_EDGE):
                        if x + cx < WIDTH and y + cy < HEIGHT and column & 1 << cy:
                            self.pixel(x + cx, y + cy, c)

            # Stores the bytearray representations for each legal character mapped in the object char_cols
            def get_char_cols(self):
                if (self.char_cols == {}):
                    try:
                        with open("font-pet-me-128.dat", "rb") as font_file:
                            font = bytearray(font_file.read())
                            for char in range(32, 127):
                                self.char_cols[chr(char)] = []
                                # Store the byte representation for each column
                                for col in range(8):
                                    self.char_cols[chr(char)].append(
                                        font[(char-32)*8 + col])
                    except FileNotFoundError:
                        print(FILE_EXCEPTION_WARNING)


class PiicoDev_SSD1306(Framebuf.FrameBuffer):

    def __init__(self):
        super().__init__()
        # NOTE: self.width and self.height are largely unused in this implementation <- potential optimization improvement
        self.width = WIDTH
        self.height = HEIGHT
        self.pages = HEIGHT // PAGE_EDGE
        self.buffer = bytearray(self.pages * WIDTH)
        self.comms_err = False

    def init_display(self):
        """
        Initialize display by calling commands sequentially
        """
        for cmd in (
            # display off
            _SET_DISP,
            # address setting
            _SET_MEM_ADDR,
            # horizontal
            0x00,
            # resolution and layout
            # start at line 0
            _SET_DISP_START_LINE,
            # column addr 127 mapped to SEG0
            _SET_SEG_REMAP | 0x01,
            _SET_MUX_RATIO,
            HEIGHT - 1,
            # scan from COM[N] to COM0
            _SET_COM_OUT_DIR | 0x08,
            _SET_DISP_OFFSET,
            0x00,
            _SET_COM_PIN_CFG,
            0x12,
            # timing and driving scheme
            _SET_DISP_CLK_DIV,
            0x80,
            _SET_PRECHARGE,
            0xF1,
            # set 0.83*Vcc
            _SET_VCOM_DESEL,
            0x30,
            # set display constrast to maximum
            _SET_CONTRAST,
            0xFF,
            # output follows RAM contents
            _SET_ENTIRE_ON,
            # display is not inverted
            _SET_NORM_INV,
            # enable internal IREF during display
            _SET_IREF_SELECT,
            0x30,
            # charge pump
            _SET_CHARGE_PUMP,
            0x14,
            # display on
            _SET_DISP | 0x01,
        ):
            self.write_cmd(cmd)

    def poweroff(self):
        """
        Call poweroff command _SET_DISP
        """
        self.write_cmd(_SET_DISP)

    def poweron(self):
        """
        Call poweron command _SET_DISP | 0x01
        """
        self.write_cmd(_SET_DISP | 0x01)

    def set_contrast(self, contrast):
        """
        Set a new constrast value
        """
        self.write_cmd(_SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        """
        Call invert (pixel on-off values) on display '_SET_NORM_INV | (invert & 1)'
        """
        self.write_cmd(_SET_NORM_INV | (invert & 1))

    # ! Note for reviewer, is this Radians, or Degrees? This can be specified in the docstring
    def rotate(self, rotate):
        """
        Remap pixels rotating (in degrees) about the center of the display
        """
        self.write_cmd(_SET_COM_OUT_DIR | ((rotate & 1) << 3))
        self.write_cmd(_SET_SEG_REMAP | (rotate & 1))

    def show(self):
        """
        Execute commands to show the content (in self.buffer) on the display
        """
        for cmd in (
            _SET_COL_ADDR,
            0,
            WIDTH - 1,
            _SET_PAGE_ADDR,
            0,
            self.pages - 1,
            self.buffer
        ):
            self.write_cmd(cmd)

    def write_cmd(self, cmd):
        """
        Write formal param cmd to i2c bus on self.addr
        """
        try:
            self.i2c.writeto_mem(self.addr, BUFFER_START_ADDR, bytes([cmd]))
            self.comms_err = False
        except:  # TODO: Confirm which exception is thrown by i2c.writeto_mem and call except ExceptionType: specifically, calling try with hanging except is bad practice
            print(i2c_err_str.format(self.addr))
            self.comms_err = True

    # ! Mark for reviewer - What is the difference between write_data and write_cmd?
    # ! The value of write_list[0] is never redefined and is equal to BUFFER_START_ADDR,
    # ! other than buf being defined as an element of an array this is identical to write_cmd?
    def write_data(self, buf):
        try:
            self.write_list[1] = buf
            self.i2c.writeto_mem(self.addr, int.from_bytes(
                self.write_list[0], 'big'), self.write_list[1])
            self.comms_err = False
        except:  # TODO: Confirm which exception is thrown by i2c.writeto_mem and call except ExceptionType: specifically, calling try with hanging except is bad practice
            print(i2c_err_str.format(self.addr))
            self.comms_err = True

    # TODO: name t is unclear, given the comparison another 10 lines down I suspect this is a border or fill of some description?
    def circ(self, x, y, r, t=1, c=1):
        """
        Draw a circle with center (x, y) with radius r
        """
        for i in range(x-r, x+r+1):
            for j in range(y-r, y+r+1):
                if t:
                    if((i-x)**2 + (j-y)**2 < r**2):
                        self.pixel(i, j, 1)
                else:
                    if((i-x)**2 + (j-y)**2 < r**2) and ((i-x)**2 + (j-y)**2 >= (r-r*t-1)**2):
                        self.pixel(i, j, c)

    # TODO: This should have a docstring and the naming conventions aren't clear
    def arc(self, x, y, r, st_ang, en_ang, t=0, c=1):
        for i in range(r*(1-t)-1, r):
            for ta in range(st_ang, en_ang, 1):
                X = int(i*cos(radians(ta)) + x)
                Y = int(i*sin(radians(ta)) + y)
                self.pixel(X, Y, c)

    # ! TODO: This docstring can be more useful, personally
    # ! I find the naming conventions are also a little unclear,
    # ! but that is a personal preference
    def load_pbm(self, filename, c):
        """
        Load a specified PBM file into the display buffer
        """
        try:
            with open(filename, 'rb') as file:
                line = file.readline()
                if line.startswith(b'P4') is False:
                    print('Not a valid pbm P4 file')
                    return
                line = file.readline()
                while line.startswith(b'#') is True:
                    line = file.readline()
                data_piicodev = bytearray(file.read())
            for byte in range(WIDTH // PAGE_EDGE * HEIGHT):
                for bit in range(PAGE_EDGE):
                    if data_piicodev[byte] & 1 << bit != 0:
                        x_coord = ((8-bit) + (byte * 8)) % WIDTH
                        y_coord = byte * 8 // WIDTH
                        # ! NOTE for reviwer: An idea would be to wrapper this condition into pixel,
                        # ! I've noticed this is called a few times in this implementation and shouldn't
                        # ! be much slower if it is in pixel
                        if x_coord < WIDTH and y_coord < HEIGHT:
                            self.pixel(x_coord, y_coord, c)
        except FileNotFoundError:
            print(FILE_EXCEPTION_WARNING)

    class Graph2D:
        """
        TODO: Should have a docstring for the class here to define its usage
        """

        def __init__(self, origin_x=0, origin_y=HEIGHT-1, width=WIDTH, height=HEIGHT, min_value=0, max_value=255, c=1, bars=False):
            # ! NOTE for reviewer: we've got 11 instance variables here (and nine formal params in init) for a single public method.
            # ! This can be refactored into an object or similar to reduce code complexity
            self.min_value = min_value
            self.max_value = max_value
            self.origin_x = origin_x
            self.origin_y = origin_y
            self.width = width
            self.height = height
            self.c = c
            self.m = (1-height)/(max_value-min_value)
            self.offset = origin_y-self.m*min_value
            self.bars = bars
            self.data = []

    # ! NOTE for reviwer: renamed from updateGraph2D to meet snake case
    # ! and because the class itself is defined as a 2D graph this method
    # ! can simply be named update
    def update(self, graph, value):
        """
        TODO: Include a docstring for this method (my suggestion is to define what the formal parameter graph is for, seems like the formal parameters are being manipulated as a shallow copy here)
        """
        graph.data.insert(0, value)
        if len(graph.data) > graph.width:
            graph.data.pop()
        x = graph.origin_x+graph.width-1
        m = graph.c
        # ! NOTE for reviewer: bad practice using the same name for a variable as the formal parameter value
        for value in graph.data:
            y = round(graph.m*value + graph.offset)
            if graph.bars:
                for idx in range(y, graph.origin_y+1):
                    if graph.origin_x <= x < graph.origin_x+graph.width and graph.origin_y-graph.height < idx <= graph.origin_y:
                        self.pixel(x, idx, m)
            else:
                if graph.origin_x <= x < graph.origin_x+graph.width and graph.origin_y-graph.height < y <= graph.origin_y:
                    self.pixel(x, y, m)
            x -= 1


class PiicoDev_SSD1306_MicroPython(PiicoDev_SSD1306):
    def __init__(self, bus=None, freq=None, sda=None, scl=None, addr=DEFAULT_ADDR):
        self.i2c = create_unified_i2c(bus=bus, freq=freq, sda=sda, scl=scl)
        self.addr = addr
        self.temp = bytearray(2)
        self.write_list = [b'\x40', None]  # Co=0, D/C#=1
        self.init_display()
        super().__init__(self.buffer, WIDTH, HEIGHT, Framebuf.MONO_VLSB)
        self.fill(0)
        self.show()


class PiicoDev_SSD1306_MicroBit(PiicoDev_SSD1306):
    def __init__(self, bus=None, freq=None, sda=None, scl=None, addr=DEFAULT_ADDR):
        self.i2c = create_unified_i2c(bus=bus, freq=freq, sda=sda, scl=scl)
        self.addr = addr
        self.temp = bytearray(2)
        self.write_list = [b'\x40', None]  # Co=0, D/C#=1
        self.init_display()
        self.fill(0)
        self.show()


class PiicoDev_SSD1306_Linux(PiicoDev_SSD1306):
    def __init__(self, bus=None, freq=None, sda=None, scl=None, addr=DEFAULT_ADDR):
        self.i2c = create_unified_i2c(bus=bus, freq=freq, sda=sda, scl=scl)
        self.addr = addr
        self.temp = bytearray(2)
        self.write_list = [b'\x40', None]  # Co=0, D/C#=1
        self.init_display()
        self.fill(0)
        self.show()


def create_PiicoDev_SSD1306(addr=None, bus=None, freq=None, sda=None, scl=None, addr_switch=None):
    """
    Factory-Pattern to create a PiicoDev_SSD1306 object of
    the appropriate type based on the system type
    """
    addr = determine_addr(addr, addr_switch)
    check_compatibility()

    if _SYSNAME == 'microbit':
        return PiicoDev_SSD1306_MicroBit(addr=addr, freq=freq)

    if _SYSNAME == 'linux':
        return PiicoDev_SSD1306_Linux(addr=addr, freq=freq)

    return PiicoDev_SSD1306_MicroPython(addr=addr, bus=bus, freq=freq, sda=sda, scl=scl)


def determine_addr(addr, addr_switch):
    """
    Return the default address to use based on
    function parameters unless it is overridden by addr
    (or the address switch is declared high)
    """
    if addr is None:
        if addr_switch == 1 | addr_switch == True:
            return DEFAULT_ADDR + 1
        return DEFAULT_ADDR
    return addr


def check_compatibility():  # See PiicoDev_Unified.py
    """
    Prints an error message if the compatible index is incorrect in Unified
    """
    if compat_ind < 1:
        print('\nUnified PiicoDev library out of date. Get the latest module: https://piico.dev/unified\n')
