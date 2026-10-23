import struct
from crc import Calculator, Crc8

class DataframeBase:
    def __init__(self, struct_def, endian_fmt='<'):
        self.var_name = []
        self.var_type = []
        self.var_data = []
        for t, name, value in struct_def:
            self.var_name.append(name)
            self.var_type.append(t)
            self.var_data.append(value)
        self.endian_fmt = endian_fmt

    def get_var(self, var_name):
        return self.var_data[self.var_name.index(var_name)]

    def set_var(self, var_name, value):
        self.var_data[self.var_name.index(var_name)] = value

    def get_struct_fmt(self):
        # https://docs.python.org/3/library/struct.html
        fmt = [self.endian_fmt]
        for t in self.var_type:
            if t == 'uint8_t':
                fmt.append('B')
            elif t == 'int16_t':
                fmt.append('h')
            elif t == 'int32_t':
                fmt.append('l')
            elif t == 'int64_t':
                fmt.append('q')
            elif t == 'float32':
                # For Atmega328p float data
                # little-endian 4-bytes IEEE-754 format
                # (Answered by Google AI)
                fmt.append('f')
            else:
                raise ValueError('Unknown data type: {}'.format(t))
        return ''.join(fmt)

    def to_bytes(self):
        args = [self.get_struct_fmt()]
        args.extend(self.var_data)
        return struct.pack(*args)

    def from_bytes(self, raw_data):
        data = struct.unpack(self.get_struct_fmt(), raw_data)
        for i in range(len(data)):
            self.var_data[i] = data[i]

    def get_size(self):
        return struct.calcsize(self.get_struct_fmt())


###
# Utilitiy functions for DataframeBase class:
###

def compute_checksum(bytes):
    calculator = Calculator(Crc8.CCITT)
    return calculator.checksum(bytes)

def verify_checksum(frame: DataframeBase):
    return compute_checksum(frame.to_bytes()[:-1]) == frame.get_var('checksum')

def update_checksum(frame: DataframeBase):
    frame.set_var('checksum', compute_checksum(frame.to_bytes()[:-1]))
