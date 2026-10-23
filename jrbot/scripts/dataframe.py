from dataframe_base import DataframeBase

FRAMEID_ROS_COMMAND     = 0xA5
FRAMEID_ROS_STATUS      = 0xA6
FRAMEID_ROS_ODOM        = 0xA7

class ROSCommandFrame(DataframeBase):
    def __init__(self):
        super().__init__(
            struct_def=[
                ['uint8_t', 'header',           FRAMEID_ROS_COMMAND],
                ['int16_t', 'sequence_id',      0],
                ['uint8_t', 'toggle',           0],
                ['float32', 'cmd_x',            0.0],
                ['float32', 'cmd_yaw',          0.0],
                ['uint8_t', 'checksum',         0x00]
            ],
            endian_fmt='<'
        )

class ROSStatusFrame(DataframeBase):
    def __init__(self):
        super().__init__(
            struct_def=[
                ['uint8_t', 'header',           FRAMEID_ROS_STATUS],
                ['int16_t', 'sequence_id',      0],
                ['uint8_t', 'indicator',        0],
                # ['float32', 'cmd_x_reply',      0.0],
                # ['float32', 'cmd_yaw_reply',    0.0],
                ['float32', 'x',                0.0],
                ['float32', 'y',                0.0],
                ['float32', 'yaw',              0.0],
                ['float32', 'v',                0.0],
                ['float32', 'vyaw',             0.0],

                ['float32', 'v_left',           0.0],
                ['float32', 'v_right',          0.0],
                ['float32', 'left_err',         0.0],
                ['float32', 'right_err',        0.0],

                ['uint8_t', 'left_pwm',         0],
                ['uint8_t', 'right_pwm',        0],
                ['uint8_t', 'debug_code',       0],
                ['int16_t', 'battery_adc_value',0],
                ['uint8_t', 'checksum',         0x00]
            ],
            endian_fmt='<'
        )

# class ROSOdomFrame(DataframeBase):
    # def __init__(self):
        # super().__init__(
            # struct_def=[
                # ['uint8_t', 'header',           FRAMEID_ROS_ODOM],
                # ['int16_t', 'sequence_id',      0],
                # ['float32', 'x',                0.0],
                # ['float32', 'y',                0.0],
                # ['float32', 'yaw',              0.0],
                # ['float32', 'v',                0.0],
                # ['float32', 'vyaw',             0.0],
                # ['uint8_t', 'checksum',         0x00]
            # ],
            # endian_fmt='<'
        # )
