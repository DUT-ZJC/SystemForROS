import os
import pathlib

SAMPLES_DIRECTORY = pathlib.Path(os.path.expanduser('~/.ros2/easy_handeye2/samples'))
CALIBRATIONS_DIRECTORY = pathlib.Path(os.path.expanduser('~/.ros2/easy_handeye2/calibrations'))

# CALIBRATION_NAMESPACE = ''
EXECUTE_NAMESPACE = ''

TOSTART_TOPIC = '/move_to_start'
KEEPMOVE_TOPIC = '/keep_move'
STOPMOVE_TOPIC = '/move_stop'
AUTOMOVE_TOPIC = '/auto_move'
STOPBACK_TOPIC = '/stop_and_back'

