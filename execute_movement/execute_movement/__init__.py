import os
import pathlib

SAMPLES_DIRECTORY = pathlib.Path(os.path.expanduser('~/.ros2/easy_handeye2/samples'))
CALIBRATIONS_DIRECTORY = pathlib.Path(os.path.expanduser('~/.ros2/easy_handeye2/calibrations'))

# CALIBRATION_NAMESPACE = ''
EXECUTE_NAMESPACE = ''

TOSTART_TOPIC = EXECUTE_NAMESPACE + 'move_to_start'
KEEPMOVE_TOPIC = EXECUTE_NAMESPACE + 'keep_move'
STOPMOVE_TOPIC = EXECUTE_NAMESPACE + 'move_stop'
AUTOMOVE_TOPIC = EXECUTE_NAMESPACE + 'auto_move'
STOPBACK_TOPIC = EXECUTE_NAMESPACE + 'stop_and_back'

