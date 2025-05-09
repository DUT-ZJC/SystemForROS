
import os
import pathlib

STL_MODEL_DIRECTORY = pathlib.Path(os.path.expanduser('~/easy_handeye2/install/registration/share/registration/model'))

# CALIBRATION_NAMESPACE = ''
CALIBRATION_NAMESPACE = '/easy_handeye2/calibration/'

LIST_ALGORITHMS_TOPIC = CALIBRATION_NAMESPACE + 'list_algorithms'