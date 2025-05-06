from PyQt5.QtCore import QThread, Qt
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
from pathlib import Path

model_path = Path.home() / 'easy_handeye2' / 'install' / 'path_plan' / 'share' / 'path_plan' / 'model' / 'vitual_model.stl'

class Display(QThread):
    def __init__(self,Qwidget):
        super().__init__()
        self.widget = Qwidget
        