import sys
import cv2
from PIL import Image
from PIL.ImageQt import ImageQt
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QSlider, 
    QLabel, QApplication)
from PyQt5.QtCore import *
from PyQt5.QtGui import QPixmap, QImage, QPalette
from PyQt5.QtWidgets import *
from tomato_depth_realtime12speed import Ui_Dialog

class Test(QDialog):
    def __init__(self,parent=None):
        super(Test, self).__init__(parent)
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
 
 
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Test()
    window.setStyleSheet("background-color: rgb(173, 216, 230);")
    window.show()
    sys.exit(app.exec_())
