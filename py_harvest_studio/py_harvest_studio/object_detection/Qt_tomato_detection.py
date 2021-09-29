#!/usr/bin/python3
# -*- coding: utf-8 -*-


import sys
from PyQt5.QtWidgets import (QWidget, QSlider, 
    QLabel, QApplication)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap


class Example(QWidget):

    def __init__(self):
        super().__init__()

        self.initUI()


    def initUI(self):      

        # 水平方向のスライダー作成
        sld = QSlider(Qt.Horizontal, self)
        # スライダーがフォーカスされないようにする
        sld.setFocusPolicy(Qt.NoFocus)
        sld.setGeometry(30, 40, 100, 30)
        # スライダーが動くとchangeValue関数が呼び出される
        sld.valueChanged[int].connect(self.changeValue)

        # ラベル作成
        self.label = QLabel(self)
        # ラベルに初期画像設定
        self.label.setPixmap(QPixmap('golira.jpg'))
        self.label.setGeometry(160, 40, 80, 30)

        self.setGeometry(300, 300, 280, 170)
        self.setWindowTitle('QSlider')
        self.show()


    def changeValue(self, value):

        # スライダーの位置によって画像を変化させる
        self.label.setText(str(value))


if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())