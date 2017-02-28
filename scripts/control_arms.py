import sys
from PyQt4 import QtGui, QtCore
from PyQt4.Qt import QMessageBox
from PyQt4.QtGui import QWidget

class Example(QtGui.QWidget):
    
    def __init__(self):
        super(Example, self).__init__()
        
        self.initUI()
        
    def initUI(self):      
        
        slider_geometry = [130, 40, 1000, 30]
        label_geometry = [40, 40, 80, 30]
        i = 0
        for joint_name in ['joint1', 'joint2' , 'joint3' , 'joint4']:
            sld = QtGui.QSlider(QtCore.Qt.Horizontal, self)
            sld.setFocusPolicy(QtCore.Qt.NoFocus)
            sld.setGeometry(130 , 40 + i * 50, 1000, 30)
            sld.valueChanged[int].connect(eval('self.changeValue'+i.__str__()))
            
            self.label = QtGui.QLabel(joint_name,self)
            self.label.setGeometry(40, 40 + i * 50, 80, 30)
            
            i+=1
            
        self.setGeometry(300, 300, 1200, 370)
        self.setWindowTitle('ECM')
        self.show()
        
    def changeValue0(self, value):
        QMessageBox.about(QWidget(), '1', '1')
    def changeValue1(self, value):
        QMessageBox.about(QWidget(), '2', '2')
    def changeValue2(self, value):
        pass
    def changeValue3(self, value):
        pass
        
def main():
    
    app = QtGui.QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()    