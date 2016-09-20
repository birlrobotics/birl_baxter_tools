#!/usr/bin/env python

PACKAGE = "point_cloud_selector"

import roslib; roslib.load_manifest(PACKAGE)
import roslib.packages

import rospy
import sys

from point_cloud_selector.srv import *
from std_msgs.msg import *
from visualization_msgs.msg import *

from PySide.QtCore import *
from PySide.QtGui import *
from PySide.QtUiTools import QUiLoader

class CylinderSelectorUINode(object):
    def __init__(self, namespace="cylinder_selector"):
        self.app = QApplication(sys.argv)
        loader = QUiLoader()
        file = QFile(roslib.packages.get_pkg_dir(PACKAGE) + "/ui/cylinder_selector.ui")
        file.open(QFile.ReadOnly)
        self.window = loader.load(file)
        file.close()
        self.window.setWindowTitle("Cylinder Selector")
        self.window.show()

        # start up the ros part
        rospy.init_node("cylinder_selector_ui")

        self.namespace = namespace

        service_topics = [self.namespace + '{0}'.format(i) for i in ["/cylinder_properties"]]

        # wait for all the services to become available
        map(rospy.wait_for_service, service_topics)

        # create service clients for all the services
        self.set_cylinder_properties = rospy.ServiceProxy(service_topics[0], SetCylinderProperties)

        # set up the button slots
        self.window.height_spinner.valueChanged.connect(self.apply_changes)
        self.window.radius_spinner.valueChanged.connect(self.apply_changes)
        self.window.transparency_slider.valueChanged.connect(self.apply_changes)
        self.window.color_button.clicked.connect(self.select_color)

        self.color = ColorRGBA(r=0,g=255,b=0,a=.5)

        # now wait until a marker is published so that gui can be initialized
        self.marker = None

        init_sub = rospy.Subscriber("point_cloud_selector/cylinder_selector", Marker, self.initialize_callback)

        while self.marker is None and not rospy.is_shutdown():
            pass
        init_sub.unregister()

        self.window.height_spinner.setValue(self.marker.scale.z)
        self.window.radius_spinner.setValue(self.marker.scale.x)
        self.window.transparency_slider.setValue(int(self.marker.color.a*100))
        self.window.color_button.setStyleSheet("background-color: rgb(%i,%i,%i)" % \
                                               (int(self.marker.color.r*255),
                                                int(self.marker.color.g*255),
                                                int(self.marker.color.b*255)))

    def apply_changes(self, *args, **kwargs):
        height = self.window.height_spinner.value()
        radius = self.window.radius_spinner.value()
        self.color.a = float(self.window.transparency_slider.value())/100.0

        self.set_cylinder_properties.call(height=height, radius=radius, color=self.color)

    def select_color(self):
        # color dialog stuff
        color = QColorDialog.getColor()
        self.color.r = color.red()/255.0
        self.color.g = color.green()/255.0
        self.color.b = color.blue()/255.0

        self.window.color_button.setStyleSheet(u'background-color: rgb(%i,%i,%i)' % \
                                               (color.red(), color.green(), color.blue()))

        # now apply the changes
        self.apply_changes()

    def initialize_callback(self, data):
        self.marker = data

if __name__ == "__main__":
    ros_node = CylinderSelectorUINode()

    sys.exit(ros_node.app.exec_())
