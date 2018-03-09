import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QPushButton
from python_qt_binding.QtCore import Slot

from pitasc.srv import Load, Run
from std_srvs.srv import Empty, Trigger
from std_msgs.msg import Bool

import rospkg
import rospy
import os
import json
import thread
from functools import partial

class CalibratorPlugin(Plugin):

    def __init__(self, context):
        super(CalibratorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('CalibratorPlugin')
        rp = rospkg.RosPack()

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('rqt_pi_teacher'), 'resource', 'pi_teacher.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.b_load.clicked.connect(self.loadfile)
        self._widget.b_stop.clicked.connect(lambda: self.call_empty('stop'))
        self._widget.b_pause.clicked.connect(lambda: self.call_empty('pause'))
        self._widget.b_resume.clicked.connect(lambda: self.call_empty('resume'))

        self.btns = []

        self.sub_is_running = rospy.Subscriber("pitasc_executor_node/is_running",
            Bool, self.is_running)

    def is_running(self, data):
        try:
            self._widget.progressBar.setRange(0, int(not data.data))
        except:
            pass

    @Slot()
    def loadfile(self):
        fname = QFileDialog.getOpenFileName(self._widget, 'Open file', '', "pitasc application files (*.xml)")

        #path = QFileInfo(fname).path() # store path for next time

        #print fname[0].encode("utf-8")
        fname = fname[0].encode("utf-8")
        init_step = 0
        try:
            # Load file
            rospackage = rospkg.get_package_name(fname)
            init_step = 1
            rel_path = os.path.relpath(fname, rospkg.RosPack().get_path(rospackage))
            init_step = 2

            resp = rospy.ServiceProxy('pitasc_executor_node/load', Load)(
                package=rospackage, file_name=rel_path)

            if resp.success:
                self._widget.line_file.setText(rospackage + " | " + rel_path)
                self._widget.line_file.setStyleSheet("color: rgb(0, 153, 0);")
            else:
                self._widget.line_file.setText(resp.message)
                self._widget.line_file.setStyleSheet("color: rgb(204, 0, 0);")

            init_step = 3
            # Get list of all apps
            resp2 = rospy.ServiceProxy('pitasc_executor_node/list', Trigger)()
            apps = json.loads(resp2.message.replace('\'', '\"'))

            init_step = 4
            # Deinitalize buttons
            for btn in self.btns:
                self._widget.btn_layout.removeWidget(btn)
                btn.deleteLater()
            del self.btns[:]

            init_step = 5
            # Initialze buttons
            for app in apps:
                btn = QPushButton('Start: '+app)
                self.btns.append(btn)
                self._widget.btn_layout.addWidget(btn)
                btn.clicked.connect(partial(self.run_app, app))

        except:
            message = 'ERROR'
            if init_step == 1:
                print rospackage
                if rospackage is None:
                    message = "Could not find rospackage of selected file."
                else:
                    message = "CATKIN_IGNORE? Could not initialize path to rospackage. Ensure, that the selected rospackage is in your working directory. Possibly resource roscore terminal."
            elif init_step == 2:
                message = "Calling load-srv failed. Is executor_node running?"
            elif init_step == 3:
                message = "Calling list-srv failed."
            elif init_step == 4:
                message = "Deinitalization of app-buttons failed."
            elif init_step == 5:
                message = "initialization of app-buttons failed."
            else:
                message = "Step {} failed".format(init_step)

            self._widget.line_file.setText(message)
            self._widget.line_file.setStyleSheet("color: rgb(204, 0, 0);")
            return

    @Slot()
    def run_app(self, name):
        print "Starting App: '{}'".format(name)
        thread.start_new_thread(self.srv_call_run, (name, self.sender()))

    def srv_call_run(self, name, sender):
        try:
            for btn in self.btns:
                btn.setEnabled(False)
            sender.setStyleSheet("background: yellow")
            resp = rospy.ServiceProxy('pitasc_executor_node/run', Run)(
                app=name)
        except:
            pass
        finally:
            sender.setStyleSheet("")
            for btn in self.btns:
                btn.setEnabled(True)

    @Slot()
    def call_empty(self, srv_name):
        try:
            resp = rospy.ServiceProxy('pitasc_executor_node/'+srv_name, Empty)()
        except:
            pass

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
