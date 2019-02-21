#!/usr/bin/env python
import gi
import rospy
from gi.repository import Gtk
from std_msgs.msg import String
gi.require_version('Gtk', '3.0')
import signal
import subprocess
import time


class MyWindow(Gtk.Window):

    def __init__(self):
        Gtk.Window.__init__(self, title="Simple grasp GUI")
        width = 400
        height = width / 3.0
        Gtk.Window.set_default_size(self, width, height)
        Gtk.Window.move(self, Gtk.Window.get_screen(self).get_width() / 2.0 - width /
                        2.0, Gtk.Window.get_screen(self).get_height() / 2.0 - height)
        Gtk.Window.set_keep_above(self, True)

        # signal.signal(signal.SIGINT, signal.SIG_DFL)
        # signal.signal(signal.SIGTERM, self.handle_sigterm)

        # ros stuff
        rospy.init_node('gui_talker', anonymous=True)
        self.pub_string = rospy.Publisher('/nl_command_parsed', String, queue_size=10)

        self.hbox1 = Gtk.HBox(spacing=6)

        self.vbox = Gtk.VBox()
        self.add(self.vbox)
        self.vbox.add(self.hbox1)

        commands_normal_grasps = ['tsOfs123h', 'tsOfs123l',
                                  'pOfs123', 'tsOp', 'tsOs12', 'my_full_grasp_no_index']

        commands = ["open", 'full tip', 'tripod', '3f', '2f', 'ulnar pinch']
        commands.append("3_per_finger")
        commands.append("no_contact")

        for command in commands:
            self.add_button_grasp(self.hbox1, command)

        self.hbox2 = Gtk.HBox(spacing=6)
        self.vbox.add(self.hbox2)

        for command in commands_normal_grasps:
            self.add_button_grasp(self.hbox2, command, min_height=50)

        # self.connect("delete-event", Gtk.main_quit)

        signal.signal(signal.SIGINT, signal.SIG_DFL)  # This works !!
        self.show_all()

    def add_button_grasp(self, hbox, grasp_string, min_width=100, min_height=100):
        button = Gtk.Button.new_with_label(grasp_string)
        button.set_size_request(min_width, min_height)
        fun = lambda x: self.send(grasp_string)
        button.connect("clicked", fun)
        hbox.pack_start(button, True, True, 0)

    def send(self, grasp_string):
        rospy.loginfo(grasp_string)
        self.pub_string.publish(grasp_string)

    def main(self):
        Gtk.main()


if __name__ == '__main__':

    try:
        win = MyWindow()
        win.main()

    except KeyboardInterrupt:
        print('shutdown keyboard')

    # except rospy.ROSInterruptException:
    #     print('ros inter')
    #     pass
