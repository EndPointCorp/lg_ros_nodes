#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import threading
import signal
import sys
import rospy
from std_msgs.msg import String


class ButtonApp(threading.Thread):
    def onclick(*args, **kwargs):
        p = rospy.Publisher('/touchscreen/toggle', String, queue_size=10)
        rospy.sleep(0.1)
        p.publish(String())

    def signal_handler(self, *args, **kwargs):
        print("Caught signal")
        #self.root.quit()
        sys.exit()
        self.root.destroy()
        print("destroyed")
        sys.exit()
        print("exited")

    def run(self):
        self.root = tk.Tk()
        self.root.geometry("200x50")
        button = ttk.Button(self.root, text="Toggle touchscreen", command=self.onclick)
        button.pack(fill=tk.BOTH, expand=1)
        self.root.mainloop()

def main():
    print("in main")
    rospy.init_node('real_button')
    app = ButtonApp()
    signal.signal(signal.SIGINT, app.signal_handler)
    app.start()

if __name__ == '__main__':
    main()
#root.attributes('-topmost', True)
#def background():
#    while not rospy.is_shutdown():
#        rospy.logerr("BBBB")
#        root.attributes('-topmost', True)
#        root.lift()
#        root.update()
#        rospy.sleep(1)
#t = threading.Thread(name='thread', target=background)
#t.start()
