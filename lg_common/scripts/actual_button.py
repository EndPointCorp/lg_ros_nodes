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
        self.root.quit()
        sys.exit()

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
    for i in [x for x in dir(signal) if x.startswith("SIG")]:
        try:
            signum = getattr(signal,i)
            signal.signal(signum, app.signal_handler)
        except (OSError, RuntimeError, ValueError) as m: #OSError for Python3, RuntimeError for 2
            print ("Skipping {}".format(i))
    app.start()

if __name__ == '__main__':
    main()
