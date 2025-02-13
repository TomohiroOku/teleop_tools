#!/usr/bin/env python

import math
import signal
import threading
import tkinter
from tkinter import Event

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

X, Y, W = 0, 1, 2


class MouseTeleopApp:
    tags = ["v_x", "v_y", "v_w"]
    format_xy = "linear velocity: ({:.2f}, {:.2f}) m/s"
    format_w = "angular velocity: {:.2f} deg/s"

    def __init__(self, scale: list[float], holonomic: bool) -> None:
        self.scale = scale
        self.frequency = 50
        self.x = self.y = self.c_x = self.c_y = 0.0
        self.v = [0.0, 0.0, 0.0]

        self.root = tkinter.Tk()
        self.root.title("Mouse Teleop")
        self.root.resizable(0, 0)

        self.canvas = tkinter.Canvas(self.root, bg="white")
        self.canvas.create_line(0, 0, 0, 0, fill="blue", width=4, tag=self.tags[X])
        self.canvas.create_line(0, 0, 0, 0, fill="blue", width=4, tag=self.tags[Y])
        self.canvas.create_arc(
            0,
            0,
            0,
            0,
            fill="red",
            outline="red",
            width=1,
            style=tkinter.PIESLICE,
            start=90.0,
            tag=self.tags[W],
        )

        self.text_xy = tkinter.StringVar()
        tkinter.Label(self.root, anchor=tkinter.W, textvariable=self.text_xy).pack()

        self.text_w = tkinter.StringVar()
        tkinter.Label(self.root, anchor=tkinter.W, textvariable=self.text_w).pack()

        self.canvas.bind("<Configure>", self.configure)
        self.canvas.bind("<Button>", self.start)
        self.canvas.bind("<ButtonRelease-1>", self.release)
        self.canvas.bind("<B1-Motion>", self.mouse_motion_angular)
        self.canvas.bind("<Button-4>", self.mouse_motion_wheelup)
        self.canvas.bind("<Button-5>", self.mouse_motion_wheeldown)
        if holonomic:
            self.canvas.bind("<ButtonRelease-2>", self.release)
            self.canvas.bind("<ButtonRelease-3>", self.release)
            self.canvas.bind("<B2-Motion>", self.mouse_motion_linear)
            self.canvas.bind("<B3-Motion>", self.mouse_motion_turn)
        self.canvas.pack()

        self.root.bind("<Control-c>", lambda _: self.root.quit())
        signal.signal(signal.SIGINT, lambda *_: self.root.quit())
        self.root.after(self.frequency, self.check)

    def check(self) -> None:
        self.root.after(self.frequency, self.check)

    def configure(self, e: Event) -> None:
        self.c_x, self.c_y = e.width / 2.0, e.height / 2.0
        self.text_xy.set(self.format_xy.format(self.v[X], self.v[Y]))
        self.text_w.set(self.format_w.format(self.v[W]))
        self.draw_w()

    def start(self, e: Event) -> None:
        self.x, self.y = e.x, e.y

    def release(self, _: Event) -> None:
        self.v = [0.0, 0.0, 0.0]
        self.draw_xy()
        self.draw_w()
        self.update_label()

    def mouse_motion_angular(self, e: Event) -> None:
        self.v[W], self.v[X] = self.relative_motion(e.x, e.y)
        self.draw_xy()
        self.draw_w()
        self.update_label()

    def mouse_motion_linear(self, e: Event) -> None:
        self.v[Y], self.v[X] = self.relative_motion(e.x, e.y)
        self.draw_xy()
        self.update_label()

    def mouse_motion_turn(self, e: Event) -> None:
        self.v[W], _ = self.relative_motion(e.x, e.y)
        self.draw_w()
        self.update_label()

    def mouse_motion_wheelup(self, _: Event) -> None:
        self.v[X] = min(self.v[X] + 0.1, 1.0)
        self.draw_xy()
        self.update_label()

    def mouse_motion_wheeldown(self, _: Event) -> None:
        self.v[X] = max(self.v[X] - 0.1, -1.0)
        self.draw_xy()
        self.update_label()

    def relative_motion(self, x: float, y: float) -> tuple[float, float]:
        dx = (self.x - x) / self.c_x
        dy = (self.y - y) / self.c_y

        dx = max(-1.0, min(dx, 1.0))
        dy = max(-1.0, min(dy, 1.0))

        return dx, dy

    def update_coords(
        self, tag: str, x0: float, y0: float, x1: float, y1: float
    ) -> None:
        x0 += self.c_x
        y0 += self.c_y
        x1 += self.c_x
        y1 += self.c_y
        self.canvas.coords(tag, (x0, y0, x1, y1))

    def draw_xy(self) -> None:
        x = -self.v[X] * self.c_y
        y = -self.v[Y] * self.c_x
        self.update_coords(self.tags[X], 0, 0, 0, x)
        self.update_coords(self.tags[Y], 0, 0, y, 0)

    def draw_w(self) -> None:
        r = min(self.c_x, self.c_y) * 0.5
        self.update_coords(self.tags[W], -r, -r, r, r)
        w = self.v[W] * math.degrees(self.scale[W])
        self.canvas.itemconfig(self.tags[W], extent=w)

    def update_label(self) -> None:
        x, y, w = self.get_scaled_twist()
        self.text_xy.set(self.format_xy.format(x, y))
        self.text_w.set(self.format_w.format(w))

    def get_scaled_twist(self) -> None:
        return [v * s for v, s in zip(self.v, self.scale)]

    def __exit__(self, *_) -> None:
        self.root.quit()

    def __enter__(self) -> tkinter.Tk:
        return self.root


class MouseTeleop(Node):
    def __init__(self) -> None:
        super().__init__("mouse_teleop")

        x = self.declare_parameter("scale.x", 0.5).get_parameter_value().double_value
        y = self.declare_parameter("scale.y", x).get_parameter_value().double_value
        w = self.declare_parameter("scale.yaw", 0.5).get_parameter_value().double_value
        publish_rate = (
            self.declare_parameter("publish_rate", 1.0)
            .get_parameter_value()
            .double_value
        )
        if publish_rate > 1000.0:
            self.get_logger().warn(
                "publish rate is 1000.0 Hz at maximum. setting 1000.0 Hz"
            )
            publish_rate = 1000.0
        elif publish_rate < 0.001:
            self.get_logger().warn(
                "publish rate is 0.001 Hz at minimum. setting 0.001 Hz"
            )
            publish_rate = 0.001
        holonomic = (
            self.declare_parameter("allow_holonomic", False)
            .get_parameter_value()
            .bool_value
        )
        use_stamped_twist = (
            self.declare_parameter("twist_stamped", False)
            .get_parameter_value()
            .bool_value
        )

        self.app = MouseTeleopApp((x, y, w), holonomic)
        if use_stamped_twist:
            msg_type, callback = TwistStamped, self.publish_stamped
        else:
            msg_type, callback = Twist, self.publish
        self.pub = self.create_publisher(
            msg_type, "~/cmd_vel", qos_profile_system_default
        )
        self.timer = self.create_timer(1.0 / publish_rate, callback)

    def publish(self) -> None:
        msg = Twist()
        x, y, yaw = self.app.get_scaled_twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = yaw
        self.pub.publish(msg)

    def publish_stamped(self) -> None:
        twist = Twist()
        x, y, yaw = self.app.get_scaled_twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = yaw
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "mouse_teleop"
        msg.twist = twist
        self.pub.publish(msg)

    def run(self) -> None:
        with self.app as a:
            a.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MouseTeleop()

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
