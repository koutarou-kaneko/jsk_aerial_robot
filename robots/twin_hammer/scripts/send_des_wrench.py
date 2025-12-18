#!/usr/bin/env python3
import rospy
import yaml
import time
from geometry_msgs.msg import WrenchStamped
from pathlib import Path


class HapticsWrenchPublisher:
    def __init__(self):
        config_path = Path(
            "/home/kaneko/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/twin_hammer/scripts/send_des_wrench_params.yaml"
        )

        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)

        self.active = self.config["active_components"]
        self.increment = self.config["increment"]
        self.min_val = self.config["min"]
        self.max_val = self.config["max"]

        self.update_interval = self.config["update_interval"]
        self.publish_rate = self.config.get("publish_rate", 100)
        self.frame_id = self.config.get("frame_id", "base_link")

        # 現在の目標レンチ
        self.current_wrench = {
            "fx": 0.0,
            "fy": 0.0,
            "fz": 0.0,
            "tx": 0.0,
            "ty": 0.0,
            "tz": 0.0,
        }

        # 初期値は min
        for key in self.current_wrench.keys():
            self.current_wrench[key] = self.min_val[key]

        # フェーズ
        # ramp -> hold_max (1 update step) -> zero (1 update step) -> done
        self.phase = "ramp"

        self.last_update_time = time.time()

        self.pub = rospy.Publisher(
            "/twin_hammer/haptics_wrench",
            WrenchStamped,
            queue_size=1
        )

    def update_if_needed(self):
        now = time.time()
        if now - self.last_update_time < self.update_interval:
            return False

        self.last_update_time = now
        return True

    def update_target_wrench(self):
        if self.phase != "ramp":
            return

        reached_max = True

        for key in self.current_wrench.keys():
            if not self.active.get(key, False):
                continue

            self.current_wrench[key] += self.increment.get(key, 0.0)

            self.current_wrench[key] = max(
                self.min_val[key],
                min(self.current_wrench[key], self.max_val[key])
            )

            if self.current_wrench[key] < self.max_val[key]:
                reached_max = False

        if reached_max:
            self.phase = "hold_max"

    def set_zero_wrench(self):
        for key in self.current_wrench.keys():
            self.current_wrench[key] = 0.0

    def publish(self):
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        msg.wrench.force.x = self.current_wrench["fx"]
        msg.wrench.force.y = self.current_wrench["fy"]
        msg.wrench.force.z = self.current_wrench["fz"]
        msg.wrench.torque.x = self.current_wrench["tx"]
        msg.wrench.torque.y = self.current_wrench["ty"]
        msg.wrench.torque.z = self.current_wrench["tz"]

        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            update_tick = self.update_if_needed()

            if self.phase == "ramp":
                if update_tick:
                    self.update_target_wrench()

            elif self.phase == "hold_max":
                # max を 1 update_interval 分保持
                if update_tick:
                    self.phase = "zero"

            elif self.phase == "zero":
                # 0 を 1 update_interval 分保持
                if update_tick:
                    self.set_zero_wrench()
                    self.phase = "done"

            elif self.phase == "done":
                rospy.loginfo("Haptics wrench sequence finished.")
                rospy.signal_shutdown("Finished wrench publishing sequence")

            self.publish()
            rate.sleep()


def main():
    rospy.init_node("send_des_wrench")

    node = HapticsWrenchPublisher()
    node.run()


if __name__ == "__main__":
    main()
