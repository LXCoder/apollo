import json
import os
import time
from datetime import datetime
from pathlib import Path
from threading import Thread

import click
from matplotlib import pyplot as plt

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.external_command_msgs.command_status_pb2 import (
    CommandStatus,
    CommandStatusType,
)
from modules.common_msgs.planning_msgs.planning_command_pb2 import PlanningCommand

CHANNEL_CONTROL = "/apollo/control"
CHANNEL_CANBUS_CHASSIS = "/apollo/canbus/chassis"
CHANNEL_PLANNING_COMMAND_STATUS = "/apollo/planning/command_status"


class FollowModelRecorder:
    def __init__(self, save_folder, record_interval: int = 500) -> None:
        """
        record_interval: 记录间隔，单位 ms
        """
        self._node = cyber.Node("FollowModelNode")
        self._record_interval = record_interval
        self._save_folder = save_folder
        self._is_start_record = False
        self._is_running = False
        self._record_data: dict = {}
        self._control_data = ControlCommand()
        self._chassis_data = Chassis()
        self._planning_command_data = PlanningCommand()
        self._planning_command_status_data = CommandStatus()
        self.control_reader = self._node.create_reader(
            CHANNEL_CONTROL,
            ControlCommand,
            self._control_reader_callback,
        )
        self.chassis_reader = self._node.create_reader(
            CHANNEL_CANBUS_CHASSIS,
            Chassis,
            self._cahssis_reader_callback,
        )
        self._planning_command_status_reader = self._node.create_reader(
            CHANNEL_PLANNING_COMMAND_STATUS,
            CommandStatus,
            self._planning_command_status_reader_callback,
        )
        if not os.path.exists(save_folder):
            path = Path(save_folder)
            path.mkdir(parents=True, exist_ok=True)

    def run(self):
        self._is_running = True
        t = Thread(target=self._start_spin, name="record_data", daemon=True)
        t.start()
        self._record()

    def _start_spin(self):
        self._node.spin()

    def _control_reader_callback(self, data):
        self._control_data.CopyFrom(data)

    def _cahssis_reader_callback(self, data):
        self._chassis_data.CopyFrom(data)

    def _planning_command_reader_callback(self, data):
        self._planning_command_data.CopyFrom(data)

    def _planning_command_status_reader_callback(self, data):
        self._planning_command_data.CopyFrom(data)

    def _planning_command_status_reader_callback(self, data):
        self._planning_command_status_data.CopyFrom(data)
        status = self._planning_command_status_data.status
        if status == CommandStatusType.RUNNING:
            if not self._is_start_record:
                self._is_start_record = True
                print("start record")
        else:
            if self._is_start_record:
                self._is_start_record = False
                print("stop record")

    def _record(self):
        print("enter record data")
        try:
            while self._is_running:
                while self._is_running and self._is_start_record:
                    if "v" not in self._record_data:
                        self._record_data.setdefault("v", [])
                    if "a" not in self._record_data:
                        self._record_data.setdefault("a", [])

                    v_point = [
                        self._chassis_data.header.timestamp_sec,
                        self._chassis_data.speed_mps,
                    ]
                    a_point = [
                        self._control_data.header.timestamp_sec,
                        self._control_data.acceleration,
                    ]

                    self._record_data["v"].append(v_point)
                    self._record_data["a"].append(a_point)

                    time.sleep(self._record_interval / 1000)

                if self._record_data.keys():
                    # 绘制图片
                    self.draw()
                    # 清空数据
                    self._record_data.clear()
        except InterruptedError:
            if self._is_start_record and self._record_data.keys():
                # 绘制图片
                self.draw()
                # 清空数据
                self._record_data.clear()
                self._is_start_record = False

    def draw(self):
        v = []
        v_t = []
        a = []
        a_t = []
        v_start_time = self._record_data["v"][0][0]
        a_start_time = self._record_data["a"][0][0]

        for item in self._record_data["v"]:
            v_t.append(item[0] - v_start_time)
            v.append(item[1])

        for item in self._record_data["a"]:
            a_t.append(item[0] - a_start_time)
            a.append(item[1])

        plt.figure(figsize=(10, 6))
        plt.plot(v_t, v, label="v (m/s)", color="blue", linewidth=2)
        plt.plot(a_t, a, label="a (m/s²)", color="red", linestyle="--", linewidth=2)

        # 添加标题和坐标轴标签
        plt.title("v & a - t", fontsize=16)
        plt.xlabel("t (s)", fontsize=14)
        plt.ylabel("value", fontsize=14)

        # 添加网格和图例
        plt.grid(True, linestyle="--", alpha=0.5)
        plt.legend(fontsize=12)
        filename = datetime.now().strftime("%Y%m%d%H%M%S")
        dest_path = os.path.join(self._save_folder, f"{filename}.png")

        plt.savefig(dest_path, dpi=300, bbox_inches="tight")

        data_dict = {"v": {"x": v_t, "y": v}, "a": {"x": a_t, "y": a}}
        with open(os.path.join(self._save_folder, f"{filename}.json"), "w") as f:
            f.write(json.dumps(data_dict))
        print(f"save data images: {dest_path}")


def draw_plt(apollo_data_path, follow_model_data_path, output_path):
    apollo_data = None
    follow_model_data = None
    with open(apollo_data_path, "r") as f:
        apollo_data = json.loads(f.read())

    with open(follow_model_data_path, "r") as f:
        follow_model_data = json.loads(f.read())

    plt.figure(figsize=(10, 6))

    plt.plot(
        apollo_data["v"]["x"],
        apollo_data["v"]["y"],
        label="v (m/s) -- apollo",
        color="blue",
        linewidth=2,
    )
    plt.plot(
        apollo_data["a"]["x"],
        apollo_data["a"]["y"],
        label="a (m/s²) -- apollo",
        color="blue",
        linestyle="--",
        linewidth=2,
    )
    plt.plot(
        follow_model_data["v"]["x"],
        follow_model_data["v"]["y"],
        label="v (m/s) -- follow model",
        color="red",
        linewidth=2,
    )
    plt.plot(
        follow_model_data["a"]["x"],
        follow_model_data["a"]["y"],
        label="a (m/s²) -- follow model",
        color="red",
        linestyle="--",
        linewidth=2,
    )

    # 添加标题和坐标轴标签
    plt.title("v & a - t", fontsize=16)
    plt.xlabel("t (s)", fontsize=14)
    plt.ylabel("value", fontsize=14)

    # 添加网格和图例
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend(fontsize=12)
    filename = datetime.now().strftime("%Y%m%d%H%M%S")
    dest_path = os.path.join(output_path, f"{filename}.png")

    plt.savefig(dest_path, dpi=300, bbox_inches="tight")
    print(f"save data images: {dest_path}")


@click.group()
def cli():
    """A CLI tool for recording and drawing"""
    pass


@cli.command()
@click.option("--path", default="/apollo/data/imgs", help="data saving path")
@click.option(
    "-i", "--inrerval", default=500, type=int, help="time interval for data collection"
)
def record(path, inrerval):
    """Activate recording function"""
    cyber.init()

    recorder = FollowModelRecorder(path, inrerval)
    try:
        recorder.run()
    finally:
        cyber.shutdown()
        recorder._is_running = False


@cli.command()
@click.option("--apollo", help="apollo data file path")
@click.option("--compare", help="comparing file paths")
@click.option("-o", "--output", help="catalog of output charts")
def draw(apollo, compare, output):
    """Draw velocity and acceleration maps based on data files"""
    draw_plt(apollo, compare, output)


if __name__ == "__main__":
    cli()
