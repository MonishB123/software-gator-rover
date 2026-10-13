#!/usr/bin/env python3
"""Mission runner.
 
Loads trees/mission_tree.xml and executes it, printing each step in the
terminal. Nodes supported (subset of BehaviorTree.CPP v4 XML):
Sequence, Fallback, RetryUntilSuccessful, Repeat, Inverter, SubTree, Action.
 
Modes
  --mock      Every action is simulated (no ROS needed). Dig/Dump are always
              the 5 s mocks from mock_actions.py.
  (default)   Localize / DriveTo / ClearCostmaps / BackUp use Nav2 through
              nav2_simple_commander; Dig / Dump stay mocked.
 
Examples
  python3 mission_node.py --mock
  python3 mission_node.py --mock --fail-drive 3     # force drive failures to exercise recovery
  python3 mission_node.py --mock --step             # press Enter before every action
  ros2 run rover_autonomy mission_node              # real Nav2
"""
import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from enum import Enum
from pathlib import Path
 
try:
    from rover_autonomy.mock_actions import MockDig, MockDump
except ImportError:  # running the file directly
    from mock_actions import MockDig, MockDump
 
# ---- Mission waypoints (map frame): x [m], y [m], yaw [rad]. EDIT FOR THE FIELD.
WAYPOINTS = {
    "dig_zone": (4.0, 1.0, 0.0),
    "berm": (1.0, 5.0, math.pi / 2),
}
 
 
class Status(Enum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
 
 
# --------------------------------------------------------------------------
# Action backends: each is callable(params: dict) -> bool
# --------------------------------------------------------------------------
class MockNavBackend:
    """Fake localize / nav so the loop runs without ROS."""
 
    def __init__(self, fail_drive=0, time_scale=1.0):
        self.fail_drive = fail_drive
        self.scale = time_scale
 
    def _wait(self, s):
        time.sleep(s * self.scale)
 
    def localize(self, p):
        self._wait(1.0)
        return True
 
    def drive_to(self, p):
        target = p["target"]
        print(f"    driving to '{target}' {WAYPOINTS[target]}")
        self._wait(2.0)
        if self.fail_drive > 0:
            self.fail_drive -= 1
            print("    (mock) path planning failed")
            return False
        return True
 
    def clear_costmaps(self, p):
        print("    (mock) costmaps cleared")
        return True
 
    def back_up(self, p):
        print(f"    (mock) backed up {p.get('distance', '0.5')} m")
        self._wait(1.0)
        return True
 
    def shutdown(self):
        pass
 
 
class Nav2Backend:
    """Real Nav2 via nav2_simple_commander. Requires a running Nav2 stack."""
 
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
        from tf2_ros import Buffer, TransformListener
 
        self._rclpy = rclpy
        self._PoseStamped = PoseStamped
        self._TaskResult = TaskResult
        rclpy.init()
        self.nav = BasicNavigator()
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self.nav)
        print("Waiting for Nav2 to become active...")
        self.nav.waitUntilNav2Active()
 
    def localize(self, p, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._rclpy.spin_once(self.nav, timeout_sec=0.2)
            if self.tf_buffer.can_transform("map", "base_link", self._rclpy.time.Time()):
                return True
        print("    no map->base_link transform")
        return False
 
    def drive_to(self, p):
        x, y, yaw = WAYPOINTS[p["target"]]
        pose = self._PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        self.nav.goToPose(pose)
        while not self.nav.isTaskComplete():
            time.sleep(0.2)
        return self.nav.getResult() == self._TaskResult.SUCCEEDED
 
    def clear_costmaps(self, p):
        self.nav.clearAllCostmaps()
        return True
 
    def back_up(self, p):
        self.nav.backup(backup_dist=float(p.get("distance", 0.5)), backup_speed=0.1)
        while not self.nav.isTaskComplete():
            time.sleep(0.2)
        return self.nav.getResult() == self._TaskResult.SUCCEEDED
 
    def shutdown(self):
        try:
            self.nav.cancelTask()
        finally:
            self.nav.destroy_node()
            self._rclpy.shutdown()
 
 
# --------------------------------------------------------------------------
# Tiny behavior-tree interpreter
# --------------------------------------------------------------------------
class BTRunner:
    def __init__(self, root, actions, step=False):
        self.root = root
        self.actions = actions
        self.step = step
        self.trees = {bt.get("ID"): bt for bt in root.findall("BehaviorTree")}
 
    def validate(self):
        for node in self.root.iter("Action"):
            if node.get("ID") not in self.actions:
                raise ValueError(f"No implementation registered for Action '{node.get('ID')}'")
        for node in self.root.iter("SubTree"):
            if node.get("ID") not in self.trees:
                raise ValueError(f"SubTree '{node.get('ID')}' not defined")
 
    def run(self):
        self.validate()
        main = self.root.get("main_tree_to_execute")
        print(f"=== Mission start (tree: {main}) ===")
        status = self._tick_tree(main, {}, 0)
        print(f"=== Mission {status.value} ===")
        return status
 
    def _tick_tree(self, tree_id, bb, depth):
        bt = self.trees[tree_id]
        return self._tick(list(bt)[0], bb, depth)
 
    @staticmethod
    def _resolve(attrs, bb):
        out = {}
        for k, v in attrs.items():
            if v.startswith("{") and v.endswith("}"):
                out[k] = bb.get(v[1:-1])
            else:
                out[k] = v
        return out
 
    def _tick(self, node, bb, depth):
        tag, pad = node.tag, "  " * depth
        label = node.get("name") or node.get("ID") or ""
 
        if tag == "Action":
            params = self._resolve({k: v for k, v in node.attrib.items() if k != "ID"}, bb)
            name = node.get("ID")
            print(f"{pad}> Action {name} {params if params else ''}")
            if self.step:
                input(f"{pad}  [Enter to run]")
            ok = self.actions[name](params)
            st = Status.SUCCESS if ok else Status.FAILURE
            print(f"{pad}  -> {st.value}")
            return st
 
        if tag == "SubTree":
            sub_bb = self._resolve({k: v for k, v in node.attrib.items() if k != "ID"}, bb)
            print(f"{pad}> SubTree {node.get('ID')} {sub_bb if sub_bb else ''}")
            return self._tick_tree(node.get("ID"), sub_bb, depth + 1)
 
        print(f"{pad}> {tag} {label}")
        children = list(node)
 
        if tag == "Sequence":
            for c in children:
                if self._tick(c, bb, depth + 1) == Status.FAILURE:
                    return Status.FAILURE
            return Status.SUCCESS
 
        if tag == "Fallback":
            for i, c in enumerate(children):
                if self._tick(c, bb, depth + 1) == Status.SUCCESS:
                    return Status.SUCCESS
                if i < len(children) - 1:
                    print(f"{pad}  ! child failed, falling back")
            return Status.FAILURE
 
        if tag == "RetryUntilSuccessful":
            n = int(node.get("num_attempts", 1))
            for attempt in range(1, n + 1):
                if self._tick(children[0], bb, depth + 1) == Status.SUCCESS:
                    return Status.SUCCESS
                print(f"{pad}  ! attempt {attempt}/{n} failed")
            return Status.FAILURE
 
        if tag == "Repeat":
            n = int(node.get("num_cycles", 1))
            for i in range(1, n + 1):
                print(f"{pad}  cycle {i}/{n}")
                if self._tick(children[0], bb, depth + 1) == Status.FAILURE:
                    return Status.FAILURE
            return Status.SUCCESS
 
        if tag == "Inverter":
            st = self._tick(children[0], bb, depth + 1)
            return Status.FAILURE if st == Status.SUCCESS else Status.SUCCESS
 
        raise ValueError(f"Unsupported node type <{tag}>")
 
 
# --------------------------------------------------------------------------
def find_tree_file(cli_path):
    if cli_path:
        return Path(cli_path)
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("rover_autonomy")) / "trees" / "mission_tree.xml"
    except Exception:
        # source-tree fallback: rover_autonomy/rover_autonomy/ -> rover_autonomy/trees/
        return Path(__file__).resolve().parents[1] / "trees" / "mission_tree.xml"
 
 
def main(argv=None):
    ap = argparse.ArgumentParser(description="Rover mission runner")
    ap.add_argument("--tree", help="path to mission_tree.xml")
    ap.add_argument("--mock", action="store_true", help="simulate everything (no ROS)")
    ap.add_argument("--step", action="store_true", help="wait for Enter before each action")
    ap.add_argument("--fail-drive", type=int, default=0,
                    help="(mock) force the first N drive attempts to fail")
    ap.add_argument("--time-scale", type=float, default=1.0,
                    help="(mock) scale nav sleep times and dig/dump duration")
    args = ap.parse_args(argv)
 
    tree_path = find_tree_file(args.tree)
    root = ET.parse(tree_path).getroot()
 
    if args.mock:
        nav = MockNavBackend(args.fail_drive, args.time_scale)
    else:
        nav = Nav2Backend()
 
    dig = MockDig(duration=5.0 * args.time_scale)
    dump = MockDump(duration=5.0 * args.time_scale)
    actions = {
        "Localize": nav.localize,
        "DriveTo": nav.drive_to,
        "ClearCostmaps": nav.clear_costmaps,
        "BackUp": nav.back_up,
        "Dig": dig,
        "Dump": dump,
    }
 
    runner = BTRunner(root, actions, step=args.step)
    try:
        status = runner.run()
    except KeyboardInterrupt:
        print("\nInterrupted, stopping.")
        status = Status.FAILURE
    finally:
        nav.shutdown()
    return 0 if status == Status.SUCCESS else 1
 
 
if __name__ == "__main__":
    sys.exit(main())