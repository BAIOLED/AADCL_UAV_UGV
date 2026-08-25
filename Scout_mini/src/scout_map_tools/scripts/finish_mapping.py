#!/usr/bin/env python3
"""Save the filtered cloud and finalize all map artifacts with one service."""

import subprocess
import threading

import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class MappingFinisher:
    def __init__(self):
        self.map_name = rospy.get_param("~map_name")
        self.save_service_name = rospy.get_param(
            "~mapper_save_service",
            "/scout_pointcloud_mapper/save_map",
        )
        self._lock = threading.Lock()
        self._service = rospy.Service(
            "/finish_mapping",
            Trigger,
            self.finish,
        )
        rospy.loginfo(
            "Mapping finisher ready: map_name=%s, service=/finish_mapping",
            self.map_name,
        )

    def finish(self, _request):
        if not self._lock.acquire(False):
            return TriggerResponse(False, "Map finalization is already running")

        try:
            rospy.loginfo("Waiting for filtered-map save service")
            rospy.wait_for_service(self.save_service_name, timeout=5.0)
            save_map = rospy.ServiceProxy(self.save_service_name, Trigger)
            save_result = save_map(TriggerRequest())
            if not save_result.success:
                return TriggerResponse(
                    False,
                    "Filtered PCD save failed: " + save_result.message,
                )

            rospy.loginfo("Filtered PCD saved; generating public PCD and maps")
            command = [
                "rosrun",
                "scout_map_tools",
                "finalize_map.py",
                self.map_name,
                "--replace-raw",
            ]
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            output = result.stdout.strip()
            if result.returncode != 0:
                rospy.logerr("Map finalization failed:\n%s", output)
                return TriggerResponse(
                    False,
                    "PCD was saved, but finalization failed; inspect node log",
                )

            rospy.loginfo("Map finalization completed:\n%s", output)
            return TriggerResponse(
                True,
                "Map '{}' saved and finalized successfully".format(
                    self.map_name
                ),
            )
        except (rospy.ROSException, rospy.ServiceException, OSError) as error:
            rospy.logerr("Finish mapping failed: %s", error)
            return TriggerResponse(False, str(error))
        finally:
            self._lock.release()


if __name__ == "__main__":
    rospy.init_node("scout_mapping_finisher")
    MappingFinisher()
    rospy.spin()
