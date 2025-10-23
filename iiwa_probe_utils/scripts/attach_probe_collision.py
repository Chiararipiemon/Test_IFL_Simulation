#!/usr/bin/env python3
import rospy, sys
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface, roscpp_initialize

"""
Attacca al link 'tool0' un box che approssima la sonda (asse lungo Z).
Parametri ROS (_underscore):
  _link=tool0
  _length=0.12     # m
  _radius=0.02     # m -> box X,Y = 2*radius
  _name=probe_body
  _frame_id=tool0  # pose riferita al link
  _z_offset=0.0    # m (opzionale)
"""

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("attach_probe_collision")

    link      = rospy.get_param("~link", "tool0")
    length    = float(rospy.get_param("~length", 0.12))
    radius    = float(rospy.get_param("~radius", 0.02))
    name      = rospy.get_param("~name", "probe_body")
    frame_id  = rospy.get_param("~frame_id", link)
    z_off     = float(rospy.get_param("~z_offset", 0.0))

    psi = PlanningSceneInterface(synchronous=True)
    rospy.sleep(1.0)

    size_x = 2.0*radius
    size_y = 2.0*radius
    size_z = length

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = (length/2.0) + z_off
    pose.pose.orientation.w = 1.0

    # rimuovi eventuali oggetti omonimi
    try: psi.remove_attached_object(link, name=name)
    except Exception: pass
    try: psi.remove_world_object(name)
    except Exception: pass
    rospy.sleep(0.3)

    # aggiungi e ATTACCA
    psi.add_box(name, pose, size=(size_x, size_y, size_z))
    rospy.sleep(0.3)
    psi.attach_box(link, name, pose=pose, size=(size_x, size_y, size_z))
    rospy.loginfo("Attached '%s' to '%s' (%.3f x %.3f x %.3f m)",
                  name, link, size_x, size_y, size_z)
    rospy.spin()

if __name__ == "__main__":
    main()
