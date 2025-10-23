#!/usr/bin/env python3
import os, sys, csv, time, rospy
import tf2_ros, tf2_geometry_msgs  # noqa
from geometry_msgs.msg import PoseStamped

"""
Logga la posa di probe_tip rispetto a un frame target (default: world) in CSV.
Parametri:
  _source_frame := probe_tip
  _target_frame := world
  _rate_hz      := 50
  _outfile      := path CSV (default: ~/iiwa_stack_ws/bags/probe_tip_<timestamp>.csv)
  _publish_pose := true/false (se vero pubblica anche /probe_tip_pose)
"""

def main():
    rospy.init_node("log_probe_tip_csv")
    source_frame = rospy.get_param("~source_frame", "probe_tip")
    target_frame = rospy.get_param("~target_frame", "world")
    rate_hz      = float(rospy.get_param("~rate_hz", 50))
    publish_pose = bool(rospy.get_param("~publish_pose", True))
    outfile      = rospy.get_param("~outfile", "")

    if not outfile:
        ts = time.strftime("%Y%m%d_%H%M%S")
        outfile = os.path.expanduser(f"~/iiwa_stack_ws/bags/probe_tip_{ts}.csv")
    else:
        outfile = os.path.expanduser(outfile)
        os.makedirs(os.path.dirname(outfile), exist_ok=True)

    buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    lis = tf2_ros.TransformListener(buf)  # noqa

    pub = None
    if publish_pose:
        pub = rospy.Publisher("probe_tip_pose", PoseStamped, queue_size=10)

    f = open(outfile, "w", newline="")
    w = csv.writer(f)
    w.writerow(["sec", "nsec", "x", "y", "z", "qx", "qy", "qz", "qw"])

    r = rospy.Rate(rate_hz)
    rospy.loginfo("Logging %s -> %s in %s", source_frame, target_frame, outfile)
    while not rospy.is_shutdown():
        try:
            tr = buf.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.1))
            t = tr.header.stamp
            trn = tr.transform.translation
            rot = tr.transform.rotation
            w.writerow([t.secs, t.nsecs, trn.x, trn.y, trn.z, rot.x, rot.y, rot.z, rot.w])
            if pub is not None:
                ps = PoseStamped()
                ps.header.stamp = t
                ps.header.frame_id = target_frame
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = trn.x, trn.y, trn.z
                ps.pose.orientation = rot
                pub.publish(ps)
        except Exception:
            pass
        r.sleep()

    f.close()
    rospy.loginfo("CSV saved to %s", outfile)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
