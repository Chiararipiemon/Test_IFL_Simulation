#!/usr/bin/env python3
import rospy, numpy as np, math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def make_arrow(p, n, frame_id, mid, scale=0.02, color=(0.1,0.8,1.0,0.9)):
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = "cloud_normals"
    m.id = mid
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.scale.x = scale * 0.6  # shaft length
    m.scale.y = scale * 0.2  # shaft diameter
    m.scale.z = scale * 0.2  # head diameter
    m.color.r, m.color.g, m.color.b, m.color.a = color
    s = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
    e = Point(x=float(p[0]+n[0]*scale), y=float(p[1]+n[1]*scale), z=float(p[2]+n[2]*scale))
    m.points = [s, e]
    return m

class NormalsMarkers:
    def __init__(self):
        self.topic_in   = rospy.get_param("~topic_in", "/cloud_with_normals")
        self.scale      = float(rospy.get_param("~scale", 0.02))     # lunghezza frecce [m]
        self.step       = int(rospy.get_param("~step", 50))          # 1 freccia ogni N punti
        self.max_markers= int(rospy.get_param("~max_markers", 500))  # limite superiore
        self.topic_out  = rospy.get_param("~topic_out", "/cloud_normals_markers")

        self.pub = rospy.Publisher(self.topic_out, MarkerArray, queue_size=1, latch=True)
        self.sub = rospy.Subscriber(self.topic_in, PointCloud2, self.cb, queue_size=1)
        rospy.loginfo("Waiting for cloud on %s ...", self.topic_in)

    def cb(self, msg: PointCloud2):
        # usa il frame della cloud per evitare mismatch
        fid = msg.header.frame_id or "world"

        # leggi i campi con ordine ESPERTO (niente set)
        fields = ['x','y','z','normal_x','normal_y','normal_z']
        has_needed = all(any(f.name==req for f in msg.fields) for req in fields)
        if not has_needed:
            rospy.logerr("Cloud does not have required fields: %s", fields)
            return

        pts, nrms = [], []
        for rec in pc2.read_points(msg, field_names=fields, skip_nans=True):
            x,y,z,nx,ny,nz = rec
            pts.append([x,y,z])
            nrms.append([nx,ny,nz])

        if not pts:
            rospy.logwarn("Empty cloud received.")
            return

        P = np.asarray(pts, float)
        N = np.asarray(nrms, float)
        # normalizza le normali
        nn = np.linalg.norm(N, axis=1, keepdims=True); nn[nn==0]=1.0
        N = N / nn

        # decimazione controllata
        step = max(1, self.step)
        if self.max_markers > 0:
            step = max(step, int(math.ceil(len(P) / float(self.max_markers))))

        arr = MarkerArray()
        mid = 0
        for i in range(0, len(P), step):
            arr.markers.append(make_arrow(P[i], N[i], fid, mid, scale=self.scale))
            mid += 1

        self.pub.publish(arr)
        rospy.loginfo("Published %d normal arrows on %s (frame=%s, step=%d, scale=%.3f)",
                      len(arr.markers), self.topic_out, fid, step, self.scale)

def main():
    rospy.init_node("normals_markers_from_cloud")
    NormalsMarkers()
    rospy.spin()

if __name__ == "__main__":
    main()
