#!/usr/bin/env python3
# Codice per loggare le info che ImFusion vuole 
import os
import csv
import math
import rospy
import numpy as np
import tf2_ros
from std_srvs.srv import Trigger, TriggerResponse
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix

class CsvLogger:
    def __init__(self):
        # Parametri vari
        self.fixed_frame = rospy.get_param("~fixed_frame", "world")
        self.ee_frame    = rospy.get_param("~ee_frame", "iiwa_link_ee")  
        self.rate_hz     = float(rospy.get_param("~rate_hz", 20.0))
        self.units       = rospy.get_param("~units", "mm")               # "mm" 
        self.tip_offset_m = float(rospy.get_param("~tip_offset_m", 0.0)) 
        self.output_dir  = os.path.expanduser(rospy.get_param("~output_dir", "~/iiwa_csv"))
        self.prefix      = rospy.get_param("~prefix", "us_poses")

        os.makedirs(self.output_dir, exist_ok=True)

        # TF
        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)

        # Stato
        self.is_recording = False
        self.csv_file = None
        self.writer = None

        # Services
        self.srv_start = rospy.Service("~start", Trigger, self._start_cb)
        self.srv_stop  = rospy.Service("~stop",  Trigger, self._stop_cb)

        rospy.loginfo("[csv_logger] Pronto. Call ~start per iniziare a registrare, ~stop per fermare.")

    def _start_cb(self, _req):
        if self.is_recording:
            return TriggerResponse(success=False, message="Già in recording fermati")
        ts = rospy.get_time()
        fname = f"{self.prefix}_{rospy.Time.now().to_sec():.0f}.csv"
        path = os.path.join(self.output_dir, fname)
        try:
            self.csv_file = open(path, "w", newline="")
            self.writer = csv.writer(self.csv_file)
            # intestazione identica al tuo file
            self.writer.writerow(["x","y","z","qx","qy","qz","qw"])
            self.csv_file.flush()
            self.is_recording = True
            rospy.loginfo(f"[csv_logger] Recording ON -> {path}")
            return TriggerResponse(success=True, message=f"Recording in {path}")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Errore apertura file: {e}")

    def _stop_cb(self, _req):
        if not self.is_recording:
            return TriggerResponse(success=False, message="Non in recording.")
        try:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self.writer = None
            self.is_recording = False
            rospy.loginfo("[csv_logger] Recording OFF.")
            return TriggerResponse(success=True, message="Recording fermato")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Errore chiusura file: {e}")

    def _lookup(self):
        try:
            tf: TransformStamped = self.tf_buf.lookup_transform(
                self.fixed_frame, self.ee_frame, rospy.Time(0), rospy.Duration(0.2)
            )
            # posizione in metri
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            tz = tf.transform.translation.z
            qx = tf.transform.rotation.x
            qy = tf.transform.rotation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w

            # Offset opzionale della punta lungo l'asse -Z del tool
            if self.tip_offset_m != 0.0:
                R = quaternion_matrix([qx,qy,qz,qw])[:3,:3]
                offset_tool = np.array([0.0, 0.0, -self.tip_offset_m])  # -Z
                offset_world = R.dot(offset_tool)
                tx += float(offset_world[0])
                ty += float(offset_world[1])
                tz += float(offset_world[2])

            # Conversione unità, in dubbio su questa cosa ma in teroia va bene
            if self.units.lower() == "mm":
                sx = 1000.0; sy = 1000.0; sz = 1000.0
            else:
                sx = sy = sz = 1.0

            return (tx*sx, ty*sy, tz*sz, qx, qy, qz, qw)

        except TransformException as e:
            rospy.logwarn_throttle(1.0, f"[csv_logger] TF non disponibile: {e}")
            return None

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.is_recording and self.writer is not None:
                data = self._lookup()
                if data is not None:
                    self.writer.writerow([f"{data[0]:.6f}", f"{data[1]:.6f}", f"{data[2]:.6f}",
                                          f"{data[3]:.6f}", f"{data[4]:.6f}", f"{data[5]:.6f}", f"{data[6]:.6f}"])
                    # flush leggero per sicurezza
                    self.csv_file.flush()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("csv_logger", anonymous=False)
    CsvLogger().spin()
