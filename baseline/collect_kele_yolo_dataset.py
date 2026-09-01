#!/usr/bin/env python3
"""从无3DGS固定场景采集RGB并用场景几何生成训练标签。

标签只用于训练/诊断；最终运行时YOLO不读取布局或GT。
"""
from __future__ import annotations
import argparse, json, time
from pathlib import Path
import cv2, numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from scipy.spatial.transform import Rotation
from discoverse.robots.mmk2.mmk2_fk import MMK2FK

RGB_TOPIC='/head_camera/color/image_raw'
INFO_TOPIC='/head_camera/color/camera_info'
ODOM_TOPIC='/slamware_ros_sdk_server_node/odom'
JOINT_TOPIC='/joint_states'
LAYOUT='/workspace/supermarket_sorting_task/examples/supermarket_sorting/retail_competition_layout.json'

class Collector(Node):
    def __init__(self, out: Path, frames: int, interval: float):
        super().__init__('collect_kele_yolo_dataset')
        self.out=out; self.frames=frames; self.interval=interval
        (out/'images').mkdir(parents=True, exist_ok=True); (out/'labels').mkdir(parents=True, exist_ok=True)
        self.bridge=CvBridge(); self.K=None; self.rgb=None; self.base=None; self.joints={}; self.last_save=0.; self.saved=0
        self.fk=MMK2FK(); self.layout=json.loads(Path(LAYOUT).read_text())
        self.create_subscription(CameraInfo,INFO_TOPIC,self.on_info,10)
        self.create_subscription(Image,RGB_TOPIC,self.on_rgb,qos_profile_sensor_data)
        self.create_subscription(Odometry,ODOM_TOPIC,self.on_odom,qos_profile_sensor_data)
        self.create_subscription(JointState,JOINT_TOPIC,self.on_joints,qos_profile_sensor_data)
        self.create_timer(0.05,self.tick)
    def on_info(self,m): self.K=np.array(m.k,dtype=float).reshape(3,3)
    def on_rgb(self,m):
        try: self.rgb=self.bridge.imgmsg_to_cv2(m,'bgr8')
        except Exception: self.rgb=None
    def on_odom(self,m):
        p=m.pose.pose.position; q=m.pose.pose.orientation
        self.base=([float(p.x),float(p.y),float(p.z)],[float(q.w),float(q.x),float(q.y),float(q.z)])
    def on_joints(self,m): self.joints={n:float(m.position[i]) for i,n in enumerate(m.name) if i<len(m.position)}
    def camera_t(self):
        if self.base is None or self.K is None: return None
        self.fk.set_base_pose(self.base[0],self.base[1])
        self.fk.set_slide_joint(self.joints.get('slide_joint',0.0))
        self.fk.set_head_joints([self.joints.get('head_yaw_joint',0.0),self.joints.get('head_pitch_joint',0.0)])
        self.fk.set_left_arm_joints([0.0]*6); self.fk.set_right_arm_joints([0.0]*6)
        pos,quat=self.fk.get_head_camera_pose()
        T=np.eye(4); T[:3,3]=pos; T[:3,:3]=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix()
        return T
    def tick(self):
        if self.saved>=self.frames or self.rgb is None: return
        now=time.monotonic()
        if now-self.last_save<self.interval: return
        T=self.camera_t()
        if T is None: return
        h,w=self.rgb.shape[:2]; inv=np.linalg.inv(T); fx,fy=self.K[0,0],self.K[1,1]; cx,cy=self.K[0,2],self.K[1,2]
        labels=[]; overlay=self.rgb.copy()
        for s in self.layout:
            if s.get('object_kind')!='kele': continue
            pw=np.array(list(s['world_position'])+[1.0]); pc=inv@pw
            if pc[2]<=0.05: continue
            u=fx*pc[0]/pc[2]+cx; v=fy*pc[1]/pc[2]+cy
            if not(0<=u<w and 0<=v<h): continue
            # Approximate bbox for small shelf products; training augmentation handles scale jitter.
            bw,bh=22.0,42.0
            x0=max(0,int(u-bw/2)); y0=max(0,int(v-bh/2)); x1=min(w-1,int(u+bw/2)); y1=min(h-1,int(v+bh/2))
            if x1-x0<4 or y1-y0<4: continue
            labels.append(f"0 {((x0+x1)/2)/w:.6f} {((y0+y1)/2)/h:.6f} {(x1-x0)/w:.6f} {(y1-y0)/h:.6f}")
            cv2.rectangle(overlay,(x0,y0),(x1,y1),(0,255,0),1)
        idx=self.saved; ip=self.out/'images'/f'frame_{idx:04d}.png'; lp=self.out/'labels'/f'frame_{idx:04d}.txt'; dp=self.out/'debug'/f'frame_{idx:04d}.png'; dp.parent.mkdir(exist_ok=True)
        cv2.imwrite(str(ip),self.rgb); lp.write_text('\n'.join(labels)+'\n',encoding='utf-8'); cv2.imwrite(str(dp),overlay)
        self.saved+=1; self.last_save=now; self.get_logger().info(f'saved={self.saved}/{self.frames} labels={len(labels)} pose={self.base[0][:2]}')
        if self.saved>=self.frames: rclpy.shutdown()

def main():
    p=argparse.ArgumentParser(); p.add_argument('--out',required=True); p.add_argument('--frames',type=int,default=12); p.add_argument('--interval',type=float,default=0.5); a=p.parse_args()
    rclpy.init(); n=Collector(Path(a.out),a.frames,a.interval)
    try:
        while rclpy.ok() and n.saved<a.frames: rclpy.spin_once(n,timeout_sec=0.1)
    except KeyboardInterrupt: pass
    n.destroy_node();
    if rclpy.ok(): rclpy.shutdown()
    print(f'COLLECTED={n.saved}')
if __name__=='__main__': main()
