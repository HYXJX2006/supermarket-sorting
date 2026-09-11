#!/usr/bin/env python3
"""在已停稳、正对货架的位置采集头部相机上/中/下三层图像。
只控制 head_pitch，不控制底盘、机械臂、夹爪或升降柱。
"""
from __future__ import annotations
import argparse, json, math, time
from datetime import datetime
from pathlib import Path
import cv2, numpy as np, rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray

RGB='/head_camera/color/image_raw'; RESULT='/multiclass/result_image'; DEPTH='/head_camera/aligned_depth_to_color/image_raw'; ODOM='/slamware_ros_sdk_server_node/odom'; JOINT='/joint_states'; HEAD='/head_forward_position_controller/commands'
LAYERS=(('upper',-0.35),('middle',-0.60),('lower',-0.85))

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
class Capture(Node):
 def __init__(self,out:Path,shelf:str,settle:float):
  super().__init__('capture_layers'); self.out=out; self.out.mkdir(parents=True,exist_ok=True); self.shelf=shelf; self.settle=max(.8,settle); self.bridge=CvBridge(); self.rgb=None; self.result=None; self.depth=None; self.rgb_at=0.; self.result_at=0.; self.pitch=None; self.odom=None; self.stage=0; self.started=time.monotonic(); self.stage_started=0.; self.records=[]
  self.pub=self.create_publisher(Float64MultiArray,HEAD,10)
  self.create_subscription(Image,RGB,self.on_rgb,qos_profile_sensor_data); self.create_subscription(Image,RESULT,self.on_result,qos_profile_sensor_data); self.create_subscription(Image,DEPTH,self.on_depth,qos_profile_sensor_data); self.create_subscription(JointState,JOINT,self.on_joint,qos_profile_sensor_data); self.create_subscription(Odometry,ODOM,self.on_odom,qos_profile_sensor_data)
  self.get_logger().info(f'开始 {shelf} 三层采样，输出={out}')
 def on_rgb(self,m):
  try:self.rgb=self.bridge.imgmsg_to_cv2(m,'bgr8'); self.rgb_at=time.monotonic()
  except Exception:pass
 def on_result(self,m):
  try:self.result=self.bridge.imgmsg_to_cv2(m,'bgr8'); self.result_at=time.monotonic()
  except Exception:pass
 def on_depth(self,m):
  try:self.depth=self.bridge.imgmsg_to_cv2(m,'passthrough')
  except Exception:pass
 def on_joint(self,m):
  for i,n in enumerate(m.name):
   if n=='head_pitch_joint' and i<len(m.position): self.pitch=float(m.position[i]); break
 def on_odom(self,m):
  p=m.pose.pose; self.odom={'x':float(p.position.x),'y':float(p.position.y),'yaw':yaw(p.orientation),'vx':float(m.twist.twist.linear.x),'wz':float(m.twist.twist.angular.z)}
 def command(self,p): self.pub.publish(Float64MultiArray(data=[0.,float(p)]))
 def save(self,name,p):
  if self.rgb is None:return False
  rgb=self.out/f'{self.shelf}_{name}_rgb.jpg'; result=self.out/f'{self.shelf}_{name}_result.jpg'; depth=self.out/f'{self.shelf}_{name}_depth.jpg'
  cv2.imwrite(str(rgb),self.rgb,[cv2.IMWRITE_JPEG_QUALITY,60]); cv2.imwrite(str(result),self.result if self.result is not None else self.rgb,[cv2.IMWRITE_JPEG_QUALITY,60])
  if self.depth is not None:
   d=np.asarray(self.depth); valid=d[np.isfinite(d)&(d>0)]
   if valid.size:
    lo,hi=np.percentile(valid,[2,98]); q=np.clip((d.astype(np.float32)-lo)*255/max(float(hi-lo),1),0,255).astype(np.uint8); cv2.imwrite(str(depth),cv2.applyColorMap(q,cv2.COLORMAP_TURBO),[cv2.IMWRITE_JPEG_QUALITY,45])
  rec={'shelf':self.shelf,'layer':name,'pitch_command':p,'pitch_feedback':self.pitch,'base':self.odom,'files':{'rgb':rgb.name,'result':result.name,'depth':depth.name if depth.exists() else None},'captured_at':datetime.now().astimezone().isoformat()}; self.records.append(rec); (self.out/f'{self.shelf}_{name}.json').write_text(json.dumps(rec,ensure_ascii=False,indent=2),encoding='utf-8'); self.get_logger().info(f'已保存 {self.shelf}/{name} pitch={self.pitch}'); return True
 def tick(self):
  if self.stage>=len(LAYERS): return
  name,p=LAYERS[self.stage]; self.command(p); now=time.monotonic(); pitch_ok=self.pitch is None or abs(self.pitch-p)<.08; fresh=now-self.rgb_at<.8 and now-self.result_at<.8
  if pitch_ok and fresh:
   if not self.stage_started:self.stage_started=now
   if now-self.stage_started>=self.settle:
    if self.save(name,p): self.stage+=1; self.stage_started=0.
  else:self.stage_started=0.
 def done(self): return self.stage>=len(LAYERS) or time.monotonic()-self.started>30
 def finalize(self):
  self.command(-.60); (self.out/'manifest.json').write_text(json.dumps({'shelf':self.shelf,'count':len(self.records),'records':self.records},ensure_ascii=False,indent=2),encoding='utf-8')
def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--output-dir',required=True); ap.add_argument('--shelf',required=True); ap.add_argument('--settle',type=float,default=1.0); a=ap.parse_args(); rclpy.init(); n=Capture(Path(a.output_dir),a.shelf,a.settle)
 try:
  while rclpy.ok() and not n.done(): rclpy.spin_once(n, timeout_sec=.05); n.tick()
 finally:n.finalize();n.destroy_node();rclpy.shutdown()
 return 0 if len(n.records)==3 else 1
if __name__=='__main__': raise SystemExit(main())
