#!/usr/bin/env python3
"""DG-202606 plan-only 放置控制器。

执行阶段：升降柱降到配送高度 -> 保持右臂 -> 松开右夹爪 -> 等待落台。
真实执行必须 --execute --confirm place；默认不发命令。
"""
from __future__ import annotations
import argparse, json, time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
SPINE_TOPIC="/spine_forward_position_controller/commands"; RIGHT_TOPIC="/right_arm_forward_position_controller/commands"; JOINT_TOPIC="/joint_states"; PLAN_TOPIC="/competition/place_plan"; STATUS_TOPIC="/competition/place_status"
PLACE_SLIDE=0.17; GRIP_OPEN=1.0; GRIP_CLOSE=0.08
INIT_ARM_R=[0.0,-0.166,0.032,0.0,-1.571,-2.223]
class PlaceController(Node):
 def __init__(self,execute,confirm,timeout):
  super().__init__('place_controller'); self.execute_enabled=execute and confirm=='place'; self.rejected=execute and confirm!='place'; self.timeout=max(1.,float(timeout)); self.started=time.monotonic(); self.last=0.; self.done=False; self.success=False; self.joints={}; self.seen=False; self.stage='lower'
  self.spine=self.create_publisher(Float64MultiArray,SPINE_TOPIC,10); self.right=self.create_publisher(Float64MultiArray,RIGHT_TOPIC,10); self.plan=self.create_publisher(String,PLAN_TOPIC,10); self.status=self.create_publisher(String,STATUS_TOPIC,10); self.create_subscription(JointState,JOINT_TOPIC,self._j,qos_profile_sensor_data); self.create_timer(.1,self._tick)
  self.get_logger().warning(f"PLACE mode={'EXECUTE' if self.execute_enabled else 'PLAN-ONLY'}；不控制底盘")
 def _j(self,m): self.seen=True; self.joints={n:float(m.position[i]) for i,n in enumerate(m.name) if i<len(m.position)}
 def _arm(self): return [self.joints.get(f'right_arm_joint{i}',INIT_ARM_R[i-1]) for i in range(1,7)]
 def _emit(self,state,reason): m=String(); m.data=json.dumps({'schema_version':1,'state':state,'reason':reason,'stage':self.stage},ensure_ascii=False,separators=(',',':')); self.status.publish(m)
 def _tick(self):
  if self.done:return
  if self.rejected:self.get_logger().error('拒绝执行：必须使用 --execute --confirm place');self.done=True;return
  if time.monotonic()-self.started>self.timeout:self._emit('failed','超时');self.done=True;return
  if not self.seen:self._emit('waiting','等待 joint_states');self._log('等待 joint_states，不发送放置命令');return
  plan={'schema_version':1,'mode':'execute' if self.execute_enabled else 'plan-only','action':'place','stage':self.stage,'slide_target':PLACE_SLIDE,'right_arm_joints':self._arm(),'right_gripper':GRIP_OPEN if self.stage=='release' else GRIP_CLOSE,'mechanical_commands_sent':False,'base_commands_sent':False}
  m=String();m.data=json.dumps(plan,ensure_ascii=False,separators=(',',':'));self.plan.publish(m)
  if not self.execute_enabled:self.success=True;self.done=True;self.get_logger().info('PLAN-ONLY 放置计划：'+m.data);return
  if self.stage=='lower':
   self.spine.publish(Float64MultiArray(data=[PLACE_SLIDE])); self.right.publish(Float64MultiArray(data=self._arm()+[GRIP_CLOSE]))
   if abs(self.joints.get('slide_joint',999)-PLACE_SLIDE)<.02:self.stage='release';self._emit('active','升降柱到配送高度');
  else:
   self.right.publish(Float64MultiArray(data=self._arm()+[GRIP_OPEN]));
   if time.monotonic()-self.started>2.0:self._emit('reached','商品已释放，等待裁判/视觉确认');self.success=True;self.done=True
  self._log(f'PLACE stage={self.stage} slide={self.joints.get("slide_joint")}')
 def _log(self,t):
  if time.monotonic()-self.last>2:self.get_logger().info(t);self.last=time.monotonic()
def main():
 p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');p.add_argument('--confirm',default='');p.add_argument('--timeout',type=float,default=20.);a=p.parse_args()
 if a.execute and a.confirm!='place':print('拒绝执行：必须使用 --confirm place');return 2
 rclpy.init();n=PlaceController(a.execute,a.confirm,a.timeout)
 try:
  while rclpy.ok() and not n.done:rclpy.spin_once(n,timeout_sec=.1)
 except (KeyboardInterrupt,ExternalShutdownException):pass
 finally:
  result=n.success;n.destroy_node()
  if rclpy.ok():rclpy.shutdown()
 return 0 if result else 1
if __name__=='__main__':raise SystemExit(main())
