"""Supported demo, A toggles manual control; Space pauses; R resets."""
from pathlib import Path
import argparse,time,math,json
import mujoco,mujoco.viewer
P=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--free',action='store_true');parser.add_argument('--render',action='store_true');args=parser.parse_args()
m=mujoco.MjModel.from_xml_path(str(P/('platform.xml' if args.free else 'scene_demo.xml')));d=mujoco.MjData(m)
state={'auto':not args.free,'pause':False,'reset':False}
def key(k):
 if k==32:state['pause']=not state['pause']
 elif k in [65,97]:state['auto']=not state['auto'] if not args.free else False
 elif k in [82,114]:state['reset']=True
def control():
 if state['auto']:
  t=d.time;d.ctrl[:]=[math.radians(35*(1-math.cos(2*math.pi*t/12))),math.radians(20*(1-math.cos(2*math.pi*t/6))),0]
if args.render:
 import struct,zlib
 for i in range(2200):control();mujoco.mj_step(m,d)
 cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.32];cam.distance=1.15;cam.azimuth=70;cam.elevation=-18
 with mujoco.Renderer(m,height=960,width=1280) as renderer:
  opt=mujoco.MjvOption();opt.geomgroup[3]=0;renderer.update_scene(d,camera=cam,scene_option=opt)
  rgb=renderer.render();h,w,_=rgb.shape
  def chunk(t,b):return struct.pack('!I',len(b))+t+b+struct.pack('!I',zlib.crc32(t+b)&0xffffffff)
  raw=b''.join(b'\x00'+row.tobytes() for row in rgb)
  (P/'preview.png').write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))
else:
 with mujoco.viewer.launch_passive(m,d,key_callback=key) as v:
  v.cam.lookat[:]=[0,0,.32];v.cam.distance=1.15;v.cam.azimuth=70;v.cam.elevation=-18;v.opt.geomgroup[3]=0
  while v.is_running():
   start=time.monotonic()
   if state['reset']:mujoco.mj_resetData(m,d);state['reset']=False
   if not state['pause']:
    for _ in range(16):control();mujoco.mj_step(m,d)
   v.sync();time.sleep(max(0,.016-(time.monotonic()-start)))
