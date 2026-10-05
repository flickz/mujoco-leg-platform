from pathlib import Path
import json,math
import numpy as np
import mujoco
P=Path(__file__).resolve().parent
cad=json.loads((P/'cad_export.json').read_text());poses=json.loads((P/'cad_poses.json').read_text());build=json.loads((P/'build_report.json').read_text())
m=mujoco.MjModel.from_xml_path(str(P/'scene_demo.xml'));d=mujoco.MjData(m)
def setpose(h,k):
 d.qpos[:]=m.qpos0
 for j,v in [('hip',h),('knee',k),('shin_pivot',k),('rod_pivot',-k)]:d.joint(j).qpos[0]=math.radians(v)
 mujoco.mj_forward(m,d)
errors=[]
for pose in poses:
 setpose(pose['hip_deg'],pose['knee_deg'])
 for g in ['thigh','horn','shin','rod']:
  old=np.array(cad['transforms']['PL_leg_'+g+':1']).reshape(4,4);new=np.array(pose['transforms'][g]).reshape(4,4)
  x=np.r_[np.array(build['origins_m'][g])*100,1];expected=(new@np.linalg.inv(old)@x)[:3]/100
  err=float(np.linalg.norm(d.body(g).xpos-expected));errors.append({'h':pose['hip_deg'],'k':pose['knee_deg'],'group':g,'error_m':err})
print('pose errors',errors)
print('compile',m.nq,m.nv,m.nu,m.neq)
setpose(0,0);maxloop=0;tracking=0
for step in range(12000):
 t=step*m.opt.timestep
 h=math.radians(35*(1-math.cos(2*math.pi*t/12)));k=math.radians(20*(1-math.cos(2*math.pi*t/6)))
 d.ctrl[:]=[h,k,0];mujoco.mj_step(m,d)
 maxloop=max(maxloop,float(np.linalg.norm(d.site('rod_end').xpos-d.site('shin_end').xpos)))
 tracking=max(tracking,abs(d.joint('hip').qpos[0]-h),abs(d.joint('knee').qpos[0]-k))
 assert np.isfinite(d.qpos).all()
print('dynamics',maxloop,math.degrees(tracking),'warnings',d.warning.number.tolist(),'q',d.qpos)
report={'cad_max_error_m':max(p['error_m'] for p in errors),'cad_poses':errors,'demo_duration_s':12,'loop_max_error_m':maxloop,'tracking_max_error_deg':math.degrees(tracking),'warnings':d.warning.number.tolist()}
(P/'validation_report.json').write_text(json.dumps(report,indent=2))
assert report['cad_max_error_m']<1e-6
assert maxloop<.0005 and tracking<math.radians(3)
assert not np.any(d.warning.number)
# Independent freely sliding, unpowered contact smoke test.
mf=mujoco.MjModel.from_xml_path(str(P/'platform.xml'));df=mujoco.MjData(mf);contacts=set();max_force=0
for i in range(5000):
 mujoco.mj_step(mf,df)
 assert np.isfinite(df.qpos).all()
 for co in df.contact:
  contacts.add(tuple(mf.geom(g).name for g in [co.geom1,co.geom2]))
report['free_slider']={'duration_s':5,'warnings':df.warning.number.tolist(),'contacts':[list(x) for x in sorted(contacts)],'final_qpos':df.qpos.tolist()}
assert not np.any(df.warning.number)
assert any('tyre_collision' in x for x in contacts),'Wheel never touched platform'
report['passed']=False
(P/'validation_report.json').write_text(json.dumps(report,indent=2))
print('All validation checks passed; free contacts:',contacts)
# Check compiled mass/inertia and actual gravitational acceleration.
audit=json.loads((P/'mass_audit.json').read_text())['groups']
for name,entry in audit.items():
 b=mf.body(name)
 assert np.isclose(b.mass[0],entry['mass_kg']) and b.mass[0]>0
 assert np.all(b.inertia>0)
assert np.allclose(mf.opt.gravity,[0,0,-9.81])
for model in [m,mf]:
 for name in ['hip','knee']:
  a=model.actuator(name+('_servo' if model is m else '_motor'))
  assert np.allclose(a.forcerange,[-11,11])
# Same zero-velocity pose, compare acceleration with gravity on/off.
dg=mujoco.MjData(mf);mujoco.mj_forward(mf,dg)
gacc=float(dg.joint('carriage_z').qacc[0])
mf.opt.gravity[:]=0;dz=mujoco.MjData(mf);mujoco.mj_forward(mf,dz)
zacc=float(dz.joint('carriage_z').qacc[0])
assert abs((gacc-zacc)+9.81)<1e-5 and abs(zacc)<1e-4  # Tiny closure stabilization at exported pose.
report['passed']=True
report['mass_gravity_checks']={'body_mass_kg':{n:float(mf.body(n).mass[0]) for n in audit},'carriage_accel_gravity_m_s2':gacc,'carriage_accel_zero_gravity_m_s2':zacc,'RS06_default_torque_cap_Nm':11,'passed':True}
(P/'validation_report.json').write_text(json.dumps(report,indent=2))
print('Mass, positive inertia, motor caps and gravity checks passed:',report['mass_gravity_checks'])
# Isolate floor contact with a temporary drop probe outside the plywood footprint.
import xml.etree.ElementTree as ET
root=ET.parse(P/'platform.xml').getroot();root.find('compiler').set('meshdir',str(P/'meshes'))
probe=ET.SubElement(root.find('worldbody'),'body',name='floor_probe',pos='0.8 0 0.3')
ET.SubElement(probe,'freejoint');ET.SubElement(probe,'geom',name='probe_geom',type='sphere',size='.02',mass='.1')
ET.SubElement(probe,'inertial',pos='0 0 0',mass='.1',diaginertia='.000016 .000016 .000016')
mp=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));dp=mujoco.MjData(mp);hit=False
for _ in range(2000):
 mujoco.mj_step(mp,dp)
 for ct in dp.contact:
  if set([mp.geom(ct.geom1).name,mp.geom(ct.geom2).name])=={'floor','probe_geom'}:hit=True
z=float(dp.body('floor_probe').xpos[2]);assert hit and abs(z-.02)<.001 and not np.any(dp.warning.number)
report['floor_test']={'contact':hit,'settled_probe_center_m':z,'expected_center_m':.02,'warnings':dp.warning.number.tolist()}
(P/'validation_report.json').write_text(json.dumps(report,indent=2));print('Floor probe passed',z)
