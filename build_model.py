"""Rebuild portable MJCF from the current Fusion snapshot. SI throughout."""
from pathlib import Path
import json,math,collections,xml.etree.ElementTree as E
import numpy as np
P=Path(__file__).resolve().parent
M=json.loads((P/'cad_export.json').read_text());parts=M['parts'];J=M['joints']
def vec(x):return ' '.join(f'{v:.10g}' for v in x)
def add(p,t,**kw):return E.SubElement(p,t,{k:str(v) for k,v in kw.items()})
def group(p):
 o=p['occurrence'];s=p['source']
 if o.startswith('PL_leg_'):
  g=o.split(':')[0][7:]
  if g=='fixed':return 'carriage'
  if 'wheel_body:' in s or ('DD9015' in s and p['body'].endswith('| Body37')):return 'wheel'
  return g
 return 'carriage' if o in M['moving_occurrences'] else 'frame'
def obj(path):
 v=[];f=[]
 for l in path.read_text().splitlines():
  s=l.split()
  if s[0]=='v':v.append([float(x) for x in s[1:4]])
  elif s[0]=='f':f.append([int(x)-1 for x in s[1:4]])
 return np.array(v),np.array(f)
def build():
 cfgfile=P/'config.json'
 if not cfgfile.exists():raise FileNotFoundError('config.json is required; preserve the supplied motor and mass settings.')
 c=json.loads(cfgfile.read_text());c['RS06_total_kg']=c['RS06']['mass_kg'];orig={'frame':np.zeros(3)}
 def jp(name):
  p=np.array(J['PL - '+name]['origin_m']);p[1]=0;return p
 O=jp('Hip');K=jp('Knee follower');H=jp('Pushrod upper');T=jp('Pushrod lower')
 tyre=next(p for p in parts if 'wheel_body:' in p['source']);bb=np.array(tyre['bounds_m']);W=bb.mean(axis=0)
 orig.update(carriage=O,thigh=O,horn=O,rod=H,shin=K,wheel=W)
 radius=(bb[1,0]-bb[0,0])/2;width=(bb[1,1]-bb[0,1])/2
 # Mass buckets count each bought motor once, split between housing and output.
 masses={};buckets=collections.defaultdict(list);methods={}
 for p in parts:
  s=p['source'];g=group(p);volume=p['volume_m3']
  if not p['inertia_per_kg'] or volume<=0:continue
  if 'RS06' in s:
   motor='hip' if s.startswith('hip_mount') else 'knee';output=g==('thigh' if motor=='hip' else 'horn');buckets[(motor,'output' if output else 'housing')].append(p);continue
  if 'DD9015' in s:buckets[('wheel','output' if g=='wheel' else 'housing')].append(p);continue
  if 'Flanged Ball Bearing' in s:buckets[('bearing','all')].append(p);continue
  fraction=1
  if 'wheel_body:' in s:material='TPU';fraction=c['TPU_solid_fraction']
  elif s:
   if s=='hip_fixed_mount:1' or 'hip_output_adapter' in s:material='aluminium'
   else:material='PETG';fraction=c['print_solid_fraction']
  else:
   ap=p['appearance'].lower();material=next((x for x in ['plywood','delrin','aluminium','steel'] if x in ap),'steel')
  masses[p['id']]=volume*c['density'][material]*fraction;methods[p['id']]=f'{material} volume × density × {fraction} solid fraction'
 for (motor,role),pp in buckets.items():
  total=c['bearings_total_kg'] if motor=='bearing' else (c['DD9015_output_kg'] if role=='output' else c['DD9015_total_kg']-c['DD9015_output_kg']) if motor=='wheel' else (c['RS06_output_kg'] if role=='output' else c['RS06_total_kg']-c['RS06_output_kg'])
  volume=sum(p['volume_m3'] for p in pp)
  for p in pp:masses[p['id']]=total*p['volume_m3']/volume;methods[p['id']]=f'{motor} {role}, volume-distributed nominal total {total} kg'
 inert={}
 for g in orig:
  pp=[p for p in parts if group(p)==g and p['id'] in masses];mass=sum(masses[p['id']] for p in pp)
  com=sum((masses[p['id']]*np.array(p['com_m']) for p in pp),np.zeros(3))/mass;I=np.zeros((3,3))
  for p in pp:
   m=masses[p['id']];axes=np.array(p['axes']).T;off=np.array(p['com_m'])-com
   I+=m*(axes@np.diag(p['inertia_per_kg'])@axes.T+np.eye(3)*np.dot(off,off)-np.outer(off,off))
  assert np.linalg.eigvalsh(I).min()>0
  inert[g]={'mass_kg':mass,'com_m':com.tolist(),'inertia_kg_m2':I.tolist()}
 (P/'mass_audit.json').write_text(json.dumps({'groups':inert,'parts':[{'id':p['id'],'mass_kg':masses.get(p['id'],0),'method':methods.get(p['id'],'surface representation; mass included in device bucket')} for p in parts]},indent=2))
 grouped=collections.defaultdict(list)
 for p in parts:grouped[(group(p),tuple(p['rgba']))].append(p)
 (P/'meshes').mkdir(exist_ok=True);assets=[]
 for index,((g,rgba),pp) in enumerate(grouped.items()):
  name=f'{g}_{index:02d}';vv=[];ff=[];n=0
  for p in pp:
   v,f=obj(P/p['mesh']);vv.append(v-orig[g]);ff.append(f+n);n+=len(v)
  with (P/'meshes'/f'{name}.obj').open('w') as f:
   for v in np.concatenate(vv):f.write('v '+vec(v)+'\n')
   for row in np.concatenate(ff):f.write('f '+' '.join(str(x+1) for x in row)+'\n')
  assets.append((name,g,rgba))
 def model(demo):
  root=E.Element('mujoco',model='Leg test platform '+('supported demo' if demo else 'slider dynamics'))
  add(root,'compiler',angle='radian',meshdir='meshes',autolimits='true',inertiafromgeom='false')
  add(root,'option',timestep=c['timestep_s'],gravity=vec(c['gravity_m_s2']),integrator='implicitfast',iterations=100,tolerance='1e-10')
  vis=add(root,'visual');add(vis,'global',offwidth=1280,offheight=960);add(vis,'headlight',ambient='.4 .4 .4',diffuse='.6 .6 .6')
  add(root,'statistic',center='0 0 .33',extent='.9')
  de=add(root,'default');add(de,'joint',axis='0 1 0',damping='.06',armature='.001');add(de,'geom',mass='0',friction='.8 .005 .0001',solref='.008 1')
  v=add(de,'default',**{'class':'visual'});add(v,'geom',type='mesh',contype=0,conaffinity=0,group=1)
  cl=add(de,'default',**{'class':'collision'});add(cl,'geom',group=3,rgba='.1 .7 .8 .3')
  floor=c['floor']
  if floor['solref'][0]<2*c['timestep_s']:raise ValueError('Floor contact time constant must be >= twice timestep')
  asset=add(root,'asset')
  add(asset,'texture',name='ground_checker',type='2d',builtin='checker',rgb1=vec(floor['rgb1']),rgb2=vec(floor['rgb2']),width=512,height=512)
  repeat=1/(2*floor['checker_square_m'])
  add(asset,'material',name='ground_material',texture='ground_checker',texuniform='true',texrepeat=vec([repeat,repeat]),reflectance=0,specular=.1,shininess=.1)
  for name,g,rgba in assets:add(asset,'mesh',name=name,file=name+'.obj',inertia='shell')
  world=add(root,'worldbody');add(world,'light',pos='0 -1 2',dir='0 .3 -1',directional='true')
  add(world,'geom',name='floor',type='plane',pos=vec([0,0,floor['height_m']]),size='0 0 .05',material='ground_material',contype=1,conaffinity=1,condim=floor['condim'],priority=1,friction=vec(floor['friction']),solref=vec(floor['solref']),solimp=vec(floor['solimp']),margin=0)
  nodes={'frame':add(world,'body',name='frame')}
  nodes['carriage']=add(world,'body',name='carriage',pos=vec(O))
  slide=J['RIG - vertical carriage travel']
  add(nodes['carriage'],'joint',name='carriage_z',type='slide',axis='0 0 1',ref=slide['value'],range=vec(slide['range']),damping='2',armature='0')
  spec=[('thigh','carriage','hip',J['PL - Hip']['value'],c['hip_range_deg']),('horn','thigh','knee',J['PL - Knee']['value'],c['knee_range_deg']),('rod','horn','rod_pivot',-J['PL - Knee']['value'],None),('shin','thigh','shin_pivot',J['PL - Knee']['value'],None),('wheel','shin','wheel',0,None)]
  for g,pa,j,ref,limits in spec:
   nodes[g]=add(nodes[pa],'body',name=g,pos=vec(orig[g]-orig[pa]))
   kw={'range':vec(np.radians(limits))} if limits else {}
   add(nodes[g],'joint',name=j,type='hinge',ref=ref,**kw)
  for g,node in nodes.items():
   # Fixed frame retains mass but remains grounded (no free joint).
   info=inert[g];I=np.array(info['inertia_kg_m2'])
   add(node,'inertial',mass=info['mass_kg'],pos=vec(np.array(info['com_m'])-orig[g]),fullinertia=vec([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
   for name,ag,rgba in assets:
    if ag==g:add(node,'geom',name=name,mesh=name,rgba=vec(rgba),**{'class':'visual'})
  for g in ['rod','shin']:add(nodes[g],'site',name=g+'_end',pos=vec(T-orig[g]),size='.002',group=4)
  add(nodes['wheel'],'site',name='wheel_axis',size='.002',group=4)
  add(nodes['carriage'],'site',name='carriage_imu',size='.002',group=4)
  # The guides are an ideal slider, not wheel/rail contact meshes.
  for p in parts:
   if p['occurrence'] in ['rig_deck:1','rig_upright_left:1','rig_upright_right:1','rig_foot_left:1','rig_foot_right:1','rig_base_crossbar_front:1','rig_base_crossbar_rear:1','rig_top_crossbar:1','rig_carriage:1']:
    bb=np.array(p['bounds_m']);g=group(p)
    add(nodes[g],'geom',name='collision_'+p['id'],type='box',pos=vec(bb.mean(axis=0)-orig[g]),size=vec((bb[1]-bb[0])/2),**{'class':'collision'})
  for g,p1,p2 in [('thigh',O,K),('shin',K,W)]:
   p1=p1.copy();p2=p2.copy();p1[1]=p2[1]=-.004
   # Narrow central span proxies avoid turning concave enclosures into solid hulls.
   start=p1+.15*(p2-p1);end=p2-.15*(p2-p1)
   add(nodes[g],'geom',name=g+'_collision',type='capsule',fromto=vec(np.r_[start-orig[g],end-orig[g]]),size='.012',**{'class':'collision'})
  add(nodes['wheel'],'geom',name='tyre_collision',type='cylinder',size=vec([radius,width]),quat='.7071067812 .7071067812 0 0',condim=4,**{'class':'collision'})
  eq=add(root,'equality');add(eq,'connect',name='parallel_loop',site1='rod_end',site2='shin_end',solref='.003 1',solimp='.999 .9999 .0001')
  if demo:add(eq,'joint',name='carriage_lock',joint1='carriage_z',polycoef='0 0 0 0 0')
  con=add(root,'contact');add(con,'exclude',body1='frame',body2='carriage');add(con,'exclude',body1='carriage',body2='horn')
  ac=add(root,'actuator')
  for j in ['hip','knee','wheel']:
   mode=c['demo_RS06_torque_mode' if demo else 'free_RS06_torque_mode']
   if mode not in ('rated','peak'):raise ValueError('RS06 torque mode must be rated or peak')
   tq=c['wheel_torque_limit_Nm'] if j=='wheel' else c['RS06'][mode+'_torque_Nm']
   if demo and j!='wheel':add(ac,'position',name=j+'_servo',joint=j,kp=c['servo_kp'],kv=c['servo_kv'],ctrlrange=vec(np.radians(c[j+'_range_deg'])),forcerange=vec([-tq,tq]))
   else:add(ac,'motor',name=j+'_motor',joint=j,gear='1',ctrlrange=vec([-tq,tq]),forcerange=vec([-tq,tq]))
  sensor=add(root,'sensor')
  for j in ['hip','knee','wheel','carriage_z']:
   add(sensor,'jointpos',name=j+'_position',joint=j);add(sensor,'jointvel',name=j+'_velocity',joint=j)
  add(sensor,'accelerometer',name='carriage_accel',site='carriage_imu')
  add(sensor,'gyro',name='carriage_gyro',site='carriage_imu')
  E.indent(root);E.ElementTree(root).write(P/('scene_demo.xml' if demo else 'platform.xml'),encoding='utf-8',xml_declaration=True)
 model(True);model(False)
 (P/'build_report.json').write_text(json.dumps({'source':M['document'],'bodies':len(parts),'visual_meshes':len(assets),'origins_m':{k:v.tolist() for k,v in orig.items()},'wheel_radius_m':radius,'moving_mass_kg':sum(v['mass_kg'] for k,v in inert.items() if k!='frame'),'frame_mass_kg':inert['frame']['mass_kg']},indent=2))
 print((P/'build_report.json').read_text())
if __name__=='__main__':build()
