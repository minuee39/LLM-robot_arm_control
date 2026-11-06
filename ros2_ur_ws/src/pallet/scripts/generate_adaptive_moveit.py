from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
base=Path(__file__).resolve().parents[2]
c=base/'pallet_physical_moveit_config/config'
r=ET.parse(base/'pallet/urdf/pallet_adaptive.urdf').getroot()
ET.SubElement(r,'link',name='gripper_motor_reference')
j=ET.SubElement(r,'joint',name='gripper_motor',type='revolute')
ET.SubElement(j,'parent',link='gripper');ET.SubElement(j,'child',link='gripper_motor_reference');ET.SubElement(j,'axis',xyz='0 1 0');ET.SubElement(j,'limit',lower='-12',upper='12',velocity='2.0',effort='1')
ET.indent(r);ET.ElementTree(r).write(c/'pallet_adaptive_control.urdf',encoding='utf-8',xml_declaration=True)
s=ET.parse(c/'pallet.srdf').getroot()
s.set('name',r.get('name'))
for e in list(s):
    if e.tag=='disable_collisions' and ({e.get('link1'),e.get('link2')}&{'gripper_l','gripper_r'}):s.remove(e)
# Tree-adjacent parts and the cut loop's shared pad pin are physical connections.
# Keep all other self collisions and all environment collisions enabled.
for side in ('l', 'r'):
    for a,b in [('pad','wrist'),('inner_upper','pad')]:
        ET.SubElement(s,'disable_collisions',link1=f'{side}_{a}_link',
                      link2=f'{side}_{b}_link',reason='Adjacent')
s.find("group[@name='gripper']/joint").set('name','gripper_motor')
for name,q in [('open',11.5),('middle',0),('closed_measured',-11.5)]:
    g=ET.SubElement(s,'group_state',name=name,group='gripper');ET.SubElement(g,'joint',name='gripper_motor',value=str(q))
for j in r.findall('joint'):
    if j.get('type')!='fixed' and j.get('name') not in [f'J{i}' for i in range(1,7)]+['gripper_motor']:
        ET.SubElement(s,'passive_joint',name=j.get('name'))
        ET.SubElement(s.find("group[@name='gripper']"),'joint',name=j.get('name'))
ET.indent(s);ET.ElementTree(s).write(c/'pallet_adaptive.srdf',encoding='utf-8',xml_declaration=True)
ctrl=yaml.safe_load((c/'moveit_controllers.yaml').read_text());ctrl['moveit_simple_controller_manager']['pallet_gripper_controller']['joints']=['gripper_motor'];(c/'adaptive_moveit_controllers.yaml').write_text(yaml.safe_dump(ctrl,sort_keys=False))
limits=yaml.safe_load((c/'joint_limits.yaml').read_text());limits['joint_limits'].pop('g_r',None);limits['joint_limits']['gripper_motor']={'has_velocity_limits':True,'max_velocity':1.8,'has_acceleration_limits':True,'max_acceleration':4.5};(c/'adaptive_joint_limits.yaml').write_text(yaml.safe_dump(limits,sort_keys=False))

# Nonlinear linkage lookup for the MoveIt Joints preview (no hardware commands).
from adaptive_gripper_core import AdaptiveGripper
import json
model=AdaptiveGripper.load(base/'pallet/config/adaptive_gripper/geometry.json')
states=[model.pose(-12.+i*.05)['joints'] for i in range(481)]
names=list(states[0])
(c/'adaptive_preview_states.json').write_text(json.dumps({
    'lower_rad':-12.,'step_rad':.05,'joint_names':names,
    'positions':[[state[n] for n in names] for state in states]},separators=(',',':'))+'\n')
