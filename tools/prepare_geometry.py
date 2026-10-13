"""Extract joint-origin facts from the bundled pinned Franka Xacro.

Only the explicit numeric origins used by this revision are accepted; no Xacro
expressions or upstream programs are executed. Python standard library only.
"""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
xml = ET.parse(root / 'sources/franka_arm.xacro').getroot()
macro = xml.find('{http://www.ros.org/wiki/xacro}macro')
replacements = {'${-pi/2}': str(-math.pi/2), '${pi/2}': str(math.pi/2)}
data = dict(source='franka_description/robots/common/franka_arm.xacro',
            revision='ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98',
            xyz_rows=[], rpy_rows=[], axis_rows=[])
for number in range(1, 9):
    joint = next(x for x in macro.findall('joint') if x.get('name') == '${arm_id}_joint'+str(number))
    origin = joint.find('origin')
    xyz = origin.get('xyz')
    rpy = origin.get('rpy')
    for expression, value in replacements.items():
        rpy = rpy.replace(expression, value)
    data['xyz_rows'].append([float(x) for x in xyz.split()])
    data['rpy_rows'].append([float(x) for x in rpy.split()])
    if number <= 7:
        data['axis_rows'].append([float(x) for x in joint.find('axis').get('xyz').split()])
(root / 'sources/joint_origins.json').write_text(json.dumps(data, indent=2)+'\n', newline='\n')
print('Prepared eight manufacturer joint origins')
