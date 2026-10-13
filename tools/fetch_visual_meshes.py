"""Retrieve pinned manufacturer COLLADA geometry and convert it to binary STL.

Development-time only; MATLAB runtime needs no Python or network access.
Scene-node transforms are applied before writing link-local geometry.
"""
from pathlib import Path
import hashlib
import json
import math
import struct
import urllib.request
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
revision = 'ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98'
folder = root / 'sources/meshes'
folder.mkdir(parents=True, exist_ok=True)
manifest = []


def convert(source, destination):
    xml = ET.parse(source).getroot()
    ns = {'c': xml.tag.split('}')[0][1:]}
    assert float(xml.find('c:asset/c:unit', ns).get('meter')) == 1
    assert xml.find('c:asset/c:up_axis', ns).text == 'Z_UP'
    scene = xml.find('c:library_visual_scenes', ns)
    identity = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    assert not scene.findall('.//c:rotate', ns) and not scene.findall('.//c:scale', ns)
    assert not scene.findall('.//c:translate', ns)
    def multiply(a,b):
        return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
    used = []
    def visit(node,parent):
        local=identity
        for transform in node.findall('c:matrix',ns):
            values=[float(x) for x in transform.text.split()]
            # Blender COLLADA writes the affine matrix as four explicit rows.
            matrix=[values[i:i+4] for i in range(0,16,4)]
            assert matrix[3]==[0,0,0,1]
            local=multiply(local,matrix)
        world=multiply(parent,local)
        for instance in node.findall('c:instance_geometry',ns):used.append((instance.get('url')[1:],world))
        for child in node.findall('c:node',ns):visit(child,world)
    for visual_scene in scene.findall('c:visual_scene',ns):
        for node in visual_scene.findall('c:node',ns):visit(node,identity)
    geometries = {x.get('id'): x for x in xml.findall('c:library_geometries/c:geometry', ns)}
    triangles = []
    for gid,transform in used:
        mesh = geometries[gid].find('c:mesh', ns)
        sources = {}
        for item in mesh.findall('c:source', ns):
            array = [float(x) for x in item.find('c:float_array', ns).text.split()]
            stride = int(item.find('c:technique_common/c:accessor', ns).get('stride'))
            sources[item.get('id')] = [array[i:i+stride] for i in range(0,len(array),stride)]
        vertices = {x.get('id'): x.find("c:input[@semantic='POSITION']",ns).get('source')[1:]
                    for x in mesh.findall('c:vertices', ns)}
        for primitive in list(mesh):
            kind = primitive.tag.split('}')[-1]
            if kind not in ['triangles','polylist']:
                continue
            inputs = primitive.findall('c:input', ns)
            stride = max(int(x.get('offset')) for x in inputs)+1
            vertex = next(x for x in inputs if x.get('semantic')=='VERTEX')
            offset = int(vertex.get('offset'))
            positions = sources[vertices[vertex.get('source')[1:]]]
            indices = [int(x) for x in primitive.find('c:p', ns).text.split()]
            indices = indices[offset::stride]
            counts = [3]*int(primitive.get('count')) if kind=='triangles' else [int(x) for x in primitive.find('c:vcount',ns).text.split()]
            cursor=0
            for count in counts:
                polygon = indices[cursor:cursor+count];cursor += count
                for j in range(1,count-1):
                    tri=[]
                    for index in [0,j,j+1]:
                        v=positions[polygon[index]][:3]+[1]
                        tri.append([sum(transform[row][col]*v[col] for col in range(4)) for row in range(3)])
                    triangles.append(tri)
            assert cursor==len(indices)
    with destination.open('wb') as out:
        out.write(b'Pinned Franka visual geometry, conversion only'.ljust(80,b'\0'))
        out.write(struct.pack('<I',len(triangles)))
        for a,b,c in triangles:
            u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
            n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
            length=math.sqrt(sum(x*x for x in n))
            if length:n=[x/length for x in n]
            out.write(struct.pack('<12fH',*(n+a+b+c),0))
    return len(triangles)


for link in range(8):
    url=f'https://raw.githubusercontent.com/frankarobotics/franka_ros/{revision}/franka_description/meshes/visual/link{link}.dae'
    source=folder/f'link{link}.dae';target=folder/f'link{link}.stl'
    if not source.exists():
        for attempt in range(3):
            try:
                with urllib.request.urlopen(url,timeout=45) as response:data=response.read()
                ET.fromstring(data)  # Never persist a partial download.
                source.write_bytes(data)
                break
            except (TimeoutError, OSError):
                if attempt==2:raise
    count=convert(source,target)
    manifest.append(dict(path=source.relative_to(root).as_posix(),url=url,revision=revision,sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
    manifest.append(dict(path=target.relative_to(root).as_posix(),source=source.relative_to(root).as_posix(),conversion='COLLADA scene transforms applied to binary STL',triangles=count,sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    print(f'link{link}: {count} triangles',flush=True)
(root/'sources/mesh_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',newline='\n')
