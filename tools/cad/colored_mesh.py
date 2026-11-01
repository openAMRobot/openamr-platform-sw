# SPDX-License-Identifier: MIT
"""Write one material-capable visual asset without changing triangle coordinates."""
import hashlib
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np

NS='http://www.collada.org/2005/11/COLLADASchema'
E.register_namespace('',NS)
def node(parent,tag,text=None,**attrs):
    child=E.SubElement(parent,'{'+NS+'}'+tag,{k:str(v) for k,v in attrs.items()})
    if text is not None:child.text=text
    return child

def write_dae(path,parts):
    """parts: ordered (triangles in metres, RGBA, descriptive material id)."""
    arrays=[np.asarray(t,dtype='<f4').reshape(-1,3,3) for t,_,_ in parts]
    triangles=np.concatenate(arrays)
    root=E.Element('{'+NS+'}COLLADA',version='1.4.1');asset=node(root,'asset')
    node(asset,'created','2026-11-01T00:00:00Z');node(asset,'modified','2026-11-01T00:00:00Z')
    node(asset,'unit',name='meter',meter='1');node(asset,'up_axis','Z_UP')
    effects=node(root,'library_effects');materials=node(root,'library_materials')
    geometry=node(node(root,'library_geometries'),'geometry',id='equipment',name='equipment');mesh=node(geometry,'mesh')
    normal=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);length=np.linalg.norm(normal,axis=1);normal/=np.where(length>0,length,1)[:,None]
    def floats(values):return ' '.join(format(float(v),'.9g') for v in values.reshape(-1))
    for id_,values in [('positions',triangles.reshape(-1,3)),('normals',normal)]:
        source=node(mesh,'source',id=id_);node(source,'float_array',floats(values),id=id_+'-array',count=values.size)
        accessor=node(node(source,'technique_common'),'accessor',source='#'+id_+'-array',count=len(values),stride=3)
        for name in ['X','Y','Z']:node(accessor,'param',name=name,type='float')
    vertices=node(mesh,'vertices',id='vertices');node(vertices,'input',semantic='POSITION',source='#positions')
    scene=node(node(root,'library_visual_scenes'),'visual_scene',id='Scene');obj=node(scene,'node',id='equipment_node')
    instance=node(obj,'instance_geometry',url='#equipment');bindings=node(node(instance,'bind_material'),'technique_common')
    offset=0;records=[]
    for index,((_,rgba,label),tris) in enumerate(zip(parts,arrays)):
        mat='material_'+str(index);effect=node(effects,'effect',id=mat+'-fx')
        phong=node(node(node(effect,'profile_COMMON'),'technique',sid='common'),'phong')
        for channel in ['ambient','diffuse']:node(node(phong,channel),'color',floats(np.asarray(rgba)))
        node(node(phong,'specular'),'color','0.08 0.08 0.08 1');node(node(phong,'shininess'),'float','20')
        node(node(materials,'material',id=mat,name=label),'instance_effect',url='#'+mat+'-fx')
        item=node(mesh,'triangles',count=len(tris),material=mat);node(item,'input',semantic='VERTEX',source='#vertices',offset=0);node(item,'input',semantic='NORMAL',source='#normals',offset=1)
        indices=[]
        for f in range(offset,offset+len(tris)):
            for v in range(3):indices.extend((3*f+v,f))
        node(item,'p',' '.join(map(str,indices)));node(bindings,'instance_material',symbol=mat,target='#'+mat)
        records.append(dict(name=label,rgba=list(rgba),triangles=len(tris)));offset+=len(tris)
    node(node(root,'scene'),'instance_visual_scene',url='#Scene')
    E.indent(root,space='  ');Path(path).write_bytes(E.tostring(root,encoding='utf-8',xml_declaration=True)+b'\n')
    return {'format':'COLLADA 1.4.1, metres, Z_UP','geometry_sha256_float32':hashlib.sha256(triangles.tobytes()).hexdigest(),'embedded_materials':records}
