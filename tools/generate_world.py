#!/usr/bin/env python3
"""Generate a self-contained Fortress SDF; no Fuel/models/textures downloads."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

def add(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, attrs)
    if text is not None: e.text = str(text)
    return e

def geometry(parent, kind, dimensions):
    g = add(parent, 'geometry'); s = add(g, kind)
    if kind == 'box': add(s, 'size', dimensions)
    elif kind == 'cylinder':
        add(s, 'radius', dimensions[0]); add(s, 'length', dimensions[1])
    elif kind == 'sphere': add(s, 'radius', dimensions)
    elif kind == 'plane':
        add(s, 'normal', '0 0 1'); add(s, 'size', dimensions)

def appearance(parent, kind, dimensions, color, pose=None, collide=True, friction=None):
    if collide:
        c = add(parent, 'collision', name='collision')
        if pose: add(c, 'pose', pose)
        geometry(c, kind, dimensions)
        if friction is not None:
            s = add(c, 'surface'); f = add(s, 'friction'); ode = add(f, 'ode')
            add(ode, 'mu', friction); add(ode, 'mu2', friction)
    v = add(parent, 'visual', name='visual')
    if pose: add(v, 'pose', pose)
    geometry(v, kind, dimensions)
    m = add(v, 'material'); add(m, 'ambient', color); add(m, 'diffuse', color); add(m, 'specular', '0.02 0.02 0.02 1')

def inertia(link, mass, diagonal):
    i=add(link,'inertial'); add(i,'mass',mass); t=add(i,'inertia')
    for tag,val in zip(['ixx','iyy','izz'],diagonal): add(t,tag,val)
    for tag in ['ixy','ixz','iyz']: add(t,tag,0)

def make():
    sdf = ET.Element('sdf', version='1.8'); world = add(sdf, 'world', name='visual_search')
    p = add(world,'physics', name='default_physics', type='ignored')
    add(p,'max_step_size',0.002); add(p,'real_time_factor',1.0)
    add(world, 'gravity', '0 0 -9.81')
    for name, file in [('Physics','physics'),('UserCommands','user-commands'),('SceneBroadcaster','scene-broadcaster'),('Sensors','sensors')]:
        plug = add(world,'plugin',name='ignition::gazebo::systems::'+name,filename='libignition-gazebo-'+file+'-system.so')
        if name=='Sensors': add(plug,'render_engine','ogre2')
    scene = add(world,'scene'); add(scene,'ambient','0.8 0.8 0.8 1'); add(scene,'background','0.75 0.8 0.9 1'); add(scene,'shadows','false')
    light = add(world,'light',name='sun',type='directional'); add(light,'pose','0 0 6 0 0 0')
    add(light,'diffuse','0.8 0.8 0.8 1'); add(light,'specular','0.1 0.1 0.1 1'); add(light,'direction','-0.5 0.2 -1')
    add(light,'cast_shadows','false')
    floor=add(world,'model',name='ground'); add(floor,'static','true'); link=add(floor,'link',name='link')
    appearance(link,'plane','8 8','0.55 0.55 0.55 1')
    objects=[('red_box','1.0 -0.7 0.25 0 0 0','0.35 0.35 0.5','0.95 0.035 0.025 1'),
             ('blue_box','1.0 0.9 0.25 0 0 0','0.35 0.35 0.5','0.025 0.04 0.95 1'),
             ('obstacle','0 1.9 0.25 0 0 0','0.35 0.35 0.5','0.3 0.3 0.3 1'),
             ('wall_n','0 3 0.35 0 0 0','6.2 0.15 0.7','0.65 0.65 0.65 1'),
             ('wall_s','0 -3 0.35 0 0 0','6.2 0.15 0.7','0.65 0.65 0.65 1'),
             ('wall_e','3 0 0.35 0 0 0','0.15 6.2 0.7','0.65 0.65 0.65 1'),
             ('wall_w','-3 0 0.35 0 0 0','0.15 6.2 0.7','0.65 0.65 0.65 1')]
    for name,pose,size,color in objects:
        m=add(world,'model',name=name); add(m,'static','true'); add(m,'pose',pose)
        l=add(m,'link',name='link'); appearance(l,'box',size,color)
    robot=add(world,'model',name='visual_bot'); add(robot,'pose','-1.25 0 0.005 0 0 0')
    base=add(robot,'link',name='base_link'); inertia(base,2.0,[0.01,0.018,0.022])
    inertial=base.find('inertial'); add(inertial,'pose','0 0 0.10 0 0 0')
    appearance(base,'box','0.30 0.22 0.08','0.15 0.15 0.15 1',pose='0 0 0.10 0 0 0')
    for name,y in [('left',0.15),('right',-0.15)]:
        l=add(robot,'link',name=name+'_wheel'); add(l,'pose',f'0 {y} 0.06 0 0 0'); inertia(l,0.15,[0.00016,0.00027,0.00016])
        appearance(l,'cylinder',(0.06,0.03),'0.08 0.08 0.08 1',pose='0 0 0 1.57079632679 0 0',friction=1.0)
        j=add(robot,'joint',name=name+'_wheel_joint',type='revolute'); add(j,'parent','base_link'); add(j,'child',name+'_wheel')
        ax=add(j,'axis'); add(ax,'xyz','0 1 0'); lim=add(ax,'limit'); add(lim,'lower',-1e16); add(lim,'upper',1e16)
    # Passive low-friction ball casters: neither wheel nor base body intersects floor.
    for name,x in [('front',0.12),('rear',-0.12)]:
        l=add(robot,'link',name=name+'_caster'); add(l,'pose',f'{x} 0 0.025 0 0 0'); inertia(l,0.05,[0.000013]*3)
        appearance(l,'sphere',0.025,'0.15 0.15 0.15 1',friction=0.01)
        j=add(robot,'joint',name=name+'_caster_joint',type='fixed'); add(j,'parent','base_link'); add(j,'child',name+'_caster')
    camera = add(base,'sensor',name='camera',type='camera'); add(camera,'pose','0.16 0 0.24 0 0 0')
    add(camera,'always_on','true'); add(camera,'update_rate',10); add(camera,'topic','/camera/image_raw')
    c=add(camera,'camera'); add(c,'horizontal_fov',1.2); im=add(c,'image'); add(im,'width',320); add(im,'height',240); add(im,'format','R8G8B8')
    clip=add(c,'clip'); add(clip,'near',0.03); add(clip,'far',10)
    lidar=add(base,'sensor',name='lidar',type='gpu_lidar'); add(lidar,'pose','0.10 0 0.20 0 0 0')
    add(lidar,'always_on','true'); add(lidar,'update_rate',10); add(lidar,'topic','/scan'); add(lidar,'visualize','false')
    ray=add(lidar,'lidar'); sc=add(ray,'scan'); hz=add(sc,'horizontal')
    add(hz,'samples',360); add(hz,'resolution',1); add(hz,'min_angle',-3.14159265359); add(hz,'max_angle',3.14159265359)
    v=add(sc,'vertical'); add(v,'samples',1); add(v,'resolution',1); add(v,'min_angle',0); add(v,'max_angle',0)
    r=add(ray,'range'); add(r,'min',0.03); add(r,'max',6); add(r,'resolution',0.01)
    drive=add(robot,'plugin',filename='libignition-gazebo-diff-drive-system.so',name='ignition::gazebo::systems::DiffDrive')
    for tag,val in [('left_joint','left_wheel_joint'),('right_joint','right_wheel_joint'),('wheel_separation',0.30),
                    ('wheel_radius',0.06),('topic','/cmd_vel'),('odom_topic','/odom'),('tf_topic','/model/visual_bot/tf'),
                    ('frame_id','odom'),('child_frame_id','base_link'),('odom_publish_frequency',20),
                    ('max_linear_velocity',0.15),('min_linear_velocity',-0.15),('max_angular_velocity',0.8),('min_angular_velocity',-0.8),
                    ('max_linear_acceleration',0.4),('min_linear_acceleration',-0.4)]: add(drive,tag,val)
    ET.indent(sdf,space='  ')
    out=ROOT/'src/pai_robot/worlds/visual_search.sdf'
    ET.ElementTree(sdf).write(out,encoding='utf-8',xml_declaration=True)
    return out
if __name__=='__main__': print(make())
