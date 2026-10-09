"""Cruise gate rejects nearby cargo/obstacles and non-flat travel."""
import xml.etree.ElementTree as ET
from app import Simulation, ROOT
from slope_assembly import assembly_xml
from navigation import Navigator
root=ET.fromstring(assembly_xml(ROOT/'scene_fork_tracks.xml'))
world=root.find('worldbody');world.find("body[@name='dozer']").set('pos','-4 -3.2 .3')
sim=Simulation(scene_xml=ET.tostring(root,encoding='unicode'))
p=Navigator(sim,[[1,-3.2]],fast_empty=True)
assert p.cruise_clear(sim)
# Object directly in front, representing a pickup/carry zone. No scene motion.
ET.SubElement(world,'geom',name='test_cargo',type='box',pos='-2.7 -3.2 .5',size='.3 .3 .3')
sim2=Simulation(scene_xml=ET.tostring(root,encoding='unicode'))
assert not Navigator(sim2,[[1,-3.2]],fast_empty=True).cruise_clear(sim2)
# Explicit raised vehicle fixture: cruise must not activate on a ramp.
sim.data.qpos[2]+=.2;sim.observe()
assert not p.cruise_clear(sim)
print('PASS: clear flat corridor, nearby cargo rejection, raised vehicle rejection')
