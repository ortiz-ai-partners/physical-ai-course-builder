"""Matched physical construction trials; fast mode never applies while carrying."""
import json
import numpy as np
from app import Simulation,ROOT
from slope_assembly import SlopeAssembly,assembly_xml
reports=[]
for target in (1.,1.5,2.):
 sim=Simulation(scene_xml=assembly_xml(ROOT/'scene_fork_tracks.xml'))
 assert SlopeAssembly(sim,target).fast_empty==(target in (1.,1.5))
for target in (1.,1.5,2.):
 for fast in (False,True):
  sim=Simulation(scene_xml=assembly_xml(ROOT/'scene_fork_tracks.xml'));p=SlopeAssembly(sim,target,fast_empty=fast)
  while not p.done:
   before=sim.data.qpos.copy();a=p.action(sim);assert np.array_equal(before,sim.data.qpos)
   if abs(a[0])>1: assert p.phase in ('TO_DOWN','TO_START')
   sim.step(*a)
  errors={n:float(np.linalg.norm(sim.data.body(n).xpos[:2]-[target,y])) for n,y in [('up_1',-1),('down_1',1.05)]}
  row=dict(target=target,fast_empty=fast,success=p.success,reason=p.reason,seconds=round(sim.data.time,2),fast_steps=p.fast_steps,empty_peak_kmh=p.empty_peak*3.6,errors=errors)
  print(row,flush=True);reports.append(row)
  assert p.success and max(errors.values())<.06,row
  if fast:
   assert p.fast_steps>0 and p.empty_peak*3.6>3.,row
   row['seconds_saved']=round(reports[-2]['seconds']-row['seconds'],2)
   if target in (1.,1.5):assert row['seconds_saved']>0,row
(ROOT/'.test-results/fast-construction.json').write_text(json.dumps(reports,indent=2))
