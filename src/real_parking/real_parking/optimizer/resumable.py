"""Retain every original node/heap/cost across candidate slices; single-thread scheduling."""
import inspect,textwrap,time,math,json,pathlib,os,numpy as np

def install_resume(mod):
 source=textwrap.dedent(inspect.getsource(mod.HybridAStar.plan))
 source=source.replace('def plan(self,start,goal,gears=(1,-1)):','def plan_steps(self,start,goal,gears=(1,-1)):')
 source=source.replace('started=time.monotonic();start=', 'self.active_s=0.;self.paused_s=0.;started=time.monotonic();start=')
 source=source.replace('while heap and time.monotonic()-started<self.timeout:', 'while heap and time.monotonic()-started-self.paused_s<self.timeout:\n        self.active_s=time.monotonic()-started-self.paused_s\n        pause_started=time.monotonic()\n        yield None\n        self.paused_s+=time.monotonic()-pause_started')
 source=source.replace('time.monotonic()-started;', 'time.monotonic()-started-self.paused_s;')
 source=source.replace('{time.monotonic()-started:.1f}', '{time.monotonic()-started-self.paused_s:.1f}')
 env=dict(mod.__dict__);exec(compile(source,'<original_plan_resumable>','exec'),env)
 mod.HybridAStar.plan_steps=env['plan_steps']
 def plan(self,*args,**kw):
  gen=self.plan_steps(*args,**kw)
  while True:
   try:next(gen)
   except StopIteration as done:return done.value
 mod.HybridAStar.plan=plan
 def parking_steps(self,start,goal,road_y=None,staging_distances=(7.,)):
  c,s=math.cos(goal[2]),math.sin(goal[2]);along=np.array([c,s]);normal=np.array([-s,c]);offset=float((start[:2]-goal[:2])@normal) if road_y is None else road_y
  errors=[]
  for ahead in staging_distances:
   staging=goal.copy();staging[:2]+=ahead*along+offset*normal
   try:
    dep=mod.HybridAStar(self.geometry,self.footprints,self.grid,xy_resolution=.10,yaw_resolution=math.radians(3),timeout=self.timeout,articulation_limit_deg=math.degrees(self.articulation_limit),primitive_length=.20)
    self.current_phase=dep;self.phase_name='departure';departure=yield from dep.plan_steps(goal,staging,gears=(1,-1))
    self.completed_expanded=dep.expanded;self.current_phase=self;self.phase_name='approach';approach=yield from self.plan_steps(start,departure.states[-1])
    result=mod.combine(approach,departure.reversed());return result
   except RuntimeError as exc:errors.append(str(exc))
  raise RuntimeError('; '.join(errors))
 mod.HybridAStar.parking_steps=parking_steps

def execution_deadline(root):
 # Only supervisor budget selection; search/candidate/internal time caps unchanged.
 budget_file=pathlib.Path(os.environ.get('RESEARCH_BUDGET_FILE',str(root/'work_state.json')))
 return json.loads(budget_file.read_text())['deadline_monotonic']

def schedule(mod,start,queue,grids,geo,fp,normal,road,timeout=90.,width=3,existing_jobs=None):
 """First 3 original candidates staged at5/40/90 active seconds; rest eventually same budgets.
 No candidate is removed; a paused generator retains its actual search, and original input is immutable.
 """
 jobs=[];events=[];winner=None;campaign=time.monotonic()
 root=pathlib.Path(__file__).resolve().parents[1];deadline=execution_deadline(root)
 for item in ([] if existing_jobs is not None else queue):
  planner=mod.HybridAStar(geo,fp,grids[item['map_variant']],timeout=timeout)
  goal=np.asarray(item['goal']);gen=planner.parking_steps(start,goal,road_y=road-float(goal[:2]@normal),staging_distances=tuple(item.get('staging_distances',(7.,))))
  jobs.append({'item':item,'planner':planner,'generator':gen,'active_s':0.,'cpu_s':0.,'slices':0,'status':'PENDING','expanded':0})
 if existing_jobs is not None:jobs=existing_jobs
 for offset in range(0,len(jobs),width):
  group=jobs[offset:offset+width]
  for cap in getattr(mod,'_resume_checkpoints',(5.,40.,2*timeout+5)):
   for job in group:
    if job['status'] in ['FAILED','FOUND','REJECTED']:continue
    mod._resume_active_job=job
    began=time.monotonic();cpu=time.process_time();job['slices']+=1
    while job['active_s']+time.monotonic()-began<cap:
     if time.monotonic()>deadline-90:
      (root/'interruption.json').write_text(json.dumps({'reason':'research deadline','stage':'resumable offline','monotonic_s':time.monotonic(),'events':events,'running_candidate':job['item'],'status':'INTERRUPTED'},indent=2));raise KeyboardInterrupt('Research deadline')
     try:next(job['generator']);job['status']='PAUSED'
     except StopIteration as done:job['path']=done.value;job['status']='FOUND';winner=job;break
     except RuntimeError as e:job['error']=str(e);job['status']='FAILED';break
    job['active_s']+=time.monotonic()-began;job['cpu_s']+=time.process_time()-cpu
    p=job['planner'];job['expanded']=getattr(p,'completed_expanded',0)+getattr(getattr(p,'current_phase',p),'expanded',0)
    events.append({'index':job['item']['index'],'cap_s':cap,'active_s':job['active_s'],'expanded':job['expanded'],'status':job['status'],'elapsed_s':time.monotonic()-campaign})
    checkpoint=getattr(mod,'_resume_checkpoint_path',None)
    if checkpoint:pathlib.Path(checkpoint).write_text(json.dumps({'status':'RUNNING','events':events,'elapsed_s':time.monotonic()-campaign},indent=2))
    if winner:break
   if winner:break
  if winner:break
 return winner,jobs,events
