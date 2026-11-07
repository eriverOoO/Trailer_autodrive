"""B: complete current CDE phase, then rank the available feasible paths."""
import time,json,pathlib,queue as queues,multiprocessing as mp,numpy as np
from resumable import execution_deadline
SLICE_S=5.  # Phases after the first alternate candidates instead of running each to its cap.
GRACE_ROUNDS=1  # Rounds granted to the other candidates after the first FOUND.
PARALLEL_GRACE_S=5.  # Wall seconds the other candidates of a group get after the first FOUND.

def _candidate_worker(item,start,grid,geo,fp,normal,road,timeout,results):
 """Spawned process: one complete departure+approach search for one candidate."""
 import os,importlib
 # The real vehicle installs B through real_parking.bootstrap, not environment profiles.
 module,function=os.getenv('B_WORKER_INSTALLER','real_parking.bootstrap:install_b').split(':')
 mod=getattr(importlib.import_module(module),function)()
 planner=mod.HybridAStar(geo,fp,grid,timeout=timeout);goal=np.asarray(item['goal']);began=time.monotonic()
 try:
  gen=planner.parking_steps(start,goal,road_y=road-float(goal[:2]@normal),staging_distances=tuple(item.get('staging_distances',(7.,))))
  while True:
   try:next(gen)
   except StopIteration as done:path=done.value;break
  results.put((item['index'],'FOUND',path,'',getattr(planner,'completed_expanded',0)+planner.expanded,time.monotonic()-began))
 except RuntimeError as e:
  results.put((item['index'],'FAILED',None,str(e),getattr(planner,'completed_expanded',0)+planner.expanded,time.monotonic()-began))

def schedule(mod,start,queue,grids,geo,fp,normal,road,timeout=90.,width=3,existing_jobs=None):
 """Same candidate groups and ranking as schedule_sliced, but each group searches in parallel processes.
 A group ends when every candidate finished, or PARALLEL_GRACE_S after its first FOUND.
 Cancelled candidates restart from scratch if the caller resumes the schedule.
 """
 jobs=[];events=[];winner=None;campaign=time.monotonic()
 root=pathlib.Path(__file__).resolve().parents[1];deadline=execution_deadline(root)
 for item in ([] if existing_jobs is not None else queue):
  jobs.append({'item':item,'planner':mod.HybridAStar(geo,fp,grids[item['map_variant']],timeout=timeout),'grid':grids[item['map_variant']],'active_s':0.,'cpu_s':0.,'slices':0,'status':'PENDING','expanded':0})
 if existing_jobs is not None:jobs=existing_jobs
 context=mp.get_context('spawn');hard_cap=2*timeout+15.
 for offset in range(0,len(jobs),width):
  group=[j for j in jobs[offset:offset+width] if j['status'] in ['PENDING','CANCELLED','PAUSED']]
  if not group:
   found=[j for j in jobs[offset:offset+width] if j['status']=='FOUND']
   if found:winner=min(found,key=lambda j:j['planner'].priority_rank(j['path']));break
   continue
  results=context.Queue();running={}
  for job in group:
   proc=context.Process(target=_candidate_worker,args=(job['item'],start,job['grid'],geo,fp,normal,road,timeout,results),daemon=True)
   proc.start();job['status']='RUNNING';job['slices']+=1;running[job['item']['index']]=(job,proc,time.monotonic())
  first_found=None
  try:
   while running:
    if time.monotonic()>deadline-90:
     (root/'interruption.json').write_text(json.dumps({'reason':'research deadline','stage':'parallel candidates','monotonic_s':time.monotonic(),'events':events,'status':'INTERRUPTED'},indent=2));raise KeyboardInterrupt('Research deadline')
    try:index,status,path,error,expanded,active=results.get(timeout=.2)
    except queues.Empty:
     now=time.monotonic()
     if first_found is not None and now-first_found>=PARALLEL_GRACE_S:break
     for index,(job,proc,began) in list(running.items()):
      if not proc.is_alive() or now-began>hard_cap:
       job['status']='FAILED';job['error']='candidate process ended without result' if not proc.is_alive() else 'candidate process wall cap';running.pop(index)
     continue
    job,proc,began=running.pop(index);job['status']=status;job['expanded']=expanded;job['active_s']+=active
    if path is not None:job['path']=path
    if error:job['error']=error
    events.append({'index':index,'status':status,'active_s':active,'expanded':expanded,'elapsed_s':time.monotonic()-campaign})
    if status=='FOUND' and first_found is None:first_found=time.monotonic()
  finally:
   for job,proc,began in running.values():
    proc.terminate();proc.join(5);job['status']='CANCELLED'
    events.append({'index':job['item']['index'],'status':'CANCELLED','elapsed_s':time.monotonic()-campaign})
   for job in group:
    if job['status']=='RUNNING':job['status']='CANCELLED'
  found=[j for j in jobs[offset:offset+width] if j['status']=='FOUND']
  if found:
   winner=min(found,key=lambda j:j['planner'].priority_rank(j['path']));break
 return winner,jobs,events

def schedule_sliced(mod,start,queue,grids,geo,fp,normal,road,timeout=90.,width=3,existing_jobs=None):
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
 caps=tuple(getattr(mod,'_resume_checkpoints',(5.,40.,2*timeout+5)))
 for offset in range(0,len(jobs),width):
  group=jobs[offset:offset+width]
  for cap in caps:
   sliced=cap!=caps[0];grace=None
   while True:
    ran=False
    for job in group:
     if job['status'] in ['FAILED','FOUND','REJECTED'] or job['active_s']>=cap:continue
     limit=min(cap,job['active_s']+SLICE_S) if sliced else cap
     mod._resume_active_job=job;ran=True
     began=time.monotonic();cpu=time.process_time();job['slices']+=1
     while job['active_s']+time.monotonic()-began<limit:
      if time.monotonic()>deadline-90:
       (root/'interruption.json').write_text(json.dumps({'reason':'research deadline','stage':'resumable offline','monotonic_s':time.monotonic(),'events':events,'running_candidate':job['item'],'status':'INTERRUPTED'},indent=2));raise KeyboardInterrupt('Research deadline')
      try:next(job['generator']);job['status']='PAUSED'
      except StopIteration as done:job['path']=done.value;job['status']='FOUND';break
      except RuntimeError as e:job['error']=str(e);job['status']='FAILED';break
     job['active_s']+=time.monotonic()-began;job['cpu_s']+=time.process_time()-cpu
     p=job['planner'];job['expanded']=getattr(p,'completed_expanded',0)+getattr(getattr(p,'current_phase',p),'expanded',0)
     events.append({'index':job['item']['index'],'cap_s':cap,'active_s':job['active_s'],'expanded':job['expanded'],'status':job['status'],'elapsed_s':time.monotonic()-campaign})
     checkpoint=getattr(mod,'_resume_checkpoint_path',None)
     if checkpoint:pathlib.Path(checkpoint).write_text(json.dumps({'status':'RUNNING','events':events,'elapsed_s':time.monotonic()-campaign},indent=2))
    if not sliced or not ran:break
    if grace is None and any(j['status']=='FOUND' for j in group):grace=GRACE_ROUNDS
    elif grace is not None:
     grace-=1
     if grace<=0:break
   found=[j for j in jobs if j['status']=='FOUND']
   if found:
    winner=min(found,key=lambda j:j['planner'].priority_rank(j['path']));break
  if winner:break
 return winner,jobs,events
