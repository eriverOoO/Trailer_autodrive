"""Optional E integration: immutable same candidate snapshots, persistent search jobs.
Original latest RAW precheck and READY/execution guards still perform acceptance.
"""
import json,time,numpy as np
from pathlib import Path

def install(cls):
 original=cls.submit_plan_candidates
 def submit(self):
  context=self.plan_context
  if context['survey']:return original(self)
  def work():
   import tractor_trailer_hybrid_astar as mod
   from resumable import schedule
   if getattr(mod,'_steering_priority',False):
    from priority_schedule import schedule
   queue=[];grids={}
   for i,(grid,goal,shift,lateral) in enumerate(context['attempts']):
    key=str(id(grid));grids[key]=grid;item={'index':i,'map_variant':key,'goal':goal.tolist(),'shift':shift,'lateral':lateral}
    if getattr(self,'stop_policy','legacy')=='observed_safe':
     from observed_stop import observed_staging_distance
     ahead=observed_staging_distance(grid,goal,float(self.road_pose[:2]@context['normal']),self.footprints)
     # No geometry veto is turned into a feasible endpoint. Retain the job
     # for its normal4D failure if the observed-safe endpoint is absent.
     item['staging_distances']=[ahead] if ahead is not None else [7.]
     item['staging_source']='farthest actually observed-safe whole-rig endpoint within original7m extent' if ahead is not None else 'no observed-safe endpoint; original solver must fail safely'
    queue.append(item)
   out=Path(self.get_parameter('blocked_debug_directory').value);beg=time.monotonic()
   while True:
    winner,jobs,events=schedule(mod,context['start'],queue,grids,self.geometry,self.footprints,context['normal'],float(self.road_pose[:2]@context['normal']),timeout=context['timeout'],existing_jobs=context.get('resume_jobs'))
    context['resume_jobs']=jobs
    if getattr(mod,'_steering_priority',False):
     # Hard-filter every available candidate on the same latest RAW grid, then rank.
     import tractor_parallel_parking_auto as auto
     raw_grid=self.grid(False);ranker=mod.HybridAStar(self.geometry,self.footprints,raw_grid,timeout=context['timeout']);valid=[]
     for job in jobs:
      if job['status']!='FOUND':continue
      metrics=auto.path_metrics(raw_grid,job['path'].states,self.footprints)
      if not metrics['valid']:job['status']='REJECTED';continue
      job['priority_key']=ranker.priority_rank(job['path']);valid.append(job)
     winner=min(valid,key=lambda j:j['priority_key']) if valid else None
     with (out/'steering_priority_candidates.jsonl').open('a') as f:
      f.write(json.dumps({'elapsed_s':time.monotonic()-beg,'hard_RAW_filtered':True,'available':[{'index':j['item']['index'],'goal':j['item']['goal'],'rank':j['priority_key'],'expanded':j['expanded']} for j in valid],'chosen':winner['item']['index'] if winner else None})+'\n')
     if winner is None and any(j['status'] not in ['FAILED','REJECTED'] for j in jobs):continue
    with (out/'candidate_scheduler.jsonl').open('a') as f:
     f.write(json.dumps({'event':'schedule_end','monotonic_s':time.monotonic(),'elapsed_s':time.monotonic()-beg,'events':events,'winner':winner['item'] if winner else None})+'\n')
    if winner is None:raise RuntimeError('candidate search exhausted with preserved resumable search states')
    idx=winner['item']['index'];grid,candidate,shift,lateral=context['attempts'][idx];context['next_index']=idx+1;path=winner['path']
    # The original RAW precheck is retained exactly before a path can be returned.
    import tractor_parallel_parking_auto as auto
    raw=auto.path_metrics(self.grid(False),path.states,self.footprints)
    if not raw['valid']:
     winner['status']='REJECTED';context['failures'].append(f'goal shift={shift:.2f}, lateral={lateral:.2f}: latest RAW candidate precheck rejected');continue
    # A later original READY rejection also resumes the retained other jobs.
    return path,grid,grid is context['filled'],candidate,shift,lateral
  self.future=self.planner_pool.submit(work)
 cls.submit_plan_candidates=submit
