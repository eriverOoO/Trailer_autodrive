"""Analyze real recorded turns only after explicit user direction confirmation."""
from pathlib import Path
import argparse,json,time,math,statistics
p=argparse.ArgumentParser();p.add_argument('--event',required=True);p.add_argument('--name',required=True);p.add_argument('--direction',choices=['CCW','CW'],required=True);p.add_argument('--physical-confirmed',action='store_true',required=True);a=p.parse_args()
out=Path(__file__).resolve().parent;now=time.time_ns()
events=[json.loads(l) for l in (out/'motion_events.jsonl').read_text().splitlines()]
start=next(e['stamp_ns'] for e in reversed(events) if e['event']==a.event)
rows=[]
with (out/'actual_motion_stream.jsonl').open() as f:
 for line in f:
  try:r=json.loads(line)
  except json.JSONDecodeError:continue
  if start-5_000_000_000<=r['stamp_ns']<=now:rows.append(r)
result={'physical_direction':a.direction,'user_confirmed':True,'start_ns':start,'end_ns':now,'status':'NOT TESTED','note':'Direction evidence only; exact physical angle/mounting remains uncalibrated.'}
for kind in ['raw_imu','arrow','body']:
 stream=sorted((r for r in rows if r['kind']==kind),key=lambda r:r['stamp_ns']);values=[];last=None;value=None
 for r in stream:
  y=r['yaw_rad'];value=y if last is None else value+(y-last+math.pi)%(2*math.pi)-math.pi;last=y;values.append((r['stamp_ns'],value))
 pre=[v for t,v in values if start-5_000_000_000<=t<=start];post=[v for t,v in values if now-3_000_000_000<=t<=now]
 if pre and post:result[kind]={'pre_samples':len(pre),'post_samples':len(post),'delta_deg':math.degrees(statistics.mean(post)-statistics.mean(pre)),'post_span_deg':math.degrees(max(post)-min(post))}
states=[r['value'] for r in rows if r['kind']=='status'];pre=[r for r in states if start-5_000_000_000<=r['stamp_ns']<=start];post=[r for r in states if now-3_000_000_000<=r['stamp_ns']<=now]
if pre and post:
 x0,y0=[statistics.mean(r['display_anchor_xy'][i] for r in pre) for i in range(2)];x1,y1=[statistics.mean(r['display_anchor_xy'][i] for r in post) for i in range(2)]
 result['sensor_anchor_shift_m']=math.hypot(x1-x0,y1-y0)
 result['latest_state']={k:post[-1].get(k) for k in ['imu','slam','diagnostic_valid','paired_age_ms','position_reference','heading_alignment_verified']}
if all(k in result for k in ['raw_imu','arrow','body']):
 expected=1 if a.direction=='CCW' else -1;raw=result['raw_imu']['delta_deg'];arrow=result['arrow']['delta_deg'];body=result['body']['delta_deg']
 result.update(correction_error_arrow_deg=arrow+raw,correction_error_body_deg=body+raw)
 result['status']='PASS' if abs(raw)>=2 and arrow*expected>2 and body*expected>2 and abs(arrow+raw)<2 and abs(body+raw)<2 and result.get('latest_state',{}).get('diagnostic_valid') else 'FAIL_OR_INCONCLUSIVE'
(out/(a.name+'.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2))
phases=json.loads((out/'physical_yaw_tests.json').read_text());phases[a.name]=result;(out/'physical_yaw_tests.json').write_text(json.dumps(phases,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False,indent=2))
