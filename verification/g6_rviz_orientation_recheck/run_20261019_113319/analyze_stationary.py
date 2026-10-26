#!/usr/bin/env python3
from pathlib import Path
import json,math
import numpy as np
b=Path(__file__).parent;s=json.loads((b/'baseline/summary.json').read_text());scans=json.loads((b/'baseline/scan_summaries.json').read_text());poses=json.loads((b/'baseline/poses.json').read_text());steady=[x for x in scans if x['elapsed_s']>=5];p=[x for x in poses if x['elapsed_s']>=5]
def stats(a):
 a=np.asarray(a,dtype=float);return {'min':float(a.min()),'median':float(np.median(a)),'max':float(a.max()),'std':float(a.std())} if len(a) else None
span=steady[-1]['wall']-steady[0]['wall'] if len(steady)>1 else 0
out={'physical_fixation_confirmed_by_user':True,'data_scope':'Fresh Cartographer session; 0.15-1.0 m software range, not the previous 15 m baseline','warmup_excluded_s':5,'steady_scan_count':len(steady),'steady_scan_span_s':span,'at_least_60s_observed':span>=60,'steady_scan_hz':(len(steady)-1)/span if span else None,'scan_count':s['scan_count'],'scan_hz':s['scan_hz'],'valid_beams':s['valid_beams'],'valid_fraction':s['valid_fraction'],'map_count':s['map_count'],'map_hz':s['map_hz'],'matched_cloud_count':s['matched_cloud_count'],'matched_cloud_hz':s['matched_cloud_hz'],'submap_list_count':s['submap_list_count'],'tf_edges':s['tf_edges'],'static_tf':s['static_tf'],'tf_failures_after_warmup':[x for x in s['tf_lookup_failures'] if x['elapsed_s']>=5],'range_min_m':s['first_scan']['range_min'] if s['first_scan'] else None,'range_max_m':s['first_scan']['range_max'] if s['first_scan'] else None,'physical_target_direction_verified':False}
if p:
 xy=np.asarray([x['xyz'][:2] for x in p]);yaws=np.degrees(np.unwrap(np.radians([x['rpy_deg'][2] for x in p])));out['estimated_xy_change']={'max_distance_from_first_m':float(np.max(np.linalg.norm(xy-xy[0],axis=1))),'x_stats_m':stats(xy[:,0]),'y_stats_m':stats(xy[:,1])};out['estimated_yaw_deg']=stats(yaws);out['estimated_yaw_peak_to_peak_deg']=float(yaws.max()-yaws.min())
lines=(b/'launch.log').read_text(errors='replace').splitlines();out['log_counts_at_analysis']={key:sum(key in line for line in lines) for key in ['[ERROR]','[FATAL]','[WARN]','Checksum error','Dropped']}
(b/'stationary_result.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
