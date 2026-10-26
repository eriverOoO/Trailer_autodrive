#!/usr/bin/env python3
"""Analyze saved measurements only. No ROS publishers or configuration changes."""
from pathlib import Path
import json,gzip,math,csv
import numpy as np
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

b=Path(__file__).parent;folder=b/'baseline';scans=[]
with gzip.open(folder/'scan_raw.jsonl.gz','rt') as f:
 for line in f:scans.append(json.loads(line))
last=scans[-1];ranges=np.asarray(last['ranges']);ang=last['angle_min']+np.arange(len(ranges))*last['angle_increment'];valid=np.isfinite(ranges)&(ranges>=last['range_min'])&(ranges<=last['range_max']);xy=np.array([ranges[valid]*np.cos(ang[valid]),ranges[valid]*np.sin(ang[valid])]).T
poses=json.loads((folder/'poses.json').read_text());p=min(poses,key=lambda p:abs(p['elapsed_s']-last['elapsed_s']));yaw=math.radians(p['rpy_deg'][2]);R=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]]);world=xy@R.T+np.asarray(p['xyz'][:2])
maps=json.loads((folder/'maps.json').read_text());m=maps[-1];data=np.load(folder/f'map_{len(maps)-1:03d}.npz')['data'];res=m['resolution'];origin=np.asarray(m['origin_xyz'][:2]);ix=np.floor((world-origin)/res).astype(int);inside=(ix[:,0]>=0)&(ix[:,0]<data.shape[1])&(ix[:,1]>=0)&(ix[:,1]<data.shape[0]);cells=ix[inside];occupied=data>=65;distance=distance_transform_edt(~occupied,sampling=res);dist=distance[cells[:,1],cells[:,0]];occ=data[cells[:,1],cells[:,0]]
def stats(a):return {'min':float(a.min()),'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(a.max())} if len(a) else None
alignment={'method':'Last scan, nearest observed scan-start map-to-laser pose; no per-beam motion compensation. Last map snapshot. Diagnostic association, not ground truth or a calibrated mounting angle.','scan_stamp_ns':last['stamp_ns'],'map_stamp_ns':m['stamp_ns'],'scan_minus_map_stamp_s':(last['stamp_ns']-m['stamp_ns'])/1e9,'pose_receipt_time_difference_s':p['elapsed_s']-last['elapsed_s'],'pose_yaw_deg':p['rpy_deg'][2],'valid_points':len(xy),'points_inside_map':int(inside.sum()),'nearest_occupied_ge65_m':stats(dist),'fraction_within_0p10m_of_occupied_ge65':float(np.mean(dist<=.10)),'fraction_within_0p20m_of_occupied_ge65':float(np.mean(dist<=.20)),'endpoint_unknown_fraction':float(np.mean(occ<0)),'map_resolution_m':res}
(b/'scan_map_alignment.json').write_text(json.dumps(alignment,indent=2)+'\n')
np.savez_compressed(b/'last_scan_xy_and_map_xy.npz',laser_xy=xy,map_xy=world)
all_bins={k:{'valid':0,'total':0,'zero':0,'values':[]} for k in range(-180,180)}
for s in scans:
 a=np.asarray(s['ranges']);degrees=np.degrees(s['angle_min']+np.arange(len(a))*s['angle_increment']);bins=(np.floor(degrees+.5).astype(int)+180)%360-180;v=np.isfinite(a)&(a>=s['range_min'])&(a<=s['range_max'])
 for i,k in enumerate(bins):
  row=all_bins[int(k)];row['total']+=1;row['zero']+=int(a[i]==0)
  if v[i]:row['valid']+=1;row['values'].append(a[i])
with (b/'angle_distribution.csv').open('w') as f:
 writer=csv.writer(f);writer.writerow(['angle_deg','total_bins','valid_bins','zero_bins','valid_ratio','median_range_m'])
 for k,row in all_bins.items():writer.writerow([k,row['total'],row['valid'],row['zero'],row['valid']/row['total'] if row['total'] else '',float(np.median(row['values'])) if row['values'] else ''])
fig,axs=plt.subplots(1,2,figsize=(13,6));axs[0].scatter(xy[:,0],xy[:,1],s=2,c='red');axs[0].axhline(0,c='gray',lw=.5);axs[0].axvline(0,c='gray',lw=.5);axs[0].arrow(0,0,1,0,color='red',head_width=.12);axs[0].arrow(0,0,0,1,color='green',head_width=.12);axs[0].set(xlabel='laser_frame X (m)',ylabel='laser_frame Y (m)',title='Actual raw LaserScan\nPhysical cable heading is unconfirmed');axs[0].axis('equal');axs[0].grid(alpha=.2)
masked=np.ma.masked_where(data<0,data);extent=[origin[0],origin[0]+data.shape[1]*res,origin[1],origin[1]+data.shape[0]*res];axs[1].set_facecolor('#708986');axs[1].imshow(masked,origin='lower',extent=extent,cmap='gray_r',vmin=0,vmax=100);axs[1].scatter(world[:,0],world[:,1],s=2,c='red');axs[1].set(xlabel='map X (m)',ylabel='map Y (m)',title='OccupancyGrid + raw endpoints\nUsing observed scan-start pose');axs[1].set_aspect('equal');fig.tight_layout();fig.savefig(b/'scan_map_analysis.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(2,1,figsize=(12,6),sharex=True);axs[0].scatter(np.degrees(ang[valid]),ranges[valid],s=3,c='red');axs[0].axhline(1,c='gray',ls='--',label='1 m reference (not an identified target)');axs[0].set(ylabel='Valid range (m)',title='Recorded raw range vs ROS angle');axs[0].legend();axs[0].grid(alpha=.2);axs[1].plot(list(all_bins),[row['valid']/row['total'] if row['total'] else 0 for row in all_bins.values()]);axs[1].set(xlabel='ROS LaserScan angle (degrees)',ylabel='Valid fraction',xlim=(-180,180),ylim=(0,1.05));axs[1].set_xticks([-180,-90,0,90,180]);axs[1].grid(alpha=.2);fig.tight_layout();fig.savefig(b/'angle_distribution.png',dpi=160);plt.close(fig)
# Measured transforms, camera and screen relationship. Cable offset is deliberately unknown.
tables={}
for label in ['baseline','late_observation']:
 s=json.loads((b/label/'summary.json').read_text());rows=json.loads((b/label/'tf_samples.json').read_text());edges={}
 for parent,child in sorted(set((r['parent'],r['child']) for r in rows)):
  r=[x for x in rows if x['parent']==parent and x['child']==child];a=np.asarray([x['rpy_deg'] for x in r]);edges[parent+' -> '+child]={'rpy_deg_min':a.min(axis=0).tolist(),'rpy_deg_median':np.median(a,axis=0).tolist(),'rpy_deg_max':a.max(axis=0).tolist(),'rpy_deg_last':r[-1]['rpy_deg'],'xyz_last':r[-1]['xyz']}
 tables[label]={'dynamic_edges':edges,'map_to_laser_yaw_deg':s['map_to_laser_yaw_deg'],'camera_angle_deg':-90,'laser_x_screen_deviation_from_up_deg':s['map_to_laser_yaw_deg'],'physical_cable_offset_deg':None,'physical_left_right_verified':False}
(b/'orientation_analysis.json').write_text(json.dumps(tables,indent=2)+'\n')
print(json.dumps({'alignment':alignment,'orientation':tables},indent=2))
