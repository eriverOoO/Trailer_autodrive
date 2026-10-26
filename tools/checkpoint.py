from pathlib import Path
import sys, json, datetime
W=Path(__file__).resolve().parents[1]
stage,percent=int(sys.argv[1]),int(sys.argv[2]);note=' '.join(sys.argv[3:])
p=W/'progress.json';s=json.loads(p.read_text()) if p.exists() else {'stages':[0]*6,'history':[]}
s['stages'][stage-1]=percent;s['stage']=stage;s['overall']=sum(a*b/100 for a,b in zip(s['stages'],[15,15,20,20,20,10]));s['history'].append({'time':datetime.datetime.now().isoformat(),'stage':stage,'percent':percent,'note':note});p.write_text(json.dumps(s,indent=2))
text=f"# mvsim_slam_real — Progress\n\n## Current Status\nOverall: {s['overall']:g}%\nCurrent Stage: {stage}/6\nStage Progress: {percent}%\n\n## Completed Stages\n"+'\n'.join(f'- Stage {i+1}: {v}%' for i,v in enumerate(s['stages']))
text+='\n\n## Completed Work / checkpoints\n'+'\n'.join(f"- {h['time']} Stage {h['stage']} {h['percent']}%: {h['note']}" for h in s['history'])
text+='\n\n## Created Files\n'+'\n'.join('- '+str(x.relative_to(W)) for x in sorted(W.glob('*.md')))
text+='''

## Architecture Decisions
See PORTING_MAP.md, ARCHITECTURE.md, HARDWARE_INTERFACES.md. Preserve Final B vendor/optimizer algorithms; new ROS hardware adapters. Rear axle planning point. No simulated truth. Separate IMU ports. Hardware disabled in development mode.

## Protected Files
Entire original ~/mvsim_slam is read-only. Copied serial_communication_pkg, interfaces_pkg definitions, control/**/*.ino and original vision/LiDAR packages remain byte-identical, excluding caches/build outputs. tools/verify_preservation.py uses source_baseline.json and protected_manifest.json.

## Verification
See verification/ for actual command logs/results. No hardware verification performed. Source/copy hashes must pass at completion.

## Unresolved Problems
Unmeasured geometry/motor scale/polarities/IMU axes and yaw alignment require actual measurement. Original firmware has no receive watchdog, encoder feedback or independent cutoff. Current ROS Python has no torch, so optional YOLO runtime cannot start; model weights also require provisioning. Original setup entries lidar_obstacle_detector_node/traffic_light_detector_node have missing modules. Core LiDAR software is independent. Hardware and actual IMU fusion remain unverified. See README.

## Current Work
'''+note+'''

## Next Actions
1. Read this file, progress.json, documents and inspect actual src/ and verification/ files before resuming.
2. Continue incomplete stages; do not repeat completed source analysis.
3. Build: cd ~/mvsim_slam_real && source /opt/ros/humble/setup.bash && colcon build --symlink-install
4. Tests: source install/setup.bash && PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest tests -q
5. Preservation: /usr/bin/python3 tools/verify_preservation.py

## Restrictions
No original edits, protected copy edits, deletions/moves of old data, git push, global ROS/Python changes, hardware connection/test/calibration or firmware upload. Default enable_actuation=false. Never claim hardware PASS or measured geometry.
'''
(W/'PROGRESS.md').write_text(text+'\n')
