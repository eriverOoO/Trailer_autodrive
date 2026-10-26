#!/usr/bin/python3
"""Restore only this change, refusing to overwrite later source modifications."""
from pathlib import Path
import hashlib,json,shutil
out=Path(__file__).resolve().parent
rows=json.loads((out/'source_change_manifest.json').read_text())
for row in rows:
 p=Path(row['path'])
 if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256_after']:
  raise SystemExit('Refusing rollback: file changed after verification: '+str(p))
for row in rows:
 p=Path(row['path'])
 if row['added']:
  relative=p.relative_to('/home/venezuelachocolatemilk/mvsim_slam_real/visualization/tractor_trailer_rviz_2d')
  dest=out/'reverted_added_files'/relative;dest.parent.mkdir(parents=True,exist_ok=True);shutil.move(str(p),str(dest))
 else:shutil.copy2(row['backup'],p)
print('Visualization source restored. Rebuild only tractor_trailer_rviz_2d; G6/SLAM untouched.')
