from pathlib import Path
import hashlib,json
W=Path(__file__).resolve().parents[1];O=Path('/home/venezuelachocolatemilk/mvsim_slam');fail=[]
for rel,h in json.loads((W/'source_baseline.json').read_text()).items():
 p=O/rel
 if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=h:fail.append('ORIGINAL '+rel)
manifest=json.loads((W/'protected_manifest.json').read_text())
for rel,h in manifest.items():
 a=W/'src'/rel;b=O/'src_example'/rel
 if not a.is_file() or not b.is_file() or hashlib.sha256(a.read_bytes()).hexdigest()!=h or hashlib.sha256(b.read_bytes()).hexdigest()!=h:fail.append('COPY '+rel)
for p in (W/'src/serial_communication_pkg').rglob('*'):
 if p.is_file() and '__pycache__' not in p.parts and not any(x.endswith('.egg-info') for x in p.parts) and p.suffix!='.pyc' and str(p.relative_to(W/'src')) not in manifest:fail.append('ADDED SERIAL '+str(p))
result={'status':'FAIL' if fail else 'PASS','original_files':len(json.loads((W/'source_baseline.json').read_text())),'protected_copy_files':len(manifest),'failures':fail}
(W/'verification/preservation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));raise SystemExit(bool(fail))
