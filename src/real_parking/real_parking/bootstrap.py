"""Only outside adaptation; Final B vendor and optimizer source stays byte-identical."""
from pathlib import Path
import sys

def load():
    root=Path(__file__).absolute().parent
    for folder in ['vendor','optimizer','support']:
        path=str(root/folder)
        if path not in sys.path:sys.path.insert(0,path)

def install_b():
    load()
    import tractor_trailer_hybrid_astar as mod
    if not getattr(mod,'_real_b_installed',False):
        from variants import install
        install(mod,'clearance+shot+resume20')
        from steering_priority import install as priority
        priority(mod)
        from tractor_parallel_parking_auto import AutoParking
        from candidate_scheduler_online import install as candidates
        candidates(AutoParking)
        mod._real_b_installed=True
    return mod
