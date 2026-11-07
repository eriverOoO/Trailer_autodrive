from pathlib import Path
import sys
W=Path(__file__).resolve().parents[1]
for name in ['ebimu_driver','real_vehicle_bridge']:sys.path.insert(0,str(W/'src'/name))
