from pathlib import Path
import sys
W=Path(__file__).resolve().parents[1]
for name in ['ebimu_driver','real_vehicle_bridge']:sys.path.insert(0,str(W/'src'/name))


def pytest_configure(config):
    config.addinivalue_line('markers', 'planning: slow 4D Hybrid A* planning scenarios (deselect with -m "not planning")')
