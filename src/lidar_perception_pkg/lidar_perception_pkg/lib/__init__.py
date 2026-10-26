import importlib.util
import os
import marshal
import types


def _module_dir():
    return os.path.dirname(os.path.abspath(__file__))


def _load_pyc(module_path, module_name):
    with open(module_path, 'rb') as f:
        pyc = f.read()

    code = marshal.loads(pyc[16:])
    module = types.ModuleType(module_name)
    module.__file__ = module_path
    exec(code, module.__dict__)
    return module


def _load_py(module_path, module_name):
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_module(module_name, candidates):
    base_dir = _module_dir()
    for candidate in candidates:
        module_path = os.path.join(base_dir, candidate)
        if not os.path.exists(module_path):
            continue
        if module_path.endswith('.pyc'):
            return _load_pyc(module_path, module_name)
        return _load_py(module_path, module_name)

    raise FileNotFoundError(
        f"Could not find any module candidate for {module_name}: {candidates}"
    )


lidar_perception_func_lib = _load_module(
    'lidar_perception_func_lib',
    [
        'lidar_perception_func_lib.py',
        'lidar_perception_func_lib.cpython-310.pyc',
    ],
)
