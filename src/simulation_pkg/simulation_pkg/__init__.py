import os
import types
import marshal
import importlib.util

def get_pyc(file_name):
    file_path = os.path.join(os.path.dirname(__file__), 'lib', 'pyc', file_name)
    pyc = open(file_path, 'rb').read()
    code = marshal.loads(pyc[16:])
    module = types.ModuleType('module_name')
    exec(code, module.__dict__)
    return module

def get_py(file_name):
    file_path = os.path.join(os.path.dirname(__file__), 'lib', file_name)

    spec = importlib.util.spec_from_file_location("module_name", file_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def __getattr__(name):
    # Towing loader does not need the legacy numerical/bytecode libraries.
    if name == 'basic':
        value = get_py('012_deploy_lib.py')
        globals()['basic'] = value
        return value
    raise AttributeError(name)


__all__ = ['basic', 'get_py', 'get_pyc']
