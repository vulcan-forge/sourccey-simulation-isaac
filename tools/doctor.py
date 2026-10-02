"""Record local prerequisites without importing or starting Isaac Sim."""
from pathlib import Path
import ctypes
import json
import platform
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


class Memory(ctypes.Structure):
    _fields_ = [('length', ctypes.c_ulong), ('load', ctypes.c_ulong)] + [
        (n, ctypes.c_ulonglong) for n in ('totalPhysical', 'availablePhysical', 'totalPage',
                                         'availablePage', 'totalVirtual', 'availableVirtual', 'extended')]


def main():
    memory = Memory(); memory.length = ctypes.sizeof(memory)
    ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(memory))
    result = subprocess.run(['nvidia-smi', '--query-gpu=name,driver_version,memory.total,memory.used',
                             '--format=csv,noheader,nounits'], capture_output=True, text=True)
    report = {'platform': platform.platform(), 'ram_gb': memory.totalPhysical/2**30,
              'ram_available_gb': memory.availablePhysical/2**30,
              'disk_free_gb': shutil.disk_usage(ROOT).free/2**30,
              'gpu': result.stdout.strip(), 'runtime_installed': (ROOT/'.runtime/python.bat').exists(),
              'target_runtime': 'Isaac Sim 5.1.0',
              'hardware_limit': 'Local 8 GB GPU is below the published 16 GB minimum.'}
    (ROOT/'docs').mkdir(exist_ok=True)
    (ROOT/'docs/machine_report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
