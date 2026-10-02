"""Install the official NVIDIA Windows archive locally; no global driver changes."""
from pathlib import Path
import hashlib
import json
import os
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
VERSION = '5.1.0'
URL = f'https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-{VERSION}-windows-x86_64.zip'


def main():
    runtime = ROOT/'.runtime'
    if (runtime/'python.bat').exists() and (runtime/'installation.json').exists():
        print('Isaac Sim is already installed.', flush=True)
        return
    archive = ROOT/'.downloads'/URL.rsplit('/', 1)[1]
    archive.parent.mkdir(exist_ok=True)
    if not zipfile.is_zipfile(archive):
        offset = archive.stat().st_size if archive.exists() else 0
        request = urllib.request.Request(URL, headers={'Range': f'bytes={offset}-'})
        with urllib.request.urlopen(request, timeout=60) as response:
            append = response.status == 206 and offset > 0
            with archive.open('ab' if append else 'wb') as output:
                while True:
                    chunk = response.read(8*1024*1024)
                    if not chunk:
                        break
                    output.write(chunk)
                    print(f'Downloaded {output.tell()/1e9:.2f} GB', flush=True)
    runtime.mkdir(exist_ok=True)
    print('Extracting Isaac Sim...', flush=True)
    with zipfile.ZipFile(archive) as bundle:
        for index, member in enumerate(bundle.infolist()):
            target = (runtime/member.filename).resolve()
            if not target.is_relative_to(runtime.resolve()):
                raise ValueError('Archive path escaped the runtime directory')
            # Extended paths also work on machines without the system long-path policy.
            path = '\\\\?\\'+str(target) if os.name == 'nt' else str(target)
            if member.is_dir():
                os.makedirs(path, exist_ok=True)
            else:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with bundle.open(member) as source, open(path, 'wb') as output:
                    import shutil
                    shutil.copyfileobj(source, output, 8*1024*1024)
            if index % 10000 == 0:
                print(f'Extracted {index}/{len(bundle.infolist())} entries', flush=True)
    if not (runtime/'python.bat').exists():
        raise RuntimeError('Archive extracted, but the expected python.bat is missing')
    digest = hashlib.sha256()
    with archive.open('rb') as stream:
        for chunk in iter(lambda: stream.read(8*1024*1024), b''):
            digest.update(chunk)
    (runtime/'installation.json').write_text(json.dumps({'version': VERSION, 'url': URL,
                                                       'archive_sha256': digest.hexdigest()}, indent=2))
    print('Isaac Sim installed.', flush=True)


if __name__ == '__main__':
    main()
