#!/usr/bin/env python3
"""Download a commit-pinned, SHA256-verified source archive. Never overwrite."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import tarfile
import tempfile
import urllib.request


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive',help='Use an already downloaded archive; SHA256 is still checked')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    lock=json.loads((root/'src/realflight_modules/openvins_bridge/config/upstream.json').read_text())
    dest=root/'src/third_party/open_vins'
    marker=dest/'.grampus_upstream.json'
    if dest.exists():
        if marker.is_file() and json.loads(marker.read_text()).get('commit')==lock['commit']:
            print('Pinned source already exists; preserving all local files:',dest)
            return
        if not (dest/'.git').is_dir() or any(p.name!='.git' for p in dest.iterdir()):
            raise SystemExit('Destination already contains unverified files; inspect manually: '+str(dest))
        head=subprocess.run(['git','-C',str(dest),'rev-parse','--verify','HEAD'],capture_output=True)
        origin=subprocess.run(['git','-C',str(dest),'remote','get-url','origin'],capture_output=True,text=True)
        if head.returncode==0 or origin.returncode or origin.stdout.strip()!=lock['repository']:
            raise SystemExit('Existing checkout is not an empty matching bootstrap; no files changed')
        incomplete_git=dest/'.git'
    else:
        incomplete_git=None
    dest.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.openvins-',dir=str(dest.parent)) as temp:
        temp=Path(temp)
        archive=Path(args.archive) if args.archive else temp/'source.tar.gz'
        if not args.archive:
            url='https://codeload.github.com/rpng/open_vins/tar.gz/'+lock['commit']
            with urllib.request.urlopen(url,timeout=90) as response,archive.open('wb') as output:
                shutil.copyfileobj(response,output)
        digest=hashlib.sha256()
        with archive.open('rb') as source:
            for block in iter(lambda:source.read(1024*1024),b''):
                digest.update(block)
        if digest.hexdigest()!=lock['archive_sha256']:
            raise SystemExit('Archive SHA256 mismatch; destination was not created')
        prefix='open_vins-'+lock['commit']
        stage=temp/'source'
        stage.mkdir()
        with tarfile.open(archive,'r:gz') as tar:
            for member in tar.getmembers():
                parts=PurePosixPath(member.name).parts
                if not parts or parts[0]!=prefix or any(p in ('..','') for p in parts) or member.issym() or member.islnk():
                    raise SystemExit('Unsafe archive member: '+member.name)
                target=stage.joinpath(*parts[1:])
                if member.isdir():
                    target.mkdir(parents=True,exist_ok=True)
                elif member.isfile():
                    target.parent.mkdir(parents=True,exist_ok=True)
                    with tar.extractfile(member) as src,target.open('wb') as out:
                        shutil.copyfileobj(src,out)
                    target.chmod(member.mode & 0o777)
                else:
                    raise SystemExit('Unsupported archive entry: '+member.name)
        (stage/'.grampus_upstream.json').write_text(json.dumps(lock,indent=2)+'\n')
        if incomplete_git is None:
            stage.rename(dest)
        else:
            shutil.copytree(stage,dest,dirs_exist_ok=True)
    print('Installed pinned OpenVINS source:',lock['commit'])


if __name__=='__main__':
    main()
