"""Offline syntax, secret-pattern, checkout-hygiene and map checks."""
import ast,hashlib,json,re,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
SKIP={'.git','.venv','__pycache__','.pytest_cache','.ruff_cache','build','install','log','runtime'}
BINARY={'.png','.jpg','.jpeg','.gif','.mp4','.STL','.stl','.npz','.pcd','.pgm','.pt','.onnx','.rrd','.pdf','.pptx','.safetensors','.zip','.gz'}

def main():
    problems=[];files=[];compiled=0;texts=0
    patterns=(re.compile(rb'\bsk-[A-Za-z0-9_-]{20,}'),re.compile(rb'\b(?:gh[pousr]_|github_pat_)[A-Za-z0-9_]{20,}'),
        re.compile(rb'-----BEGIN (?:OPENSSH|RSA|EC|DSA) PRIVATE KEY-----'),
        re.compile(rb'/home/[^/\s\x22\x27]+/homebody-office-\d{8}'),
        re.compile(rb'[A-Z]:[/\\]+Users[/\\]+[^/\\\s\x22\x27]+[/\\]'))
    for p in ROOT.rglob('*'):
        rel=p.relative_to(ROOT)
        if not p.is_file() or any(part in SKIP for part in rel.parts) or rel.as_posix() in ('config/local.env','config/local.ps1'):continue
        if p.suffix in ('.pyc','.token','.key') or '.before' in p.name:problems.append(f'Unexpected file: {rel}')
        data=p.read_bytes();files.append((rel.as_posix(),data))
        if len(data)>95*1024*1024:problems.append(f'Oversized Git file: {rel}')
        if p.suffix not in BINARY:
            try:data.decode('utf-8')
            except UnicodeDecodeError:continue
            texts+=1
            if b'\r' in data:problems.append(f'CR bytes: {rel}')
            # The scanner's own regex definitions are intentionally recognizable.
            if rel.as_posix()!='scripts/check-release.py' and any(pattern.search(data) for pattern in patterns):
                problems.append(f'Sensitive pattern: {rel}')
            if p.suffix=='.py':compile(data,str(rel),'exec');compiled+=1
    sys.path.insert(0,str(ROOT/'office'))
    from office_landmarks import approach_points
    from g1_readonly_geometry import StaticRoute
    import numpy as np
    office=ROOT/'office';planner=StaticRoute(office/'real-office-navigation.npz')
    points=approach_points(office,json.loads((office/'office-waypoints.json').read_text(encoding='utf-8'))['points'])
    for label,xy in points.items():
        if not np.isfinite(xy).all() or planner.clearance_at(xy)<=planner.radius:problems.append(f'Invalid saved goal: {label}')
    readme=(ROOT/'README.md').read_text(encoding='utf-8')
    for link in re.findall(r'\]\(([^)]+)\)',readme):
        if '://' not in link and not link.startswith('#') and not (ROOT/link.split('#')[0]).exists():problems.append(f'Broken README link: {link}')
    print(json.dumps(dict(files=len(files),text_files=texts,python_sources=compiled,goals=len(points),
        center_clearance_m=planner.clearance_at(points['房间中央']),problems=problems),ensure_ascii=False,indent=2))
    return int(bool(problems))

if __name__=='__main__':raise SystemExit(main())
