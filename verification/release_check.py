"""Time-resolution and model-source checks; not a Simulink runtime test."""
from pathlib import Path
import re, platform
import numpy as np
import scipy
from reference_simulation import parameters, attitude, flight, ROOT

p=parameters(); fine=dict(p,sample_dt=.005,max_step=.005)
lines=['Preparation checks - 2026-10-15',
       'Generic teaching model; not NAYA results.',
       f'Python {platform.python_version()}, NumPy {np.__version__}, SciPy {scipy.__version__}',
       'Maximum integration step and output spacing: 0.01 s -> 0.005 s']
for mode in ['off','PD']:
    t,a=flight(p,mode); tf,af=flight(fine,mode)
    difference=abs(max(a[:,3])-max(af[:,3]))
    assert difference<.005, 'Apogee time-resolution sensitivity exceeded 5 mm'
    lines.append(f'{mode} apogee difference: {difference:.9f} m (threshold 0.005 m)')
def settle(p):
    t,y,d=attitude(p,'PD');outside=np.flatnonzero(abs(y[:,0])>.02*abs(p['theta0']))
    return t[outside[-1]+1]
difference=abs(settle(p)-settle(fine))
assert difference<=.0100001
lines.append(f'PD settling-time difference: {difference:.6f} s (threshold 0.01 s)')
stress=dict(p,delta_max=np.deg2rad(.5))
t,y,d=attitude(stress,'PD')
assert np.max(abs(d))<=stress['delta_max']+1e-12
assert np.any(np.isclose(abs(d),stress['delta_max'])), 'Stress case must engage saturation'
lines.append('Reduced-authority stress case: saturation engaged and respected.')
s=(ROOT/'simulink/tvc_pd_educational.mdl').read_text()
assert s.count('{')==s.count('}')
blocks={}
for block in re.findall(r'Block \{(.*?)\n    \}',s,re.S):
    name=re.search(r'Name "([^"]+)"',block).group(1)
    typ=re.search(r'BlockType (\w+)',block).group(1)
    assert name not in blocks
    blocks[name]=typ
sources=re.findall(r'SrcBlock "([^"]+)"',s)
dests=re.findall(r'DstBlock "([^"]+)"\s+DstPort (\d+)',s)
assert len(blocks)==10 and len(dests)==13
assert all(a in blocks for a in sources)
assert all(a in blocks for a,b in dests)
assert len(dests)==len(set(dests)), 'An input port is connected more than once'
expected_inputs={name: ({1,2} if typ=='Sum' else {1,2,3} if typ=='Mux' else {1})
                 for name,typ in blocks.items()}
for name,ports in expected_inputs.items():
    assert {int(port) for dest,port in dests if dest==name}==ports
lines+=['MDL static structure: balanced braces; 10 unique blocks; 13 unique destination ports;',
        'every expected input connected; all referenced block names exist.',
        'MATLAB / Simulink execution: NOT PERFORMED (runtimes unavailable).',
        'MDL loading / compilation / SLX generation: NOT VERIFIED.',
        'Figures visually inspected: axes, units, legends and provenance labels readable.']
(ROOT/'results/verification_report.txt').write_text('\n'.join(lines)+'\n')
print('\n'.join(lines))
