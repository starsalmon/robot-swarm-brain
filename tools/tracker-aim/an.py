import json,statistics as st
d=json.load(open('/home/cain/aim/aim_raw.json'));B=d['blocks']
t0=B[0]['t']
print('phase angle t_s mean_b sd_b mean_s echo')
for r in B: print(r['phase'],r['angle'],round(r['t']-t0),round(r['mean_b'],4),round(r['sd_b'],4),round(r['mean_s'],4),[round(x) for x in r['echo']][:3])
# detrend: fit linear drift on all blocks with angle effect via least squares: b = a + c*t + angle dummies is overfit; instead use ABAB paired diffs
ab=[r for r in B if r['phase'].startswith('AB')]
import numpy as np
ref=[r for r in ab if r['phase']=='AB_ref'];bst=[r for r in ab if r['phase']=='AB_best']
# interpolate drift: diff best - mean(neighbouring refs)
diffs=[]
for i,b in enumerate(bst):
    nb=[ref[i]['mean_b']]+([ref[i+1]['mean_b']] if i+1<len(ref) else [])
    if len(nb)==2: diffs.append(b['mean_b']-st.mean(nb))
print('best-minus-ref (interp, mV):',[round(x*1000,2) for x in diffs])
print('raw pair diffs mV (best-ref prior):',[round((bst[i]['mean_b']-ref[i]['mean_b'])*1000,2) for i in range(len(bst))])
# drift slope from all blocks
t=np.array([r['t']-t0 for r in B]);y=np.array([r['mean_b'] for r in B])
print('overall drift mV/min',np.polyfit(t,y,1)[0]*60000)
# coarse detrended: fwd vs rev
for ph in('fwd','rev','fine','fine_rev'):
    print(ph,[(r['angle'],round(r['mean_b'],4)) for r in B if r['phase']==ph])
