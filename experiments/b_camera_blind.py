"""Experiment A2: same patches, but searched over the WHOLE 3 km corridor (no prior position)."""
import numpy as np, cv2
E='data/processed/'
gA=np.load(E+'gA.npy'); gB=np.load(E+'gB.npy'); vA=gA>0; vB=gB>0
rng=np.random.default_rng(1)
print('corridor grid', gA.shape, 'valid share 2018', round(vA.mean(),2), '2020', round(vB.mean(),2))
for P in (64,96,160):
    errs=[];pk=[];second=[];tried=0
    while len(errs)<150 and tried<50000:
        tried+=1
        y=rng.integers(0,gA.shape[0]-P); x=rng.integers(0,gA.shape[1]-P)
        if vB[y:y+P,x:x+P].mean()<0.999 or vA[y:y+P,x:x+P].mean()<0.999: continue
        m=cv2.matchTemplate(gA,gB[y:y+P,x:x+P],cv2.TM_CCOEFF_NORMED)
        _,mx,_,loc=cv2.minMaxLoc(m)
        errs.append(np.hypot(loc[0]-x,loc[1]-y)); pk.append(mx)
        m2=m.copy(); cv2.circle(m2,loc,40,-1,-1); second.append(m2.max())
    e=np.array(errs);pk=np.array(pk);second=np.array(second)
    print(f'patch {P} m, search whole corridor, n={len(e)} | median error {np.median(e):.1f} m | <=10 m {(e<=10).mean()*100:.0f}% | gross miss >50 m {(e>50).mean()*100:.0f}% | best score {np.median(pk):.2f}, runner-up elsewhere {np.median(second):.2f}')
# degrade the drone view: blur + brightness change + noise, as a cheap stand-in for a worse camera and other light
P=96; errs=[];tried=0
while len(errs)<150 and tried<50000:
    tried+=1
    y=rng.integers(0,gA.shape[0]-P); x=rng.integers(0,gA.shape[1]-P)
    if vB[y:y+P,x:x+P].mean()<0.999 or vA[y:y+P,x:x+P].mean()<0.999: continue
    t=gB[y:y+P,x:x+P].astype(np.float32); t=cv2.GaussianBlur(t,(0,0),2.0)*0.6+rng.normal(0,12,t.shape); t=np.clip(t,0,255).astype(np.uint8)
    m=cv2.matchTemplate(gA,t,cv2.TM_CCOEFF_NORMED); _,mx,_,loc=cv2.minMaxLoc(m); errs.append(np.hypot(loc[0]-x,loc[1]-y))
e=np.array(errs); print(f'degraded view (blur 2 m, 40% darker, noise), patch 96 m, whole corridor | median error {np.median(e):.1f} m | <=10 m {(e<=10).mean()*100:.0f}% | gross miss {(e>50).mean()*100:.0f}%')
