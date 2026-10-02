"""Experiment A: does camera map matching work on flat Taiwanese farmland across two dates?
Patches from the 2020 image are searched in the 2018 image by zero-normalised cross-correlation (heading assumed known)."""
import numpy as np, rasterio, cv2
from rasterio.warp import reproject, Resampling
from rasterio.transform import from_origin
import os; os.makedirs('data/processed',exist_ok=True)
rng=np.random.default_rng(0)
a=rasterio.open('data/raw/aerial/wufeng_2018-05-03_x4.tif'); b=rasterio.open('data/raw/aerial/wufeng_2020-03-23_x4.tif')
l=max(a.bounds.left,b.bounds.left); r=min(a.bounds.right,b.bounds.right); bo=max(a.bounds.bottom,b.bounds.bottom); t=min(a.bounds.top,b.bounds.top)
def grid(ds,res):
    W=int((r-l)/res); H=int((t-bo)/res); T=from_origin(l,t,res,res)
    out=np.zeros((3,H,W),np.uint8)
    for i in range(3):
        reproject(rasterio.band(ds,i+1),out[i],dst_transform=T,dst_crs=ds.crs,resampling=Resampling.average)
    return out
res=1.0
A=grid(a,res); B=grid(b,res)
gA=cv2.cvtColor(np.moveaxis(A,0,-1),cv2.COLOR_RGB2GRAY); gB=cv2.cvtColor(np.moveaxis(B,0,-1),cv2.COLOR_RGB2GRAY)
vA=(A.sum(0)>0); vB=(B.sum(0)>0)
np.save('data/processed/gA.npy',gA); np.save('data/processed/gB.npy',gB)
for P,S in ((96,150),):
    errs=[]; peaks=[]; tried=0
    while len(errs)<300 and tried<40000:
        tried+=1
        y=rng.integers(S,gA.shape[0]-P-S); x=rng.integers(S,gA.shape[1]-P-S)
        if vB[y:y+P,x:x+P].mean()<0.999 or vA[y-S:y+P+S,x-S:x+P+S].mean()<0.999: continue
        m=cv2.matchTemplate(gA[y-S:y+P+S,x-S:x+P+S],gB[y:y+P,x:x+P],cv2.TM_CCOEFF_NORMED)
        _,mx,_,loc=cv2.minMaxLoc(m)
        errs.append(np.hypot(loc[0]-S,loc[1]-S)*res); peaks.append(mx)
    e=np.array(errs); pk=np.array(peaks)
    wrong=e>50
    print(f'patch {P} m, search +-{S} m, n={len(e)} | median error {np.median(e):.1f} m | <=5 m {(e<=5).mean()*100:.0f}% | <=10 m {(e<=10).mean()*100:.0f}% | <=20 m {(e<=20).mean()*100:.0f}% | gross miss >50 m {wrong.mean()*100:.0f}% | score right {np.median(pk[~wrong]):.2f} vs wrong {np.median(pk[wrong]) if wrong.any() else float("nan"):.2f}')
    if P==96 and S==150:
        # can the score tell right from wrong? keep only confident matches
        for th in (0.3,0.4,0.5):
            k=pk>=th
            if k.any(): print(f'     keep score>={th}: {k.mean()*100:.0f}% of patches kept, of those gross miss {(wrong&k).sum()/k.sum()*100:.0f}%, median error {np.median(e[k]):.1f} m')
