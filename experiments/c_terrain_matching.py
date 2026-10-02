"""Experiment B2: terrain matching by particle filter with the wind as part of the state (my first filter diverged:
it had no way to represent a constant unknown wind, so the particles lagged and then collapsed in the wrong place)."""
import numpy as np
E='data/processed/'
import os
if not os.path.exists(E+'dem_strip.npy'):
    # 23.90-24.20 N, 120-121 E from the Copernicus 30 m elevation model (free, read over HTTP)
    os.environ.update(GDAL_HTTP_TIMEOUT='60',GDAL_DISABLE_READDIR_ON_OPEN='EMPTY_DIR',CPL_VSIL_CURL_ALLOWED_EXTENSIONS='.tif',GDAL_HTTP_MAX_RETRY='5',GDAL_HTTP_RETRY_DELAY='2')
    import rasterio
    from rasterio.windows import Window
    os.makedirs(E,exist_ok=True)
    with rasterio.open('/vsicurl/https://copernicus-dem-30m.s3.amazonaws.com/Copernicus_DSM_COG_10_N24_00_E120_00_DEM/Copernicus_DSM_COG_10_N24_00_E120_00_DEM.tif') as ds:
        r0=int((25.0-24.20)*3600); r1=int((25.0-23.90)*3600)
        np.save(E+'dem_strip.npy',ds.read(1,window=Window(0,r0,3600,r1-r0)).astype(np.float32))
d=np.load(E+'dem_strip.npy'); dy=30.9; dx=30.9*np.cos(np.radians(24.05)); H,W=d.shape
row=int((24.20-24.05)*3600); y0=(H-1-row)*dy; prof=d[row]
def elev(x,y):
    # bilinear interpolation of the elevation grid
    fx=np.clip(x/dx,0,W-1.001); fy=np.clip(H-1-y/dy,0,H-1.001); c=fx.astype(int); r=fy.astype(int); a=fx-c; b=fy-r
    return d[r,c]*(1-a)*(1-b)+d[r,c+1]*a*(1-b)+d[r+1,c]*(1-a)*b+d[r+1,c+1]*a*b
def run(a,b,terrain=True,N=4000,v=20.0,dt=1.5,wind=(1.5,1.0),sig_h=4.0,sig_m=8.0,seed=0,fix_every=None,sig_fix=10.0):
    rng=np.random.default_rng(seed); steps=int((b-a)/((v+wind[0])*dt))
    xt,yt=a,y0; drx,dry=a,y0
    px=a+rng.normal(0,200,N); py=y0+rng.normal(0,200,N); wx=rng.normal(0,3,N); wy=rng.normal(0,3,N)
    err=[];dr=[];sp=[]
    for i in range(steps):
        xt+=(v+wind[0])*dt; yt+=wind[1]*dt; drx+=v*dt
        wx+=rng.normal(0,0.02,N); wy+=rng.normal(0,0.02,N)
        px+=(v+wx)*dt+rng.normal(0,0.5,N); py+=wy*dt+rng.normal(0,0.5,N)
        lw=np.zeros(N)
        if terrain:
            z=elev(np.array([xt]),np.array([yt]))[0]+rng.normal(0,sig_h); lw+=-0.5*((z-elev(px,py))/sig_m)**2
        if fix_every and i%fix_every==0 and i>0:           # optional camera position fix
            fx=xt+rng.normal(0,sig_fix); fy=yt+rng.normal(0,sig_fix); lw+=-0.5*(((fx-px)**2+(fy-py)**2)/sig_fix**2)
        w=np.exp(lw-lw.max()); w/=w.sum()
        if 1/np.sum(w**2)<N/2:
            idx=rng.choice(N,N,p=w); px=px[idx]+rng.normal(0,5,N); py=py[idx]+rng.normal(0,5,N); wx=wx[idx]+rng.normal(0,0.05,N); wy=wy[idx]+rng.normal(0,0.05,N); w=np.full(N,1/N)
        mx,my=np.sum(w*px),np.sum(w*py)
        err.append(np.hypot(mx-xt,my-yt)); dr.append(np.hypot(drx-xt,dry-yt)); sp.append(np.sqrt(np.sum(w*((px-mx)**2+(py-my)**2))))
    return np.array(err),np.array(dr),np.array(sp)
coast=np.argmax(prof>1.0)*dx/1000
print(f'flight line 24.05 N, coast {coast:.0f} km east of 120E. drone 20 m/s, unknown wind 1.8 m/s, laser+map error 4 m, start uncertainty 200 m')
segs=[('open sea',5e3,35e3),('coastal plain',39e3,55e3),('foothills',58e3,74e3),('mountains',78e3,98e3)]
for name,a,b in segs:
    rel=prof[int(a/dx):int(b/dx)]
    R=[run(a,b,seed=s) for s in range(6)]
    last=lambda r:r[int(len(r)*0.6):]
    e=np.median([last(r[0]).mean() for r in R]); worst=np.max([last(r[0]).mean() for r in R]); drE=np.mean([r[1][-1] for r in R]); sp=np.median([last(r[2]).mean() for r in R])
    print(f'  {name:14s} {(b-a)/1000:.0f} km | elevation {rel.min():4.0f}-{rel.max():4.0f} m, std {rel.std():6.1f} | dead reckoning ends {drE:5.0f} m off | terrain matching: median {e:5.0f} m, worst run {worst:5.0f} m, filter says +-{sp:4.0f} m')
print('same legs with laser+map error 10 m instead of 4 m:')
for name,a,b in segs[1:]:
    R=[run(a,b,seed=s,sig_h=10.0,sig_m=14.0) for s in range(6)]; last=lambda r:r[int(len(r)*0.6):]
    print(f'  {name:14s} median {np.median([last(r[0]).mean() for r in R]):5.0f} m, worst {np.max([last(r[0]).mean() for r in R]):5.0f} m')
print('coastal plain, terrain OFF, camera fix (10 m) every 100 m of flight vs every 1 km:')
for every,label in ((3,'every ~100 m'),(31,'every ~1 km')):
    R=[run(39e3,55e3,terrain=False,seed=s,fix_every=every) for s in range(6)]; last=lambda r:r[int(len(r)*0.6):]
    print(f'  {label}: median {np.median([last(r[0]).mean() for r in R]):5.0f} m, filter says +-{np.median([last(r[2]).mean() for r in R]):4.0f} m')
print('open sea, nothing but heading and airspeed, how error grows with distance (wind 1.8 m/s unknown):')
e,dr,sp=run(5e3,35e3,terrain=False,seed=0)
for km in (5,10,20,30): 
    i=min(len(dr)-1,int(km*1000/((21.5)*1.5))); print(f'  after {km:2d} km: dead reckoning {dr[i]:5.0f} m off')
