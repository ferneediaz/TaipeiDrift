"""Experiment C: can the wind learned while position fixes are available carry the drone across water?
16 km over land with a camera fix every 1 km, then 30 km with no fixes at all. The wind drifts slowly during the flight."""
import numpy as np
def run(seed,wind_change=0.5,N=4000,v=20.0,dt=1.5,land=16e3,sea=30e3,sig_fix=10.0,head_sigma_deg=0.0):
    rng=np.random.default_rng(seed)
    wind=np.array([1.5,1.0]); T=int((land+sea)/(v*dt)); step_sd=wind_change/np.sqrt(sea/(v*dt))  # wind random walk: ~wind_change m/s over the crossing
    xt=np.zeros(2); px=rng.normal(0,200,(N,2)); pw=rng.normal(0,3,(N,2)); dr=np.zeros(2)
    hb=np.radians(rng.normal(0,head_sigma_deg))             # constant heading error of the compass
    out={}; lost=None
    for i in range(T):
        wind=wind+rng.normal(0,step_sd,2)
        xt=xt+(np.array([v,0])+wind)*dt
        od=np.array([v*np.cos(hb),v*np.sin(hb)])              # what the drone believes it flew through the air
        dr=dr+od*dt
        pw+=rng.normal(0,0.02,(N,2)); px+=(od+pw)*dt+rng.normal(0,0.5,(N,2))
        over_land=xt[0]<land
        if over_land and i%33==0 and i>0:
            f=xt+rng.normal(0,sig_fix,2); lw=-0.5*np.sum((f-px)**2,1)/sig_fix**2; w=np.exp(lw-lw.max()); w/=w.sum()
            idx=rng.choice(N,N,p=w); px=px[idx]+rng.normal(0,3,(N,2)); pw=pw[idx]+rng.normal(0,0.05,(N,2))
        if not over_land:
            if lost is None: lost=xt[0]; dr_at_loss=dr.copy(); x_at_loss=xt.copy()
            km=(xt[0]-lost)/1000
            for k in (5,10,20,30):
                if k not in out and km>=k-0.02:
                    out[k]=(np.hypot(*(px.mean(0)-xt)), np.hypot(*((x_at_loss+(dr-dr_at_loss))-xt)), np.hypot(*px.std(0)))
    return out
for label,kw in (('wind steady',dict(wind_change=0.0)),('wind changes by ~0.5 m/s during crossing',dict(wind_change=0.5)),('wind changes by ~1.5 m/s',dict(wind_change=1.5)),('wind changes 0.5 m/s + compass off by 1 degree',dict(wind_change=0.5,head_sigma_deg=1.0)),('wind changes 0.5 m/s + compass off by 5 degrees (cheap magnetometer)',dict(wind_change=0.5,head_sigma_deg=5.0))):
    R=[run(s,**kw) for s in range(8)]
    print(label)
    for k in (5,10,20,30):
        est=np.median([r[k][0] for r in R]); raw=np.median([r[k][1] for r in R]); sp=np.median([r[k][2] for r in R])
        print(f'   {k:2d} km after last fix: with learned wind {est:5.0f} m off (filter says +-{sp:4.0f} m) | plain dead reckoning from last fix {raw:5.0f} m off')
