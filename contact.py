"""Conservative flat-ground foot plants, solved into FK (MMD IK stays off).

Detection is source-only, in leg-length units. Corrections have compact support:
frames with neither detected foot plant preserve the uncorrected animation.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.ndimage import gaussian_filter1d


def runs(mask):
    edges=np.diff(np.r_[False,mask,False].astype(int))
    return list(zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)))


def plants(ankle,toe,length,fps=30,rotation=None):
    n=len(ankle); weights=np.zeros(n)
    if n<5 or length<=0: return weights,[],None
    # A slow elevated foot is not a plant: both ankle and toe must be low.
    ah=float(np.percentile(ankle[:,1],5)); th=float(np.percentile(toe[:,1],5))
    av=np.linalg.norm(np.gradient(ankle,1/fps,axis=0),axis=1)/length
    tv=np.linalg.norm(np.gradient(toe,1/fps,axis=0),axis=1)/length
    # Reject a foot hovering above the other foot's floor via a shared reference
    # supplied by the caller; the local low percentile is an additional gate.
    mask=(ankle[:,1]<ah+.025*length)&(toe[:,1]<th+.025*length)&(av<.16)&(tv<.16)
    if rotation is not None:
        spin=(rotation[:-1].inv()*rotation[1:]).magnitude()*fps
        speed=np.maximum(np.r_[spin,0],np.r_[0,spin])
        mask&=speed<np.deg2rad(45)
    segments=[]
    for start,end in runs(mask):
        if end-start<12:continue
        ramp=min(8,(end-start)//2)
        w=np.ones(end-start)
        x=np.arange(ramp)/ramp; smooth=x*x*x*(10+x*(-15+6*x))
        w[:ramp]=smooth;w[-ramp:]=smooth[::-1]
        weights[start:end]=w;segments.append((int(start),int(end)))
    return weights,segments,dict(ankle_low=ah,toe_low=th)


def swing_between(a,b):
    a=np.asarray(a,float);b=np.asarray(b,float)
    an=np.linalg.norm(a);bn=np.linalg.norm(b)
    if min(an,bn)<1e-10:raise ValueError('接地補正: 長さゼロの脚ボーンです')
    a=a/an;b=b/bn;dot=np.clip(a@b,-1,1)
    if dot<-.999999:
        axis=np.cross(a,np.eye(3)[np.argmin(abs(a))]);axis/=np.linalg.norm(axis)
        return R.from_rotvec(axis*np.pi)
    return R.from_quat(np.r_[np.cross(a,b),1+dot])


def solve_leg(hip,knee,ankle,goal,upper_rotation,lower_rotation):
    """Analytic two-bone IK, fixed lengths and original bend half-plane."""
    upper=knee-hip;lower=ankle-knee
    l1=np.linalg.norm(upper);l2=np.linalg.norm(lower)
    if min(l1,l2)<1e-7:raise ValueError('接地補正: 脚の長さがゼロです')
    direction=goal-hip;distance=np.linalg.norm(direction)
    axis=direction/distance if distance>1e-9 else (ankle-hip)/max(np.linalg.norm(ankle-hip),1e-9)
    if np.linalg.norm(axis)<.5:axis=np.array([0.,-1,0])
    reach=np.clip(distance,abs(l1-l2)+1e-6,l1+l2-1e-6)
    along=(l1*l1+reach*reach-l2*l2)/(2*reach)
    spread=np.sqrt(max(l1*l1-along*along,0))
    # Transport the original bend plane with the hip-to-ankle direction.
    # Projecting the upper leg directly onto the NEW direction can reverse the
    # knee when a planted ankle crosses that line, even though the source is smooth.
    original_axis=ankle-hip
    original_axis=original_axis/np.linalg.norm(original_axis) if np.linalg.norm(original_axis)>1e-9 else axis.copy()
    pole=upper-original_axis*(upper@original_axis)
    reference=upper_rotation.apply([0.,0.,-1.])
    reference-=original_axis*(reference@original_axis)
    magnitude=np.linalg.norm(pole);refnorm=np.linalg.norm(reference)
    if refnorm>1e-8 and np.linalg.norm(ankle-hip)>.8*(l1+l2):
        reference/=refnorm
        source_pole=pole/max(magnitude,1e-9)
        confidence=np.clip(magnitude/(.15*l1),0,1)
        confidence=confidence*confidence*(3-2*confidence)
        pole=reference*(1-confidence)+source_pole*confidence
    pole=swing_between(original_axis,axis).apply(pole)
    if np.linalg.norm(pole)<.005*l1:
        pole=upper_rotation.apply([0.,0.,-1.]);pole-=axis*(pole@axis)
    if np.linalg.norm(pole)<1e-8:
        pole=np.cross(axis,np.eye(3)[np.argmin(abs(axis))])
    pole/=np.linalg.norm(pole)
    new_knee=hip+axis*along+pole*spread;new_ankle=hip+axis*reach
    uq=swing_between(upper,new_knee-hip)*upper_rotation
    lq=swing_between(lower,new_ankle-new_knee)*lower_rotation
    return uq,lq,new_knee,new_ankle,float(np.linalg.norm(goal-new_ankle))


def correct_feet(legs,shift,strength=1.,floor=0.):
    """legs contains source ankle/toe and target FK positions/world rotations."""
    if not np.isfinite([strength,floor]).all() or not 0<=strength<=1:
        raise ValueError('接地の強さは0〜100%、床高さは有限値を指定してください')
    n=len(shift); report={'enabled':True,'strength':strength,'floor':floor,'sides':{}}
    result={};weighted=[];weights=[]
    # Shared toe floor prevents a permanently raised foot from defining its own floor.
    source_floor=float(np.percentile(np.concatenate([v['source_toe'][:,1] for v in legs.values()]),5))
    for side,v in legs.items():
        base_w,segments,levels=plants(v['source_ankle'],v['source_toe'],v['source_length'],rotation=v.get('source_q'))
        w=base_w.copy()
        # A whole take recorded in the air cannot be distinguished from a raised
        # stage automatically. Users can disable correction; this is a flat-floor model.
        accepted=[]; w[:]=0
        for start,end in segments:
            if np.median(v['source_toe'][start:end,1])>source_floor+.045*v['source_length']:continue
            accepted.append((start,end))
        goal=v['ankle'].copy();fq=v['foot_q'].as_quat().copy()
        for start,end in accepted:
            w[start:end]=base_w[start:end]*strength
            mid=(start+end)//2;anchor=np.median(v['ankle'][start:end],axis=0)
            fixed=v['foot_q'][mid]
            # Support the lowest rigid shoe vertex, including the heel offset.
            # The ankle-height fallback keeps models without usable samples working.
            sole=v.get('sole_points')
            if sole is None:sole=np.array([[0.,-v['rest_ankle_height'],0.]])
            anchor[1]=floor-float(np.min(fixed.apply(sole)[:,1]))
            goal[start:end]=anchor
            current=v['foot_q'][start:end]
            fq[start:end]=(current*R.from_rotvec((current.inv()*fixed).as_rotvec()*w[start:end,None])).as_quat()
        target_length=float(np.linalg.norm(v['knee'][0]-v['hip'][0])+np.linalg.norm(v['ankle'][0]-v['knee'][0]))
        weighted.append((goal[:,1]-v['ankle'][:,1])*w);weights.append(w)
        result[side]=dict(weight=w,goal=goal,foot_q=R.from_quat(fq),length=target_length,segments=accepted)
        report['sides'][side]=dict(sole_sample_count=0 if v.get('sole_points') is None else len(v['sole_points']),segments=[[a,b] for a,b in accepted],anchors=[dict(start=a,end=b,position=goal[(a+b)//2].tolist()) for a,b in accepted],plant_frames=int(np.count_nonzero(w)),full_lock_frames=int(np.count_nonzero(w>=.999)),source_levels=levels)
    total=np.sum(weights,axis=0);support=np.max(weights,axis=0)
    lift=np.sum(weighted,axis=0)/np.maximum(total,1)
    limit=.10*min(v['length'] for v in result.values())
    lift=np.clip(lift,-limit,limit)
    # Smooth only inside support. No smoothing spills into flight frames.
    if n>1:lift=gaussian_filter1d(lift,1,mode='nearest')*support
    # Lower the pelvis only as far as needed to keep planted ankles reachable.
    # The same compact contact weights preserve flight and blend boundaries.
    for side,v in legs.items():
        r=result[side];horizontal=np.linalg.norm((r['goal']-v['hip'])[:,[0,2]],axis=1)
        # A small extension reserve avoids the IK singularity at a straight knee.
        vertical=np.sqrt(np.maximum((r['length']*.995)**2-horizontal**2,0))
        cap=r['goal'][:,1]+vertical-v['hip'][:,1]
        correction=np.minimum(cap-lift,0)*r['weight']
        lift=np.clip(lift+correction,-limit,limit)
    new_shift=shift.copy();new_shift[:,1]+=lift
    for side,v in legs.items():
        r=result[side];w=r['weight'];uq=v['upper_q'].as_quat().copy();lq=v['lower_q'].as_quat().copy()
        solved=v['ankle'].copy();solved[:,1]+=lift
        errors=[];before=[];after=[]
        for frame in np.flatnonzero(w>0):
            delta=np.array([0.,lift[frame],0.]);h=v['hip'][frame]+delta;k=v['knee'][frame]+delta;a=v['ankle'][frame]+delta
            # Solve the plant, then ease rotations. Blending the target position
            # through full extension produces a square-root knee snap at release.
            goal=r['goal'][frame]
            horizontal=np.linalg.norm((goal-h)[[0,2]])
            cap=goal[1]+np.sqrt(max((r['length']*.995)**2-horizontal**2,0))
            virtual_delta=np.array([0.,min(cap-h[1],0),0.]) if w[frame]<.999999 else np.zeros(3)
            u,l,_,p,error=solve_leg(h+virtual_delta,k+virtual_delta,a+virtual_delta,goal,v['upper_q'][frame],v['lower_q'][frame])
            base_u=v['upper_q'][frame];base_l=v['lower_q'][frame]
            u=base_u*R.from_rotvec((base_u.inv()*u).as_rotvec()*w[frame])
            l=base_l*R.from_rotvec((base_l.inv()*l).as_rotvec()*w[frame])
            p=h+u.apply(base_u.inv().apply(k-h))+l.apply(base_l.inv().apply(a-k))
            uq[frame]=u.as_quat();lq[frame]=l.as_quat();solved[frame]=p;errors.append(error)
            if w[frame]>=.999:
                before.append(np.linalg.norm(v['ankle'][frame]-r['goal'][frame]));after.append(np.linalg.norm(p-r['goal'][frame]))
        r.update(upper_q=R.from_quat(uq),lower_q=R.from_quat(lq),solved_ankle=solved)
        report['sides'][side].update(max_unreachable_error=max(errors,default=0.),unreachable_frames=int(np.count_nonzero(np.array(errors)>1e-4)),mean_full_lock_error_before=float(np.mean(before)) if before else None,mean_full_lock_error_after=float(np.mean(after)) if after else None)
    report.update(blend_frames=8,source_toe_floor=source_floor,max_pelvis_shift=float(np.max(abs(lift))),no_support_frames=int(np.count_nonzero(support==0)),sole_model='lowest rigid PMX shoe vertex; fallback: projected rest ankle height; floor in MMD units',warnings=['つま先立ち・足の回転中・短い接地は固定しません','段差・階段・移動床は未対応です','届かない足目標は脚長で制限し残差を記録します'])
    return result,new_shift,report
