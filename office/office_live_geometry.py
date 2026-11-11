"""Offline geometry functions. This module has no ROS or robot command interface."""
import numpy as np
from scipy import ndimage
from scipy.optimize import minimize
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def voxel(points,size):
    return points[np.unique(np.floor(points/size).astype(np.int32),axis=0,return_index=True)[1]]


def structural(points):
    return points[(points[:,2]>-.95)&(points[:,2]<.6)&(np.linalg.norm(points[:,:2],axis=1)>.6)]


def gravity_alignment(up):
    up=np.asarray(up,float)
    if up.shape!=(3,) or not np.isfinite(up).all() or np.linalg.norm(up)<.9:
        raise ValueError('Waiting for initial IMU gravity direction')
    up=up/np.linalg.norm(up)
    target=np.array([0.,0.,1.])
    axis=np.cross(up,target)
    sine=np.linalg.norm(axis)
    cosine=np.clip(up@target,-1.,1.)
    if sine<1e-8:
        R=np.eye(3) if cosine>0 else Rotation.from_euler('x',np.pi).as_matrix()
    else:
        R=Rotation.from_rotvec(axis/sine*np.arctan2(sine,cosine)).as_matrix()
    T=np.eye(4);T[:3,:3]=R
    return T


def estimate_floor(points):
    """Fit a broad low horizontal plane; fail instead of guessing a height."""
    points=voxel(np.asarray(points,float),.07)
    points=points[np.isfinite(points).all(axis=1)]
    if len(points)<500:raise ValueError('地面结构点不足')
    low,high=np.percentile(points[:,2],[1,35])
    edges=np.arange(low-.04,high+.081,.04)
    counts,edges=np.histogram(points[:,2],edges)
    for index in np.argsort(counts)[::-1][:8]:
        height=(edges[index]+edges[index+1])/2
        selected=points[np.abs(points[:,2]-height)<.055]
        if len(selected)<80:continue
        matrix=np.column_stack([selected[:,:2],np.ones(len(selected))])
        fit=np.linalg.lstsq(matrix,selected[:,2],rcond=None)[0]
        residual=np.abs(matrix@fit-selected[:,2])
        keep=residual<.025
        if keep.sum()<80:continue
        selected=selected[keep]
        if np.any(np.ptp(selected[:,:2],axis=0)<.8):continue
        fit=np.linalg.lstsq(np.column_stack([selected[:,:2],np.ones(len(selected))]),selected[:,2],rcond=None)[0]
        if np.linalg.norm(fit[:2])>.10:continue
        return {'height_m':float(fit[2]),'slope_xy':fit[:2].tolist(),'points':len(selected)}
    raise ValueError('未找到可靠的水平地面')


class HistoricalMatcher:
    def __init__(self,path):
        with path.open('rb') as f:
            header=[]
            while True:
                line=f.readline()
                if not line:raise ValueError('PCD lacks DATA')
                header.append(line.decode('ascii').strip())
                if line.startswith(b'DATA'):break
            if header[-1]!='DATA binary' or 'FIELDS x y z intensity normal_x normal_y normal_z curvature' not in header:
                raise ValueError('Unexpected historical map format')
            points=np.frombuffer(f.read(),dtype='<f4').reshape(-1,8)[:,:3].copy()
        if not np.isfinite(points).all():raise ValueError('Nonfinite historical map')
        self.floor=estimate_floor(points)
        self.target=voxel(points[(points[:,2]>-1.05)&(points[:,2]<.75)],.07)
        self.tree=cKDTree(self.target)
        flat=voxel(structural(points),.1)
        self.origin=np.floor(flat[:,:2].min(axis=0)/.1)*.1-1.
        self.upper=np.ceil(flat[:,:2].max(axis=0)/.1)*.1+1.
        shape=np.ceil((self.upper-self.origin)/.1).astype(int)+1
        occupied=np.zeros((shape[1],shape[0]),dtype=bool)
        ij=np.floor((flat[:,:2]-self.origin)/.1).astype(int)
        occupied[ij[:,1],ij[:,0]]=True
        self.distance=ndimage.distance_transform_edt(~occupied)*.1

    @staticmethod
    def planar(points,p):
        c,s=np.cos(p[2]),np.sin(p[2])
        return np.column_stack([points[:,:2]@np.array([[c,s],[-s,c]])+p[:2],points[:,2]])

    def metrics(self,points,T):
        d=self.tree.query(points@T[:3,:3].T+T[:3,3])[0]
        return {'points':len(points),'mean_clipped_m':float(np.minimum(d,.65).mean()),
                'median_distance_m':float(np.median(d)),'within_0_2m_fraction':float(np.mean(d<.2))}

    def refine(self,points,T):
        T=T.copy()
        for i in range(75):
            moved=points@T[:3,:3].T+T[:3,3]
            d,idx=self.tree.query(moved)
            keep=d<(.35 if i<15 else .2)
            if keep.sum()<120:raise ValueError('Too few correspondences')
            a,b=moved[keep],self.target[idx[keep]]
            ac,bc=a.mean(axis=0),b.mean(axis=0)
            u,_,vt=np.linalg.svd((a-ac).T@(b-bc));R=vt.T@u.T
            if np.linalg.det(R)<0:vt[-1]*=-1;R=vt.T@u.T
            shift=bc-R@ac
            increment=np.eye(4);increment[:3,:3]=R;increment[:3,3]=shift
            T=increment@T
            if np.linalg.norm(shift)<1e-5 and Rotation.from_matrix(R).magnitude()<1e-6:break
        return T

    def global_match(self,points,up=None):
        alignment=np.eye(4) if up is None else gravity_alignment(up)
        aligned=points@alignment[:3,:3].T
        shift=0.
        if up is not None:
            floor=estimate_floor(aligned)
            shift=self.floor['height_m']-floor['height_m']
            if abs(shift)>1.5:raise ValueError('地面高度差超出配准范围')
            aligned=aligned+[0,0,shift]
            alignment[2,3]=shift
        T,quality=self._global_match_aligned(aligned)
        quality['gravity_alignment_used']=up is not None
        quality['height_shift_m']=shift
        return T@alignment,quality

    def prior_allows(self,T,prior):
        office=np.asarray(prior['T_office_from_history'])@T
        x,y=office[:2,3]
        lo_x,hi_x,lo_y,hi_y=prior['office_xy_bounds_m']
        yaw=np.rad2deg(np.arctan2(office[1,0],office[0,0]))
        error=abs((yaw-prior['heading_degrees']+180)%360-180)
        return lo_x<=x<=hi_x and lo_y<=y<=hi_y and error<=prior['heading_tolerance_degrees']

    def current_structure(self,points,T):
        """Select by historical height after the complete fitted transform."""
        moved=points@T[:3,:3].T+T[:3,3]
        keep=(moved[:,2]>-.95)&(moved[:,2]<.6)&(np.linalg.norm(points[:,:2],axis=1)>.6)
        return voxel(points[keep],.1)

    def match_with_prior(self,points,up,prior):
        alignment=gravity_alignment(up)
        level=points@alignment[:3,:3].T
        floor=estimate_floor(level)
        shift=self.floor['height_m']-floor['height_m']
        if abs(shift)>1.5:raise ValueError('地面高度差超出配准范围')
        prepared=alignment.copy();prepared[2,3]=shift
        source=voxel(structural(level+[0,0,shift]),.1)
        if len(source)<500:raise ValueError('等待更完整的结构点云')
        rng=np.random.default_rng(20261109)
        coarse=source[rng.choice(len(source),min(650,len(source)),replace=False)]
        fine=source[rng.choice(len(source),min(4000,len(source)),replace=False)]
        office=np.asarray(prior['T_office_from_history'])
        history=np.linalg.inv(office)
        lo_x,hi_x,lo_y,hi_y=prior['office_xy_bounds_m']
        def transform(p):
            t=np.eye(4);t[:3,:3]=Rotation.from_euler('z',p[2]).as_matrix();t[:3,3]=[*p[:2],0]
            return t
        def allowed(p):return self.prior_allows(transform(p)@prepared,prior)
        def score(p):
            if not allowed(p):return 2.
            return self.metrics(coarse,transform(p))['mean_clipped_m']
        seeds=[]
        offset=np.arctan2((office@alignment)[1,0],(office@alignment)[0,0])
        center=np.deg2rad(prior['heading_degrees'])-offset
        tolerance=prior['heading_tolerance_degrees']
        for x in np.arange(lo_x+.05,hi_x,.3):
            for y in np.arange(lo_y+.05,hi_y,.3):
                xy=(history@[x,y,0,1])[:2]
                for yaw in center+np.deg2rad(np.arange(-tolerance,tolerance+1,10)):
                    p=np.array([*xy,yaw]);seeds.append((score(p),p))
        distinct=[]
        for value,p in sorted(seeds,key=lambda item:item[0]):
            if value>=2:continue
            if any(np.linalg.norm(p[:2]-a[:2])<.65 and abs(np.angle(np.exp(1j*(p[2]-a[2]))))<.3 for a in distinct):continue
            distinct.append(p)
            if len(distinct)>=24:break
        refined=[]
        for seed in distinct:
            fit=minimize(score,seed,method='Nelder-Mead',options={'maxiter':300,'xatol':.001,'fatol':.0001})
            if not allowed(fit.x):continue
            try:t=self.refine(fine,transform(fit.x))
            except ValueError:continue
            complete=t@prepared
            # ICP is unconstrained: always check the final pose again.
            if not self.prior_allows(complete,prior):continue
            if np.rad2deg(np.arccos(np.clip(t[2,2],-1,1)))>5:continue
            quality=self.metrics(fine,t)
            refined.append((quality['mean_clipped_m'],complete,quality))
        refined.sort(key=lambda item:item[0])
        if not refined:raise ValueError('现场区域内没有可靠匹配')
        value,best,quality=refined[0]
        alternate=next((v for v,t,q in refined if np.linalg.norm(t[:2,3]-best[:2,3])>.8 or
            abs(np.angle(np.exp(1j*(np.arctan2(t[1,0],t[0,0])-np.arctan2(best[1,0],best[0,0])))))>np.deg2rad(18)),None)
        self.last_search={'source_points':len(source),'height_shift_m':shift,'current_floor':floor,
            'historical_floor':self.floor,'candidates':[{'T':t.tolist(),'quality':q} for v,t,q in refined]}
        if quality['within_0_2m_fraction']<.72 or quality['median_distance_m']>.12:
            raise ValueError('现场区域内点云吻合度不足')
        if alternate is not None and alternate-value<.05:
            raise ValueError('现场区域内仍存在多个相近候选')
        quality={**quality,'gravity_alignment_used':True,'height_shift_m':shift,
            'onsite_prior_used':True,'next_distinct_refined_score_m':alternate,
            'best_refined_score_m':value,'searched_seeds':len(distinct)}
        return best,quality

    def _global_match_aligned(self,points):
        source=voxel(structural(points),.1)
        if len(source)<250:raise ValueError('Collecting more structural points')
        rng=np.random.default_rng(20261106)
        coarse=source[rng.choice(len(source),min(650,len(source)),replace=False)]
        fine=source[rng.choice(len(source),min(4000,len(source)),replace=False)]
        translations=np.array(np.meshgrid(np.arange(self.origin[0],self.upper[0],.4),np.arange(self.origin[1],self.upper[1],.4),indexing='xy')).reshape(2,-1).T
        results=[]
        for yaw in np.deg2rad(np.arange(-180,180,10)):
            xy=self.planar(coarse,[0,0,yaw])[:,:2][None,:,:]+translations[:,None,:]
            cols=(xy[:,:,0]-self.origin[0])/.1;rows=(xy[:,:,1]-self.origin[1])/.1
            d=ndimage.map_coordinates(self.distance,[rows.ravel(),cols.ravel()],order=1,mode='constant',cval=1.).reshape(len(translations),-1)
            scores=np.minimum(d,.6).mean(axis=1)
            for i in np.argpartition(scores,20)[:20]:results.append((float(scores[i]),*translations[i],yaw))
        seeds=[]
        for score,x,y,yaw in sorted(results):
            if any(np.hypot(x-a[0],y-a[1])<.8 and abs(np.angle(np.exp(1j*(yaw-a[2]))))<np.deg2rad(18) for a in seeds):continue
            seeds.append((x,y,yaw))
            if len(seeds)>=24:break
        solutions=[]
        def score(p):return float(np.minimum(self.tree.query(self.planar(coarse,p))[0],.65).mean())
        for seed in seeds:
            fit=minimize(score,seed,method='Nelder-Mead',options={'maxiter':240,'xatol':.001,'fatol':.0001})
            p=fit.x;p[2]=np.angle(np.exp(1j*p[2]))
            val=float(np.minimum(self.tree.query(self.planar(fine,p))[0],.65).mean())
            solutions.append((val,p))
        solutions.sort(key=lambda x:x[0])
        best=solutions[0][1]
        alternate=next((v for v,p in solutions if np.linalg.norm(p[:2]-best[:2])>.8 or abs(np.angle(np.exp(1j*(p[2]-best[2]))))>np.deg2rad(18)),None)
        refined=[]
        for _,p in solutions[:5]:
            T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',p[2]).as_matrix();T[:3,3]=[*p[:2],0]
            T=self.refine(fine,T)
            refined.append((self.metrics(fine,T)['mean_clipped_m'],T))
        refined.sort(key=lambda x:x[0])
        T=refined[0][1]
        quality=self.metrics(fine,T)
        quality['next_distinct_planar_score_m']=alternate
        quality['best_planar_score_m']=solutions[0][0]
        self.last_search={'quality':quality,'source_points':len(source),
            'planar_candidates':[{'score_m':v,'xyyaw':p.tolist()} for v,p in solutions],
            'refined_candidates':[{'T':t.tolist(),'quality':self.metrics(fine,t)} for v,t in refined]}
        if quality['within_0_2m_fraction']<.72 or quality['median_distance_m']>.12:
            raise ValueError('Global match quality too low')
        if alternate is None or alternate-solutions[0][0]<.05:
            raise ValueError('Global match ambiguous in searched candidates')
        return T,quality
