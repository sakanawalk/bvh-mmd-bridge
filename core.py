"""BVH -> PMX-calibrated FK VMD. No application or scene side effects."""
from pathlib import Path
import json, math, re, struct, hashlib
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp


class BVH:
    def __init__(self, path):
        text = Path(path).read_text(encoding='utf-8-sig')
        header, motion = re.split(r'\bMOTION\b', text, maxsplit=1)
        header = header.strip()
        if not header.startswith('HIERARCHY'): header='HIERARCHY\n'+header
        tokens = iter(re.findall(r'[^\s{}]+|[{}]', header))
        self.bones = []; self.width = 0
        def expect(value):
            if next(tokens) != value: raise ValueError('BVH階層が不正です: '+value)
        def joint(parent, end=False):
            name = self.bones[parent]['name']+'__end' if end else next(tokens)
            i = len(self.bones)
            row = dict(name=name, parent=parent, channels=[], indices=[], offset=None)
            self.bones.append(row); expect('{'); expect('OFFSET')
            row['offset'] = [float(next(tokens)) for _ in range(3)]
            while True:
                token = next(tokens)
                if token == '}': break
                if token == 'CHANNELS':
                    n = int(next(tokens)); row['channels'] = [next(tokens) for _ in range(n)]
                    row['indices'] = list(range(self.width, self.width+n)); self.width += n
                elif token == 'JOINT': joint(i)
                elif token == 'End': expect('Site'); joint(i, True)
                else: raise ValueError('未知のBVH項目: '+token)
        try:
            expect('HIERARCHY'); expect('ROOT'); joint(-1)
            if next(tokens, None) is not None: raise ValueError('複数ルートには未対応です')
        except (StopIteration, IndexError) as e: raise ValueError('BVH階層が途中で終了しています') from e
        match = re.fullmatch(r'\s*Frames:\s*(\d+)\s*Frame Time:\s*([^\s]+)\s*(.*)', motion, re.S)
        if not match: raise ValueError('BVHの時刻ヘッダーが不正です')
        self.count = int(match[1]); self.dt = float(match[2])
        if self.count < 1 or not math.isfinite(self.dt) or self.dt <= 0: raise ValueError('フレーム数・時間が不正です')
        # Strict conversion rejects trailing garbage instead of silently ignoring it.
        values = np.array(match[3].split(), dtype=float)
        if values.size != self.count*self.width or not np.isfinite(values).all(): raise ValueError('BVHのデータ数または数値が不正です')
        self.data = values.reshape(self.count, self.width)
        self.names = {b['name']: i for i,b in enumerate(self.bones)}
        if len(self.names) != len(self.bones): raise ValueError('BVHに重複ボーン名があります')
        for b in self.bones:
            cs = b['channels']
            if len(cs)!=len(set(cs)) or any(c not in [a+t for a in 'XYZ' for t in ('position','rotation')] for c in cs):
                raise ValueError('BVHチャンネル指定が不正です')
            rs = [c for c in cs if c.endswith('rotation')]
            if len(rs) not in (0,3): raise ValueError('回転は3軸のBVHに対応しています')
            if not np.isfinite(b['offset']).all(): raise ValueError('BVH OFFSETが不正です')

    def sample(self, times, basis):
        source_times = np.arange(self.count)*self.dt
        positions=[]; rotations=[]
        for b in self.bones:
            p = np.broadcast_to(b['offset'], (len(times),3)).copy()
            order=''; ids=[]
            for c,k in zip(b['channels'],b['indices']):
                if c.endswith('position'): p[:, 'XYZ'.index(c[0])] += np.interp(times,source_times,self.data[:,k])
                else: order+=c[0]; ids.append(k)
            q = R.from_euler(order,self.data[:,ids],degrees=True) if ids else R.identity(self.count)
            q = Slerp(source_times,q)(times) if self.count>1 else R.from_quat(np.repeat(q.as_quat(),len(times),axis=0))
            p = p@basis.T; q=R.from_matrix(basis@q.as_matrix()@basis.T)
            parent=b['parent']
            if parent>=0: p=positions[parent]+rotations[parent].apply(p); q=rotations[parent]*q
            positions.append(p); rotations.append(q)
        return positions, rotations


class Reader:
    def __init__(self,path): self.data=Path(path).read_bytes(); self.pos=0
    def take(self,fmt):
        size=struct.calcsize('<'+fmt)
        if self.pos+size>len(self.data): raise ValueError('PMXが途中で終了しています')
        x=struct.unpack_from('<'+fmt,self.data,self.pos); self.pos+=size
        return x[0] if len(x)==1 else list(x)
    def skip(self,n):
        if n<0 or self.pos+n>len(self.data): raise ValueError('PMXの長さが不正です')
        self.pos+=n
    def count(self):
        n=self.take('i')
        if n<0 or n>len(self.data): raise ValueError('PMXの要素数が不正です')
        return n
    def string(self):
        n=self.count(); start=self.pos; self.skip(n)
        return self.data[start:self.pos].decode(self.encoding)
    def index(self,n): return self.take({1:'b',2:'h',4:'i'}[n])


class PMX:
    def __init__(self,path):
        r=Reader(path)
        if r.data[:4]!=b'PMX ': raise ValueError('PMXファイルを指定してください')
        r.skip(4); version=r.take('f')
        if not (abs(version-2)<.001 or abs(version-2.1)<.001): raise ValueError('PMX 2.0 / 2.1に対応しています')
        sizes=[r.take('B') for _ in range(r.take('B'))]
        if len(sizes)<8 or sizes[0] not in (0,1) or sizes[1]>4 or any(x not in (1,2,4) for x in sizes[2:8]): raise ValueError('PMXヘッダーが不正です')
        r.encoding='utf-16-le' if sizes[0]==0 else 'utf-8'
        self.name=r.string()
        for _ in range(3): r.string()
        self.vertices=[]
        for _ in range(r.count()):
            pos=r.take('3f');r.skip(20+sizes[1]*16); mode=r.take('B')
            if mode==0: indices=[r.index(sizes[5])];weights=[1.]
            elif mode in (1,3):
                indices=[r.index(sizes[5]) for _ in range(2)];w=r.take('f');weights=[w,1-w]
                if mode==3:r.skip(36)
            elif mode in (2,4):
                indices=[r.index(sizes[5]) for _ in range(4)];weights=r.take('4f')
            else:raise ValueError('未知のPMXウェイト形式です')
            r.skip(4);self.vertices.append((pos,indices,weights))
        r.skip(r.count()*sizes[2])
        for _ in range(r.count()): r.string()
        for _ in range(r.count()):
            r.string(); r.string(); r.skip(65+2*sizes[3]+1); shared=r.take('B')
            if shared not in (0,1): raise ValueError('PMX Toon指定が不正です')
            r.skip(1 if shared else sizes[3]); r.string(); r.skip(4)
        self.bones=[]
        for i in range(r.count()):
            b=dict(name=r.string(),english=r.string(),position=r.take('3f'),parent=r.index(sizes[5]),layer=r.take('i'),flags=r.take('H'))
            flags=b['flags']; b['tail']=r.index(sizes[5]) if flags&1 else r.take('3f')
            if flags&0x300: b['inherit']=[r.index(sizes[5]),r.take('f')]
            if flags&0x400: b['fixed_axis']=r.take('3f')
            if flags&0x800: r.skip(24)
            if flags&0x2000: r.skip(4)
            if flags&0x20:
                r.skip(sizes[5]+8)
                for _ in range(r.count()):
                    r.skip(sizes[5]); limit=r.take('B')
                    if limit not in (0,1): raise ValueError('PMX IK制限が不正です')
                    if limit: r.skip(24)
            self.bones.append(b)
        self.names={b['name']:i for i,b in enumerate(self.bones)}
        if len(self.names)!=len(self.bones): raise ValueError('PMXに重複ボーン名があります')
        for i,b in enumerate(self.bones):
            if b['parent'] < -1 or b['parent']>=len(self.bones) or b['parent']==i or not np.isfinite(b['position']).all(): raise ValueError('PMXの親・位置が不正です')


def foot_support_points(model, ankle, driven):
    """Rigid shoe samples only; animated toes and deforming ankle blends excluded."""
    aliases={};visiting=set()
    def canonical(i):
        if i<0:return i
        if i in aliases:return aliases[i]
        if i in visiting:return i
        visiting.add(i);b=model.bones[i];answer=i
        if i not in driven and b.get('inherit') and b['flags']&0x100 and not b['flags']&0x3280:
            donor,w=b['inherit']
            if 0<=donor<len(model.bones) and abs(w-1)<1e-7:
                d=model.bones[donor]
                if np.allclose(b['position'],d['position'],atol=1e-6) and canonical(b['parent'])==canonical(d['parent']):answer=canonical(donor)
        visiting.remove(i);aliases[i]=answer;return answer
    rigid={i for i in range(len(model.bones)) if canonical(i)==ankle}
    for _ in range(len(model.bones)):
        add={i for i,b in enumerate(model.bones) if i not in driven and b['parent'] in rigid and 'inherit' not in b and not b['flags']&0x3020}
        if add<=rigid:break
        rigid|=add
    points=[];origin=np.array(model.bones[ankle]['position'])
    for pos,indices,weights in model.vertices:
        if abs(sum(weights)-1)>1e-5 or any(w<0 for w in weights):continue
        if sum(w for i,w in zip(indices,weights) if i in rigid)<.99999:continue
        if np.isfinite(pos).all() and pos[1]<=origin[1]:points.append(np.array(pos)-origin)
    return np.array(points) if len(points)>=3 else None


ROLES={'hips':('Hips','下半身'), 'spine':('Spine','上半身'), 'chest':('Spine1','上半身2'), 'neck':('Neck','首'), 'head':('Head','頭')}
for side,jp in [('Left','左'),('Right','右')]:
    for src,dst in [('Shoulder','肩'),('Arm','腕'),('ForeArm','ひじ'),('Hand','手首'),('UpLeg','足'),('Leg','ひざ'),('Foot','足首'),('ToeBase','つま先')]:
        ROLES[side+src]=(side+src,jp+dst)


def auto_mapping(bvh,pmx):
    def norm(n): return re.sub(r'[^a-z0-9]', '', n.split(':')[-1].lower())
    source={norm(n):n for n in bvh.names}
    result={}
    for role,(src,dst) in ROLES.items():
        if norm(src) in source and dst in pmx.names: result[role]={'source':source[norm(src)],'target':dst}
    return result


def align(a,b):
    a=np.asarray(a,float); b=np.asarray(b,float)
    if min(np.linalg.norm(a),np.linalg.norm(b))<1e-8: raise ValueError('長さゼロのボーンは姿勢補正できません')
    a/=np.linalg.norm(a); b/=np.linalg.norm(b); dot=np.clip(a@b,-1,1)
    if dot<-.999999:
        axis=np.cross(a,np.eye(3)[np.argmin(abs(a))]); axis/=np.linalg.norm(axis)
        return R.from_rotvec(axis*np.pi)
    return R.from_quat(np.r_[np.cross(a,b),1+dot])


def encode_name(name,width):
    try: raw=name.encode('cp932')
    except UnicodeEncodeError as e: raise ValueError('VMDに保存できない文字のボーン名: '+name) from e
    if len(raw)>width: raise ValueError(f'VMDの{width}バイト上限を超える名前: {name}')
    return raw.ljust(width,b'\0')


def write_vmd(path, name, keys, ik):
    # Model label is informational; bone names must never be truncated.
    label=name.encode('cp932',errors='replace')[:20].decode('cp932',errors='ignore')
    encoded={k:encode_name(k,15) for k in keys}; iknames=[encode_name(k,20) for k in ik]
    linear=bytes([20,20,0,0,20,20,20,20,107,107,107,107,107,107,107,107])*4
    with Path(path).open('xb') as f:
        f.write(b'Vocaloid Motion Data 0002'.ljust(30,b'\0')); f.write(encode_name(label,20))
        f.write(struct.pack('<I',sum(len(p) for p,q in keys.values())))
        for name,(p,q) in keys.items():
            if not np.isfinite(p).all() or not np.isfinite(q).all(): raise ValueError('出力に非有限値があります')
            q=q.copy()
            for frame in range(1,len(q)):
                if q[frame-1]@q[frame]<0: q[frame]*=-1
            for frame in range(len(p)): f.write(encoded[name]+struct.pack('<I3f4f',frame,*p[frame],*q[frame])+linear)
        f.write(struct.pack('<4I',0,0,0,0))
        f.write(struct.pack('<IIBI',1,0,1,len(ik)))
        for name in iknames: f.write(name+b'\0')


def convert(bvh_path,pmx_path,out,config=None):
    config=config or {}; out=Path(out)
    if out.exists(): raise FileExistsError('既存の出力は上書きしません: '+str(out))
    b=BVH(bvh_path); m=PMX(pmx_path); mapping=config.get('mapping',auto_mapping(b,m))
    required=['hips','spine','LeftArm','LeftForeArm','LeftHand','RightArm','RightForeArm','RightHand','LeftUpLeg','LeftLeg','LeftFoot','RightUpLeg','RightLeg','RightFoot']
    missing=[r for r in required if r not in mapping]
    if missing: raise ValueError('必須ボーンの対応を設定してください: '+', '.join(missing))
    if len({v['target'] for v in mapping.values()})!=len(mapping): raise ValueError('同じPMXボーンに複数の対応があります')
    for v in mapping.values():
        if v['source'] not in b.names or v['target'] not in m.names: raise ValueError('保存設定とファイルのボーンが一致しません')
    center=config.get('center','センター')
    if center not in m.names or center in {v['target'] for v in mapping.values()}: raise ValueError('移動用のセンターボーンを指定してください')
    basis=np.array(config.get('basis',[[1,0,0],[0,1,0],[0,0,-1]]),float)
    if basis.shape!=(3,3) or not np.allclose(basis@basis.T,np.eye(3)): raise ValueError('座標変換は直交行列で指定してください')
    duration=(b.count-1)*b.dt; n=int(math.ceil(duration*30-1e-9))+1
    if n>300000: raise ValueError('この版の上限は30fpsで300000フレームです')
    t=np.minimum(np.arange(n)/30,duration); sp,sq=b.sample(t,basis)
    def si(role): return b.names[mapping[role]['source']]
    def ti(role): return m.names[mapping[role]['target']]
    def tp(role): return np.array(m.bones[ti(role)]['position'])
    scales=[]
    for side in ('Left','Right'):
        a,c,d=[side+x for x in ('UpLeg','Leg','Foot')]
        source=sum(np.linalg.norm(b.bones[si(k)]['offset']) for k in (c,d))
        if source<1e-8: raise ValueError('BVH脚の長さがゼロです')
        scales.append((np.linalg.norm(tp(c)-tp(a))+np.linalg.norm(tp(d)-tp(c)))/source)
    scale=float(config.get('scale',0)) or float(np.mean(scales))
    height=float(config.get('height_offset',0))
    if not math.isfinite(scale) or scale<=0 or not math.isfinite(height): raise ValueError('倍率・高さが不正です')
    children={'LeftShoulder':'LeftArm','LeftArm':'LeftForeArm','LeftForeArm':'LeftHand','LeftUpLeg':'LeftLeg','LeftLeg':'LeftFoot','LeftFoot':'LeftToeBase'}
    children.update({k.replace('Left','Right'):v.replace('Left','Right') for k,v in list(children.items())})
    desired={}; corrections={}
    for role,v in mapping.items():
        fix=R.identity(); child=children.get(role)
        if child in mapping:
            if b.bones[si(child)]['parent']!=si(role): raise ValueError('対応するBVH関節は直接の親子である必要があります: '+role)
            fix=align(tp(child)-tp(role),basis@np.array(b.bones[si(child)]['offset']))
        elif role.endswith('Hand'):
            fore=role.replace('Hand','ForeArm')
            fix=align(tp(role)-tp(fore),basis@np.array(b.bones[si(role)]['offset']))
        corrections[role]=fix.as_quat().tolist(); desired[ti(role)]=sq[si(role)]*fix
    # Motion relative to its first root sample. The initial PMX hip height is retained.
    root=sp[si('hips')]; shift=(root-root[0])*scale; shift[:,1]+=height
    zero=np.zeros((n,3)); identity=R.identity(n)
    world={}; local={}; keys={}; active=set()
    ci=m.names[center]
    def add_ancestors(i):
        visited=set()
        while i>=0:
            if i in visited: raise ValueError('PMXの親階層に循環があります')
            visited.add(i); active.add(i); i=m.bones[i]['parent']
    for i in desired: add_ancestors(i)
    if ci not in active: raise ValueError('センターが身体ボーンの祖先ではありません')
    for i in desired:
        a=i
        while a>=0 and a!=ci: a=m.bones[a]['parent']
        if a!=ci: raise ValueError('身体ボーンすべてが移動用センターの子孫である必要があります')
    visiting=set()
    def evaluate(i):
        if i<0: return identity
        if i in world: return world[i]
        if i in visiting: raise ValueError('PMXの付与・親に循環があります')
        visiting.add(i); row=m.bones[i]; flags=row['flags']; parent=evaluate(row['parent']); grant=identity
        if flags&0x2000 or flags&0x1000: raise ValueError('身体階層の外部親・物理後変形には未対応です: '+row['name'])
        if 'inherit' in row:
            donor,weight=row['inherit']
            if donor<0 or donor>=len(m.bones) or not math.isfinite(weight): raise ValueError('付与親が不正です')
            if flags&0x80 or flags&0x200: raise ValueError('身体階層のローカル付与・移動付与には未対応です: '+row['name'])
            # Inactive append donors have no VMD motion. Active donors use their local result.
            if donor in active:
                evaluate(donor); grant=R.from_rotvec(local[donor].as_rotvec()*weight)
        if i in desired:
            if flags&0x400: raise ValueError('対応ボーンの固定軸には未対応です: '+row['name'])
            total=parent.inv()*desired[i]; anim=total*grant.inv()
            keys[row['name']]=(zero,anim.as_quat())
        else: total=grant
        local[i]=total; world[i]=parent*total; visiting.remove(i)
        return world[i]
    for i in sorted(active): evaluate(i)
    if np.max(world[ci].magnitude())>1e-6: raise ValueError('回転付与されたセンターには未対応です')
    contact_report={'enabled':False}
    if config.get('contact_enabled',False):
        from contact import correct_feet
        positions={}
        def position(i):
            if i in positions:return positions[i]
            row=m.bones[i];par=row['parent'];p=np.broadcast_to(row['position'],(n,3)).copy()
            if par>=0:
                p=position(par)+world[par].apply(p-np.array(m.bones[par]['position']))
            if i==ci:p+=shift
            positions[i]=p;return p
        legs={}
        for side in ('Left','Right'):
            a,k,f,toe=[side+x for x in ('UpLeg','Leg','Foot','ToeBase')]
            if toe not in mapping:raise ValueError('接地補正には左右のToeBase（つま先）の対応が必要です')
            if m.bones[ti(k)]['parent']!=ti(a) or m.bones[ti(f)]['parent']!=ti(k):
                raise ValueError('接地補正はPMXの足→ひざ→足首が直接の親子であるモデルに対応しています')
            for role in (a,k,f):
                if 'inherit' in m.bones[ti(role)]:raise ValueError('接地補正対象の脚ボーン自身への付与回転には未対応です: '+mapping[role]['target'])
            legs[side]=dict(source_ankle=sp[si(f)],source_toe=sp[si(toe)],source_q=sq[si(f)],source_length=sum(np.linalg.norm(b.bones[si(role)]['offset']) for role in (k,f)),hip=position(ti(a)),knee=position(ti(k)),ankle=position(ti(f)),upper_q=world[ti(a)],lower_q=world[ti(k)],foot_q=world[ti(f)],rest_ankle_height=float(tp(f)[1]),sole_points=foot_support_points(m,ti(f),set(desired)))
        corrected,shift,contact_report=correct_feet(legs,shift,float(config.get('contact_strength',1)),float(config.get('floor_height',0)))
        for side,values in corrected.items():
            for role,key in [('UpLeg','upper_q'),('Leg','lower_q'),('Foot','foot_q')]:desired[ti(side+role)]=values[key]
        world.clear();local.clear();keys.clear()
        for i in sorted(active):evaluate(i)
    keys[center]=(shift,identity.as_quat())
    ik=[row['name'] for row in m.bones if row['flags']&0x20]
    write_vmd(out,m.name,keys,ik)
    report=dict(version='0.3.0-rc1',mode='contact_corrected_fk' if contact_report['enabled'] else 'experimental_fk',contact=contact_report,frames=n,fps=30,source_frames=b.count,source_dt=b.dt,source_duration=duration,scale=scale,mapping=mapping,rest_corrections=corrections,ik_disabled=ik,
        input_sha256={str(p):hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in (bvh_path,pmx_path)},
        output_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),
        limitations=['接地は低速・低位置の区間を推定しFKへ焼き込みます 段差・移動床は未対応です','靴底は足首に剛体追従するPMX頂点から推定します 取得できないモデルは足首高へフォールバックします','指・表情・物理は生成しません','各モデル・モーションの接地品質はMMDで確認してください'])
    return report
