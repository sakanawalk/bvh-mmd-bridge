import json, struct, tempfile, unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from core import BVH, PMX, convert, align
TEST_TEMP=Path(__file__).parent/'test_temp'
TEST_TEMP.mkdir(exist_ok=True)
tempfile.tempdir=str(TEST_TEMP.resolve())


def make_pmx(path, bvh):
    from core import ROLES
    raw=bytearray(b'PMX '+struct.pack('<fB',2.,8)+bytes([1,0,4,4,4,4,4,4]))
    def text(s):
        x=s.encode(); raw.extend(struct.pack('<i',len(x))+x)
    for s in ['Synthetic','Synthetic','','']:text(s)
    raw.extend(struct.pack('<4i',0,0,0,0))
    rows=[('センター',[0,10,0],-1)]
    lookup={}; rest={}
    mapping={src:dst for src,dst in ROLES.values()}
    for i,b in enumerate(bvh.bones):
        p=np.array(b['offset'],float)
        if b['parent']>=0:p+=rest[b['parent']]
        rest[i]=p
        if b['name'] not in mapping:continue
        parent=b['parent']
        while parent>=0 and parent not in lookup:parent=bvh.bones[parent]['parent']
        lookup[i]=len(rows)
        rows.append((mapping[b['name']],(p*.1+np.array([0,10,0])).tolist(),lookup.get(parent,0)))
    raw.extend(struct.pack('<i',len(rows)))
    for name,pos,parent in rows:
        text(name);text('');raw.extend(struct.pack('<3fiiH3f',*pos,parent,0,0x1a,0,1,0))
    path.write_bytes(raw)


def parse_vmd(path):
    data=path.read_bytes(); assert data[:30]==b'Vocaloid Motion Data 0002'.ljust(30,b'\0')
    count=struct.unpack_from('<I',data,50)[0]; rows={}
    for i in range(count):
        offset=54+i*111; name=data[offset:offset+15].split(b'\0')[0].decode('cp932')
        values=struct.unpack_from('<I7f',data,offset+15); rows.setdefault(name,[]).append(values)
    tail=54+count*111
    assert struct.unpack_from('<4I',data,tail)==(0,0,0,0)
    n,frame,visible,ik=struct.unpack_from('<IIBI',data,tail+16)
    assert (n,frame,visible)==(1,0,1)
    assert len(data)==tail+16+13+21*ik
    for values in rows.values():
        a=np.array(values); assert np.isfinite(a).all(); assert np.allclose(np.linalg.norm(a[:,4:8],axis=1),1,atol=1e-6)
    return rows


class CoreTests(unittest.TestCase):
    def test_translation_channel_order_and_rotation(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'test.bvh';path.write_text('''HIERARCHY
ROOT Hips { OFFSET 1 2 3 CHANNELS 6 Zrotation Xposition Yrotation Zposition Xrotation Yposition
JOINT Child { OFFSET 1 0 0 CHANNELS 3 Zrotation Xrotation Yrotation } }
MOTION
Frames: 2
Frame Time: 0.5
90 10 0 30 0 20 0 0 0
90 20 0 40 0 30 0 0 0
''')
            b=BVH(path);p,q=b.sample(np.array([0,.25,.5]),np.eye(3))
            np.testing.assert_allclose(p[0],[[11,22,33],[16,27,38],[21,32,43]])
            np.testing.assert_allclose(p[1]-p[0],np.tile([0,1,0],(3,1)),atol=1e-10)
    def test_end_to_end_and_no_overwrite(self):
        source=Path(__file__).parent/'sample/sample.bvh'
        with tempfile.TemporaryDirectory() as d:
            d=Path(d); b=BVH(source); pmx=d/'target.pmx';make_pmx(pmx,b)
            output=d/'motion.vmd'; report=convert(source,pmx,output)
            rows=parse_vmd(output)
            self.assertEqual(len(rows['センター']),report['frames'])
            self.assertAlmostEqual(report['scale'],.1,places=6)
            self.assertEqual(rows['センター'][0][1:4],(0.,0.,0.))
            # Independently reconstruct target FK from serialized VMD and compare
            # arm directions with the sampled source, including the reflected Z axis.
            target=PMX(pmx); matrices={}; points={}
            f=min(20,report['frames']-1)
            for i,bone in enumerate(target.bones):
                parent=bone['parent']; entry=rows.get(bone['name'])
                q=R.from_quat(entry[f][4:8]) if entry else R.identity()
                delta=np.array(entry[f][1:4]) if entry else np.zeros(3)
                pos=np.array(bone['position'])
                if parent>=0:
                    pos=points[parent]+matrices[parent].apply(pos-np.array(target.bones[parent]['position'])+delta)
                    q=matrices[parent]*q
                else:pos+=delta
                points[i]=pos;matrices[i]=q
            source_pos,_=b.sample(np.array([min(f/30,(b.count-1)*b.dt)]),np.diag([1,1,-1]))
            for side,jp in [('Left','左'),('Right','右')]:
                a=points[target.names[jp+'ひじ']]-points[target.names[jp+'腕']]
                z=source_pos[b.names[side+'ForeArm']][0]-source_pos[b.names[side+'Arm']][0]
                np.testing.assert_allclose(a/np.linalg.norm(a),z/np.linalg.norm(z),atol=1e-6)
            with self.assertRaises(FileExistsError):convert(source,pmx,output)
    def test_alignment_antiparallel(self):
        np.testing.assert_allclose(align([1,0,0],[-1,0,0]).apply([1,0,0]),[-1,0,0],atol=1e-9)
    def test_malformed_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'bad.bvh';p.write_text('ROOT Hips { OFFSET 0 0 0 CHANNELS 0 }\nMOTION\nFrames: 1\nFrame Time: -1\n')
            with self.assertRaises(ValueError):BVH(p)
            p.write_bytes(b'PMX '+struct.pack('<fB',2.,8))
            with self.assertRaises(ValueError):PMX(p)


if __name__=='__main__':unittest.main()
