import sys,glob,re,os,math,json
sys.path.insert(0,'.')
from swmesh import read_mesh_full, parse_transform
import zrender
def origin(t):
    m=re.match(r'mega_island_(\d+)_(\d+)$',t); return int(m.group(1))*1000-8000,int(m.group(2))*1000-12000
def tris_world(work, rom, wx, wz, radius, terrain=True):
    out=[]
    tiles=set()
    for f in glob.glob(f'{rom}/data/tiles/mega_island_*_*.xml'):
        t=os.path.basename(f)[:-4]
        if not re.match(r'mega_island_\d+_\d+$',t): continue
        ox,oz=origin(t)
        if abs(ox-wx)<=500+radius and abs(oz-wz)<=500+radius: tiles.add(t)
    for t in tiles:
        ox,oz=origin(t)
        files=[(f,None) for f in glob.glob(f'{work}/track_meshes/{t}_*.mesh')]
        if terrain:
            text=open(f'{rom}/data/tiles/{t}.xml',encoding='utf-8').read()
            for m in re.finditer(r'<mesh id="[^"]*"(?: parent_id="[^"]*")? file_name="([^"]+)">\s*<transform([^/]*)/>',text):
                p=os.path.join(rom,*m.group(1).split('/'))
                if os.path.exists(p): files.append((p,parse_transform(m.group(2))))
        for f,M in files:
            mesh=read_mesh_full(f); V=mesh.vertices
            def tp(v):
                if M is None: return (v[0]+ox,v[1],v[2]+oz)
                return (v[0]*M[0][0]+v[1]*M[1][0]+v[2]*M[2][0]+M[3][0]+ox, v[0]*M[0][1]+v[1]*M[1][1]+v[2]*M[2][1]+M[3][1], v[0]*M[0][2]+v[1]*M[1][2]+v[2]*M[2][2]+M[3][2]+oz)
            for i in range(0,len(mesh.indices),3):
                tri=[V[mesh.indices[i+j]] for j in range(3)]
                P=[tp(v) for v in tri]
                if all(math.hypot(p[0]-wx,p[2]-wz)>radius for p in P): continue
                n=tri[0][7:10]
                if M is not None:
                    n=(n[0]*M[0][0]+n[1]*M[1][0]+n[2]*M[2][0], n[0]*M[0][1]+n[1]*M[1][1]+n[2]*M[2][1], n[0]*M[0][2]+n[1]*M[1][2]+n[2]*M[2][2])
                out.append((P[0],P[2],P[1],P[1],n,tri[0][3:7],None))
    return out
if __name__=='__main__':
    work,rom=sys.argv[1],sys.argv[2]; wx,wz,r=map(float,sys.argv[3:6]); eye=tuple(map(float,sys.argv[6].split(','))); tgt=tuple(map(float,sys.argv[7].split(','))); out=sys.argv[8]
    T=tris_world(work,rom,wx,wz,r)
    zrender.render(T,eye,tgt,out,fov=60,title=out)
