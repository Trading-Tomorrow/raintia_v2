#!/usr/bin/env python3
"""
Resolve a ordem otima de visita aos 4 pontos (problema do caixeiro-viajante).
Usa A* sobre a grelha do mapa para medir a distancia REAL entre cada par de
pontos (a contornar o carro/paredes), e depois testa todas as ordens possiveis
(base -> 4 pontos -> base) para escolher a mais curta.
"""
import heapq, math, itertools
import numpy as np, cv2, yaml

MAPA='/home/nuno/Documents/ArgusAI/map.pgm'
YAML='/home/nuno/Documents/ArgusAI/map.yaml'
INFLAR_M=0.25   # margem de seguranca a obstaculos (m)

PONTOS = {
    'base':   (0.0,  0.0),
    'frente': (10.17, 4.86),
    'tras':   (3.10,  4.49),
    'esq':    (6.50,  7.24),
    'dir':    (6.77,  2.11),
}

with open(YAML) as f: meta=yaml.safe_load(f)
res=meta['resolution']; ox,oy,_=meta['origin']
img=cv2.imread(MAPA, cv2.IMREAD_GRAYSCALE); H,W=img.shape

# transitavel = livre OU desconhecido, menos obstaculos inflacionados
occ=(img<100).astype(np.uint8)
k=int(INFLAR_M/res)
occ_inf=cv2.dilate(occ, np.ones((2*k+1,2*k+1),np.uint8))
livre = occ_inf==0   # True onde se pode andar

def to_px(x,y):
    return (H-1-int((y-oy)/res), int((x-ox)/res))   # (row,col)

def snap(rc):
    # se o ponto cair em obstaculo, procura a celula livre mais perto
    r,c=rc
    if 0<=r<H and 0<=c<W and livre[r,c]: return (r,c)
    for raio in range(1,15):
        for dr in range(-raio,raio+1):
            for dc in range(-raio,raio+1):
                rr,cc=r+dr,c+dc
                if 0<=rr<H and 0<=cc<W and livre[rr,cc]: return (rr,cc)
    return rc

DIRS=[(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),(-1,-1,1.414),(-1,1,1.414),(1,-1,1.414),(1,1,1.414)]
def astar(ini,fim):
    ini=snap(ini); fim=snap(fim)
    def h(a): return math.hypot(a[0]-fim[0],a[1]-fim[1])
    oset=[(h(ini),0.0,ini)]; g={ini:0.0}; vis=set()
    while oset:
        _,gc,n=heapq.heappop(oset)
        if n==fim: return gc*res
        if n in vis: continue
        vis.add(n)
        for dr,dc,cost in DIRS:
            m=(n[0]+dr,n[1]+dc)
            if not (0<=m[0]<H and 0<=m[1]<W) or not livre[m[0],m[1]]: continue
            ng=gc+cost
            if ng<g.get(m,1e18):
                g[m]=ng; heapq.heappush(oset,(ng+h(m),ng,m))
    return float('inf')

# matriz de distancias (A* entre todos os pares)
nomes=list(PONTOS); px={n:to_px(*PONTOS[n]) for n in nomes}
D={}
for a,b in itertools.combinations(nomes,2):
    d=astar(px[a],px[b]); D[(a,b)]=d; D[(b,a)]=d
print('Distancias A* (m):')
for a,b in itertools.combinations(nomes,2):
    print(f'  {a:7s} <-> {b:7s} : {D[(a,b)]:.2f}')

# TSP: base fixo no inicio e fim; testar todas as ordens dos 4 pontos
alvos=['frente','tras','esq','dir']
melhor=None
for perm in itertools.permutations(alvos):
    seq=['base']+list(perm)+['base']
    tot=sum(D[(seq[i],seq[i+1])] for i in range(len(seq)-1))
    if melhor is None or tot<melhor[0]: melhor=(tot,perm)

tot,ordem=melhor
print(f'\nORDEM OTIMA: base -> {" -> ".join(ordem)} -> base')
print(f'DISTANCIA TOTAL: {tot:.2f} m')
print('\nPara o navegacao.py (ordem + nomes):')
for nome in ordem:
    x,y=PONTOS[nome]
    print(f"  {nome}: ({x}, {y})")
