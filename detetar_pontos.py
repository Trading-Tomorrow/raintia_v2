#!/usr/bin/env python3
"""
Deteta o carro no mapa (obstaculo central) e calcula 4 pontos de inspecao
(frente, tras, esquerda, direita) com margem segura, virados para o carro.
Valida que cada ponto cai em espaco livre. Imprime as poses no frame do mapa.
"""
import numpy as np, cv2, math, yaml

MAPA='/home/nuno/Documents/ArgusAI/map.pgm'
YAML='/home/nuno/Documents/ArgusAI/map.yaml'
MARGEM_LONGO=1.4   # distancia extra alem do meio-comprimento do carro
MARGEM_CURTO=1.6   # distancia extra alem da meia-largura

with open(YAML) as f: meta=yaml.safe_load(f)
res=meta['resolution']; ox,oy,_=meta['origin']
img=cv2.imread(MAPA, cv2.IMREAD_GRAYSCALE); H,W=img.shape

def px(x,y):           # mapa->pixel (linha,col)
    c=int((x-ox)/res); r=H-1-int((y-oy)/res); return r,c
def mundo(r,c):        # pixel->mapa
    x=c*res+ox; y=(H-1-r)*res+oy; return x,y

occ=(img<100).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(occ,8)
# carro = componente grande mais perto do centro da imagem, que NAO toca a borda
cx_img,cy_img=W/2,H/2; melhor=None
for i in range(1,n):
    a=stats[i,cv2.CC_STAT_AREA]
    if a<60: continue
    x0,y0,w0,h0=stats[i,cv2.CC_STAT_LEFT],stats[i,cv2.CC_STAT_TOP],stats[i,cv2.CC_STAT_WIDTH],stats[i,cv2.CC_STAT_HEIGHT]
    if x0<=1 or y0<=1 or x0+w0>=W-1 or y0+h0>=H-1: continue  # toca borda = parede
    d=math.hypot(cent[i][0]-cx_img,cent[i][1]-cy_img)
    if melhor is None or d<melhor[0]: melhor=(d,i)
ci=melhor[1]
pts=np.column_stack(np.where(lab==ci))[:,::-1].astype(np.float32)  # (col,row)
rect=cv2.minAreaRect(pts)   # ((cx,cy),(w,h),ang)
(ccx,ccy),(rw,rh),ang=rect
carx,cary=mundo(ccy,ccx)    # centro do carro no mapa (cuidado: row=ccy,col=ccx)
# dimensoes em metros
L=max(rw,rh)*res; Wd=min(rw,rh)*res
# angulo do eixo LONGO em rad (no frame do mapa)
aL=math.radians(ang if rw>=rh else ang+90)
aS=aL+math.pi/2
print(f'CARRO: centro=({carx:.2f},{cary:.2f}) comprimento={L:.2f}m largura={Wd:.2f}m eixo_longo={math.degrees(aL):.0f}deg')

def livre(x,y,raio=0.3):
    r,c=px(x,y)
    rr=int(raio/res)
    sub=img[max(0,r-rr):r+rr,max(0,c-rr):c+rr]
    return sub.size>0 and np.all(sub>200)   # tudo livre na vizinhanca

def ponto(ang_dir, dist, nome):
    x=carx+dist*math.cos(ang_dir); y=cary+dist*math.sin(ang_dir)
    th=math.atan2(cary-y,carx-x)   # virado para o carro
    ok=livre(x,y,0.3)
    # se nao livre, afasta mais ate 1m extra
    d=dist
    while not ok and d<dist+1.0:
        d+=0.2; x=carx+d*math.cos(ang_dir); y=cary+d*math.sin(ang_dir); ok=livre(x,y,0.3)
    estado = "LIVRE" if ok else "BLOQUEADO!"
    print(f'  {nome}: ({x:.2f}, {y:.2f}, {math.degrees(th):.0f}deg)  d={d:.1f}m  {estado}')
    return (round(x,2),round(y,2),round(th,3),ok)

print('PONTOS (x, y, theta) no frame do mapa:')
P=[]
P.append(('frente', ponto(aL, L/2+MARGEM_LONGO, 'frente')))
P.append(('tras',   ponto(aL+math.pi, L/2+MARGEM_LONGO, 'tras')))
P.append(('esq',    ponto(aS, Wd/2+MARGEM_CURTO, 'esquerda')))
P.append(('dir',    ponto(aS+math.pi, Wd/2+MARGEM_CURTO, 'direita')))
print('\nLista para o navegacao.py:')
print('self.pontos_inspecao = [')
for nome,(x,y,th,ok) in P:
    print(f'    ({x}, {y}, {th}),   # {nome}')
print(']')
