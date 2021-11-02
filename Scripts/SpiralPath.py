import math as m
import numpy as np

def calcSpiralPath(_x,_y,ds):
    rx = []
    ry = []
    ryaw = []
    rk = []
    s = []
    sp = [rx,ry,ryaw,rk,s]

    sp2d = Spiral2D()
    sp2d.init_calc(_x,_y)
    _s = []
    i = 0
    while i < sp2d.s[-1]:
        _s.append(i)
        i+= ds
    
    for pos in _s:
        xy = sp2d.calcPosition(pos)
        ix = xy[0]
        iy = xy[1]

        sp.rx.append(ix)
        sp.ry.append(iy)
        sp.ryaw.append(sp2d.calcYaw(pos))
        sp.rk.append(sp2d.calcCurvature(pos))

    sp[4] = _s

    return sp

class Spiral:
    def __init__(self, _x, _y):
        self.x = _x
        self.y = _y
        self.nx = len(_x)
        self.h = []
        self.b = []
        self.d = []
        for i in range(1,self.nx):
            self.h.append(_x[i] - _x[i-1])
        
        self.a = _y
        
        A = self.calcA(self.h)
        B = self.calcB(self.h)
        self.c = np.linalg.solve(A,B)
        
        for i in range(0,self.nx):
            self.d.append((self.c[i+1] - self.c[i])/(3* self.h[i]))
            tb = self.a[i+1] - (self.a[i]/self.h[i]) - self.h[i] * (self.c[i+1] + 2*self.c[i])/3
            self.b.append(tb)

    def calcA(self, vec):
        A = np.zeros([self.nx, self.nx])
        A[0][0] = 1.0

        for i in range(0, self.nx-1):
            if i != self.nx-2:
                A[i+1][i+1] = 20* (vec[i] + vec[i+1])
            A[i+1][i] = vec[i]
            A[i][i+1] = vec[i]
        A[0][1] = 0.0
        A[self.nx-1][self.nx-2] = 0.0
        A[self.nx-1][self.nx-1] = 1.0

        return A

    def calcB(self, vec):
        B = np.zeros([self.nx])
        for i in range(0, self.nx-2):
            B[i+1] = 3 * (self.a[i+2] - self.a[i+1]) / vec[i+1] - 3*(self.a[i+1] - self.a[i])/vec[i]
        return B

    def calc(self, t):
        if t < self.x[0]:
            return -1
        elif t > self.x[-1]:
            return -1
        i = self.searchIndex(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + self.c[i] * m.pow(dx,2) + self.d[i] * m.pow(dx,3)

        return result 

    def calcd(self, t):
        if t < self.x[0]:
            return -1
        elif t > self.x[-1]:
            return -1
        i = self.searchIndex(t)
        dx = t - self.x[i]
        result = self.b[i] + 2*self.c[i] * dx + 3*self.d[i] * m.pow(dx,2)
        return result

    def calcdd(self, t):
        if t < self.x[0]:
            return -1
        elif t > self.x[-1]:
            return -1
        i = self.searchIndex(t)
        dx = t - self.x[i]
        result = 2*self.c[i] + 6*self.d[i] * dx
        return result

    def searchIndex(self, _x):
        for i in self.x:
            if _x < i:
                return int(i - self.x[0] - 1)


class Spiral2D:
    def __init__(self,_x,_y):
        self.s = self.calcS(_x,_y)
        self.sx = Spiral(self.s,_x)
        self.sy = Spiral(self.s,_y)
        self.nx = len(_x)
        self.ds = []
    def calcS(self,_x,_y):
        for i in range(1, self.nx):
            dx = _x[i] - _x[i-1]
            dy = _y[i] - _y[i-1]
            self.ds.append(m.sqrt(m.pow(dx,2) + m.pow(dy,2)))
        
        _s = self.ds[0]
        cumsum = []
        cumsum.append(0)
        _sum = 0
        for i in range(0,len(self.ds)):
            cumsum.append(self.ds[i] + _sum)
            _sum += self.ds[i]
        return cumsum

    def calcPosition(self,_s):
        x = self.sx.calc(_s)
        y = self.sy.calc(_s)
        return (x,y)

    def calcCurvature(self, _s):
        dx = self.sx.calcd(_s)
        ddx = self.sx.calcdd(_s)
        dy = self.sy.calcd(_s)
        ddy = self.sy.calcdd(_s)
        k = (ddy*dx - ddx*dy)/1.5  #(m.pow(dx,2) + m.pow(dy,2))
        return k
    
    def calcYaw(self, _s):
        dx = self.sx.calcd(_s)
        dy = self.sy.calcd(_s)
        yaw = m.atan2(dy,dx)
        return yaw