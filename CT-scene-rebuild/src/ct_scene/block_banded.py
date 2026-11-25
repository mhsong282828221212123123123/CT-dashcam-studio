"""Linear-time block-pentadiagonal SPD solve and diagonal inverse blocks."""
import numpy as np


def solve(diagonal,lower1,lower2,rhs,inverse_diagonal=False):
    a=np.asarray(diagonal,float);n,m,_=a.shape
    d=np.zeros_like(a);di=np.zeros_like(a);l1=np.zeros_like(a);l2=np.zeros_like(a)
    y=np.asarray(rhs,float).copy()
    for i in range(n):
        if i>=2:l2[i]=lower2[i]@di[i-2]
        if i>=1:
            cross=lower1[i]-(l2[i]@d[i-2]@l1[i-1].T if i>=2 else 0.)
            l1[i]=cross@di[i-1]
        d[i]=a[i]
        if i>=1:d[i]-=l1[i]@d[i-1]@l1[i].T;y[i]-=l1[i]@y[i-1]
        if i>=2:d[i]-=l2[i]@d[i-2]@l2[i].T;y[i]-=l2[i]@y[i-2]
        d[i]=(d[i]+d[i].T)/2
        if np.linalg.eigvalsh(d[i]).min()<=0:raise ValueError('Non-positive block precision')
        di[i]=np.linalg.inv(d[i])
    z=np.einsum('nij,nj->ni',di,y);x=z.copy()
    for i in range(n-1,-1,-1):
        if i+1<n:x[i]-=l1[i+1].T@x[i+1]
        if i+2<n:x[i]-=l2[i+2].T@x[i+2]
    if not inverse_diagonal:return x
    cov=np.zeros_like(a);upper1=np.zeros_like(a);upper2=np.zeros_like(a)
    for i in range(n-1,-1,-1):
        if i+2<n:
            upper2[i]=-l1[i+1].T@upper1[i+1]-l2[i+2].T@cov[i+2]
        if i+1<n:
            upper1[i]=-l1[i+1].T@cov[i+1]
            if i+2<n:upper1[i]-=l2[i+2].T@upper1[i+1].T
        cov[i]=di[i]
        if i+1<n:cov[i]-=l1[i+1].T@upper1[i].T
        if i+2<n:cov[i]-=l2[i+2].T@upper2[i].T
        cov[i]=(cov[i]+cov[i].T)/2
    return x,cov
