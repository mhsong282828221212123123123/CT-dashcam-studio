"""Shared oriented footprint checks in host X/Z coordinates."""
import math


def footprints_overlap(a,b,position_a=None):
    ax,az=position_a if position_a is not None else (a.x,a.z)
    delta=(b.x-ax,b.z-az)
    def axes(actor):
        c,s=math.cos(actor.yaw),math.sin(actor.yaw)
        return (c,-s),(s,c)
    A,B=axes(a),axes(b)
    dot=lambda u,v:u[0]*v[0]+u[1]*v[1]
    for axis in (*A,*B):
        radius_a=(a.width*abs(dot(A[0],axis))+a.length*abs(dot(A[1],axis)))/2
        radius_b=(b.width*abs(dot(B[0],axis))+b.length*abs(dot(B[1],axis)))/2
        if abs(dot(delta,axis))>=radius_a+radius_b-1e-8:return False
    return True
