"""Fuse independent camera-space curve hypotheses and paint segmentation.

This module never infers the number of lanes from vehicles or map tags.
Unknown road edges stay unknown. It is experimental until sequence validation.
"""
import cv2
import numpy as np


def segmented_curves(paint_mask,road_mask,objects=()):
    """Extract visible longitudinal components without a fixed lane-count cap."""
    height,width=paint_mask.shape
    mask=(paint_mask.astype(bool)&cv2.dilate(road_mask.astype(np.uint8),np.ones((15,15),np.uint8)).astype(bool)).astype(np.uint8)
    mask[:int(height*.54)]=0;mask[int(height*.93):]=0
    for obj in objects:
        x,y,w,h=map(round,obj['bbox'])
        cv2.rectangle(mask,(max(0,x-3),max(0,y-3)),(min(width-1,x+w+3),min(height-1,y+h+3)),0,-1)
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    curves=[]
    for label in range(1,count):
        x,y,w,h,area=stats[label]
        if h<height*.065 or w/max(h,1)>3 or area<height*.15:continue
        points=[]
        for row in range(y,y+h,max(1,height//160)):
            pixels=np.flatnonzero(labels[row]==label)
            if not len(pixels) or len(pixels)>width*.045:continue
            points.append([float(np.median(pixels)),float(row)])
        if len(points)<6:continue
        points=np.array(points);ys=points[:,1]/height
        if np.ptp(ys)<.065:continue
        polynomial=np.polyfit(ys,points[:,0]/width,2)
        errors=abs(np.polyval(polynomial,ys)*width-points[:,0])
        if np.percentile(errors,90)>width*.01:continue
        samples=np.linspace(ys.min(),ys.max(),30)
        fitted=np.column_stack([np.polyval(polynomial,samples)*width,samples*height])
        curves.append(dict(id=f'paint-{label}',points=fitted.tolist(),
                           source='longitudinal_paint_component',boundary_type='unknown',
                           observed=True,extrapolated=False,support_points=points.tolist()))
    return curves


def supported_curves(hypotheses,paint_mask,road_mask,objects=()):
    height,width=paint_mask.shape
    paint=paint_mask.astype(np.uint8).copy()
    occluded=np.zeros_like(paint)
    for obj in objects:
        x,y,w,h=map(round,obj['bbox'])
        cv2.rectangle(occluded,(max(0,x-3),max(0,y-3)),
                      (min(width-1,x+w+3),min(height-1,y+h+3)),1,-1)
    paint[occluded!=0]=0
    road=cv2.dilate(road_mask.astype(np.uint8),np.ones((15,15),np.uint8))
    output=[]
    for hypothesis in hypotheses:
        points=np.asarray(hypothesis['points'],float)
        accepted=[];eligible=0
        for x,y in points:
            ix,iy=round(x),round(y)
            if not (0<=ix<width and .48*height<iy<.93*height):continue
            if occluded[iy,ix]:continue
            eligible+=1
            radius=max(5,round(width*.015))
            left,right=max(0,ix-radius),min(width,ix+radius+1)
            near=np.flatnonzero(paint[iy,left:right]&road[iy,left:right])+left
            if len(near):accepted.append([float(near[np.argmin(abs(near-ix))]),float(iy)])
        if len(accepted)<5 or len(accepted)<max(1,eligible)*.5:continue
        accepted=np.array(accepted);ys=accepted[:,1]/height
        if np.ptp(ys)<.12:continue
        polynomial=np.polyfit(ys,accepted[:,0]/width,2)
        residual=abs(np.polyval(polynomial,ys)*width-accepted[:,0])
        keep=residual<width*.015
        if keep.sum()<5 or keep.mean()<.8:continue
        polynomial=np.polyfit(ys[keep],accepted[keep,0]/width,2)
        samples=np.linspace(ys[keep].min(),ys[keep].max(),40)
        xs=np.polyval(polynomial,samples)*width
        if np.any((xs<0)|(xs>=width)):continue
        output.append(dict(id=hypothesis['id'],points=np.column_stack([xs,samples*height]).tolist(),
                           source='independent_curve_and_paint_support',boundary_type='unknown',
                           observed=True,extrapolated=False,support_points=accepted[keep].tolist(),
                           support_fraction=len(accepted)/max(1,eligible)))
    return output


def road_cross_sections(mask):
    """Keep sampled road extents, without filling intersections with lane grids."""
    mask=mask.astype(np.uint8)
    height,width=mask.shape
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    candidates=[i for i in range(1,count)
                if stats[i,cv2.CC_STAT_AREA]>height*width*.02
                and np.any(labels[int(height*.7):int(height*.9),int(width*.3):int(width*.7)]==i)]
    if not candidates:return []
    selected=max(candidates,key=lambda i:stats[i,cv2.CC_STAT_AREA])
    sections=[]
    for y in range(int(height*.50),int(height*.91),max(1,height//80)):
        pixels=np.flatnonzero(labels[y]==selected)
        if len(pixels)<width*.08:continue
        sections.append([[float(pixels[0]),float(y)],[float(pixels[-1]),float(y)]])
    return sections
