"""Opt-in image-only face localization for label-left, round-button-right strips.

No scene geometry or floor identity enters this function. OpenCV is an optional
perception-runtime dependency, not a dependency of the Rax behavior API.
"""
import math

METHOD = 'right-round-face-v1'


def localize_right_round_face(rgb, box):
    return _localize_right_round_face(rgb,box,edge_threshold=50,method=METHOD)


def localize_right_round_face_low_contrast(rgb, box):
    """Explicit low-contrast mode; never a fallback after a refused target."""
    return _localize_right_round_face(rgb,box,edge_threshold=20,method='right-round-face-v2')


def _localize_right_round_face(rgb, box, *, edge_threshold, method):
    import cv2
    import numpy as np
    image=np.asarray(rgb)
    if image.ndim!=3 or image.shape[2]!=3 or image.dtype!=np.uint8:
        raise ValueError('Expected an RGB uint8 image')
    if len(box)!=4 or any(type(v) not in (int,float) or not math.isfinite(v) for v in box):
        raise ValueError('Invalid detection box')
    x0,y0,x1,y1=box;h=y1-y0;w=x1-x0
    if not (0<=x0<x1<image.shape[1] and 0<=y0<y1<image.shape[0] and h>=24 and 1.4<=w/h<=4):
        raise ValueError('Detection does not match the calibrated strip layout')
    pad=math.ceil(.12*h);left=max(0,math.floor(x1-1.1*h));top=max(0,math.floor(y0)-pad)
    crop=image[top:min(image.shape[0],math.ceil(y1)+pad),left:min(image.shape[1],math.ceil(x1)+pad)]
    gray=cv2.cvtColor(crop,cv2.COLOR_RGB2GRAY)
    candidates=[]
    for kernel in (5,7,9):
        smooth=cv2.GaussianBlur(gray,(kernel,kernel),0)
        circles=cv2.HoughCircles(smooth,cv2.HOUGH_GRADIENT,1,max(12,int(.28*h)),
            param1=edge_threshold,param2=12,minRadius=int(.25*h),maxRadius=int(.55*h))
        valid=[]
        for x,y,r in (() if circles is None else circles[0]):
            x,y,r=float(x+left),float(y+top),float(r)
            # The entire face must fit inside the detection; no clipped rim.
            if x>=x0+w/2 and x-r>=x0 and x+r<=x1 and y-r>=y0 and y+r<=y1:
                valid.append([x,y,r])
        if len(valid)!=1:
            raise ValueError('Missing or ambiguous round button face')
        candidates.append(valid[0])
    median=np.median(candidates,axis=0)
    spread=max(float(np.linalg.norm(np.asarray(c[:2])-median[:2])) for c in candidates)
    radius_spread=max(abs(c[2]-median[2]) for c in candidates)
    if spread>3. or radius_spread>3.:
        raise ValueError('Button face is unstable across smoothing scales')
    return dict(method=method,edge_threshold=edge_threshold,pixel=median[:2].tolist(),radius_px=float(median[2]),
        center_spread_px=spread,radius_spread_px=float(radius_spread),candidates=candidates,
        smoothing_kernels=[5,7,9],detection_box=list(box),opencv_version=cv2.__version__)
