"""Pure observation association and conservative press-region admission.

No detector, source geometry, motion, or contact permission is supplied here.
Coordinates for association are image pixels; press polygons are metric panel
coordinates supplied by a separately validated observation provider.
"""
import math


def _number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('Expected finite numeric geometry')
    return float(value)


def _polygon(value):
    if not isinstance(value,(list,tuple)) or not 3<=len(value)<=64:
        raise ValueError('Expected a bounded convex face polygon')
    if any(not isinstance(v,(list,tuple)) or len(v)!=2 for v in value):
        raise ValueError('Expected two-dimensional vertices')
    p=[tuple(_number(x) for x in v) for v in value]
    if any(len(v)!=2 for v in p) or len(set(p))!=len(p):
        raise ValueError('Invalid face vertices')
    area=sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(p,p[1:]+p[:1]))
    if not math.isfinite(area) or abs(area)<1e-12:raise ValueError('Degenerate or unbounded face')
    if area<0:p.reverse()
    # Every other vertex must be on the inward side of every edge. This also
    # excludes self-intersecting and concave inputs, without taking their hull.
    for a,b in zip(p,p[1:]+p[:1]):
        if math.dist(a,b)<1e-9:raise ValueError('Degenerate edge')
        if any((b[0]-a[0])*(v[1]-a[1])-(b[1]-a[1])*(v[0]-a[0]) < -1e-12 for v in p):
            raise ValueError('Face must be convex and ordered')
    return p


def safe_press_region(face_m, *, perception_error_m, calibration_error_m,
                      pose_error_m, reserve_m, tool_radius_m=.008,
                      control_error_m=.004):
    """Erode an observed convex face by the complete conservative error budget.

    Error bounds are inputs, not inferred confidence. The current tool and
    controller allowance are fixed. This is an extra gate, not permission to
    alter the existing metric, source, stop or contact checks.
    """
    polygon=_polygon(face_m)
    values=[_number(v) for v in (perception_error_m,calibration_error_m,pose_error_m,reserve_m,tool_radius_m,control_error_m)]
    perception,calibration,pose,reserve,tool,control=values
    if not 0<perception<=.007 or min(calibration,pose)<0 or reserve<=0 or tool!=.008 or control!=.004:
        raise ValueError('Invalid uncertainty or changed existing limits')
    distance=sum(values)
    if not math.isfinite(distance):raise ValueError('Unbounded error budget')
    clipped=list(polygon);edges=[]
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        length=math.dist(a,b);normal=(-(b[1]-a[1])/length,(b[0]-a[0])/length)
        offset=sum(x*y for x,y in zip(normal,a))+distance;edges.append((normal,offset))
        if not clipped:break
        result=[]
        for u,v in zip(clipped,clipped[1:]+clipped[:1]):
            du=sum(x*y for x,y in zip(normal,u))-offset
            dv=sum(x*y for x,y in zip(normal,v))-offset
            if du>=0:result.append(u)
            if (du>=0)!=(dv>=0):
                t=du/(du-dv);result.append(tuple(x+t*(y-x) for x,y in zip(u,v)))
        clipped=result
    if len(clipped)<3:
        raise ValueError('No safe press region under combined error budget')
    area=abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(clipped,clipped[1:]+clipped[:1])))/2
    if area<=1e-12:raise ValueError('No positive-area safe press region')
    point=tuple(sum(v[i] for v in clipped)/len(clipped) for i in (0,1))
    clearance=min(sum(x*y for x,y in zip(n,point))-o for n,o in edges)
    if clearance < -1e-12:raise ValueError('Safe point verification failed')
    return dict(point_m=list(point),safe_polygon_m=[list(p) for p in clipped],
                combined_inset_m=distance,additional_clearance_m=max(0.,clearance),
                error_bounds_m=dict(perception=perception,calibration=calibration,pose=pose,control=control),
                tool_radius_m=tool,reserve_m=reserve,contact_authorized=False)


def associate_panel(labels, faces):
    """Find one globally unique label-strip/independent-face assignment.

    A label record contains id, text, score, readable, and its detected strip
    box. Face records contain id and an independently supplied pixel polygon.
    No missing face or unreadable label is synthesized from panel layout.
    """
    if not 1<=len(labels)<=16 or not 1<=len(faces)<=32:
        raise ValueError('Invalid panel candidate count')
    if any(not isinstance(x.get('id'),str) or not x['id'] for x in [*labels,*faces]):
        raise ValueError('Candidate identity missing')
    if len({x['id'] for x in labels})!=len(labels) or len({x['id'] for x in faces})!=len(faces):
        raise ValueError('Duplicate candidate identity')
    if len({x.get('text') for x in labels})!=len(labels):raise ValueError('Duplicate panel labels')
    floor_keys=[]
    for label in labels:
        try:floor_keys.append(floor_control(label.get('text'),readable=label.get('readable'),score=label.get('score'))['floor'])
        except ValueError:pass
    if len(set(floor_keys))!=len(floor_keys):raise ValueError('Conflicting floor aliases')
    if any(f.get('observed_complete') is not True for f in faces):
        raise ValueError('Face boundary visibility is unverified')
    polygons={f['id']:_polygon(f['polygon_px']) for f in faces}
    edges={}
    for label in labels:
        if not isinstance(label.get('text'),str) or not label['text'] or label.get('readable') is not True or not .6<=_number(label['score'])<=1:
            raise ValueError('Unresolved panel label')
        box=[_number(v) for v in label['strip_box_px']]
        if len(box)!=4 or not box[0]<box[2] or not box[1]<box[3]:raise ValueError('Invalid strip box')
        edges[label['id']]=[ident for ident,p in polygons.items() if all(box[0]<=x<=box[2] and box[1]<=y<=box[3] for x,y in p)]
        if not edges[label['id']]:raise ValueError('Label has no complete face candidate')
    order=sorted(edges,key=lambda k:len(edges[k]));solutions=[];visits=0
    def search(i,mapping,used):
        nonlocal visits
        visits+=1
        if visits>10000:raise ValueError('Panel association search budget exceeded')
        if len(solutions)>1:return
        if i==len(order):solutions.append(dict(mapping));return
        key=order[i]
        for face in edges[key]:
            if face not in used:
                mapping[key]=face;search(i+1,mapping,used|{face});mapping.pop(key)
    search(0,{},set())
    if len(solutions)!=1:raise ValueError('Missing or ambiguous panel association')
    return dict(assignment=solutions[0],candidate_edges=edges,contact_authorized=False,
                scope='Unique association of supplied observations; no metric pose or contact qualification')


def floor_control(text, *, readable, score):
    """Interpret explicit released floor tokens; never infer from row position.

    Preserve the raw token and the main-floor marker. Service controls and
    unreadable outputs are not floor-selection candidates.
    """
    if readable is not True or not .6<=_number(score)<=1 or not isinstance(text,str):
        raise ValueError('Unresolved floor label')
    main=False;token=text
    if text=='star1':token='1';main=True
    if not token or not token.isascii() or not token.isdecimal() or (len(token)>1 and token.startswith('0')):
        raise ValueError('Not an explicit supported floor token')
    return dict(kind='floor',floor=token,main_floor_marker=main,raw_text=text)


def panel_floor_inventory(detections):
    """Account for every detector output without treating every text as a floor."""
    accepted=[];excluded=[];by_floor={}
    for index,detection in enumerate(detections):
        try:
            control=floor_control(detection.get('text'),readable=detection.get('text_readable'),score=detection.get('score'))
            record=dict(index=index,**control)
            accepted.append(record);by_floor.setdefault(control['floor'],[]).append(index)
        except ValueError as error:
            excluded.append(dict(index=index,raw_text=detection.get('text'),reason=str(error)))
    conflicts={floor:indices for floor,indices in by_floor.items() if len(indices)>1}
    return dict(floors=accepted,excluded=excluded,conflicts=conflicts,
                unambiguous=not conflicts,contact_authorized=False)
