import math
import pytest
from api.elevator_surface import safe_press_region,associate_panel


def face(radius=.016,n=64):
    return [[radius*math.cos(i*2*math.pi/n),radius*math.sin(i*2*math.pi/n)] for i in range(n)]


def check(p,**kw):
    return safe_press_region(p,perception_error_m=.001,calibration_error_m=.0005,pose_error_m=.0005,reserve_m=.0005,**kw)


def test_combined_budget_preserves_original_limits():
    result=check(face());assert math.dist(result['point_m'],[0,0])<1e-10
    assert result['combined_inset_m']==pytest.approx(.0145)
    assert result['additional_clearance_m']>0
    with pytest.raises(ValueError,match='safe press region'):
        safe_press_region(face(),perception_error_m=.007,calibration_error_m=0,pose_error_m=0,reserve_m=.0005)
    for kw in ({'tool_radius_m':.007},{'control_error_m':.003}):
        with pytest.raises(ValueError):check(face(),**kw)


@pytest.mark.parametrize('polygon',[[[0,0],[1,0],[.5,.1],[1,1],[0,1]],[[0,0],[1,1],[1,0],[0,1]],[[0,0],[1,0],[2,0]]])
def test_invalid_polygon_refuses(polygon):
    with pytest.raises(ValueError):check(polygon)


@pytest.mark.parametrize('angle',[0,.3,1.9])
def test_safe_polygon_is_inside_every_original_edge(angle):
    c,s=math.cos(angle),math.sin(angle)
    polygon=[[.12+c*x-s*y,-.03+s*x+c*y] for x,y in [[-.023,-.018],[.023,-.018],[.023,.018],[-.023,.018]]]
    result=check(polygon[::-1])
    for p in [result['point_m'],*result['safe_polygon_m']]:
        for a,b in zip(polygon,polygon[1:]+polygon[:1]):
            signed=((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]))/math.dist(a,b)
            assert signed>=result['combined_inset_m']-1e-12


def labels():
    return [dict(id='label2',text='2',score=.9,readable=True,strip_box_px=[0,0,100,100]),dict(id='label3',text='3',score=.9,readable=True,strip_box_px=[50,0,100,100])]


def faces():
    return [dict(id='a',observed_complete=True,polygon_px=[[10,20],[30,20],[30,40],[10,40]]),dict(id='b',observed_complete=True,polygon_px=[[60,20],[80,20],[80,40],[60,40]])]


def test_global_assignment_resolves_local_ambiguity_without_reusing_face():
    result=associate_panel(labels(),faces())
    assert result['assignment']=={'label3':'b','label2':'a'}
    assert not result['contact_authorized']


@pytest.mark.parametrize('case',['ambiguous','duplicate','missing','unreadable','nan','duplicate_face','clipped'])
def test_association_refuses_bad_observations(case):
    ls,fs=labels(),faces()
    if case=='ambiguous':ls[1]['strip_box_px']=[0,0,100,100]
    if case=='duplicate':ls[1]['text']='2'
    if case=='missing':fs=fs[:1]
    if case=='unreadable':ls[0]['readable']=False
    if case=='nan':ls[0]['score']=float('nan')
    if case=='duplicate_face':fs[1]['id']='a'
    if case=='clipped':fs[0]['polygon_px'][0][0]=-1
    with pytest.raises(ValueError):associate_panel(ls,fs)


@pytest.mark.parametrize('value',[float('nan'),float('inf'),-1.,True])
def test_invalid_uncertainty_refuses(value):
    with pytest.raises(ValueError):
        safe_press_region(face(),perception_error_m=.001,calibration_error_m=value,pose_error_m=0,reserve_m=.0005)


def test_overflow_geometry_refuses():
    with pytest.raises(ValueError):check([[0,0],[1e300,0],[1e300,1e300],[0,1e300]])


def test_partial_face_cannot_be_admitted_as_complete():
    fs=faces();fs[0]['observed_complete']=False
    with pytest.raises(ValueError,match='visibility'):associate_panel(labels(),fs)


def test_starred_floor_keeps_raw_meaning():
    from api.elevator_surface import floor_control,panel_floor_inventory
    assert floor_control('star1',readable=True,score=.84)==dict(kind='floor',floor='1',main_floor_marker=True,raw_text='star1')
    assert floor_control('4',readable=True,score=.93)['floor']=='4'
    ds=[dict(text=t,text_readable=True,score=.9) for t in ('star1','1','4','alarm','><')]
    inventory=panel_floor_inventory(ds)
    assert inventory['conflicts']=={'1':[0,1]} and not inventory['unambiguous']
    assert len(inventory['floors'])+len(inventory['excluded'])==len(ds)


@pytest.mark.parametrize('text', ['star2','blur','hazy','alarm','01',' 1','1?','１','door_open'])
def test_floor_semantics_does_not_guess(text):
    from api.elevator_surface import floor_control
    with pytest.raises(ValueError):floor_control(text,readable=True,score=.99)


def test_unreadable_number_cannot_be_recovered_from_layout():
    from api.elevator_surface import panel_floor_inventory
    r=panel_floor_inventory([dict(text='6',text_readable=False,score=.94)])
    assert not r['floors'] and len(r['excluded'])==1


def test_association_refuses_competing_main_floor_aliases():
    ls=labels();ls[0]['text']='star1';ls[1]['text']='1'
    with pytest.raises(ValueError,match='Conflicting floor aliases'):associate_panel(ls,faces())
