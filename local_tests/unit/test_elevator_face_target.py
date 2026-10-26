from dataclasses import replace
import pytest
from test_elevator import RobotContext, settings, posture
from auto_research_agent.api.elevator import Elevator
from auto_research_agent.api.base import PrimitiveError


class FaceContext(RobotContext):
    def __init__(self,change=None):super().__init__();self.change=change
    def value(self,tool,**args):
        value=super().value(tool,**args)
        if tool=='rax_detect_buttons':
            camera=value['cameras'][args['camera']];camera.update(image_sha256='a'*64,processed_image_sha256='b'*64)
            d=value['detections'][0];d['box']=[200,200,400,280]
            d['press_target']=dict(method='right-round-face-v1',pixel=[355,240],radius_px=30.,center_spread_px=1.,radius_spread_px=1.,detection_box=d['box'].copy(),capture_us=camera['capture_us'],image_sha256='a'*64,processed_image_sha256='b'*64)
            if self.change:self.change(d['press_target'])
        return value


def test_face_pixel_replaces_strip_center_only_when_explicit():
    c=FaceContext();e=Elevator(c,settings(button_target_method='right-round-face-v1'))
    assert e.observe_button('2','right')
    assert [a for t,a in c.calls if t=='rax_camera_ray'][-1]['pixel']==[355.,240.]
    c=FaceContext();e=Elevator(c,settings());e.observe_button('2','right')
    assert [a for t,a in c.calls if t=='rax_camera_ray'][-1]['pixel']==[300.,240.]


@pytest.mark.parametrize('key,value',[('method','other'),('image_sha256','c'*64),('processed_image_sha256','c'*64),('capture_us',0),('center_spread_px',3.01),('radius_spread_px',float('nan')),('pixel',[210,240]),('radius_px',100.),('detection_box',[0,0,1,1])])
def test_bad_face_refuses_before_ray_or_motion(key,value):
    c=FaceContext(lambda t:t.update({key:value}));e=Elevator(c,settings(button_target_method='right-round-face-v1'))
    with pytest.raises((PrimitiveError,ValueError)):e.observe_button('2','right')
    assert not any(t=='rax_camera_ray' or t.startswith('evo_move') for t,a in c.calls)


def staged_settings():
    cfg=settings(synchronized_view=True)
    p=replace(cfg.arms[1],preparation_joints=(posture('right',.8),),stow_joints=(posture('right',.8),))
    return replace(cfg,arms=(p,))


def test_waypoints_have_checked_prepare_and_stow_without_initial_unfold():
    c=RobotContext();e=Elevator(c,staged_settings());e.stow_arms()
    assert len([x for x in c.calls if x[0]=='evo_move_joints'])==1
    e.prepare_button_view(('right',));e.stow_arms()
    actions=[a['targets'] for t,a in c.calls if t=='evo_move_joints']
    assert [q['right_joint4'] for q in actions]==[0.,.8,1.,.8,0.]
    assert not e._viewing_arms


def test_failed_waypoint_stop_prevents_view_dispatch():
    c=RobotContext();c.fail_action='evo_move_joints';e=Elevator(c,staged_settings())
    with pytest.raises(Exception):e.prepare_button_view(('right',))
    assert len([x for x in c.calls if x[0]=='evo_move_joints'])==1


@pytest.mark.parametrize('method',['right-round-face-v2','right-round-face-v3'])
def test_low_contrast_target_requires_exact_explicit_mode(method):
    c=FaceContext(lambda target:target.update(method=method))
    e=Elevator(c,settings(button_target_method=method))
    assert e.observe_button('2','right')
    assert [args for tool,args in c.calls if tool=='rax_detect_buttons'][-1]['target_method']==method
    c=FaceContext(lambda target:target.update(method=method))
    e=Elevator(c,settings(button_target_method='right-round-face-v1'))
    with pytest.raises(PrimitiveError):e.observe_button('2','right')
    assert not any(tool=='rax_camera_ray' for tool,args in c.calls)
