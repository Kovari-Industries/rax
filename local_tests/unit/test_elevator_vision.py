import pytest
np=pytest.importorskip("numpy")
cv2=pytest.importorskip("cv2")
from auto_research_agent.api.elevator_vision import localize_right_round_face, localize_right_round_face_denoised


def image():
    rgb=np.full((140,240,3),120,np.uint8)
    cv2.circle(rgb,(180,70),28,(235,235,235),-1)
    cv2.putText(rgb,'2',(45,85),cv2.FONT_HERSHEY_SIMPLEX,1,(240,240,240),2)
    return rgb


def test_round_face_not_strip_center():
    face=localize_right_round_face(image(),[30,30,220,110])
    assert np.linalg.norm(np.array(face['pixel'])-[180,70])<3
    assert face['center_spread_px']<=3


@pytest.mark.parametrize('box',([30,30,220,110],[30,30,220,50],[30,30,220,140]))
def test_missing_or_invalid_face_refuses(box):
    with pytest.raises(ValueError):localize_right_round_face(np.zeros((140,240,3),np.uint8),box)


@pytest.mark.parametrize('localizer',[localize_right_round_face,localize_right_round_face_denoised])
def test_multiple_candidates_refuse(monkeypatch,localizer):
    monkeypatch.setattr(cv2,'HoughCircles',lambda *a,**k:np.array([[[35,50,20],[60,50,20]]],dtype=np.float32))
    with pytest.raises(ValueError,match='ambiguous'):localizer(image(),[30,30,220,110])


@pytest.mark.parametrize('localizer',[localize_right_round_face,localize_right_round_face_denoised])
def test_unstable_candidates_refuse(monkeypatch,localizer):
    values=iter([40,50,60])
    monkeypatch.setattr(cv2,'HoughCircles',lambda *a,**k:np.array([[[next(values),50,20]]],dtype=np.float32))
    with pytest.raises(ValueError,match='unstable'):localizer(image(),[30,30,220,110])


def test_explicit_low_contrast_mode_keeps_center_and_refuses_blank():
    from auto_research_agent.api.elevator_vision import localize_right_round_face_low_contrast
    rgb=np.full((140,240,3),30,np.uint8)
    cv2.circle(rgb,(180,70),28,(50,50,50),-1)
    face=localize_right_round_face_low_contrast(rgb,[30,30,220,110])
    assert face['method']=='right-round-face-v2' and face['edge_threshold']==20
    assert np.linalg.norm(np.array(face['pixel'])-[180,70])<3
    assert face['center_spread_px']<=3 and face['radius_spread_px']<=3
    with pytest.raises(ValueError):localize_right_round_face_low_contrast(np.full_like(rgb,30),[30,30,220,110])


def test_explicit_median_mode_retains_stability_and_missing_face_refusal():
    from auto_research_agent.api.elevator_vision import localize_right_round_face_denoised
    rgb=image().astype(float)
    rgb+=np.random.default_rng(12).normal(0,12,rgb.shape)
    face=localize_right_round_face_denoised(np.clip(rgb,0,255).astype(np.uint8),[30,30,220,110])
    assert face['method']=='right-round-face-v3' and face['median_kernel']==5
    assert np.linalg.norm(np.array(face['pixel'])-[180,70])<=3
    assert face['center_spread_px']<=3 and face['radius_spread_px']<=3
    with pytest.raises(ValueError):localize_right_round_face_denoised(np.zeros((140,240,3),np.uint8),[30,30,220,110])


def test_median_mode_refuses_a_face_clipped_by_the_detection():
    with pytest.raises(ValueError):localize_right_round_face_denoised(image(),[30,30,190,110])
