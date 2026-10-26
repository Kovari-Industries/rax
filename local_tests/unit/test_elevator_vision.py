import pytest
np=pytest.importorskip("numpy")
cv2=pytest.importorskip("cv2")
from auto_research_agent.api.elevator_vision import localize_right_round_face


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


def test_multiple_candidates_refuse(monkeypatch):
    monkeypatch.setattr(cv2,'HoughCircles',lambda *a,**k:np.array([[[35,50,20],[60,50,20]]],dtype=np.float32))
    with pytest.raises(ValueError,match='ambiguous'):localize_right_round_face(image(),[30,30,220,110])


def test_unstable_candidates_refuse(monkeypatch):
    values=iter([40,50,60])
    monkeypatch.setattr(cv2,'HoughCircles',lambda *a,**k:np.array([[[next(values),50,20]]],dtype=np.float32))
    with pytest.raises(ValueError,match='unstable'):localize_right_round_face(image(),[30,30,220,110])
