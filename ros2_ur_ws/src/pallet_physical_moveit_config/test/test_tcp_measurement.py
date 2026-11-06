"""Exercise fixed-point calibration, degeneracy and record preservation."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('tcp',Path(__file__).parents[1]/'scripts/tcp_measurement.py')
tcp=importlib.util.module_from_spec(spec);spec.loader.exec_module(tcp)


def samples():
    actual=np.array([.01,-.02,.12]);fixed=np.array([.3,.1,.2])
    quats=[[0,0,0,1],[np.sin(.4),0,0,np.cos(.4)],
           [0,np.sin(.5),0,np.cos(.5)],[0,0,np.sin(.6),np.cos(.6)]]
    return [dict(base_frame='base_footprint',tool_frame='gripper',opening_mm=30.,
                 point_id='fixed_center',model_sha256='test_model',
                 quaternion_xyzw=q,translation_m=(fixed-tcp.rotation(q)@actual).tolist()) for q in quats]


def test_recovers_tcp_in_tool_frame():
    result=tcp.solve(samples())
    np.testing.assert_allclose(result['tcp_translation_in_gripper_m'],[.01,-.02,.12],atol=1e-12)
    np.testing.assert_allclose(result['fixed_point_in_base_m'],[.3,.1,.2],atol=1e-12)
    assert result['residual_rms_mm'] < 1e-9
    assert result['tcp_orientation']=='not_observed_by_pivot_calibration'


def test_rejects_same_orientation_and_mixed_opening():
    data=samples()
    with pytest.raises(ValueError,match='orientation diversity'):tcp.solve([data[0]]*4)
    data[1]['opening_mm']=40.
    with pytest.raises(ValueError,match='same frames'):tcp.solve(data)


def test_rejects_nonfinite_pose():
    data=samples();data[1]['translation_m'][0]=float('nan')
    with pytest.raises(ValueError,match='finite translation'):tcp.solve(data)


def test_does_not_overwrite_measurement(tmp_path):
    path=tmp_path/'capture.json';tcp.write(path,{'original':True})
    with pytest.raises(FileExistsError):tcp.write(path,{'original':False})
    assert 'true' in path.read_text()


def test_missing_transform_diagnostics_distinguish_absence_and_stale():
    assert 'TF unavailable' in tcp.missing_tf_message('unknown frame',None)
    assert '2.000s old' in tcp.missing_tf_message('',2.)
    assert 'future' in tcp.missing_tf_message('',-1.)
    assert 'Motor Studio alone' in tcp.missing_tf_message('',None)
