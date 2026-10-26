"""Shared flight analysis contracts stay strict at recording boundaries."""
import json
import pytest
from jsonschema import Draft202012Validator, ValidationError
from bullet_msgs import schema_bytes

SOURCE = dict(physics_id='p', episode_id='e', stream_id='s', step_index=0, simulation_time_s=0., measurement_us=1)
STATE = dict(episode_id='e', step_index=0, simulation_time_s=0., drone_id='drone_1', position_m=[0.,0.,1.], orientation_xyzw=[0.,0.,0.,1.], linear_velocity_m_s=[0.,0.,0.], angular_velocity_rad_s=[0.,0.,0.], rotor_speeds_rad_s=[0.]*4)


@pytest.mark.parametrize('name,value', [
 ('analysis_ground_truth',dict(schema_version=1,source=SOURCE,ground_truth=STATE,publisher_drops=0)),
 ('analysis_sensors',dict(schema_version=1,source=SOURCE,sensors=dict(imu=None,barometer=None,rangefinder=None),attitude_estimate=None,publisher_drops=0)),
 ('analysis_config',dict(schema_version=1,source=SOURCE,role='physics',settings={},revisions={'repo':None})),
 ('analysis_msp',dict(schema_version=1,source=SOURCE,backend='betaflight',timestamp_kind='host_receipt',
                      firmware={'api':[0,1,48],'variant':[66,84,70,76],'version':[26,6,2]},error=None,
                      rc_frames_sent=2,last_rc_sent_at_s=1.,
                      msp_delivery=dict(requests_hz=10,responses_hz=10,queue_delay_mean_s=0.,queue_delay_max_s=0.,
                                        response_turnaround_mean_s=.001,response_turnaround_max_s=.001),
                      sensor_feedback=dict(window_s=1.,expected_hz=120.,observed_hz=120.,feedback_age_s=.001,
                                           attitude_age_s=.001,barometer_age_s=.001,image_age_s=.01,
                                           image_attitude_offset_s=.002,vision_reason='usable',mission_phase='LOS_ACQUIRE'),
                      attitude=None, altitude=None,
                      status={'mode_flags':1,'arming_disabled_flags':0,'received_at':1.})),
 ('image_metadata',dict(schema_version=1,image_topic='camera/front/image',image_log_time_ns=10,image_record_sequence=0,metadata=dict(session='s',sequence=1,capture_ns=1000,source=SOURCE))),
 ('recording_health',dict(schema_version=1,complete=True,reason='closed',outcome='stop',skipped_images=0,peak_queue_bytes=0,counts={},launch_id=None)),
])
def test_analysis_schema_validates_and_rejects_unknown_fields(name,value):
    validator=Draft202012Validator(json.loads(schema_bytes(name+'.json')))
    validator.check_schema(validator.schema)
    validator.validate(value)
    with pytest.raises(ValidationError):
        validator.validate(dict(value,unexpected=True))
    with pytest.raises(ValidationError):
        validator.validate({k:v for k,v in value.items() if k!='schema_version'})


def test_revision_snapshots_report_dirty_and_unavailable(monkeypatch,tmp_path):
    from bullet_msgs import provenance
    def git(command,**kwargs):
        from pathlib import Path
        name=Path(command[2]).name
        if name=='bullet_render':
            raise OSError('git unavailable')
        if command[-1]=='HEAD':
            return 'a'*40+'\n'
        return ' M settings\n' if name=='bullet_app' else ''
    monkeypatch.setattr(provenance.subprocess,'check_output',git)
    result=provenance.revisions(tmp_path)
    assert len(result)==7
    assert result['bullet_app']=='a'*40+'+dirty'
    assert result['bullet_render'] is None
    assert result['bullet_drone']=='a'*40
