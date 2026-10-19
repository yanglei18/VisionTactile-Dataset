import hashlib
import json

import pytest

from vt_multisensor_alignment.geometry_materials import GeometryMaterials


def write_sidecar(tmp_path):
    bag = tmp_path/'bag'
    bag.mkdir()
    sidecar = tmp_path/'bag.spatial'
    sidecar.mkdir()
    geometry = dict(config=dict(id='cam', serial='serial', color_frame_id='color', depth_frame_id='depth', tracker_role='left_wrist'),
                    extrinsic=dict(status='VALID', camera=dict(name='cam', serial='serial', frame_id='color'),
                                   tracker=dict(role='left_wrist', tracker_id='tracker', frame_id='tracker_frame'),
                                   transform=dict(translation_m=[0, 0, 0], quaternion_xyzw=[0, 0, 0, 1],
                                                  parent_frame='tracker_frame', child_frame='color', semantics='parent_from_child')),
                    color_intrinsics={'width': 2}, depth_intrinsics={'width': 2},
                    depth_to_color=dict(translation_m=[0, 0, 0], quaternion_xyzw=[0, 0, 0, 1]),
                    depth_units_m=.001, depth_units_source='realsense_ros_z16_mm',
                    identity=dict(verified=True, serial='serial', observed_ns=100), geometry_revision=1)
    content = {key: geometry[key] for key in ('config', 'extrinsic', 'color_intrinsics', 'depth_intrinsics',
                                              'depth_to_color', 'depth_units_m', 'depth_units_source')}
    geometry['geometry_sha256'] = hashlib.sha256(json.dumps(content, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    data = json.dumps(geometry).encode()
    artifact = 'geometry.json'
    (sidecar/artifact).write_bytes(data)
    revision = dict(camera_id='cam', artifact=artifact, artifact_sha256=hashlib.sha256(data).hexdigest(),
                    geometry_revision=1, geometry_sha256=geometry['geometry_sha256'], observed_realtime_ns=100,
                    time_epoch=0, coordinate_epoch=0, scene_epoch='scene')
    manifest = dict(schema_version=1, kind='vt.spatial.geometry', bag_path=str(bag), session_id='session',
                    world_frame='vive_map', camera_config=[geometry['config']], complete=True,
                    geometry_status='available', revisions=[revision], epochs=[], updated_realtime_ns=1000)
    (sidecar/'manifest.json').write_text(json.dumps(manifest))
    return bag, sidecar, manifest


def test_missing_old_bag_geometry_reports_unavailable(tmp_path):
    bag = tmp_path/'bag'
    bag.mkdir()
    material = GeometryMaterials(bag)
    assert material.select('cam', 100).status == 'unavailable'


def test_select_checks_time_generation_and_actual_materials(tmp_path):
    bag, _, _ = write_sidecar(tmp_path)
    material = GeometryMaterials(bag)
    assert material.select('cam', 99).reason == 'before_geometry_segment'
    assert material.select('cam', 500, time_epoch=0, coordinate_epoch=0).status == 'ready'
    assert material.select('cam', 500, time_epoch=1, coordinate_epoch=0).status == 'unavailable'
    assert material.select('cam', 2000).reason == 'after_recorded_geometry'
    current = material.select('cam', 500)
    current.geometry['depth_units_m'] = 10
    assert material.select('cam', 500).geometry['depth_units_m'] == .001


def test_tampered_artifact_or_escape_path_fails_closed(tmp_path):
    bag, sidecar, manifest = write_sidecar(tmp_path)
    (sidecar/'geometry.json').write_text('{}')
    with pytest.raises(ValueError, match='hash'):
        GeometryMaterials(bag)
    manifest['revisions'][0]['artifact'] = '../elsewhere.json'
    (sidecar/'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='artifact path'):
        GeometryMaterials(bag)


def test_realtime_overlap_needs_explicit_epoch_and_never_uses_previous_generation(tmp_path):
    bag, sidecar, manifest = write_sidecar(tmp_path)
    second = dict(manifest['revisions'][0], time_epoch=1, observed_realtime_ns=200)
    manifest['revisions'].append(second)
    (sidecar/'manifest.json').write_text(json.dumps(manifest))
    material = GeometryMaterials(bag)
    assert material.select('cam', 500).reason == 'ambiguous_generation'
    assert material.select('cam', 150, time_epoch=1, coordinate_epoch=0).reason == 'before_geometry_segment'
    assert material.select('cam', 500, time_epoch=1, coordinate_epoch=0).status == 'ready'

def add_intervals(sidecar,manifest,intervals):
    revision=manifest['revisions'][0]
    manifest['updated_realtime_ns']=30_000_000_000
    manifest['identity_intervals']=[dict(camera_id='cam',identity_revision=0,serial='serial',verified=True,
        geometry_revision=1,geometry_sha256=revision['geometry_sha256'],time_epoch=0,coordinate_epoch=0,scene_epoch='scene',
        start_realtime_ns=start,last_observed_ns=last,end_realtime_ns=end) for start,last,end in intervals]
    (sidecar/'manifest.json').write_text(json.dumps(manifest))


def test_recorded_identity_renewals_extend_lease_without_duplicating_geometry(tmp_path):
    bag,sidecar,manifest=write_sidecar(tmp_path)
    add_intervals(sidecar,manifest,[(100,10_000_000_000,15_000_000_000)])
    selected=GeometryMaterials(bag).select('cam',12_000_000_000)
    assert selected.status=='ready'
    assert selected.valid_until_ns==15_000_000_000
    assert GeometryMaterials(bag).select('cam',15_000_000_000).reason=='identity_interval_unverified'


def test_interval_gap_and_foreign_identity_never_borrow_a_current_camera(tmp_path):
    bag,sidecar,manifest=write_sidecar(tmp_path)
    add_intervals(sidecar,manifest,[(100,100,5_000_000_100),(10_000_000_000,10_000_000_000,15_000_000_000)])
    material=GeometryMaterials(bag)
    assert material.select('cam',7_000_000_000).reason=='identity_interval_unverified'
    assert material.select('cam',12_000_000_000).status=='ready'
    manifest['identity_intervals'][1]['serial']='foreign'
    (sidecar/'manifest.json').write_text(json.dumps(manifest))
    assert GeometryMaterials(bag).select('cam',12_000_000_000).reason=='identity_interval_unverified'


def test_manifest_without_intervals_keeps_original_five_second_lease(tmp_path):
    bag,sidecar,manifest=write_sidecar(tmp_path)
    manifest['updated_realtime_ns']=30_000_000_000
    (sidecar/'manifest.json').write_text(json.dumps(manifest))
    assert GeometryMaterials(bag).select('cam',12_000_000_000).reason=='identity_observation_expired'


def test_malformed_or_ambiguous_identity_intervals_fail_closed(tmp_path):
    bag,sidecar,manifest=write_sidecar(tmp_path)
    add_intervals(sidecar,manifest,[(100,100,5_000_000_100)])
    manifest['identity_intervals'].append(dict(manifest['identity_intervals'][0]))
    (sidecar/'manifest.json').write_text(json.dumps(manifest))
    assert GeometryMaterials(bag).select('cam',500).reason=='ambiguous_identity_interval'
    manifest['identity_intervals'][0]['end_realtime_ns']=10_000_000_000
    (sidecar/'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError,match='lease'): GeometryMaterials(bag)

@pytest.mark.parametrize('field,value',[('tracker_frame_id','foreign_tracker_frame'),('model','foreign_model')])
def test_recorded_configuration_binding_must_match_extrinsic(tmp_path,field,value):
    bag,sidecar,manifest=write_sidecar(tmp_path)
    geometry=json.loads((sidecar/'geometry.json').read_text())
    geometry['config'][field]=value
    content={key:geometry[key] for key in ('config','extrinsic','color_intrinsics','depth_intrinsics','depth_to_color','depth_units_m','depth_units_source')}
    geometry['geometry_sha256']=hashlib.sha256(json.dumps(content,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    raw=json.dumps(geometry).encode(); (sidecar/'geometry.json').write_bytes(raw)
    manifest['revisions'][0].update(geometry_sha256=geometry['geometry_sha256'],artifact_sha256=hashlib.sha256(raw).hexdigest())
    (sidecar/'manifest.json').write_text(json.dumps(manifest))
    assert GeometryMaterials(bag).select('cam',500).reason=='extrinsic_identity_mismatch'
