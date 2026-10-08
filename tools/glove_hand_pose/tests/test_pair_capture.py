"""Bounded file-only capture extraction; no Qt, serial or VQF required."""
import csv
import hashlib
import io
import json

import numpy as np
import pytest

from vt_glove_pose.config import fingerprint, load_config
from vt_glove_pose.pair_capture import load_pair_capture
from vt_glove_pose.stream import CSV_FIELDS


def sample(node, t=10., value=1, status=None):
    return dict(node_id=node, values=[value,node,-value,100,200,8000],
                host_monotonic_s=t, host_wall_s=1000.+t, status=status)


def frame(t=10., value=1, first=1):
    return [sample(node,t,value) for node in range(first,first+6)]


def csv_text(samples, fields=CSV_FIELDS):
    stream=io.StringIO(newline="")
    writer=csv.DictWriter(stream,fieldnames=fields)
    writer.writeheader()
    for s in samples:
        row=dict(zip(CSV_FIELDS,[s['host_monotonic_s'],s['host_wall_s'],s['node_id'],
                                 *s['values'],s['status']]))
        if "phase" in fields: row['phase']='moving'
        writer.writerow({k:row[k] for k in fields})
    return stream.getvalue()


def csv_file(tmp_path, samples, fields=CSV_FIELDS):
    path=tmp_path/'capture.csv'
    path.write_bytes(csv_text(samples,fields).encode())
    return path


def header():
    config=load_config()
    return dict(type='header',kind='vt_glove_pose.diagnostic_session',schema_version=1,
                time_basis='host_observed_not_sensor_latency',
                checkpoint=dict(config=config,config_fingerprint=fingerprint(config)))


def json_file(tmp_path, events):
    path=tmp_path/'capture.jsonl'
    path.write_text(''.join(json.dumps(dict(event,seq=i),allow_nan=False)+'\n'
                            for i,event in enumerate([header(),*events])),encoding='utf8')
    return path


def batch(samples):
    return dict(type='batch',samples=samples,performance={})


def test_csv_same_usb_stamp_preserves_ordered_frame_identity(tmp_path):
    path=csv_file(tmp_path,frame(value=11)+frame(value=22)+frame(value=33,first=7))
    result=load_pair_capture(path)
    np.testing.assert_array_equal(result['prox_gyro_raw'][:,0],[11,22])
    np.testing.assert_array_equal(result['dist_gyro_raw'][:,0],[11,22])
    np.testing.assert_array_equal(result['prox_acc_raw'],[[100,200,8000]]*2)
    np.testing.assert_allclose(result['prox_gyro_dps'],result['prox_gyro_raw']*.00875)
    np.testing.assert_array_equal(result['status'],[[-1,-1],[-1,-1]])
    meta=result['metadata']
    assert meta['matched_pairs']==meta['pairs']==2
    assert meta['input_samples']==18 and meta['dropped_pair_candidates']==0
    assert meta['stop_reason']=='eof'
    assert meta['bytes_read']==path.stat().st_size
    assert meta['sha256_read_prefix']==hashlib.sha256(path.read_bytes()).hexdigest()
    assert not meta['units_verified'] and not meta['sample_rate_verified']
    assert not meta['physical_synchronization_verified']


def test_reverse_roles_and_nonadjacent_ids_follow_wire_order(tmp_path):
    path=csv_file(tmp_path,frame(first=7))
    result=load_pair_capture(path,proximal=11,distal=8)
    assert result['prox_gyro_raw'][0,1]==11
    assert result['dist_gyro_raw'][0,1]==8
    assert result['metadata']['pairs']==1


@pytest.mark.parametrize('nodes,expected,drop',[
    ([5,1,2,3,4,6,5,6],[3],1),
    ([5,5,6],[3],1),
    ([6,5,6],[2],0),
    ([5,7,6],[],1),
    ([5],[],1),
])
def test_missing_repeated_or_reordered_nodes_are_never_timestamp_joined(tmp_path,nodes,expected,drop):
    # Every input intentionally has the same stamp; order is the only identity.
    counter=0; samples=[]
    for node in nodes:
        if node==5: counter+=1
        samples.append(sample(node,value=counter+1))
    path=csv_file(tmp_path,samples)
    result=load_pair_capture(path)
    assert result['prox_gyro_raw'][:,0].tolist()==expected
    assert result['metadata']['dropped_pair_candidates']==drop


def test_nonadjacent_pair_rejects_missing_intermediate_node(tmp_path):
    path=csv_file(tmp_path,[sample(2),sample(4),*frame(value=7)])
    result=load_pair_capture(path,proximal=2,distal=4)
    assert result['prox_gyro_raw'][:,0].tolist()==[7]
    assert result['metadata']['dropped_order_candidates']==1


def test_ordered_pair_with_different_host_stamps_is_dropped(tmp_path):
    path=csv_file(tmp_path,[sample(5,10),sample(6,10.001),sample(5,11),sample(6,11)])
    result=load_pair_capture(path)
    assert result['metadata']['pairs']==1
    assert result['metadata']['dropped_timestamp_candidates']==1


def test_intermediate_node_also_requires_same_frame_stamp(tmp_path):
    path=csv_file(tmp_path,[sample(2,10),sample(3,11),sample(4,10)])
    result=load_pair_capture(path,proximal=2,distal=4)
    assert result['metadata']['pairs']==0
    assert result['metadata']['dropped_timestamp_candidates']==1


def test_time_window_is_host_relative_half_open_and_does_not_retime(tmp_path):
    path=csv_file(tmp_path,frame(20)+frame(21,value=2)+frame(22,value=3)+frame(23,value=4))
    result=load_pair_capture(path,start_s=1,duration_s=2)
    assert result['prox_gyro_raw'][:,0].tolist()==[2,3]
    np.testing.assert_array_equal(result['host_monotonic_s'],[[21,21],[22,22]])
    assert result['metadata']['pairs_before_window']==1
    assert result['metadata']['stop_reason']=='duration'
    assert result['metadata']['bytes_read']<path.stat().st_size


def test_max_pairs_stops_before_unread_bad_tail(tmp_path):
    path=csv_file(tmp_path,frame())
    path.write_bytes(path.read_bytes()+b'invalid later row\n')
    result=load_pair_capture(path,max_pairs=1)
    assert result['metadata']['pairs']==1 and result['metadata']['stop_reason']=='max_pairs'
    assert result['metadata']['bytes_read']<path.stat().st_size


def test_fixed_prefix_excludes_partial_line_and_hashes_actual_bytes(tmp_path):
    path=csv_file(tmp_path,frame()+frame(11))
    raw=path.read_bytes(); lines=raw.splitlines(keepends=True)
    limit=sum(map(len,lines[:7]))+5
    result=load_pair_capture(path,byte_limit=limit)
    assert result['metadata']['pairs']==1
    assert result['metadata']['partial_line_bytes_ignored']==5
    assert result['metadata']['complete_line_bytes']==limit-5
    assert result['metadata']['bytes_read']==limit
    assert result['metadata']['sha256_read_prefix']==hashlib.sha256(raw[:limit]).hexdigest()
    assert result['metadata']['stop_reason']=='partial_line_at_byte_limit'


def test_default_byte_budget_is_bounded_even_if_target_nodes_missing(tmp_path,monkeypatch):
    import vt_glove_pose.pair_capture as module
    path=csv_file(tmp_path,[sample(1)]*30)
    lines=path.read_bytes().splitlines(keepends=True)
    limit=sum(map(len,lines[:4]));monkeypatch.setattr(module,'DEFAULT_BYTE_LIMIT',limit)
    result=load_pair_capture(path)
    assert result['metadata']['stop_reason']=='default_byte_limit'
    assert result['metadata']['bytes_read']==limit
    for key in ('prox_gyro_dps','dist_gyro_dps','prox_acc_raw','dist_acc_raw'):
        assert result[key].shape==(0,3)
    assert result['near_limit_mask'].shape==(0,2)


def test_raw_saturation_mask_keeps_min_int16_without_overflow(tmp_path):
    p,d=sample(5,status=255),sample(6,status=2)
    p['values'][0]=-32768;d['values'][1]=32700
    result=load_pair_capture(csv_file(tmp_path,[p,d]))
    np.testing.assert_array_equal(result['near_limit_mask'],[[True,True]])
    np.testing.assert_array_equal(result['status'],[[255,2]])
    assert result['prox_gyro_dps'][0,0]==-32768*.00875


def test_csv_optional_status_and_phase_and_utf8_bom(tmp_path):
    path=csv_file(tmp_path,frame(),fields=(*CSV_FIELDS[:-1],'phase'))
    path.write_bytes(b'\xef\xbb\xbf'+path.read_bytes())
    assert load_pair_capture(path)['metadata']['pairs']==1


def test_jsonl_cross_batch_pair_and_ignored_snapshot_preserve_raw_order(tmp_path):
    records=frame(value=2)+frame(value=3)
    path=json_file(tmp_path,[batch(records[:5]),dict(type='snapshot',now_s=10,snapshot={}),
                             batch(records[5:]),dict(type='end',complete=True,reason='manual')])
    result=load_pair_capture(path)
    assert result['prox_gyro_raw'][:,0].tolist()==[2,3]
    assert result['metadata']['observed_end_complete'] is True
    assert result['metadata']['last_event_seq']==4
    assert result['metadata']['source_configuration']['sample_rate_hz']==120


def test_jsonl_header_scales_never_override_explicit_arguments(tmp_path):
    path=json_file(tmp_path,[batch(frame())])
    result=load_pair_capture(path,gyro_dps_per_lsb=.1,sample_rate_hz=100)
    assert result['prox_gyro_dps'][0,0]==.1
    assert result['metadata']['gyro_dps_per_lsb']==.1
    assert result['metadata']['sample_rate_hz']==100
    assert result['metadata']['observed_end_complete'] is None


@pytest.mark.parametrize('mutation,match',[
    (lambda e:e.update(seq=9),'sequence'),
    (lambda e:e.update(type='execute'),'unsupported'),
    (lambda e:e.update(samples='bad'),'sample batch'),
    (lambda e:e['samples'][0].update(node_id=13),'node_id'),
    (lambda e:e['samples'][0].update(values=[0]*5),'six'),
    (lambda e:e['samples'][0].update(unknown=1),'sample fields'),
    (lambda e:e['samples'][0].update(host_monotonic_s=float('nan')),'Nonfinite'),
])
def test_bad_jsonl_events_raise_instead_of_returning_partial_success(tmp_path,mutation,match):
    event=dict(batch(frame()),seq=1);mutation(event)
    path=tmp_path/'bad.jsonl'
    path.write_text(json.dumps(dict(header(),seq=0))+'\n'+json.dumps(event)+'\n')
    with pytest.raises(ValueError,match=match): load_pair_capture(path)


def test_duplicate_json_keys_and_records_after_footer_rejected(tmp_path):
    path=json_file(tmp_path,[dict(type='end',complete=True,reason='manual'),batch(frame())])
    with pytest.raises(ValueError,match='after diagnostic end'):load_pair_capture(path)
    path.write_text(json.dumps(dict(header(),seq=0))+'\n'+
                    '{"seq":1,"seq":1,"type":"batch","samples":[]}\n')
    with pytest.raises(ValueError,match='Duplicate JSON key'):load_pair_capture(path)


@pytest.mark.parametrize('problem', ['extra_field','bad_integer','backwards','missing_newline','duplicate_header'])
def test_malformed_csv_rejected(tmp_path,problem):
    path=csv_file(tmp_path,frame()+frame(11))
    data=path.read_bytes()
    if problem=='extra_field':data+=b'1,2,3\n'
    elif problem=='bad_integer':data=data.replace(b',5,1,5,-1,',b',5,1.2,5,-1,',1)
    elif problem=='backwards':data+=csv_text([sample(5,9)]).split('\n',1)[1].encode()
    elif problem=='missing_newline':data=data[:-2]
    else:data=data.replace(b'node_id',b'gx_lsb',1)
    path.write_bytes(data)
    with pytest.raises(ValueError):load_pair_capture(path)


def test_empty_headers_and_oversized_lines(tmp_path,monkeypatch):
    path=tmp_path/'empty.csv';path.write_bytes(b'')
    with pytest.raises(ValueError,match='header'):load_pair_capture(path)
    import vt_glove_pose.pair_capture as module
    monkeypatch.setattr(module,'MAX_LINE_BYTES',256)
    path=csv_file(tmp_path,frame())
    path.write_bytes(path.read_bytes()+b'x'*257+b'\n')
    with pytest.raises(ValueError,match='exceeds'):load_pair_capture(path)


def test_named_pipe_is_rejected_before_open(tmp_path):
    import os
    path=tmp_path/'pipe.csv'
    os.mkfifo(path)
    with pytest.raises(ValueError,match='regular file'):load_pair_capture(path)


@pytest.mark.parametrize('event',[
    dict(type='snapshot',snapshot={}),dict(type='checkpoint',checkpoint={}),
    dict(type='marker',label='invented'),dict(type='command',name='execute',args={}),
])
def test_malformed_auxiliary_events_raise(tmp_path,event):
    path=json_file(tmp_path,[event,batch(frame())])
    with pytest.raises(ValueError):load_pair_capture(path)


@pytest.mark.parametrize('kwargs',[
    dict(proximal=0),dict(proximal=True),dict(distal=13),dict(proximal=5,distal=5),
    dict(proximal=6,distal=7),dict(sample_rate_hz=float('nan')),dict(sample_rate_hz=True),
    dict(gyro_dps_per_lsb=0),dict(gyro_dps_per_lsb=float('inf')),dict(start_s=-1),
    dict(duration_s=0),dict(duration_s=float('nan')),dict(byte_limit=-1),dict(byte_limit=True),
    dict(max_pairs=0),dict(max_pairs=1.2),dict(max_pairs=True),
])
def test_invalid_parameters_fail_before_open(tmp_path,kwargs):
    with pytest.raises(ValueError):load_pair_capture(tmp_path/'never-opened.csv',**kwargs)
