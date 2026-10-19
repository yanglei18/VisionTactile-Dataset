"""Both replay paths reject external links and bound metadata before parsing."""
from pathlib import Path

import pytest

from vt_multisensor_alignment.topic_replay_worker import _file_inventory
from vt_multisensor_alignment.spatial_replay_worker import _inventory


@pytest.fixture(params=[_file_inventory, _inventory], ids=['topic', 'spatial'])
def inventory(request):
    return request.param


def metadata(bag, names='[data.mcap]'):
    (bag / 'metadata.yaml').write_text(f'rosbag2_bagfile_information:\n  relative_file_paths: {names}\n')


@pytest.mark.parametrize('kind', ['file', 'directory', 'metadata', 'root'])
def test_replay_rejects_links_at_every_path_component(tmp_path, inventory, kind):
    outside = tmp_path / 'outside'
    outside.mkdir()
    (outside / 'data.mcap').write_bytes(b'synthetic')
    metadata(outside)
    bag = tmp_path / 'bag'
    if kind == 'root':
        bag.symlink_to(outside, target_is_directory=True)
    else:
        bag.mkdir()
        metadata(bag)
        if kind == 'file':
            (bag / 'data.mcap').symlink_to(outside / 'data.mcap')
        elif kind == 'directory':
            metadata(bag, '[linked/data.mcap]')
            (bag / 'linked').symlink_to(outside, target_is_directory=True)
        else:
            (bag / 'data.mcap').write_bytes(b'synthetic')
            (bag / 'metadata.yaml').unlink()
            (bag / 'metadata.yaml').symlink_to(outside / 'metadata.yaml')
    with pytest.raises(ValueError):
        inventory(bag)


def test_replay_bounds_metadata_before_parsing(tmp_path, inventory):
    metadata(tmp_path, '[]')
    with (tmp_path / 'metadata.yaml').open('a') as stream:
        stream.write('#' + 'x' * (4 * 1024 * 1024))
    with pytest.raises(ValueError):
        inventory(tmp_path)


@pytest.mark.parametrize('kind', ['fifo', 'directory'])
def test_replay_rejects_nonregular_storage_without_blocking(tmp_path, inventory, kind):
    import os
    metadata(tmp_path)
    if kind == 'fifo':
        os.mkfifo(tmp_path / 'data.mcap')
    else:
        (tmp_path / 'data.mcap').mkdir()
    with pytest.raises(ValueError):
        inventory(tmp_path)


def test_nested_regular_storage_is_compatible(tmp_path, inventory):
    (tmp_path / 'split').mkdir()
    (tmp_path / 'split' / 'data.mcap').write_bytes(b'synthetic')
    metadata(tmp_path, '[split/data.mcap]')
    inventory(tmp_path)


@pytest.mark.parametrize('kind', ['topic', 'spatial'])
def test_source_does_not_resolve_away_forbidden_root_link(tmp_path, monkeypatch, kind):
    from vt_multisensor_alignment import topic_replay_worker, spatial_replay_worker
    outside = tmp_path / 'outside'
    outside.mkdir()
    metadata(outside, '[]')
    bag = tmp_path / 'linked-bag'
    bag.symlink_to(outside, target_is_directory=True)
    def forbidden_reader(path):
        pytest.fail('unsafe path reached native reader')
    with pytest.raises(ValueError):
        if kind == 'topic':
            topic_replay_worker.TopicReplaySource(bag, reader_factory=forbidden_reader)
        else:
            monkeypatch.setattr(spatial_replay_worker, '_open_reader', forbidden_reader)
            spatial_replay_worker.BagReplaySource(bag)
