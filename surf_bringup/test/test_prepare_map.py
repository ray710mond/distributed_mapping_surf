import argparse
import importlib.util
import json
import pathlib
import stat
import struct
import sys


SCRIPT = pathlib.Path(__file__).parents[1] / 'scripts' / 'prepare_map.py'
SPEC = importlib.util.spec_from_file_location('prepare_map', SCRIPT)
prepare_map = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = prepare_map
SPEC.loader.exec_module(prepare_map)


def write_binary_pcd(path, points=((1.0, 2.0, 3.0),)):
    header = (
        '# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
        'TYPE F F F\nCOUNT 1 1 1\n'
        f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n'
    ).encode('ascii')
    with path.open('wb') as stream:
        stream.write(header)
        for point in points:
            stream.write(struct.pack('<fff', *point))


def make_fake_converters(directory):
    bonxai = directory / 'bonxai_converter'
    bonxai.write_text(
        '''#!/usr/bin/env python3
import hashlib, pathlib, struct, sys
command = sys.argv[1]
with pathlib.Path(__file__).with_name("calls.log").open("a") as stream:
    stream.write(command + "\\n")
if command == "inspect-bonxai":
    print(1 if pathlib.Path(sys.argv[2]).read_bytes().startswith(b"occupied") else 0)
elif command == "inspect-pcd":
    header = pathlib.Path(sys.argv[2]).read_bytes().split(b"DATA", 1)[0]
    points = next(line for line in header.splitlines() if line.startswith(b"POINTS "))
    print(int(points.split()[1]))
elif command == "pcd-to-bonxai":
    digest = hashlib.sha256(pathlib.Path(sys.argv[2]).read_bytes()).hexdigest().encode()
    pathlib.Path(sys.argv[3]).write_bytes(b"occupied:" + digest)
elif command in ("bonxai-to-pcd", "chunks-to-pcd", "normalize-pcd"):
    output = pathlib.Path(sys.argv[3])
    value = 9.0 if command == "bonxai-to-pcd" and b"new" in pathlib.Path(sys.argv[2]).read_bytes() else 1.0
    header = ("# .PCD v0.7\\nVERSION 0.7\\nFIELDS x y z\\nSIZE 4 4 4\\n"
              "TYPE F F F\\nCOUNT 1 1 1\\nWIDTH 1\\nHEIGHT 1\\n"
              "POINTS 1\\nDATA binary\\n").encode("ascii")
    output.write_bytes(header + struct.pack("<fff", value, 2.0, 3.0))
else:
    raise SystemExit(2)
''',
        encoding='utf-8')

    chunks = directory / 'pcd_to_chunks'
    chunks.write_text(
        '''#!/usr/bin/env python3
import pathlib, sys
with pathlib.Path(__file__).with_name("calls.log").open("a") as stream:
    stream.write("chunks\\n")
root = pathlib.Path(sys.argv[2])
chunk = root / "0_0_0"
chunk.mkdir(parents=True)
resolution = sys.argv[sys.argv.index("--resolution") + 1]
chunk_size = sys.argv[sys.argv.index("--chunk-size") + 1]
(root / "metadata.yaml").write_text(
    "format: rolling_bonxai_chunk_store\\n"
    "format_version: 1\\n"
    f"resolution: {resolution}\\n"
    f"chunk_size: {chunk_size}\\n"
    "occupied_cells: 1\\n"
    "chunks: 1\\n"
    "validated: true\\n"
)
(chunk / "0_0_0.chunk").write_bytes(b"chunk")
''',
        encoding='utf-8')

    occupancy = directory / 'pcd_to_occupancy'
    occupancy.write_text(
        '''#!/usr/bin/env python3
import pathlib, sys
with pathlib.Path(__file__).with_name("calls.log").open("a") as stream:
    stream.write("occupancy\\n")
stem = pathlib.Path(sys.argv[2])
resolution = sys.argv[sys.argv.index("--resolution") + 1]
stem.with_suffix(".pgm").write_bytes(b"P5\\n1 1\\n255\\n\\x00")
stem.with_suffix(".yaml").write_text(
    f"image: {stem.name}.pgm\\n"
    "mode: trinary\\n"
    f"resolution: {resolution}\\n"
    "origin: [0.0, 0.0, 0.0]\\n"
    "negate: 0\\n"
    "occupied_thresh: 0.65\\n"
    "free_thresh: 0.196\\n"
)
stem.with_name(stem.name + "_metadata.yaml").write_text("valid: true\\n")
''',
        encoding='utf-8')

    for executable in (bonxai, chunks, occupancy):
        executable.chmod(executable.stat().st_mode | stat.S_IXUSR)
    return bonxai, chunks, occupancy


def arguments(root, name, converters, **overrides):
    values = dict(
        map_root=root,
        map_name=name,
        bonxai_converter=converters[0],
        chunk_converter=converters[1],
        occupancy_converter=converters[2],
        voxel_resolution=0.05,
        chunk_size=10.0,
        map_2d_resolution=0.2,
    )
    values.update(overrides)
    return argparse.Namespace(**values)


def calls(directory):
    path = directory / 'calls.log'
    return path.read_text().splitlines() if path.exists() else []


def clear_calls(directory):
    (directory / 'calls.log').unlink(missing_ok=True)


def write_chunk_store(path):
    chunk = path / '0_0_0'
    chunk.mkdir(parents=True)
    (path / 'metadata.yaml').write_text(
        'format: rolling_bonxai_chunk_store\n'
        'format_version: 1\n'
        'resolution: 0.05\n'
        'chunk_size: 10.0\n'
        'occupied_cells: 1\n'
        'chunks: 1\n'
        'validated: true\n'
    )
    (chunk / '0_0_0.chunk').write_bytes(b'chunk')


def test_missing_directory_creates_non_localized_map(tmp_path):
    converters = tuple(tmp_path / name for name in ('missing-a', 'missing-b', 'missing-c'))
    result = prepare_map.prepare(arguments(tmp_path / 'maps', 'new_lab', converters))

    assert result['directory_created'] is True
    assert result['localization_enabled'] is False
    assert pathlib.Path(result['map_directory']).is_dir()
    assert result['generated'] == []


def test_pcd_fills_every_missing_artifact(tmp_path):
    converters = make_fake_converters(tmp_path)
    environment = tmp_path / 'maps' / 'lab'
    environment.mkdir(parents=True)
    write_binary_pcd(environment / 'lab.pcd')

    result = prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    assert result['localization_enabled'] is True
    assert (environment / 'lab.bonxai').read_bytes().startswith(b'occupied:')
    assert (environment / 'lab.yaml').is_file()
    assert (environment / 'lab.pgm').is_file()
    assert (environment / 'lab_chunks' / '0_0_0' / '0_0_0.chunk').is_file()


def test_first_adoption_uses_pcd_over_untracked_bonxai(tmp_path):
    converters = make_fake_converters(tmp_path)
    environment = tmp_path / 'maps' / 'lab'
    environment.mkdir(parents=True)
    write_binary_pcd(environment / 'lab.pcd', ((5.0, 6.0, 7.0),))
    (environment / 'lab.bonxai').write_bytes(b'occupied-unrelated')

    clear_calls(tmp_path)
    result = prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    assert 'pcd-to-bonxai' in calls(tmp_path)
    assert 'bonxai-to-pcd' not in calls(tmp_path)
    assert (environment / 'lab.bonxai').read_bytes().startswith(b'occupied:')
    assert any('lab.bonxai.incomplete-' in path for path in result['preserved'])


def test_nonempty_bonxai_can_bootstrap_pcd_and_derivatives(tmp_path):
    converters = make_fake_converters(tmp_path)
    environment = tmp_path / 'maps' / 'lab'
    environment.mkdir(parents=True)
    (environment / 'lab.bonxai').write_bytes(b'occupied')

    result = prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    assert result['localization_enabled'] is True
    assert prepare_map.read_pcd_info(environment / 'lab.pcd').point_count == 1
    assert (environment / 'lab.yaml').is_file()
    assert (environment / 'lab_chunks' / 'metadata.yaml').is_file()


def test_chunk_store_can_bootstrap_pcd_and_other_derivatives(tmp_path):
    converters = make_fake_converters(tmp_path)
    environment = tmp_path / 'maps' / 'lab'
    write_chunk_store(environment / 'lab_chunks')

    result = prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    assert result['localization_enabled'] is True
    assert prepare_map.read_pcd_info(environment / 'lab.pcd').point_count == 1
    assert (environment / 'lab.bonxai').read_bytes().startswith(b'occupied:')
    assert (environment / 'lab.yaml').is_file()


def test_unchanged_map_uses_fingerprinted_derivatives(tmp_path):
    converters = make_fake_converters(tmp_path)
    environment = tmp_path / 'maps' / 'lab'
    environment.mkdir(parents=True)
    write_binary_pcd(environment / 'lab.pcd')
    prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    clear_calls(tmp_path)
    result = prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    assert result['generated'] == []
    assert 'pcd-to-bonxai' not in calls(tmp_path)
    assert 'bonxai-to-pcd' not in calls(tmp_path)
    assert 'occupancy' not in calls(tmp_path)
    assert 'chunks' not in calls(tmp_path)
    state = json.loads((environment / prepare_map.STATE_FILE_NAME).read_text())
    assert state['sources']['pcd']['sha256']
    assert state['derivatives']['chunks']['outputs']['file_count'] == 2


def test_one_sided_pcd_change_rebuilds_all_pcd_derivatives(tmp_path):
    converters = make_fake_converters(tmp_path)
    environment = tmp_path / 'maps' / 'lab'
    environment.mkdir(parents=True)
    write_binary_pcd(environment / 'lab.pcd')
    prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    clear_calls(tmp_path)
    write_binary_pcd(environment / 'lab.pcd', ((4.0, 5.0, 6.0), (7.0, 8.0, 9.0)))
    result = prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    assert 'pcd-to-bonxai' in calls(tmp_path)
    assert 'bonxai-to-pcd' not in calls(tmp_path)
    assert 'occupancy' in calls(tmp_path)
    assert 'chunks' in calls(tmp_path)
    assert str(environment / 'lab.yaml') in result['generated']
    assert str(environment / 'lab_chunks') in result['generated']


def test_one_sided_bonxai_change_rebuilds_pcd_and_its_derivatives(tmp_path):
    converters = make_fake_converters(tmp_path)
    environment = tmp_path / 'maps' / 'lab'
    environment.mkdir(parents=True)
    write_binary_pcd(environment / 'lab.pcd')
    prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    clear_calls(tmp_path)
    (environment / 'lab.bonxai').write_bytes(b'occupied-new')
    prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    assert 'bonxai-to-pcd' in calls(tmp_path)
    assert 'pcd-to-bonxai' not in calls(tmp_path)
    assert 'occupancy' in calls(tmp_path)
    assert 'chunks' in calls(tmp_path)
    assert (environment / 'lab.pcd').read_bytes().endswith(
        struct.pack('<fff', 9.0, 2.0, 3.0)
    )


def test_simultaneous_source_change_keeps_supplied_pair(tmp_path):
    converters = make_fake_converters(tmp_path)
    environment = tmp_path / 'maps' / 'lab'
    environment.mkdir(parents=True)
    write_binary_pcd(environment / 'lab.pcd')
    prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    clear_calls(tmp_path)
    write_binary_pcd(environment / 'lab.pcd', ((3.0, 3.0, 3.0),))
    (environment / 'lab.bonxai').write_bytes(b'occupied-saved-pair')
    prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    assert 'bonxai-to-pcd' not in calls(tmp_path)
    assert 'pcd-to-bonxai' not in calls(tmp_path)
    assert 'occupancy' in calls(tmp_path)
    assert 'chunks' in calls(tmp_path)
    assert (environment / 'lab.bonxai').read_bytes() == b'occupied-saved-pair'


def test_changed_derivative_content_is_detected_and_rebuilt(tmp_path):
    converters = make_fake_converters(tmp_path)
    environment = tmp_path / 'maps' / 'lab'
    environment.mkdir(parents=True)
    write_binary_pcd(environment / 'lab.pcd')
    prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    clear_calls(tmp_path)
    (environment / 'lab.pgm').write_bytes(b'P5\n1 1\n255\n\x01')
    chunk = environment / 'lab_chunks' / '0_0_0' / '0_0_0.chunk'
    chunk.write_bytes(b'changed-but-still-nonempty')
    result = prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    assert 'occupancy' in calls(tmp_path)
    assert 'chunks' in calls(tmp_path)
    assert (environment / 'lab.pgm').read_bytes().endswith(b'\x00')
    assert chunk.read_bytes() == b'chunk'
    assert any('.incomplete-' in path for path in result['preserved'])


def test_invalid_nav2_and_chunk_outputs_are_rebuilt(tmp_path):
    converters = make_fake_converters(tmp_path)
    environment = tmp_path / 'maps' / 'lab'
    environment.mkdir(parents=True)
    write_binary_pcd(environment / 'lab.pcd')
    prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    clear_calls(tmp_path)
    (environment / 'lab.yaml').write_text('image: lab.pgm\nresolution: nope\n')
    metadata = environment / 'lab_chunks' / 'metadata.yaml'
    metadata.write_text('validated: true\n')
    prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    assert 'occupancy' in calls(tmp_path)
    assert 'chunks' in calls(tmp_path)
    assert prepare_map.valid_occupancy_pair(
        environment / 'lab.yaml', environment / 'lab.pgm'
    )
    assert prepare_map.valid_chunk_store(environment / 'lab_chunks')


def test_conversion_parameter_change_invalidates_only_affected_outputs(tmp_path):
    converters = make_fake_converters(tmp_path)
    environment = tmp_path / 'maps' / 'lab'
    environment.mkdir(parents=True)
    write_binary_pcd(environment / 'lab.pcd')
    prepare_map.prepare(arguments(tmp_path / 'maps', 'lab', converters))

    clear_calls(tmp_path)
    prepare_map.prepare(arguments(
        tmp_path / 'maps',
        'lab',
        converters,
        map_2d_resolution=0.25,
        chunk_size=5.0,
    ))

    assert 'occupancy' in calls(tmp_path)
    assert 'chunks' in calls(tmp_path)
    assert 'pcd-to-bonxai' not in calls(tmp_path)


def test_map_name_cannot_escape_root(tmp_path):
    converters = tuple(tmp_path / name for name in ('a', 'b', 'c'))
    try:
        prepare_map.prepare(arguments(tmp_path, '../escape', converters))
    except ValueError as error:
        assert 'map_name' in str(error)
    else:
        raise AssertionError('unsafe map name was accepted')
