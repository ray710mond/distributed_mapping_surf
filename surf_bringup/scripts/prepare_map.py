#!/usr/bin/env python3
"""Prepare the canonical artifacts for one named SURF environment map."""

from __future__ import annotations

import argparse
import ast
import fcntl
import hashlib
import json
import math
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass


MAP_NAME_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*")
STATE_SCHEMA_VERSION = 1
STATE_FILE_NAME = ".surf-map-state.json"


@dataclass(frozen=True)
class PcdInfo:
    point_count: int
    conversion_compatible: bool


def log(message: str) -> None:
    print(f"prepare_map: {message}", file=sys.stderr, flush=True)


def read_pcd_info(path: pathlib.Path) -> PcdInfo:
    fields: list[str] = []
    sizes: list[int] = []
    types: list[str] = []
    counts: list[int] = []
    width = 0
    height = 1
    points = 0
    data_kind = ""

    with path.open("rb") as stream:
        for _ in range(256):
            raw = stream.readline()
            if not raw:
                raise ValueError("PCD header ended before DATA")
            try:
                words = raw.decode("ascii").strip().split()
            except UnicodeDecodeError as error:
                raise ValueError("PCD header is not ASCII") from error
            if not words or words[0].startswith("#"):
                continue
            key, values = words[0].upper(), words[1:]
            if key in ("FIELDS", "FIELD"):
                fields = values
            elif key == "SIZE":
                sizes = [int(value) for value in values]
            elif key == "TYPE":
                types = [value.upper() for value in values]
            elif key == "COUNT":
                counts = [int(value) for value in values]
            elif key == "WIDTH":
                width = int(values[0])
            elif key == "HEIGHT":
                height = int(values[0])
            elif key == "POINTS":
                points = int(values[0])
            elif key == "DATA":
                data_kind = values[0].lower()
                break
        else:
            raise ValueError("PCD header is unreasonably long")

    if not points:
        points = width * height
    if points < 0:
        raise ValueError("PCD point count is negative")
    if not counts:
        counts = [1] * len(fields)
    compatible = (
        data_kind == "binary"
        and len(fields) == len(sizes) == len(types) == len(counts)
        and all(
            axis in fields
            and sizes[fields.index(axis)] == 4
            and types[fields.index(axis)] == "F"
            and counts[fields.index(axis)] == 1
            for axis in ("x", "y", "z")
        )
    )
    return PcdInfo(point_count=points, conversion_compatible=compatible)


def run(command: list[str]) -> str:
    log("running " + " ".join(command))
    result = subprocess.run(
        command,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    if result.stdout:
        for line in result.stdout.rstrip().splitlines():
            log("  " + line)
    if result.returncode != 0:
        raise RuntimeError(
            f"converter exited with status {result.returncode}: {' '.join(command)}"
        )
    return result.stdout.strip()


def unique_backup_path(path: pathlib.Path) -> pathlib.Path:
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    candidate = path.with_name(f"{path.name}.incomplete-{stamp}")
    suffix = 1
    while candidate.exists():
        candidate = path.with_name(f"{path.name}.incomplete-{stamp}-{suffix}")
        suffix += 1
    return candidate


def preserve_existing(path: pathlib.Path, preserved: list[str]) -> None:
    if not path.exists():
        return
    backup = unique_backup_path(path)
    path.rename(backup)
    preserved.append(str(backup))
    log(f"preserved incomplete artifact as {backup}")


def inspect_bonxai(converter: pathlib.Path, path: pathlib.Path) -> int:
    output = run([str(converter), "inspect-bonxai", str(path)])
    try:
        return int(output.splitlines()[-1])
    except (IndexError, ValueError) as error:
        raise RuntimeError(
            f"could not parse occupied-cell count from {converter}"
        ) from error


def inspect_pcd(converter: pathlib.Path, path: pathlib.Path) -> int:
    output = run([str(converter), "inspect-pcd", str(path)])
    try:
        return int(output.splitlines()[-1])
    except (IndexError, ValueError) as error:
        raise RuntimeError(
            f"could not parse finite-point count from {converter}"
        ) from error


def read_bonxai_resolution(path: pathlib.Path) -> float | None:
    try:
        with path.open("rb") as stream:
            header = stream.readline().decode("ascii").strip()
    except (OSError, UnicodeDecodeError):
        return None
    match = re.search(r"\(([-+0-9.eE]+)\)$", header)
    if not match:
        return None
    try:
        resolution = float(match.group(1))
    except ValueError:
        return None
    return resolution if resolution > 0 else None


def write_bonxai_metadata(
    metadata_path: pathlib.Path,
    bonxai_path: pathlib.Path,
    pcd_path: pathlib.Path,
    pcd_points: int,
    occupied_cells: int,
    resolution: float,
    generated: list[str],
    preserved: list[str],
    *,
    force: bool = False,
) -> None:
    if not force and metadata_path.is_file() and metadata_path.stat().st_size > 0:
        return
    temporary = temporary_output(metadata_path.parent, ".yaml")
    try:
        temporary.write_text(
            "format: bonxai_static_map_v1\n"
            f"map_file: {bonxai_path.name}\n"
            f"localization_pcd: {pcd_path.name}\n"
            f"localization_points: {pcd_points}\n"
            f"occupied_cells: {occupied_cells}\n"
            "frame_id: map\n"
            f"resolution: {resolution:.12g}\n",
            encoding="utf-8",
        )
        preserve_existing(metadata_path, preserved)
        os.replace(temporary, metadata_path)
        generated.append(str(metadata_path))
    finally:
        temporary.unlink(missing_ok=True)


def _yaml_scalars(path: pathlib.Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    return values


def _valid_pgm(path: pathlib.Path) -> bool:
    """Validate the Netpbm header and payload size used by Nav2 map_server."""
    try:
        data = path.read_bytes()
        offset = 0

        def token() -> bytes:
            nonlocal offset
            while offset < len(data):
                if data[offset] in b" \t\r\n":
                    offset += 1
                    continue
                if data[offset] == ord("#"):
                    newline = data.find(b"\n", offset)
                    if newline < 0:
                        raise ValueError("unterminated PGM comment")
                    offset = newline + 1
                    continue
                break
            start = offset
            while offset < len(data) and data[offset] not in b" \t\r\n#":
                offset += 1
            if start == offset:
                raise ValueError("missing PGM token")
            return data[start:offset]

        magic = token()
        width = int(token())
        height = int(token())
        maximum = int(token())
        if magic not in (b"P2", b"P5") or width <= 0 or height <= 0:
            return False
        if maximum <= 0 or maximum > 65535:
            return False
        if magic == b"P2":
            pixels = []
            while True:
                try:
                    pixels.append(int(token()))
                except ValueError as error:
                    if "missing PGM token" not in str(error):
                        raise
                    break
            return (
                len(pixels) == width * height
                and all(0 <= value <= maximum for value in pixels)
            )

        if offset >= len(data) or data[offset] not in b" \t\r\n":
            return False
        if data[offset:offset + 2] == b"\r\n":
            offset += 2
        else:
            offset += 1
        bytes_per_pixel = 1 if maximum < 256 else 2
        return len(data) - offset == width * height * bytes_per_pixel
    except (OSError, UnicodeError, ValueError):
        return False


def valid_occupancy_pair(yaml_path: pathlib.Path, pgm_path: pathlib.Path) -> bool:
    if not yaml_path.is_file() or not pgm_path.is_file():
        return False
    if yaml_path.stat().st_size == 0 or pgm_path.stat().st_size == 0:
        return False
    try:
        values = _yaml_scalars(yaml_path)
        image_name = values["image"].strip("\"'")
        resolution = float(values["resolution"])
        origin = ast.literal_eval(values["origin"])
        negate = int(values["negate"])
        occupied = float(values["occupied_thresh"])
        free = float(values["free_thresh"])
    except (KeyError, OSError, SyntaxError, TypeError, UnicodeError, ValueError):
        return False
    if (
        not image_name
        or not math.isfinite(resolution)
        or resolution <= 0
        or not isinstance(origin, (list, tuple))
        or len(origin) != 3
        or not all(isinstance(value, (int, float)) and math.isfinite(value) for value in origin)
        or negate not in (0, 1)
        or not math.isfinite(occupied)
        or not math.isfinite(free)
        or not 0 <= free < occupied <= 1
        or values.get("mode", "trinary") not in ("trinary", "scale", "raw")
    ):
        return False
    image_path = pathlib.Path(image_name)
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    return (
        image_path.resolve() == pgm_path.resolve()
        and image_path.is_file()
        and _valid_pgm(image_path)
    )


def valid_chunk_store(path: pathlib.Path) -> bool:
    metadata = path / "metadata.yaml"
    if not path.is_dir() or not metadata.is_file() or metadata.stat().st_size == 0:
        return False
    try:
        values = _yaml_scalars(metadata)
        chunks = int(values["chunks"])
        occupied_cells = int(values["occupied_cells"])
        resolution = float(values["resolution"])
        chunk_size = float(values["chunk_size"])
        chunk_files = sorted(path.rglob("*.chunk"))
    except (KeyError, OSError, UnicodeError, ValueError):
        return False
    return (
        values.get("format") == "rolling_bonxai_chunk_store"
        and values.get("format_version") == "1"
        and values.get("validated", "").lower() == "true"
        and chunks > 0
        and occupied_cells > 0
        and math.isfinite(resolution)
        and resolution > 0
        and math.isfinite(chunk_size)
        and chunk_size >= resolution
        and len(chunk_files) == chunks
        and all(chunk.is_file() and chunk.stat().st_size > 0 for chunk in chunk_files)
    )


def fingerprint_file(path: pathlib.Path) -> dict[str, object]:
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
            size += len(block)
    return {"sha256": digest.hexdigest(), "size": size}


def fingerprint_directory(path: pathlib.Path) -> dict[str, object]:
    """Hash every regular file and its relative name in a directory tree."""
    digest = hashlib.sha256()
    files = sorted(candidate for candidate in path.rglob("*") if candidate.is_file())
    total_size = 0
    for candidate in files:
        relative = candidate.relative_to(path).as_posix().encode("utf-8")
        fingerprint = fingerprint_file(candidate)
        digest.update(len(relative).to_bytes(8, "little"))
        digest.update(relative)
        digest.update(bytes.fromhex(str(fingerprint["sha256"])))
        digest.update(int(fingerprint["size"]).to_bytes(8, "little"))
        total_size += int(fingerprint["size"])
    return {
        "sha256": digest.hexdigest(),
        "file_count": len(files),
        "total_size": total_size,
    }


def occupancy_fingerprint(
    yaml_path: pathlib.Path,
    pgm_path: pathlib.Path,
    metadata_path: pathlib.Path,
) -> dict[str, object]:
    return {
        "yaml": fingerprint_file(yaml_path),
        "pgm": fingerprint_file(pgm_path),
        "metadata": (
            fingerprint_file(metadata_path) if metadata_path.is_file() else None
        ),
    }


def conversion_record(
    source_pcd: dict[str, object],
    converter: pathlib.Path,
    parameters: dict[str, float],
    outputs: dict[str, object],
) -> dict[str, object]:
    return {
        "source_pcd": source_pcd,
        "converter": converter_fingerprint(converter),
        "parameters": parameters,
        "outputs": outputs,
    }


def conversion_is_fresh(
    record: object,
    source_pcd: dict[str, object],
    converter: pathlib.Path,
    parameters: dict[str, float],
    outputs: dict[str, object],
) -> bool:
    if not isinstance(record, dict):
        return False
    return record == conversion_record(
        source_pcd, converter, parameters, outputs
    )


def converter_fingerprint(path: pathlib.Path) -> dict[str, object]:
    try:
        return {"path": str(path.resolve()), **fingerprint_file(path)}
    except OSError:
        # Complete, fresh maps do not need to execute a converter. Keeping the
        # unavailable path in the signature still makes changes deterministic.
        return {"path": str(path), "unavailable": True}


def load_state(
    path: pathlib.Path,
    map_name: str,
    preserved: list[str],
) -> dict[str, object] | None:
    if not path.exists():
        return None
    try:
        state = json.loads(path.read_text(encoding="utf-8"))
        if (
            not isinstance(state, dict)
            or state.get("schema_version") != STATE_SCHEMA_VERSION
            or state.get("map_name") != map_name
            or not isinstance(state.get("sources"), dict)
            or not isinstance(state.get("derivatives"), dict)
        ):
            raise ValueError("state schema or map name does not match")
        return state
    except (OSError, UnicodeError, ValueError, json.JSONDecodeError) as error:
        log(f"ignoring invalid manager state {path}: {error}")
        preserve_existing(path, preserved)
        return None


def write_state_atomic(path: pathlib.Path, state: dict[str, object]) -> None:
    temporary = temporary_output(path.parent, ".json")
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(state, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        try:
            descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        except OSError:
            # The file replace is still atomic on filesystems without directory fsync.
            pass
    finally:
        temporary.unlink(missing_ok=True)


def temporary_output(parent: pathlib.Path, suffix: str) -> pathlib.Path:
    descriptor, name = tempfile.mkstemp(prefix=".surf-map-", suffix=suffix, dir=parent)
    os.close(descriptor)
    path = pathlib.Path(name)
    path.unlink()
    return path


def convert_bonxai_to_pcd(
    converter: pathlib.Path,
    bonxai_path: pathlib.Path,
    pcd_path: pathlib.Path,
    generated: list[str],
) -> None:
    temporary = temporary_output(pcd_path.parent, ".pcd")
    try:
        run([
            str(converter),
            "bonxai-to-pcd",
            str(bonxai_path),
            str(temporary),
        ])
        os.replace(temporary, pcd_path)
        generated.append(str(pcd_path))
    finally:
        temporary.unlink(missing_ok=True)


def convert_chunks_to_pcd(
    converter: pathlib.Path,
    chunk_directory: pathlib.Path,
    pcd_path: pathlib.Path,
    generated: list[str],
) -> None:
    temporary = temporary_output(pcd_path.parent, ".pcd")
    try:
        run([
            str(converter),
            "chunks-to-pcd",
            str(chunk_directory),
            str(temporary),
        ])
        os.replace(temporary, pcd_path)
        generated.append(str(pcd_path))
    finally:
        temporary.unlink(missing_ok=True)


def convert_pcd_to_bonxai(
    converter: pathlib.Path,
    pcd_path: pathlib.Path,
    bonxai_path: pathlib.Path,
    resolution: float,
    generated: list[str],
    preserved: list[str],
) -> None:
    temporary = temporary_output(bonxai_path.parent, ".bonxai")
    try:
        run([
            str(converter),
            "pcd-to-bonxai",
            str(pcd_path),
            str(temporary),
            "--resolution",
            str(resolution),
        ])
        preserve_existing(bonxai_path, preserved)
        os.replace(temporary, bonxai_path)
        generated.append(str(bonxai_path))
    finally:
        temporary.unlink(missing_ok=True)


def normalized_conversion_input(
    converter: pathlib.Path,
    pcd_path: pathlib.Path,
    info: PcdInfo,
) -> tuple[pathlib.Path, pathlib.Path | None]:
    if info.conversion_compatible:
        return pcd_path, None
    temporary = temporary_output(pcd_path.parent, ".pcd")
    run([str(converter), "normalize-pcd", str(pcd_path), str(temporary)])
    normalized = read_pcd_info(temporary)
    if not normalized.conversion_compatible or normalized.point_count != info.point_count:
        temporary.unlink(missing_ok=True)
        raise RuntimeError("normalized PCD failed compatibility or point-count validation")
    return temporary, temporary


def normalize_canonical_pcd(
    converter: pathlib.Path,
    pcd_path: pathlib.Path,
    generated: list[str],
    preserved: list[str],
) -> PcdInfo:
    temporary = temporary_output(pcd_path.parent, ".pcd")
    try:
        run([str(converter), "normalize-pcd", str(pcd_path), str(temporary)])
        normalized = read_pcd_info(temporary)
        preserve_existing(pcd_path, preserved)
        os.replace(temporary, pcd_path)
        generated.append(str(pcd_path))
        return normalized
    finally:
        temporary.unlink(missing_ok=True)


def generate_occupancy_pair(
    converter: pathlib.Path,
    pcd_path: pathlib.Path,
    output_stem: pathlib.Path,
    resolution: float,
    generated: list[str],
    preserved: list[str],
) -> None:
    with tempfile.TemporaryDirectory(prefix=".surf-map-2d-", dir=output_stem.parent) as raw:
        temporary_stem = pathlib.Path(raw) / output_stem.name
        run([
            str(converter),
            str(pcd_path),
            str(temporary_stem),
            "--resolution",
            str(resolution),
        ])
        outputs = [
            (temporary_stem.with_suffix(".pgm"), output_stem.with_suffix(".pgm")),
            (temporary_stem.with_suffix(".yaml"), output_stem.with_suffix(".yaml")),
            (
                temporary_stem.with_name(temporary_stem.name + "_metadata.yaml"),
                output_stem.with_name(output_stem.name + "_2d_metadata.yaml"),
            ),
        ]
        for temporary, final in outputs:
            if not temporary.is_file() or temporary.stat().st_size == 0:
                raise RuntimeError(f"2D converter did not produce {temporary.name}")
            preserve_existing(final, preserved)
            os.replace(temporary, final)
            generated.append(str(final))


def generate_chunk_store(
    converter: pathlib.Path,
    pcd_path: pathlib.Path,
    output: pathlib.Path,
    resolution: float,
    chunk_size: float,
    generated: list[str],
    preserved: list[str],
) -> None:
    temporary = pathlib.Path(tempfile.mkdtemp(prefix=".surf-chunks-parent-", dir=output.parent))
    temporary_output_directory = temporary / output.name
    try:
        run([
            str(converter),
            str(pcd_path),
            str(temporary_output_directory),
            "--resolution",
            str(resolution),
            "--chunk-size",
            str(chunk_size),
        ])
        if not valid_chunk_store(temporary_output_directory):
            raise RuntimeError("chunk converter produced an incomplete store")
        preserve_existing(output, preserved)
        os.replace(temporary_output_directory, output)
        generated.append(str(output))
    finally:
        shutil.rmtree(temporary, ignore_errors=True)


def prepare(args: argparse.Namespace) -> dict[str, object]:
    if not MAP_NAME_PATTERN.fullmatch(args.map_name):
        raise ValueError(
            "map_name must start with an alphanumeric character and contain only "
            "letters, numbers, underscores, and hyphens"
        )
    if args.voxel_resolution <= 0 or args.map_2d_resolution <= 0 or args.chunk_size <= 0:
        raise ValueError("map resolutions and chunk_size must be greater than zero")

    map_root = args.map_root.expanduser().resolve()
    environment = map_root / args.map_name
    directory_existed = environment.is_dir()
    if environment.exists() and not environment.is_dir():
        raise RuntimeError(f"map path exists but is not a directory: {environment}")
    environment.mkdir(parents=True, exist_ok=True)

    pcd_path = environment / f"{args.map_name}.pcd"
    bonxai_path = environment / f"{args.map_name}.bonxai"
    bonxai_metadata_path = environment / f"{args.map_name}.bonxai.yaml"
    output_stem = environment / args.map_name
    yaml_path = output_stem.with_suffix(".yaml")
    pgm_path = output_stem.with_suffix(".pgm")
    occupancy_metadata_path = output_stem.with_name(
        output_stem.name + "_2d_metadata.yaml"
    )
    chunk_directory = environment / f"{args.map_name}_chunks"
    state_path = environment / STATE_FILE_NAME
    generated: list[str] = []
    preserved: list[str] = []

    lock_path = environment / ".prepare.lock"
    with lock_path.open("a+", encoding="utf-8") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)

        state = load_state(state_path, args.map_name, preserved)
        previous_sources = state["sources"] if state is not None else {}
        previous_derivatives = state["derivatives"] if state is not None else {}
        source_history_available = (
            state is not None
            and "pcd" in previous_sources
            and "bonxai" in previous_sources
        )
        initial_pcd_fingerprint = (
            fingerprint_file(pcd_path) if pcd_path.is_file() else None
        )
        initial_bonxai_fingerprint = (
            fingerprint_file(bonxai_path) if bonxai_path.is_file() else None
        )
        pcd_changed = (
            source_history_available
            and initial_pcd_fingerprint != previous_sources.get("pcd")
        )
        bonxai_changed = (
            source_history_available
            and initial_bonxai_fingerprint != previous_sources.get("bonxai")
        )

        chunks_are_valid = valid_chunk_store(chunk_directory)
        bonxai_cells: int | None = None
        if bonxai_path.is_file():
            try:
                bonxai_cells = inspect_bonxai(args.bonxai_converter, bonxai_path)
            except RuntimeError:
                if not pcd_path.is_file() and not chunks_are_valid:
                    raise
                log(
                    "existing Bonxai file is invalid; rebuilding it from "
                    "another valid 3D source"
                )

        pcd_info: PcdInfo | None = None
        canonical_pcd_modified = False
        if pcd_path.is_file():
            try:
                pcd_info = read_pcd_info(pcd_path)
                finite_points = inspect_pcd(args.bonxai_converter, pcd_path)
                if finite_points == 0 and pcd_info.point_count > 0:
                    log(f"{pcd_path} contains no finite XYZ points")
                    preserve_existing(pcd_path, preserved)
                    pcd_info = None
                elif finite_points != pcd_info.point_count:
                    log(
                        f"removing {pcd_info.point_count - finite_points} "
                        f"non-finite points from {pcd_path}"
                    )
                    pcd_info = normalize_canonical_pcd(
                        args.bonxai_converter,
                        pcd_path,
                        generated,
                        preserved,
                    )
                    canonical_pcd_modified = True
            except (RuntimeError, ValueError):
                if not bonxai_cells and not chunks_are_valid:
                    raise
                preserve_existing(pcd_path, preserved)
                pcd_info = None

        # Once a state exists, it disambiguates external updates. A source that
        # changed alone is authoritative. Simultaneous changes are the normal
        # save/import case and are kept as a supplied pair. On first adoption,
        # the documented canonical PCD precedence establishes a known pair.
        force_pcd_from_bonxai = (
            source_history_available
            and bonxai_changed
            and not pcd_changed
            and pcd_info is not None
            and pcd_info.point_count > 0
            and bool(bonxai_cells)
        )
        force_bonxai_from_pcd = (
            pcd_info is not None
            and pcd_info.point_count > 0
            and bool(bonxai_cells)
            and (
                (not source_history_available)
                or (pcd_changed and not bonxai_changed)
                or canonical_pcd_modified
            )
        )
        if source_history_available and pcd_changed and bonxai_changed:
            log("PCD and Bonxai both changed; treating them as a supplied map pair")
        elif force_pcd_from_bonxai:
            log("Bonxai changed independently; regenerating the canonical PCD")
            preserve_existing(pcd_path, preserved)
            convert_bonxai_to_pcd(
                args.bonxai_converter, bonxai_path, pcd_path, generated
            )
            pcd_info = read_pcd_info(pcd_path)
            finite_points = inspect_pcd(args.bonxai_converter, pcd_path)
            if pcd_info.point_count == 0 or finite_points != pcd_info.point_count:
                raise RuntimeError("Bonxai conversion produced an invalid canonical PCD")
            canonical_pcd_modified = True
            force_bonxai_from_pcd = False
        elif force_bonxai_from_pcd:
            log("PCD is authoritative; regenerating its Bonxai counterpart")

        if (pcd_info is None or pcd_info.point_count == 0) and bonxai_cells:
            if pcd_path.exists():
                preserve_existing(pcd_path, preserved)
            convert_bonxai_to_pcd(
                args.bonxai_converter, bonxai_path, pcd_path, generated
            )
            pcd_info = read_pcd_info(pcd_path)
            canonical_pcd_modified = True

        if (pcd_info is None or pcd_info.point_count == 0) and chunks_are_valid:
            if pcd_path.exists():
                preserve_existing(pcd_path, preserved)
            convert_chunks_to_pcd(
                args.bonxai_converter, chunk_directory, pcd_path, generated
            )
            pcd_info = read_pcd_info(pcd_path)
            canonical_pcd_modified = True

        localization_enabled = pcd_info is not None and pcd_info.point_count > 0
        new_derivatives: dict[str, object] = {}
        if localization_enabled:
            bonxai_resolution = (
                read_bonxai_resolution(bonxai_path)
                if bonxai_path.is_file()
                else None
            )
            resolution_mismatch = (
                bonxai_resolution is not None
                and abs(bonxai_resolution - args.voxel_resolution) > 1.0e-9
            )
            if resolution_mismatch:
                log(
                    f"Bonxai resolution {bonxai_resolution:g} does not match "
                    f"the configured {args.voxel_resolution:g}; rebuilding it "
                    "from the canonical PCD"
                )
            if (
                bonxai_cells is None
                or bonxai_cells == 0
                or resolution_mismatch
                or force_bonxai_from_pcd
            ):
                convert_pcd_to_bonxai(
                    args.bonxai_converter,
                    pcd_path,
                    bonxai_path,
                    args.voxel_resolution,
                    generated,
                    preserved,
                )
                bonxai_cells = inspect_bonxai(
                    args.bonxai_converter, bonxai_path)
                bonxai_resolution = args.voxel_resolution

            current_pcd_fingerprint = fingerprint_file(pcd_path)
            current_bonxai_fingerprint = fingerprint_file(bonxai_path)
            current_metadata_fingerprint = (
                fingerprint_file(bonxai_metadata_path)
                if bonxai_metadata_path.is_file()
                else None
            )
            metadata_record = {
                "source_pcd": current_pcd_fingerprint,
                "source_bonxai": current_bonxai_fingerprint,
                "parameters": {"voxel_resolution": args.voxel_resolution},
                "output": current_metadata_fingerprint,
            }
            metadata_fresh = (
                current_metadata_fingerprint is not None
                and previous_derivatives.get("bonxai_metadata") == metadata_record
            )
            write_bonxai_metadata(
                bonxai_metadata_path,
                bonxai_path,
                pcd_path,
                pcd_info.point_count,
                bonxai_cells,
                bonxai_resolution or args.voxel_resolution,
                generated,
                preserved,
                force=not metadata_fresh,
            )
            metadata_record["output"] = fingerprint_file(bonxai_metadata_path)
            new_derivatives["bonxai_metadata"] = metadata_record

            occupancy_parameters = {
                "map_2d_resolution": args.map_2d_resolution,
            }
            occupancy_valid = valid_occupancy_pair(yaml_path, pgm_path)
            occupancy_outputs = (
                occupancy_fingerprint(
                    yaml_path, pgm_path, occupancy_metadata_path
                )
                if occupancy_valid
                else {}
            )
            occupancy_fresh = occupancy_valid and conversion_is_fresh(
                previous_derivatives.get("occupancy_2d"),
                current_pcd_fingerprint,
                args.occupancy_converter,
                occupancy_parameters,
                occupancy_outputs,
            )

            chunk_parameters = {
                "voxel_resolution": args.voxel_resolution,
                "chunk_size": args.chunk_size,
            }
            chunks_are_valid = valid_chunk_store(chunk_directory)
            chunk_outputs = (
                fingerprint_directory(chunk_directory) if chunks_are_valid else {}
            )
            chunks_are_fresh = chunks_are_valid and conversion_is_fresh(
                previous_derivatives.get("chunks"),
                current_pcd_fingerprint,
                args.chunk_converter,
                chunk_parameters,
                chunk_outputs,
            )

            if not occupancy_fresh or not chunks_are_fresh:
                conversion_input, cleanup_input = normalized_conversion_input(
                    args.bonxai_converter, pcd_path, pcd_info
                )
                try:
                    if not occupancy_fresh:
                        log("2D occupancy derivative is stale or invalid; regenerating")
                        generate_occupancy_pair(
                            args.occupancy_converter,
                            conversion_input,
                            output_stem,
                            args.map_2d_resolution,
                            generated,
                            preserved,
                        )
                        if not valid_occupancy_pair(yaml_path, pgm_path):
                            raise RuntimeError(
                                "2D converter produced a map Nav2 cannot load"
                            )
                        occupancy_outputs = occupancy_fingerprint(
                            yaml_path, pgm_path, occupancy_metadata_path
                        )
                    if not chunks_are_fresh:
                        log("chunk-store derivative is stale or invalid; regenerating")
                        generate_chunk_store(
                            args.chunk_converter,
                            conversion_input,
                            chunk_directory,
                            args.voxel_resolution,
                            args.chunk_size,
                            generated,
                            preserved,
                        )
                        chunk_outputs = fingerprint_directory(chunk_directory)
                finally:
                    if cleanup_input is not None:
                        cleanup_input.unlink(missing_ok=True)

            new_derivatives["occupancy_2d"] = conversion_record(
                current_pcd_fingerprint,
                args.occupancy_converter,
                occupancy_parameters,
                occupancy_outputs,
            )
            new_derivatives["chunks"] = conversion_record(
                current_pcd_fingerprint,
                args.chunk_converter,
                chunk_parameters,
                chunk_outputs,
            )
        else:
            log(
                f"no non-empty {args.map_name}.pcd or Bonxai source exists; "
                "fixed-map localization is disabled for this run"
            )

        write_state_atomic(
            state_path,
            {
                "schema_version": STATE_SCHEMA_VERSION,
                "map_name": args.map_name,
                "sources": {
                    "pcd": fingerprint_file(pcd_path) if pcd_path.is_file() else None,
                    "bonxai": (
                        fingerprint_file(bonxai_path)
                        if bonxai_path.is_file()
                        else None
                    ),
                },
                "derivatives": new_derivatives,
            },
        )

    return {
        "map_name": args.map_name,
        "map_directory": str(environment),
        "directory_created": not directory_existed,
        "localization_enabled": localization_enabled,
        "pcd_path": str(pcd_path),
        "bonxai_path": str(bonxai_path),
        "map_yaml": str(yaml_path) if localization_enabled else "",
        "chunk_directory": str(chunk_directory) if localization_enabled else "",
        "generated": generated,
        "preserved": preserved,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-root", required=True, type=pathlib.Path)
    parser.add_argument("--map-name", required=True)
    parser.add_argument("--bonxai-converter", required=True, type=pathlib.Path)
    parser.add_argument("--chunk-converter", required=True, type=pathlib.Path)
    parser.add_argument("--occupancy-converter", required=True, type=pathlib.Path)
    parser.add_argument("--voxel-resolution", type=float, default=0.05)
    parser.add_argument("--chunk-size", type=float, default=10.0)
    parser.add_argument("--map-2d-resolution", type=float, default=0.20)
    return parser.parse_args()


def main() -> int:
    try:
        result = prepare(parse_args())
        print(json.dumps(result, sort_keys=True))
        return 0
    except (OSError, RuntimeError, ValueError) as error:
        log(f"error: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
