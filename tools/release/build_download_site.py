#!/usr/bin/env python3
"""Build an encrypted static download site and a separate private access link.

Only the output directory is suitable for public hosting. The access link is a
bearer secret: anyone who receives it can decrypt the equipment delivery. Its
fragment stays in the browser and is never needed by the hosting service.
"""
from __future__ import annotations

import argparse
import base64
from contextlib import contextmanager, ExitStack
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
from urllib.parse import unquote, urlsplit, urlunsplit


TEMPLATE_DIRECTORY = Path(__file__).with_name("download_site")
TEMPLATE_FILES = ("index.html", "app.js", "style.css")
MAX_ARCHIVE_BYTES = 100 * 1024 * 1024
MAX_TEMPLATE_BYTES = 1024 * 1024
ADDITIONAL_DATA = b"VisionTactile download v1"
FILENAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,120}\.tar\.gz\Z")


class BuildError(ValueError):
    """A diagnostic which never contains private archive bytes or link keys."""


def _absolute_path(value, label):
    path = Path(value).expanduser()
    if ".." in path.parts or "\x00" in str(path):
        raise BuildError(f"{label} must not contain path traversal or null bytes.")
    return path.absolute()


@contextmanager
def _directory(path):
    """Keep directory handles open; no path component may be a symlink."""
    descriptor = os.open(path.anchor, os.O_RDONLY | os.O_DIRECTORY)
    try:
        for part in path.parts[1:]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
                            dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        yield descriptor
    finally:
        os.close(descriptor)


def _absent(directory, name, label):
    try:
        os.stat(name, dir_fd=directory, follow_symlinks=False)
    except FileNotFoundError:
        return
    raise BuildError(f"{label} already exists; choose a new location.")


def _read_regular(directory, name, maximum, label):
    descriptor = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK,
                         dir_fd=directory)
    with os.fdopen(descriptor, "rb") as stream:
        before = os.fstat(stream.fileno())
        if not stat.S_ISREG(before.st_mode) or not 0 < before.st_size <= maximum:
            raise BuildError(f"{label} must be a nonempty regular file within its size limit.")
        content = stream.read(maximum + 1)
        after = os.fstat(stream.fileno())
    if (len(content) != before.st_size or len(content) > maximum
            or (before.st_size, before.st_mtime_ns, before.st_ctime_ns)
            != (after.st_size, after.st_mtime_ns, after.st_ctime_ns)):
        raise BuildError(f"{label} changed while being read; retry with a stable file.")
    return content


def _base_url(value):
    if any(ord(character) <= 32 or ord(character) >= 127 for character in value):
        raise BuildError("Base URL must be an ASCII HTTPS URL without whitespace.")
    try:
        parsed = urlsplit(value)
        if (parsed.scheme != "https" or not parsed.hostname or parsed.username is not None
                or parsed.password is not None or parsed.query or parsed.fragment
                or parsed.port not in (None, 443) or "\\" in value):
            raise ValueError
        decoded_path = unquote(parsed.path)
        if ".." in decoded_path.split("/") or "\\" in decoded_path:
            raise ValueError
    except ValueError as exc:
        raise BuildError("Base URL must be HTTPS without credentials, a query or a fragment.") from exc
    return urlunsplit((parsed.scheme, parsed.netloc, parsed.path.rstrip("/") + "/", "", ""))


def _base64url(content):
    return base64.urlsafe_b64encode(content).rstrip(b"=").decode("ascii")


def _digest(content):
    return hashlib.sha256(content).hexdigest()


def _write_new(directory, name, content, mode):
    descriptor = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                         mode, dir_fd=directory)
    with os.fdopen(descriptor, "wb") as stream:
        # An unusually restrictive or permissive umask must not change the contract.
        os.fchmod(stream.fileno(), mode)
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())


def _same_entry(directory, name, identity):
    try:
        current = os.stat(name, dir_fd=directory, follow_symlinks=False)
    except FileNotFoundError:
        return False
    return (current.st_dev, current.st_ino) == identity


def build_site(input_path, output, link_file, base_url, source_commit=None, *,
               template_directory=None):
    """Encrypt a bounded archive; publish only ciphertext and trusted templates.

    The input archive is never extracted. All output entries are newly created;
    an existing output directory or access-link file is never overwritten.
    """
    input_path = _absolute_path(input_path, "Input")
    output = _absolute_path(output, "Output")
    link_file = _absolute_path(link_file, "Link file")
    templates = _absolute_path(template_directory or TEMPLATE_DIRECTORY, "Templates")
    base_url = _base_url(base_url)
    if not FILENAME.fullmatch(input_path.name):
        raise BuildError("Input filename must be a safe ASCII basename ending in .tar.gz.")
    if link_file == output or link_file.is_relative_to(output):
        raise BuildError("The private link file must be outside the public output directory.")
    if output == output.parent or link_file == link_file.parent:
        raise BuildError("Output and link file must have a filename component.")
    if source_commit is not None and not re.fullmatch(r"[0-9a-f]{7,40}", source_commit):
        raise BuildError("Source commit must contain 7 to 40 lowercase hexadecimal characters.")

    try:
        with ExitStack() as stack:
            input_directory = stack.enter_context(_directory(input_path.parent))
            output_directory = stack.enter_context(_directory(output.parent))
            link_directory = stack.enter_context(_directory(link_file.parent))
            template_handle = stack.enter_context(_directory(templates))
            _absent(output_directory, output.name, "Output directory")
            _absent(link_directory, link_file.name, "Private link file")
            static_files = {name: _read_regular(template_handle, name, MAX_TEMPLATE_BYTES,
                                                "Site template") for name in TEMPLATE_FILES}
            plaintext = _read_regular(input_directory, input_path.name, MAX_ARCHIVE_BYTES,
                                      "Input archive")
            try:
                from cryptography.hazmat.primitives.ciphers.aead import AESGCM
            except ImportError as exc:
                raise BuildError("Install the build dependency with: python3 -m pip install cryptography") from exc
            key = AESGCM.generate_key(bit_length=256)
            nonce = os.urandom(12)
            ciphertext = AESGCM(key).encrypt(nonce, plaintext, ADDITIONAL_DATA)
            cipher_sha256 = _digest(ciphertext)
            payload_name = cipher_sha256 + ".bin"
            manifest = {
                "schema_version": 1, "cipher": "AES-256-GCM", "iv": _base64url(nonce),
                "payload": "packages/" + payload_name, "size": len(plaintext),
                "cipher_size": len(ciphertext), "sha256": _digest(plaintext),
                "cipher_sha256": cipher_sha256, "filename": input_path.name,
            }
            if source_commit is not None:
                manifest["source_commit"] = source_commit
            manifest_bytes = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
            manifest_sha256 = _digest(manifest_bytes)
            private_link = (base_url + "#key=" + _base64url(key)
                            + "&manifest=" + manifest_sha256 + "\n").encode("ascii")
            output_identity = link_identity = None
            try:
                # Reserve the secret file first and keep the public tree inaccessible
                # until both products are complete. Neither location is replaceable.
                link_descriptor = os.open(link_file.name,
                                          os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                                          0o600, dir_fd=link_directory)
                with os.fdopen(link_descriptor, "wb") as stream:
                    info = os.fstat(stream.fileno())
                    link_identity = (info.st_dev, info.st_ino)
                    os.fchmod(stream.fileno(), 0o600)
                    stream.write(private_link)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.mkdir(output.name, mode=0o700, dir_fd=output_directory)
                info = os.stat(output.name, dir_fd=output_directory, follow_symlinks=False)
                output_identity = (info.st_dev, info.st_ino)
                site_handle = os.open(output.name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
                                      dir_fd=output_directory)
                try:
                    for name, content in static_files.items():
                        _write_new(site_handle, name, content, 0o644)
                    _write_new(site_handle, ".nojekyll", b"", 0o644)
                    _write_new(site_handle, "manifest.json", manifest_bytes, 0o644)
                    os.mkdir("packages", mode=0o755, dir_fd=site_handle)
                    packages_handle = os.open("packages", os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
                                              dir_fd=site_handle)
                    try:
                        os.fchmod(packages_handle, 0o755)
                        _write_new(packages_handle, payload_name, ciphertext, 0o644)
                    finally:
                        os.close(packages_handle)
                    os.fchmod(site_handle, 0o755)
                finally:
                    os.close(site_handle)
            except BaseException:
                if output_identity is not None and _same_entry(output_directory, output.name, output_identity):
                    shutil.rmtree(output.name, dir_fd=output_directory)
                if link_identity is not None and _same_entry(link_directory, link_file.name, link_identity):
                    os.unlink(link_file.name, dir_fd=link_directory)
                raise
    except OSError as exc:
        raise BuildError("Cannot access the selected paths safely; use existing, writable parents without symlinks.") from exc
    return {"output": str(output), "link_file": str(link_file), "filename": input_path.name,
            "size": len(plaintext), "cipher_size": len(ciphertext),
            "cipher_sha256": cipher_sha256, "manifest_sha256": manifest_sha256}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="Private .tar.gz archive, at most 100 MiB")
    parser.add_argument("--output", type=Path, required=True, help="New directory containing only public site files")
    parser.add_argument("--link-file", type=Path, required=True, help="New private file outside the public output")
    parser.add_argument("--base-url", required=True, help="HTTPS address of this immutable version directory")
    parser.add_argument("--source-commit", help="Optional lowercase source commit hash")
    args = parser.parse_args(argv)
    try:
        report = build_site(args.input, args.output, args.link_file, args.base_url, args.source_commit)
    except BuildError as exc:
        parser.exit(1, str(exc) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
