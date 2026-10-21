"""Encrypted delivery contracts; all inputs and secrets are synthetic fixtures."""
from __future__ import annotations

import base64
from contextlib import redirect_stdout
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from urllib.parse import parse_qs, urlsplit

try:
    from cryptography.exceptions import InvalidTag
    from cryptography.hazmat.primitives.ciphers.aead import AESGCM
except ImportError:
    AESGCM = None


SCRIPT = Path(__file__).resolve().parents[1] / "build_download_site.py"
SPEC = importlib.util.spec_from_file_location("download_site_under_test", SCRIPT)
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


def decode(value):
    return base64.urlsafe_b64decode(value + "=" * (-len(value) % 4))


@unittest.skipUnless(AESGCM is not None, "cryptography is required for encrypted download builds")
class DownloadSiteTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="vt-download-test-")
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        self.archive = self.base / "VisionTactile-Installer-fixture.tar.gz"
        self.payload = b"synthetic private archive; never upload this fixture as plaintext\n" * 20
        self.archive.write_bytes(self.payload)
        self.output = self.base / "public-site"
        self.link = self.base / "private-link.txt"
        self.templates = self.base / "templates"
        self.templates.mkdir()
        for name in builder.TEMPLATE_FILES:
            (self.templates / name).write_bytes(b"Synthetic public template: " + name.encode())
        self.url = "https://example.invalid/downloads/fixture/"

    def build(self, **overrides):
        values = {"input_path": self.archive, "output": self.output,
                  "link_file": self.link, "base_url": self.url,
                  "source_commit": "1234567", "template_directory": self.templates}
        values.update(overrides)
        return builder.build_site(**values)

    def read_build(self):
        manifest_bytes = (self.output / "manifest.json").read_bytes()
        manifest = json.loads(manifest_bytes)
        fragment = parse_qs(urlsplit(self.link.read_text()).fragment)
        key = decode(fragment["key"][0])
        cipher = (self.output / manifest["payload"]).read_bytes()
        return manifest_bytes, manifest, fragment, key, cipher

    def test_real_round_trip_matches_manifest_and_pinned_link(self):
        report = self.build()
        manifest_bytes, manifest, fragment, key, cipher = self.read_build()
        self.assertEqual(len(key), 32)
        nonce = decode(manifest["iv"])
        self.assertEqual(len(nonce), 12)
        self.assertEqual(AESGCM(key).decrypt(nonce, cipher, builder.ADDITIONAL_DATA), self.payload)
        digest = lambda content: hashlib.sha256(content).hexdigest()
        self.assertEqual(fragment["manifest"], [digest(manifest_bytes)])
        self.assertEqual(manifest["sha256"], digest(self.payload))
        self.assertEqual(manifest["cipher_sha256"], digest(cipher))
        self.assertEqual(manifest["payload"], "packages/" + digest(cipher) + ".bin")
        self.assertEqual(manifest["size"], len(self.payload))
        self.assertEqual(manifest["cipher_size"], len(self.payload) + 16)
        self.assertEqual(manifest["cipher"], "AES-256-GCM")
        self.assertEqual(manifest["schema_version"], 1)
        self.assertEqual(manifest["source_commit"], "1234567")
        self.assertEqual(report["manifest_sha256"], digest(manifest_bytes))
        self.assertEqual(urlsplit(self.link.read_text()).query, "")
        self.assertEqual(urlsplit(self.link.read_text()).path, "/downloads/fixture/")

    def test_wrong_key_changed_ciphertext_and_wrong_context_fail_authentication(self):
        self.build()
        _, manifest, _, key, cipher = self.read_build()
        nonce = decode(manifest["iv"])
        bad_key = bytes([key[0] ^ 1]) + key[1:]
        altered = cipher[:-1] + bytes([cipher[-1] ^ 1])
        for candidate_key, candidate_cipher, context in (
                (bad_key, cipher, builder.ADDITIONAL_DATA),
                (key, altered, builder.ADDITIONAL_DATA),
                (key, cipher, b"different protocol")):
            with self.subTest(context=context, tampered=candidate_cipher != cipher):
                with self.assertRaises(InvalidTag):
                    AESGCM(candidate_key).decrypt(nonce, candidate_cipher, context)

    def test_public_output_contains_only_allowlisted_files_and_no_secret(self):
        (self.templates / "unrelated-private.txt").write_text("Do not copy unknown templates")
        report = self.build()
        _, manifest, fragment, key, _ = self.read_build()
        expected = {*builder.TEMPLATE_FILES, ".nojekyll", "manifest.json", manifest["payload"]}
        actual = {path.relative_to(self.output).as_posix()
                  for path in self.output.rglob("*") if path.is_file()}
        self.assertEqual(actual, expected)
        public_bytes = b"\n".join(path.read_bytes() for path in self.output.rglob("*") if path.is_file())
        for private in (self.payload, key, fragment["key"][0].encode(), self.link.read_bytes().strip()):
            self.assertNotIn(private, public_bytes)
            self.assertNotIn(private, json.dumps(report).encode())
        self.assertNotIn(str(self.archive), json.dumps(manifest))

    def test_each_build_has_a_fresh_key_nonce_and_ciphertext(self):
        self.build()
        _, first, _, first_key, first_cipher = self.read_build()
        self.build(output=self.base / "second-site", link_file=self.base / "second-link")
        second = json.loads((self.base / "second-site/manifest.json").read_bytes())
        second_fragment = parse_qs(urlsplit((self.base / "second-link").read_text()).fragment)
        self.assertNotEqual(first_key, decode(second_fragment["key"][0]))
        self.assertNotEqual(first["iv"], second["iv"])
        self.assertNotEqual(first_cipher, (self.base / "second-site" / second["payload"]).read_bytes())

    def test_private_link_permissions_ignore_permissive_umask(self):
        previous = os.umask(0)
        try:
            self.build()
        finally:
            os.umask(previous)
        self.assertEqual(self.link.stat().st_mode & 0o777, 0o600)
        self.assertEqual(self.output.stat().st_mode & 0o777, 0o755)
        self.assertEqual((self.output / "packages").stat().st_mode & 0o777, 0o755)
        for path in self.output.rglob("*"):
            if path.is_file():
                self.assertEqual(path.stat().st_mode & 0o777, 0o644)

    def test_cli_does_not_print_key_or_private_link(self):
        output = io.StringIO()
        with patch.object(builder, "TEMPLATE_DIRECTORY", self.templates), redirect_stdout(output):
            result = builder.main(["--input", str(self.archive), "--output", str(self.output),
                                   "--link-file", str(self.link), "--base-url", self.url])
        self.assertEqual(result, 0)
        _, _, fragment, _, _ = self.read_build()
        self.assertNotIn(fragment["key"][0], output.getvalue())
        self.assertNotIn(self.link.read_text().strip(), output.getvalue())
        self.assertEqual(json.loads(output.getvalue())["output"], str(self.output))

    def test_link_must_be_outside_public_output(self):
        for link in (self.output, self.output / "link.txt", self.output / "nested/link.txt"):
            with self.subTest(link=link), self.assertRaisesRegex(builder.BuildError, "outside"):
                self.build(link_file=link)
        self.assertFalse(self.output.exists())

    def test_existing_output_or_private_link_are_never_overwritten(self):
        self.output.mkdir()
        marker = self.output / "keep-me"
        marker.write_bytes(b"existing public content")
        with self.assertRaisesRegex(builder.BuildError, "already exists"):
            self.build()
        self.assertEqual(marker.read_bytes(), b"existing public content")
        self.assertFalse(self.link.exists())
        marker.unlink()
        self.output.rmdir()
        self.link.write_bytes(b"existing secret")
        with self.assertRaisesRegex(builder.BuildError, "already exists"):
            self.build()
        self.assertEqual(self.link.read_bytes(), b"existing secret")
        self.assertFalse(self.output.exists())

    def test_symlink_inputs_outputs_and_ancestors_are_rejected(self):
        alias = self.base / "alias.tar.gz"
        alias.symlink_to(self.archive)
        with self.assertRaises(builder.BuildError):
            self.build(input_path=alias)
        alias.unlink()
        self.output.symlink_to(self.base / "missing-directory")
        with self.assertRaises(builder.BuildError):
            self.build()
        self.output.unlink()
        self.link.symlink_to(self.base / "missing-file")
        with self.assertRaises(builder.BuildError):
            self.build()
        self.link.unlink()
        parent_alias = self.base / "parent-alias"
        parent_alias.symlink_to(self.base, target_is_directory=True)
        for key, value in (("output", parent_alias / "new-site"),
                           ("link_file", parent_alias / "new-link"),
                           ("input_path", parent_alias / self.archive.name)):
            with self.subTest(key=key), self.assertRaises(builder.BuildError):
                self.build(**{key: value})
        self.assertFalse(self.output.exists())
        self.assertFalse(self.link.exists())

    def test_path_traversal_is_rejected(self):
        for key, value in (("output", self.base / ".." / "public-site"),
                           ("link_file", self.base / ".." / "private-link"),
                           ("input_path", self.base / ".." / self.archive.name)):
            with self.subTest(key=key), self.assertRaisesRegex(builder.BuildError, "traversal"):
                self.build(**{key: value})

    def test_input_requires_a_bounded_nonempty_regular_file(self):
        with patch.object(builder, "MAX_ARCHIVE_BYTES", len(self.payload) - 1):
            with self.assertRaisesRegex(builder.BuildError, "size limit"):
                self.build()
        self.archive.write_bytes(b"")
        with self.assertRaisesRegex(builder.BuildError, "nonempty regular"):
            self.build()
        self.archive.unlink()
        self.archive.mkdir()
        with self.assertRaises(builder.BuildError):
            self.build()
        self.archive.rmdir()
        os.mkfifo(self.archive)
        with self.assertRaisesRegex(builder.BuildError, "regular"):
            self.build()
        self.assertFalse(self.output.exists())
        self.assertFalse(self.link.exists())

    def test_unsafe_download_filename_is_rejected(self):
        for name in ("archive.zip", "-archive.tar.gz", "bad\nname.tar.gz", "space name.tar.gz",
                     "a" * 122 + ".tar.gz"):
            with self.subTest(name=name), self.assertRaisesRegex(builder.BuildError, "filename"):
                self.build(input_path=self.base / name)

    def test_maximum_download_filename_matches_browser_contract(self):
        longest = self.base / ("a" * 121 + ".tar.gz")
        longest.write_bytes(self.payload)
        self.build(input_path=longest)
        self.assertEqual(json.loads((self.output / "manifest.json").read_bytes())["filename"], longest.name)

    def test_base_url_requires_https_without_credentials_or_extra_parameters(self):
        for value in ("http://example.invalid/", "https://user:password@example.invalid/",
                      "https://example.invalid/?token=secret", "https://example.invalid/#fragment",
                      "https://example.invalid/../other/", "https://example.invalid/%2e%2e/",
                      "https://example.invalid\\other/", "https://example.invalid/\n",
                      "https:///missing-host/", "https://example.invalid:wrong/",
                      "https://example.invalid:8443/"):
            with self.subTest(url=value), self.assertRaisesRegex(builder.BuildError, "Base URL"):
                self.build(base_url=value)
        self.assertFalse(self.output.exists())
        self.assertFalse(self.link.exists())

    def test_source_commit_is_optional_and_validated(self):
        for commit in ("not-a-commit", "123456", "a" * 41, "ABCDEF1"):
            with self.subTest(commit=commit), self.assertRaisesRegex(builder.BuildError, "Source commit"):
                self.build(source_commit=commit)
        self.build(source_commit=None)
        self.assertNotIn("source_commit", json.loads((self.output / "manifest.json").read_bytes()))

    def test_failed_publication_removes_only_owned_output_and_link(self):
        unrelated = self.base / "keep-me"
        unrelated.write_bytes(b"unrelated data")
        with patch.object(builder, "_write_new", side_effect=OSError("simulated disk failure")):
            with self.assertRaises(builder.BuildError):
                self.build()
        self.assertFalse(self.output.exists())
        self.assertFalse(self.link.exists())
        self.assertEqual(unrelated.read_bytes(), b"unrelated data")
        self.assertEqual(self.archive.read_bytes(), self.payload)

    def test_template_symlinks_are_rejected_before_publication(self):
        template = self.templates / "app.js"
        template.unlink()
        template.symlink_to(self.archive)
        with self.assertRaises(builder.BuildError):
            self.build()
        self.assertFalse(self.output.exists())
        self.assertFalse(self.link.exists())


if __name__ == "__main__":
    unittest.main()
