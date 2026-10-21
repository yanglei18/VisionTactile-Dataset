#!/usr/bin/env python3
"""Verify an encrypted download site in an anonymous Chrome or Firefox context.

All input locations are explicit. A site built from synthetic or real input can
be tested locally, or with --online at the address in its private link file.
Reports never include private links or keys, and browser tracing is disabled.
Install the optional Playwright build dependency and its browser before use.
"""
from __future__ import annotations

import argparse
import base64
from contextlib import contextmanager
import functools
import gzip
import hashlib
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import threading
from urllib.parse import parse_qs, urlencode, urlsplit, urlunsplit

from playwright.sync_api import expect, sync_playwright


CASES = ("download", "missing_key", "wrong_key", "tampered_cipher",
         "truncated_cipher", "tampered_manifest", "manifest_retry",
         "payload_retry", "gzip_cipher", "language")
DOWNLOAD_CASES = {"download", "manifest_retry", "payload_retry", "gzip_cipher"}


def sanitize(value, *secrets):
    """Remove capability fields and URLs before persisting any browser diagnostic."""
    result = str(value)
    for secret in secrets:
        if secret:
            result = result.replace(secret, "[REDACTED]")
    return re.sub(r"https?://[^\s\"'<>]+", "[URL]", result)[:2000]


def selected_cases(value):
    cases = value.split(",")
    if not cases or any(case not in CASES for case in cases):
        raise argparse.ArgumentTypeError("Cases must be comma-separated names from: " + ", ".join(CASES))
    return cases


def launch_browser(playwright, args):
    if args.browser == "firefox":
        return playwright.firefox.launch(headless=True)
    options = {"headless": True, "args": ["--disable-background-networking"]}
    if args.chrome_executable is not None:
        options["executable_path"] = str(args.chrome_executable)
    return playwright.chromium.launch(**options)


class SilentHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, gzip_payloads, **kwargs):
        self._gzip_payloads = gzip_payloads
        super().__init__(*args, **kwargs)

    def log_message(self, *args):
        pass

    def do_GET(self):
        # A real HTTP response is essential: Playwright's route.fulfill body is
        # already decoded and cannot reproduce Content-Encoding wire semantics.
        if self._gzip_payloads.is_set() and urlsplit(self.path).path.endswith(".bin"):
            target = Path(self.translate_path(self.path))
            if target.is_file():
                body = gzip.compress(target.read_bytes(), compresslevel=1, mtime=0)
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Encoding", "gzip")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
        super().do_GET()

    def copyfile(self, source, outputfile):
        try:
            super().copyfile(source, outputfile)
        except (BrokenPipeError, ConnectionResetError):
            pass


@contextmanager
def serving(directory):
    gzip_payloads = threading.Event()
    handler = functools.partial(SilentHandler, directory=str(directory), gzip_payloads=gzip_payloads)
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}/", gzip_payloads
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def file_digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def run(args, base_url, fragment, local_base, gzip_payloads):
    manifest = json.loads((args.site_dir / "manifest.json").read_text())
    original_digest = file_digest(args.archive)
    assert original_digest == manifest["sha256"]
    assert args.archive.stat().st_size == manifest["size"]
    fields = parse_qs(fragment, strict_parsing=True)
    secret = fields["key"][0]
    wrong = base64.urlsafe_b64encode(os.urandom(32)).rstrip(b"=").decode("ascii")
    cases = args.cases or CASES
    report = {"browser": args.browser, "mode": "online" if args.online else "localhost", "archive_bytes": manifest["size"], "archive_sha256": original_digest, "cases": []}
    with sync_playwright() as playwright:
        browser = launch_browser(playwright, args)
        try:
            for name in cases:
                current_base = local_base if name == "gzip_cipher" else base_url
                parsed_base = urlsplit(current_base)
                origin = (parsed_base.scheme, parsed_base.netloc)
                if name == "gzip_cipher":
                    gzip_payloads.set()
                else:
                    gzip_payloads.clear()
                context = browser.new_context(accept_downloads=True, locale="zh-CN", service_workers="block")
                page = context.new_page()
                page.set_default_timeout(30000)
                observed = {"requests": 0, "leaks": 0, "foreign": 0, "downloads": [], "page_errors": 0, "manifest_calls": 0, "payload_calls": 0}

                def route_request(route):
                    request = route.request
                    parsed = urlsplit(request.url)
                    observed["requests"] += 1
                    network_fields = request.url + json.dumps(request.headers, sort_keys=True) + (request.post_data or "")
                    if secret in network_fields or wrong in network_fields or "#key=" in network_fields:
                        observed["leaks"] += 1
                        route.abort()
                        return
                    if (parsed.scheme, parsed.netloc) != origin:
                        observed["foreign"] += 1
                        route.abort()
                        return
                    is_manifest = parsed.path.endswith("/manifest.json")
                    is_payload = parsed.path.endswith(".bin")
                    if is_manifest:
                        observed["manifest_calls"] += 1
                    if is_payload:
                        observed["payload_calls"] += 1
                    if name == "manifest_retry" and is_manifest and observed["manifest_calls"] == 1:
                        route.fulfill(status=503, content_type="text/plain", body="Unavailable")
                    elif name == "payload_retry" and is_payload and observed["payload_calls"] == 1:
                        route.fulfill(status=503, content_type="text/plain", body="Unavailable")
                    elif name == "tampered_manifest" and is_manifest:
                        body = (args.site_dir / "manifest.json").read_bytes() + b" "
                        route.fulfill(status=200, content_type="application/json", body=body)
                    elif name in ("tampered_cipher", "truncated_cipher") and is_payload:
                        body = bytearray((args.site_dir / manifest["payload"]).read_bytes())
                        if name == "tampered_cipher":
                            body[len(body) // 2] ^= 1
                        else:
                            del body[-1:]
                        route.fulfill(status=200, content_type="application/octet-stream", body=bytes(body))
                    else:
                        route.continue_()

                def observe_response(response):
                    if name == "gzip_cipher" and urlsplit(response.url).path.endswith(".bin"):
                        observed["content_encoding"] = response.headers.get("content-encoding")
                        observed["transfer_bytes"] = int(response.headers.get("content-length", 0))

                page.route("**/*", route_request)
                page.on("response", observe_response)
                page.on("download", lambda download: observed["downloads"].append(download))
                page.on("pageerror", lambda error: observed.__setitem__("page_errors", observed["page_errors"] + 1))
                current_fragment = fragment
                if name == "missing_key":
                    current_fragment = ""
                elif name == "wrong_key":
                    current_fragment = urlencode({"key": wrong, "manifest": fields["manifest"][0]})
                try:
                    response = page.goto(current_base + ("#" + current_fragment if current_fragment else ""), wait_until="networkidle", timeout=120000)
                    assert response is not None and response.status == 200
                    assert page.evaluate("location.hash") == ""
                    assert secret not in page.content() and wrong not in page.content()
                    assert page.evaluate("localStorage.length + sessionStorage.length") == 0
                    if name == "missing_key":
                        assert page.locator("#download").is_disabled()
                        assert "完整下载链接" in page.locator("#status").inner_text()
                        assert observed["manifest_calls"] == 0
                    elif name == "tampered_manifest":
                        assert page.locator("#download").is_disabled()
                        assert "校验失败" in page.locator("#status").inner_text()
                    elif name == "language":
                        assert "已就绪" in page.locator("#status").inner_text()
                        page.locator("#language").click()
                        assert page.locator("html").get_attribute("lang") == "en"
                        assert page.locator("#download").inner_text() == "Download installer"
                        page.locator("#language").click()
                        assert page.locator("html").get_attribute("lang") == "zh-CN"
                    else:
                        if name == "manifest_retry":
                            assert "下载中断" in page.locator("#status").inner_text()
                            assert page.locator("#download").is_enabled()
                        if name in DOWNLOAD_CASES:
                            if name == "payload_retry":
                                page.locator("#download").click()
                                expect(page.locator("#status")).to_have_class(re.compile(r"\berror\b"), timeout=120000)
                                expect(page.locator("#download")).to_be_enabled()
                                assert "下载中断" in page.locator("#status").inner_text()
                            with page.expect_download(timeout=120000) as event:
                                page.locator("#download").click()
                            download = event.value
                            downloaded = Path(download.path())
                            assert download.suggested_filename == manifest["filename"]
                            assert file_digest(downloaded) == original_digest
                            assert downloaded.stat().st_size == manifest["size"]
                            expect(page.locator("#download")).to_be_enabled()
                            assert "已准备好" in page.locator("#status").inner_text()
                            assert page.locator("#save").is_visible()
                            assert len(observed["downloads"]) == 1
                            if name == "download":
                                page.screenshot(path=str(args.report.with_suffix(".png")), full_page=True)
                            if name == "manifest_retry":
                                assert observed["manifest_calls"] == 2
                            if name == "payload_retry":
                                assert observed["payload_calls"] == 2
                            if name == "gzip_cipher":
                                assert observed["content_encoding"] == "gzip"
                                assert observed["transfer_bytes"] > manifest["cipher_size"]
                        else:
                            page.locator("#download").click()
                            expect(page.locator("#status")).to_have_class(re.compile(r"\berror\b"), timeout=120000)
                            expect(page.locator("#download")).to_be_enabled()
                            assert "校验失败" in page.locator("#status").inner_text()
                            assert page.locator("#save").is_hidden()
                    if name not in DOWNLOAD_CASES:
                        assert not observed["downloads"]
                    assert observed["leaks"] == 0 and observed["foreign"] == 0 and observed["page_errors"] == 0
                    report["cases"].append({"case": name, "passed": True,
                        "transport": "online" if args.online and name != "gzip_cipher" else "localhost",
                        "requests": observed["requests"], "downloads": len(observed["downloads"]),
                        "capability_in_requests": False, "external_requests": 0, "page_errors": 0})
                except Exception as exc:
                    # Exception text may contain a navigated private URL; never print it.
                    try:
                        status_text = page.locator("#status").inner_text(timeout=1000)
                    except Exception:
                        status_text = "Unavailable"
                    report["cases"].append({
                        "case": name, "passed": False, "exception_type": type(exc).__name__,
                        "diagnostic": sanitize(exc, secret, wrong, fragment, current_fragment),
                        "status_text": sanitize(status_text, secret, wrong, fragment, current_fragment),
                        "requests": observed["requests"],
                        "capability_in_requests": bool(observed["leaks"]),
                        "external_requests": observed["foreign"],
                        "page_errors": observed["page_errors"],
                    })
                finally:
                    context.close()
        finally:
            browser.close()
    report["passed"] = all(case["passed"] for case in report["cases"])
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site-dir", type=Path, required=True, help="Built public download site directory")
    parser.add_argument("--link-file", type=Path, required=True, help="Private complete link file; never printed")
    parser.add_argument("--archive", type=Path, required=True, help="Original synthetic or real input archive")
    parser.add_argument("--report", type=Path, required=True, help="JSON result, written with private permissions")
    parser.add_argument("--online", action="store_true", help="Test the real HTTPS host instead of a localhost server")
    parser.add_argument("--browser", choices=("chrome", "firefox"), default="chrome")
    parser.add_argument("--chrome-executable", type=Path, help="Optional Chrome executable; otherwise use Playwright Chromium")
    parser.add_argument("--cases", type=selected_cases, help="Comma-separated subset; default runs all cases")
    args = parser.parse_args()
    private_url = args.link_file.read_text().strip()
    parsed = urlsplit(private_url)
    with serving(args.site_dir) as (local_base, gzip_payloads):
        if args.online:
            assert parsed.scheme == "https"
            base_url = urlunsplit((parsed.scheme, parsed.netloc, parsed.path, parsed.query, ""))
        else:
            base_url = local_base
        report = run(args, base_url, parsed.fragment, local_base, gzip_payloads)
    descriptor = os.open(args.report, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(descriptor, "w") as stream:
        os.fchmod(stream.fileno(), 0o600)
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps({"passed": report["passed"], "cases": [{"case": item["case"], "passed": item["passed"]} for item in report["cases"]], "report": str(args.report)}))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        # Setup/launch errors can contain a navigated URL too. Keep unexpected
        # failure output as conservative as individual browser-case reports.
        raise SystemExit(f"Verification setup failed ({type(exc).__name__}); no private link was logged.") from None
