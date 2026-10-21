"use strict";

// No credentials, analytics, browser storage, or external resources. The fragment
// is a delivery capability and never belongs in a server request or error log.
(() => {
  const messages = {
    zh: {
      title: "下载采集工作台", intro: "主程序、相机环境与配套 Tracker，一次下载，自动安装。",
      package: "完整安装包", "no-account": "无需注册账号或登录 GitHub。", download: "下载安装包",
      save: "再次保存安装包", "step1-title": "下载安装包", "step1": "点击上方按钮，等待浏览器保存文件。",
      "step2-title": "解压并打开", step2: "解压整个文件夹，双击 Install-VisionTactile。",
      "step3-title": "完成自动安装", step3: "点击“安装”，输入一次本机管理员密码。完成后打开工作台。",
      note: "首次安装需要联网。此安装包对应配套设备，请只将完整下载链接转发给本套设备的使用者。",
      footer: "请保留收到的完整链接，便于再次下载。", checking: "正在检查下载链接…",
      missing: "请使用随设备提供的完整下载链接打开本页面。", ready: "已就绪，点击按钮开始下载。",
      unsupported: "请使用支持安全下载的新版 Chrome、Edge 或 Firefox，并通过 HTTPS 打开。",
      invalid: "下载链接无效或不完整，请重新打开收到的完整链接。",
      network: "下载中断或暂时无法连接，请检查网络后重试。",
      integrity: "安装包校验失败，未保存文件。请重新打开原始下载链接后重试。",
      downloading: "正在下载", verifying: "下载完成，正在校验安装包…", busy: "正在准备安装包…",
      done: "已准备好安装包。如浏览器未保存，请点击下方“再次保存安装包”。", retry: "重试下载"
    },
    en: {
      title: "Download the capture workbench", intro: "The application, camera environment and matching Tracker setup in one automatic installer.",
      package: "Complete installer", "no-account": "No account or GitHub login required.", download: "Download installer",
      save: "Save installer again", "step1-title": "Download", step1: "Click the button above and wait for your browser to save the file.",
      "step2-title": "Extract and open", step2: "Extract the whole folder and double-click Install-VisionTactile.",
      "step3-title": "Install automatically", step3: "Click Install and enter your computer’s administrator password once. Then open the workbench.",
      note: "Internet access is needed for the first installation. Share this complete link only with users of the matching equipment.",
      footer: "Keep the original complete link for future downloads.", checking: "Checking the download link…",
      missing: "Open this page using the complete download link supplied with your equipment.", ready: "Ready. Click the button to download.",
      unsupported: "Use a current Chrome, Edge or Firefox browser and open this page over HTTPS.",
      invalid: "This download link is invalid or incomplete. Reopen the original complete link.",
      network: "The download was interrupted or the server is unavailable. Check your connection and retry.",
      integrity: "Installer verification failed. No file was saved. Reopen the original download link and retry.",
      downloading: "Downloading", verifying: "Download complete. Verifying the installer…", busy: "Preparing installer…",
      done: "Your installer is ready. If your browser did not save it, use “Save installer again” below.", retry: "Retry download"
    }
  };
  const button = document.getElementById("download");
  const status = document.getElementById("status");
  const progress = document.getElementById("progress");
  const save = document.getElementById("save");
  let language = navigator.language.startsWith("zh") ? "zh" : "en";
  let state = "checking", percent = null, busy = false, error = false, failed = false;
  let manifest = null, key = null, blobUrl = null, expectedManifestHash = null;
  const MAX_BYTES = 100 * 1024 * 1024;
  const hex = bytes => Array.from(new Uint8Array(bytes), value => value.toString(16).padStart(2, "0")).join("");
  const digest = async bytes => hex(await crypto.subtle.digest("SHA-256", bytes));
  const decode = value => Uint8Array.from(atob(value.replace(/-/g, "+").replace(/_/g, "/")), char => char.charCodeAt(0));

  function render() {
    document.documentElement.lang = language === "zh" ? "zh-CN" : "en";
    document.title = `Vision Tactile · ${language === "zh" ? "下载" : "Download"}`;
    for (const node of document.querySelectorAll("[data-i18n]")) node.textContent = messages[language][node.dataset.i18n];
    document.getElementById("language").textContent = language === "zh" ? "English" : "中文";
    button.textContent = messages[language][busy ? "busy" : failed ? "retry" : "download"];
    status.textContent = messages[language][state] + (percent === null ? "" : ` ${percent}%`);
    status.classList.toggle("error", error);
    button.disabled = busy || !key || (!manifest && !expectedManifestHash);
  }

  function setStatus(value, isError = false, percentage = null) {
    state = value; error = isError; percent = percentage; render();
  }

  async function readBounded(response, maximum, exact = false, onProgress = () => {}) {
    if (!response.ok || !response.body) throw new Error("network");
    const announced = response.headers.get("content-length");
    const encoding = response.headers.get("content-encoding");
    // Content-Length measures encoded transport bytes. CDNs may gzip ciphertext
    // into a slightly larger payload; fetch exposes the decoded byte stream.
    if ((!encoding || encoding.toLowerCase() === "identity") && announced !== null
        && (!/^\d+$/.test(announced) || Number(announced) > maximum)) throw new Error("integrity");
    const buffer = new Uint8Array(maximum);
    const reader = response.body.getReader();
    let offset = 0;
    try {
      while (true) {
        const {done, value} = await reader.read();
        if (done) break;
        if (offset + value.length > maximum) throw new Error("integrity");
        buffer.set(value, offset); offset += value.length; onProgress(offset);
      }
    } catch (err) {
      await reader.cancel().catch(() => {});
      throw err;
    } finally { reader.releaseLock(); }
    if (exact && offset !== maximum) throw new Error("integrity");
    return exact ? buffer : buffer.slice(0, offset);
  }

  function validateManifest(value) {
    if (value?.schema_version !== 1 || value.cipher !== "AES-256-GCM"
        || !Number.isSafeInteger(value.size) || value.size < 1 || value.size > MAX_BYTES
        || value.cipher_size !== value.size + 16 || !/^[A-Za-z0-9_-]{16}$/.test(value.iv)
        || !/^[a-f0-9]{64}$/.test(value.sha256) || !/^[a-f0-9]{64}$/.test(value.cipher_sha256)
        || value.payload !== `packages/${value.cipher_sha256}.bin`
        || !/^[A-Za-z0-9][A-Za-z0-9._-]{0,120}\.tar\.gz$/.test(value.filename)) {
      throw new Error("integrity");
    }
    return value;
  }

  const requestOptions = {credentials: "omit", referrerPolicy: "no-referrer", redirect: "error"};
  async function fetchBytes(path, maximum, exact = false, onProgress = () => {}, cache = "default") {
    const controller = new AbortController();
    let timeout;
    const touch = () => { clearTimeout(timeout); timeout = setTimeout(() => controller.abort(), 60000); };
    touch();
    try {
      const response = await fetch(path, {...requestOptions, cache, signal: controller.signal});
      return await readBounded(response, maximum, exact, count => { touch(); onProgress(count); });
    } finally { clearTimeout(timeout); }
  }
  async function loadManifest() {
    const bytes = await fetchBytes("manifest.json", 16384, false, () => {}, "no-store");
    if (await digest(bytes) !== expectedManifestHash) throw new Error("integrity");
    try { manifest = validateManifest(JSON.parse(new TextDecoder("utf-8", {fatal: true}).decode(bytes))); }
    catch { throw new Error("integrity"); }
    document.getElementById("size").textContent = `${(manifest.size / 1024 / 1024).toFixed(1)} MiB`;
  }
  let fragment = location.hash.slice(1);
  // Remove the capability before any fetch, without persisting it anywhere.
  history.replaceState(null, "", location.pathname + location.search);
  async function initialize() {
    if (!fragment) { setStatus("missing"); return; }
    if (!window.isSecureContext || !window.crypto?.subtle) { fragment = ""; setStatus("unsupported", true); return; }
    let fields = new URLSearchParams(fragment);
    fragment = "";
    let encodedKey = fields.get("key");
    const manifestHash = fields.get("manifest");
    if (fields.size !== 2 || fields.getAll("key").length !== 1 || fields.getAll("manifest").length !== 1
        || !/^[A-Za-z0-9_-]{43}$/.test(encodedKey || "") || !/^[a-f0-9]{64}$/.test(manifestHash || "")) {
      encodedKey = null; fields = null; setStatus("invalid", true); return;
    }
    fields = null;
    expectedManifestHash = manifestHash;
    const keyBytes = decode(encodedKey); encodedKey = null;
    try {
      key = await crypto.subtle.importKey("raw", keyBytes, "AES-GCM", false, ["decrypt"]);
    } finally { keyBytes.fill(0); }
    busy = true; render();
    try {
      await loadManifest();
      setStatus("ready");
    } catch (err) {
      // Retry transient network errors in memory; reject altered manifests.
      manifest = null; failed = true;
      if (err.message === "integrity") { key = null; expectedManifestHash = null; }
      setStatus(err.message === "integrity" ? "integrity" : "network", true);
    } finally { busy = false; render(); }
  }

  button.addEventListener("click", async () => {
    if (!key || !expectedManifestHash || busy) return;
    busy = true; failed = false; progress.hidden = false; progress.value = 0;
    save.hidden = true;
    if (blobUrl) { URL.revokeObjectURL(blobUrl); blobUrl = null; }
    setStatus("downloading", false, 0);
    try {
      if (!manifest) await loadManifest();
      const encrypted = await fetchBytes(manifest.payload, manifest.cipher_size, true, count => {
        const value = Math.floor(count / manifest.cipher_size * 100);
        progress.value = value; setStatus("downloading", false, value);
      });
      setStatus("verifying");
      if (await digest(encrypted) !== manifest.cipher_sha256) throw new Error("integrity");
      let plain;
      try {
        plain = await crypto.subtle.decrypt({name: "AES-GCM", iv: decode(manifest.iv), tagLength: 128,
          additionalData: new TextEncoder().encode("VisionTactile download v1")}, key, encrypted);
      } catch { throw new Error("integrity"); }
      if (plain.byteLength !== manifest.size || await digest(plain) !== manifest.sha256) throw new Error("integrity");
      blobUrl = URL.createObjectURL(new Blob([plain], {type: "application/gzip"}));
      save.href = blobUrl; save.download = manifest.filename; save.hidden = false;
      save.click(); setStatus("done");
    } catch (err) {
      failed = true; setStatus(err.message === "integrity" ? "integrity" : "network", true);
    } finally { busy = false; progress.hidden = true; render(); }
  });
  document.getElementById("language").addEventListener("click", () => {
    language = language === "zh" ? "en" : "zh"; render();
  });
  window.addEventListener("pagehide", () => { if (blobUrl) URL.revokeObjectURL(blobUrl); });
  render();
  initialize().catch(() => { key = null; manifest = null; setStatus("invalid", true); });
})();
