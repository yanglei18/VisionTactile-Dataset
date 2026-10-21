# 免登录下载托管维护

下载站为 <https://yanglei18.github.io/VisionTactile-Downloads/>，托管仓库为
[VisionTactile-Downloads](https://github.com/yanglei18/VisionTactile-Downloads)。
普通用户收到一条完整链接，打开网页后点击下载即可。安装器本身的本机管理员授权仍由操作系统要求。

## 发布边界

配套 Tracker 安装包含设备身份和网络配置，不能把原始 `.tar.gz` 或 `.vttracker`
上传公开仓库。构建工具使用 AES-256-GCM 对整个归档加密，公开目录仅包含网页、清单和密文。
每次发布生成独立随机的 32 字节密钥及 12 字节 IV；密钥只写入公开目录之外的 `0600` 链接文件。

完整链接的 `#` 片段包含密钥和清单摘要，浏览器读取后立即从地址栏移除，不写入浏览器存储，
也不发给托管服务器。页面没有统计、第三方脚本或外部字体。浏览器依次验证清单、密文、
AES-GCM 认证及明文摘要，全部通过后才保存安装包。用户不需要手工解密。

完整链接等同于设备交付凭证。只交给对应设备的使用者，不放进公开文档、Issue、CI 日志、
页面配置或 Git 历史。拿到链接的人可以转发并保存安装包；删除网页不能收回已下载的内容，
公开仓库中的旧密文也可能被保留。网站源码和 GitHub 账号仍是信任边界，不宣称加密可以防止
网站被篡改后窃取未来访问者的链接。

## 构建

维护机使用 Linux、Python 3.10 以上及 `cryptography`，这不是终端用户依赖：

```bash
python3 -m venv /tmp/vt-download-build
/tmp/vt-download-build/bin/python -m pip install 'cryptography>=41'
/tmp/vt-download-build/bin/python tools/release/build_download_site.py \
  --input /private/VisionTactile-Installer-VERSION.tar.gz \
  --output /public-site/ubuntu/VERSION \
  --link-file /private/VisionTactile-VERSION.download-link.txt \
  --base-url https://yanglei18.github.io/VisionTactile-Downloads/ubuntu/VERSION/ \
  --source-commit COMMIT_HEX
```

父目录须已存在，输出目录及链接文件须不存在。工具拒绝符号链接、路径遍历和覆盖，
输入上限 100 MiB。只复制 `tools/release/download_site/` 内的三个指定模板；归档不会被解压。
命令输出不含密钥。版本目录必须保持不变；新安装包使用新目录和新链接，以免破坏旧链接。

## 托管与验收

将生成的公开目录提交到下载仓库相应版本目录，保留仓库根目录 `.nojekyll`。
GitHub Pages 的发布来源为 `main` 分支根目录。不要提交维护机整个交付目录，
也不要在发布脚本中打印或上传私有链接文件。公开首页只展示使用说明，完整链接单独交付。

发布后使用未登录浏览器验证实际 HTTPS 地址，至少检查：

- 完整链接可下载，所得文件大小及 SHA-256 与本机原始包一致。
- 无链接凭证、错误密钥、清单或密文被篡改、密文截断均不能得到安装包。
- 网络失败可重试；连续 60 秒无网络数据时结束等待并显示错误。
- 请求 URL、请求头、控制台、DOM 和浏览器存储不包含链接密钥。
- 中文和英文页面可用，Chrome 和 Firefox 均能保存完整文件。

自动化浏览器测试不要启用包含完整 URL 的 trace、录屏或调试日志。
构建器的合成输入测试可运行：

```bash
python3 -m unittest discover -s tools/release/tests -p test_download_site.py -v
```

浏览器验收工具需要维护环境中的 `playwright`，不属于终端用户依赖。安装浏览器后可验证
本地站点；增加 `--online` 则访问链接中的真实 HTTPS 地址：

```bash
python3 tools/release/verify_download_site.py \
  --site-dir /public-site/ubuntu/VERSION \
  --link-file /private/VisionTactile-VERSION.download-link.txt \
  --archive /private/VisionTactile-Installer-VERSION.tar.gz \
  --browser firefox --report /private/browser-check.json
```

支持 Chrome/Chromium 和 Firefox。维护机使用已有 Chrome 时增加 `--chrome-executable`。
验证工具不打印完整链接，报告只记录结果与脱敏诊断。回归覆盖 CDN 对随机密文进行 gzip
传输后长度变大的情况：长度限制始终应用于浏览器解码后的内容，而非压缩传输的 Content-Length。

当前安装包约 36.7 MiB，采用浏览器内存解密，目标为桌面电脑。
GitHub Pages 的站点上限为 1 GB、月带宽软限制为 100 GB；批量客户部署或大包分发需要
迁移至对象存储/CDN。此站只用于项目软件下载，不含支付交易或 SaaS 服务，也不承诺网络 SLA。
容量与使用限制以 [GitHub 官方说明](https://docs.github.com/en/pages/getting-started-with-github-pages/github-pages-limits)为准。
