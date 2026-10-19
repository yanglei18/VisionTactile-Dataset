# Security policy

## Reporting a vulnerability

For suspected vulnerabilities, use GitHub's private **Report a vulnerability**
flow when it is enabled on the repository hosting your copy. Open its
**Security** tab and look for **Report a vulnerability**. Provide a minimal
description, impact, reproduction steps, affected version/commit, and suggested
mitigation if known.

This policy file does not enable private reporting, and its availability must
be verified on the destination repository. If that entry point is absent,
request a private reporting channel through a public issue containing only
that request, without vulnerability details or attachments. Wait for a private
channel before sharing sensitive information. No separate security email or
response-time commitment is currently documented.

Never disclose exploit details, credentials, private camera scenes, private
images, recordings, bags, or MCAP files in a public issue. Sanitize text logs
and metadata before including them in the private advisory.

Use normal GitHub Issues only for non-sensitive bugs that contain no exploit
details, credentials, or private scene data. If unsure whether a report is
sensitive, use the private vulnerability-reporting flow.

## Development-preview support

Current development work is described under `Unreleased` in the
[changelog](CHANGELOG.md#unreleased). This repository does not currently document
a security-maintenance window or supported-version matrix for this preview. Include the exact commit
and component/dependency versions in a private report so that the affected
source can be identified.
