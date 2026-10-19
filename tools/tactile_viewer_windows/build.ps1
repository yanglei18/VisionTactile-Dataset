[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$ModelBundle,
    [Parameter(Mandatory = $true)][string]$ModelLicense,
    [string]$Output,
    [string]$Iscc,
    [string]$Wheelhouse
)

$ErrorActionPreference = 'Stop'
$arguments = @('-3.12', (Join-Path $PSScriptRoot 'build.py'),
    '--model-bundle', $ModelBundle, '--model-license', $ModelLicense)
if ($Output) { $arguments += @('--output', $Output) }
if ($Iscc) { $arguments += @('--iscc', $Iscc) }
if ($Wheelhouse) { $arguments += @('--wheelhouse', $Wheelhouse) }

# The build script creates and manages its own venv. No shell activation needed.
& py @arguments
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
