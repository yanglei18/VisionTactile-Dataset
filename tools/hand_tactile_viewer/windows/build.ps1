[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Resources,
    [string]$Wheelhouse,
    [string]$Output,
    [string]$Iscc,
    [ValidateSet('left', 'right')][string]$Hand = 'right'
)

$ErrorActionPreference = 'Stop'
$arguments = @('-3.12', (Join-Path $PSScriptRoot 'build.py'), '--resources', $Resources, '--hand', $Hand)
if ($Wheelhouse) { $arguments += @('--wheelhouse', $Wheelhouse) }
if ($Output) { $arguments += @('--output', $Output) }
if ($Iscc) { $arguments += @('--iscc', $Iscc) }

# The builder creates an isolated environment; users install only the final EXE.
& py @arguments
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
