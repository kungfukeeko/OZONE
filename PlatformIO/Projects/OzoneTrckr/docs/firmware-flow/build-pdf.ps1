# Rebuild OzoneTrckr-firmware-flow.pdf from the HTML sources in this folder.
#   Run from PowerShell:  .\build-pdf.ps1
# Steps: (1) re-render the pinout overlay to PNG, (2) embed it in the doc,
#        (3) print the doc to PDF (A4 landscape). Edit the .html files first.

$ErrorActionPreference = "Stop"
$here   = $PSScriptRoot
$doc    = Join-Path $here "ozonetrckr-flow.html"
$fig    = Join-Path $here "pinout-fig.html"
$annot  = Join-Path $here "pinout_annot.png"
$out    = Join-Path $here "..\..\OzoneTrckr-firmware-flow.pdf"   # lands in the OzoneTrckr project root
$chrome = "C:\Program Files\Google\Chrome\Application\chrome.exe"
if (-not (Test-Path $chrome)) { $chrome = "C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe" }
$prof   = Join-Path $env:TEMP "chrome-pdf-prof"
$utf8   = New-Object System.Text.UTF8Encoding($false)

# pinout-fig.html references the raw PNG by an absolute file:// path. Make sure it points here:
$figHtml = [System.IO.File]::ReadAllText($fig, $utf8)
$rawUrl  = "file:///" + ((Join-Path $here "feather_s2_pinout.png") -replace '\\','/')
$figHtml = [regex]::Replace($figHtml, 'file:///[^"]*feather_s2_pinout\.png', $rawUrl)
[System.IO.File]::WriteAllText($fig, $figHtml, $utf8)

# (1) render the annotated pinout to PNG at native size
& $chrome --headless=old --disable-gpu --no-sandbox --user-data-dir="$prof" `
  --hide-scrollbars --force-device-scale-factor=1 --window-size=1828,1089 `
  --screenshot="$annot" ("file:///" + ($fig -replace '\\','/')) | Out-Null

# (2) embed that PNG into the doc as a base64 data URI (replaces the first data:image/png)
$html = [System.IO.File]::ReadAllText($doc, $utf8)
$b64  = [System.Convert]::ToBase64String([System.IO.File]::ReadAllBytes($annot))
$html = [regex]::Replace($html, 'data:image/png;base64,[A-Za-z0-9+/=]+', "data:image/png;base64,$b64", 1)
[System.IO.File]::WriteAllText($doc, $html, $utf8)

# (3) wrap + print to PDF
$build = Join-Path $env:TEMP "ozone-flow-print.html"
[System.IO.File]::WriteAllText($build, "<!doctype html>`n<meta charset=`"utf-8`">`n$html", $utf8)
& $chrome --headless=old --disable-gpu --no-sandbox --user-data-dir="$prof" `
  --no-pdf-header-footer --print-to-pdf="$out" ("file:///" + ($build -replace '\\','/')) | Out-Null

if (Test-Path $out) { Write-Host "Built: $((Resolve-Path $out).Path)" -ForegroundColor Green }
else { Write-Host "PDF was not created." -ForegroundColor Red }
