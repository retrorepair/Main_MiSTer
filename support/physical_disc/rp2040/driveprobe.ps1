# driveprobe.ps1 -- real sled speed under the textured drive, against duty, both directions.
#
# Uses the firmware's DRIVE diagnostic (fixed duty, fixed time) and the smooth HOME run as a ruler:
# HOME 0.5 takes RulerFullMs for the whole physical stroke, so its duration is the distance that was
# still to go. Distances are fractions of the PHYSICAL stroke (inner switch = 0, outer stop = 1).
#
#   powershell -File driveprobe.ps1 [-Out "0.10:1000,0.12:1000,..."] [-In "0.12:800,..."] [-Reps 2]
param(
  [string]$Out = '0.10:1000,0.12:1000,0.14:1000,0.16:800,0.18:700,0.20:500,0.24:400',
  [string]$In  = '0.10:800,0.12:800,0.14:800,0.16:700,0.18:600,0.20:500,0.24:400',
  [int]$Reps = 2, [string]$Data = 'COM6', [double]$RulerFullMs = 472, [string]$RefDrive = '0.24:500'
)
$d = New-Object System.IO.Ports.SerialPort $Data, 115200, 'None', 8, 'One'
$d.ReadTimeout = 200; $d.DtrEnable = $true
function Run($line, $timeoutMs) {
  $d.Write($line + "`n"); $buf = ''; $t = [Diagnostics.Stopwatch]::StartNew()
  while ($t.ElapsedMilliseconds -lt $timeoutMs) { Start-Sleep -Milliseconds 20; $buf += $d.ReadExisting(); if ($buf -match 'DONE[^\r\n]*\n') { break } }
  ($buf -split "`r?`n" | Where-Object { $_ -match '^DONE' } | Select-Object -First 1)
}
function Ms($done) { if ($done -match 'DONE \w+ (-?\d+) (\d+) (\w+)') { [int]$Matches[2] } else { -1 } }
function Why($done) { if ($done -match '(\w+)\s*$') { $Matches[1] } else { '?' } }
function Ruler() { $h = Run 'HOME 0.5' 9000; Start-Sleep -Milliseconds 300; (Ms $h) / $RulerFullMs }
try {
  $d.Open(); Start-Sleep -Milliseconds 500; $null = $d.ReadExisting()
  foreach ($c in 'TEX eff_out 1.0', 'TEX eff_in 1.0', 'TEX eff_in_lo 1.0') { $d.Write($c + "`n"); Start-Sleep -Milliseconds 150; $null = $d.ReadExisting() }
  $null = Run 'HOME 0.5' 9000
  '--- outward: DRIVE out <duty> <ms>, then the ruler'
  foreach ($o in $Out.Split(',')) {
    $du, $ms = $o.Split(':'); $xs = @()
    for ($i = 0; $i -lt $Reps; $i++) {
      $r = Run "DRIVE out $du $ms" ([int]$ms + 2000); Start-Sleep -Milliseconds 300
      $xs += (Ruler)
    }
    $x = ($xs | Measure-Object -Average).Average
    'out duty {0} for {1} ms: x = {2}  => {3:N3} strokes/s{4}' -f $du, $ms, (($xs | ForEach-Object { '{0:N3}' -f $_ }) -join ' / '), ($x / ([double]$ms / 1000.0)), $(if ($x -gt 0.9) { '   (SATURATED at the end stop: lower bound)' } else { '' })
  }
  '--- inward: reference outward drive, then DRIVE in <duty> <ms>, then the ruler'
  $rd, $rms = $RefDrive.Split(':'); $refs = @()
  for ($i = 0; $i -lt 3; $i++) { $null = Run "DRIVE out $rd $rms" ([int]$rms + 2000); Start-Sleep -Milliseconds 300; $refs += (Ruler) }
  $xref = ($refs | Measure-Object -Average).Average
  'reference position after DRIVE out {0} {1}: {2} => mean {3:N3}' -f $rd, $rms, (($refs | ForEach-Object { '{0:N3}' -f $_ }) -join ' / '), $xref
  foreach ($o in $In.Split(',')) {
    $du, $ms = $o.Split(':'); $sp = @()
    for ($i = 0; $i -lt $Reps; $i++) {
      $null = Run "DRIVE out $rd $rms" ([int]$rms + 2000); Start-Sleep -Milliseconds 300
      $r = Run "DRIVE in $du $ms" ([int]$ms + 2000); $why = Why $r; $took = Ms $r; Start-Sleep -Milliseconds 300
      $rem = Ruler
      $sp += (($xref - $rem) / ([double]$took / 1000.0))
    }
    'in  duty {0} for {1} ms: {2} strokes/s  (ended {3} after {4} ms; home = it covered the whole reference distance)' -f $du, $ms, (($sp | ForEach-Object { '{0:N3}' -f $_ }) -join ' / '), $why, $took
  }
  $null = Run 'HOME 0.5' 9000
  $d.Write("STOP`n")
} catch { 'ERROR: ' + $_.Exception.Message } finally { if ($d.IsOpen) { $d.Close() } }
