# chars.ps1 -- measure what the sled REALLY does under the textured drive, using the limit switch
# and a smooth timed homing run as the ruler.
#
#   outward:  HOME 0.5 (smooth), MOVE <target> <ms> (textured), then HOME 0.5 again. The smooth
#             homing time divided by the full-stroke time at that duty (472 ms) is the true distance
#             the textured move covered.
#   inward:   the same outward move, then MOVE 30 <ms> (textured return); reports how long the
#             sled actually took to reach the switch against how long it was asked to.
#
#   powershell -File chars.ps1 [-Reps 3] [-Out "700:1971,400:1000"] [-In "1923,2400"]
param([int]$Reps = 3, [string]$Data = 'COM6', [string]$Out = '700:1971', [string]$In = '1923', [double]$RulerStrokeMs = 472)
$d = New-Object System.IO.Ports.SerialPort $Data, 115200, 'None', 8, 'One'
$d.ReadTimeout = 200; $d.DtrEnable = $true
function Run($line, $timeoutMs) {
  $d.Write($line + "`n"); $buf = ''; $t = [Diagnostics.Stopwatch]::StartNew()
  while ($t.ElapsedMilliseconds -lt $timeoutMs) { Start-Sleep -Milliseconds 20; $buf += $d.ReadExisting(); if ($buf -match 'DONE[^\r\n]*\n') { break } }
  ($buf -split "`r?`n" | Where-Object { $_ -match '^DONE' } | Select-Object -First 1)
}
function Ms($done) { if ($done -match 'DONE \w+ (-?\d+) (\d+) (\w+)') { [int]$Matches[2] } else { -1 } }
try {
  $d.Open(); Start-Sleep -Milliseconds 500; $null = $d.ReadExisting()
  $null = Run 'HOME 0.5' 9000
  foreach ($o in $Out.Split(',')) {
    $tp, $tms = $o.Split(':')
    "== outward  target $tp permille in $tms ms (textured)"
    for ($i = 1; $i -le $Reps; $i++) {
      $mv = Run "MOVE $tp $tms" ([int]$tms + 3000)
      Start-Sleep -Milliseconds 500
      $hm = Run 'HOME 0.5' 9000
      $x = (Ms $hm) / $RulerStrokeMs
      "  rep {0}: {1}   smooth home took {2} ms => true travel ~{3:N2} of the stroke (asked {4:N2})" -f $i, $mv, (Ms $hm), $x, ([int]$tp / 1000.0)
    }
  }
  foreach ($t in $In.Split(',')) {
    "== inward  return to the switch asked in $t ms (textured), from the first outward target"
    $tp, $tms = $Out.Split(',')[0].Split(':')
    for ($i = 1; $i -le $Reps; $i++) {
      $null = Run "MOVE $tp $tms" ([int]$tms + 3000)
      Start-Sleep -Milliseconds 500
      $bk = Run "MOVE 30 $t" ([int]$t + 5000)
      "  rep {0}: {1}   (asked {2} ms)" -f $i, $bk, $t
      Start-Sleep -Milliseconds 500
    }
  }
  $null = Run 'HOME 0.5' 9000
  $d.Write("STOP`n")
} catch { 'ERROR: ' + $_.Exception.Message } finally { if ($d.IsOpen) { $d.Close() } }
