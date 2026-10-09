# replay.ps1 -- drive the servo firmware over its data port through the Mega CD data->CDDA seek,
# the way the MiSTer will: out to 70% of the stroke in 1971 ms with the spindle gliding down to
# the rim speed, a pause while the track starts, then back to the hub in 1923 ms.
#
#   powershell -File replay.ps1 [-Cycles 3] [-Data COM6] [-OutPermille 700] [-OutMs 1971] [-BackMs 1923]
param([int]$Cycles = 3, [string]$Data = 'COM6', [int]$OutPermille = 700, [int]$OutMs = 1971, [int]$BackMs = 1923)
$d = New-Object System.IO.Ports.SerialPort $Data, 115200, 'None', 8, 'One'
$d.ReadTimeout = 200; $d.DtrEnable = $true
function Tx($line, $waitMs) { $d.Write($line + "`n"); Start-Sleep -Milliseconds $waitMs; $r = $d.ReadExisting().Trim() -replace "`r?`n", ' | '; "{0,-22} -> {1}" -f $line, $r }
try {
  $d.Open(); Start-Sleep -Milliseconds 500; $null = $d.ReadExisting()
  Tx 'HOME' 7000
  Tx 'ST' 100
  Tx 'SPIN 431 1000' 1200
  for ($i = 1; $i -le $Cycles; $i++) {
    "--- cycle $i"
    Tx "MOVE $OutPermille $OutMs" 100
    Tx "SPIN 241 $OutMs" ($OutMs + 300)
    Tx 'ST' 100
    Start-Sleep -Milliseconds 400
    Tx "MOVE 30 $BackMs" 100
    Tx "SPIN 431 $BackMs" ($BackMs + 1500)
    Tx 'ST' 100
    Start-Sleep -Milliseconds 800
  }
  Tx 'SPIN 0 300' 600
  Tx 'ST' 100
} catch { 'ERROR: ' + $_.Exception.Message } finally { if ($d.IsOpen) { $d.Close() } }
