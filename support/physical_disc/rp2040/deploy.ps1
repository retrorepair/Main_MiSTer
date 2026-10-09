# deploy.ps1 -- copy a file onto the Pico's filesystem through the REPL console and verify it.
#
#   powershell -File deploy.ps1 [-File servo_fw.py] [-Dest /code.py] [-Console COM5] [-Data COM6]
#
# Why not just drag it onto the CIRCUITPY drive: boot.py leaves the drive read-only to the PC (so
# the running code can write /dir.txt), so files go in through the REPL as base64, 100 characters
# per line, 120 ms apart (30 ms drops characters), then are read back and checked against a CRC32.
# Afterwards the board is reset and PINGed on the data port.
param(
    [string]$File = (Join-Path $PSScriptRoot 'servo_fw.py'),
    [string]$Dest = '/code.py',
    [string]$Console = 'COM5',
    [string]$Data = 'COM6'
)
Add-Type -TypeDefinition @'
public static class PicoCrc { public static uint Of(byte[] d){ uint c=0xFFFFFFFF; foreach(byte b in d){ c^=b; for(int k=0;k<8;k++){ c=(c&1)!=0?(c>>1)^0xEDB88320u:(c>>1);} } return ~c; } }
'@ -ErrorAction SilentlyContinue
$bytes = [System.IO.File]::ReadAllBytes($File)
$b64 = [Convert]::ToBase64String($bytes)
$crc = [PicoCrc]::Of($bytes)
$p = New-Object System.IO.Ports.SerialPort $Console, 115200, 'None', 8, 'One'
$p.ReadTimeout = 500; $p.WriteTimeout = 3000; $p.DtrEnable = $true
$log = ''
try {
  $p.Open(); Start-Sleep -Milliseconds 200; $null = $p.ReadExisting()
  $p.Write([char]3); Start-Sleep -Milliseconds 500; $p.Write([char]3); Start-Sleep -Milliseconds 700
  $p.Write("`r"); Start-Sleep -Milliseconds 400; $null = $p.ReadExisting()
  $cmds = @('import binascii, gc, os', 'gc.collect()', "B = b''")
  for ($i = 0; $i -lt $b64.Length; $i += 100) { $cmds += ("B += b'" + $b64.Substring($i, [Math]::Min(100, $b64.Length - $i)) + "'") }
  $cmds += @('D = binascii.a2b_base64(B)', 'B = None', 'gc.collect()', "print('DECODED', len(D), hex(binascii.crc32(D)))",
             "g = open('$Dest', 'wb')", "print('WRITE', g.write(D))", 'g.flush()', 'g.close()',
             "R = open('$Dest','rb').read()", "print('NOW', len(R), hex(binascii.crc32(R)))")
  foreach ($c in $cmds) { $p.Write($c + "`r"); Start-Sleep -Milliseconds 120; $log += ($p.ReadExisting() -replace '[^\x20-\x7e\r\n]','?') }
  Start-Sleep -Milliseconds 1000
  $log += ($p.ReadExisting() -replace '[^\x20-\x7e\r\n]','?')
  ($log -split "`r?`n" | Where-Object { $_ -match '(DECODED|WRITE|NOW|Error|Traceback|Memory)' -and $_ -notmatch '^>>> (print|g =|D =|R =)' }) -join "`n"
  'expected: len {0}, crc 0x{1:x}' -f $bytes.Length, $crc
  $p.Write("import microcontroller`r"); Start-Sleep -Milliseconds 300
  $p.Write("microcontroller.reset()`r"); Start-Sleep -Milliseconds 300
} catch { 'ERROR: ' + $_.Exception.Message } finally { if ($p.IsOpen) { try { $p.Close() } catch { } } }
Start-Sleep -Seconds 9
$d = New-Object System.IO.Ports.SerialPort $Data, 115200, 'None', 8, 'One'
$d.ReadTimeout = 300; $d.DtrEnable = $true
try { $d.Open(); Start-Sleep -Milliseconds 500; $null = $d.ReadExisting(); $d.Write("PING`n"); Start-Sleep -Milliseconds 400; 'after restart: ' + $d.ReadExisting().Trim() }
catch { "${Data}: " + $_.Exception.Message } finally { if ($d.IsOpen) { $d.Close() } }
