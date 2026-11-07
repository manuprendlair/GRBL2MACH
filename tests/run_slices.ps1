# run_slices.ps1 - dry run of the step slicer (GRBL2MACH, plan step 2)
#
# Starts grblHAL_sim with the "file" slice sink (-o), sends a G-code program over TCP,
# waits until idle, stops the simulator cleanly (Ctrl-F), then checks the slice file:
#   - total steps per axis = final position reported by grblHAL (MPos x steps/mm) = expected;
#   - consecutive slices while moving (no gap);
#   - maximum steps per slice consistent with the requested feed rate (optional).
#
# Usage: powershell -File tests\run_slices.ps1 [-GCode "G91 G1 X10 F600; G1 Y5"] [-Expect "2500,0,0"]
#                                               [-MaxStepsPerSlice 63] [-OutDir folder] [-Port 23023]

param(
    [string]$GCode = "G21 G91 G1 X10 F600",   # lines separated by ";"
    [string]$Expect = "2500,0,0",             # expected total steps X,Y,Z
    [int]$MaxStepsPerSlice = 0,               # 0 = not checked
    [string]$OutDir = "$env:TEMP\grbl2mach_test",
    [int]$Port = 23023
)

$ErrorActionPreference = "Stop"
$sim = Join-Path $PSScriptRoot "..\build\grblHAL_sim.exe"
New-Item -ItemType Directory -Force $OutDir | Out-Null
$slices = Join-Path $OutDir "slices.csv"
$eeprom = Join-Path $OutDir "test.eep"
Remove-Item $slices, $eeprom -ErrorAction SilentlyContinue

$p = Start-Process $sim -ArgumentList "-t 0 -n -e `"$eeprom`" -b NUL -s NUL -o `"$slices`" -p $Port" `
        -PassThru -WindowStyle Hidden -RedirectStandardOutput (Join-Path $OutDir "sim_stdout.txt")
try {
    $client = $null
    for ($i = 0; $i -lt 50 -and -not $client; $i++) {
        try { $client = New-Object Net.Sockets.TcpClient("127.0.0.1", $Port) } catch { Start-Sleep -Milliseconds 100 }
    }
    if (-not $client) { throw "Simulator not reachable on port $Port" }
    $s = $client.GetStream(); $s.ReadTimeout = 200
    $rx = New-Object byte[] 65536
    function Read-All { $o = ""; try { while ($true) { $n = $s.Read($rx, 0, $rx.Length); if ($n -le 0) { break }; $o += [Text.Encoding]::ASCII.GetString($rx, 0, $n) } } catch {}; $o }
    function Send-Line([string]$t) { $b = [Text.Encoding]::ASCII.GetBytes($t); $s.Write($b, 0, $b.Length) }
    function Wait-Ok([string]$line) {
        Send-Line "$line`n"; $acc = ""
        for ($i = 0; $i -lt 300; $i++) { $acc += Read-All; if ($acc -match "(?m)^(ok|error:\d+)") { return $acc } }
        throw "No response to '$line'"
    }

    Start-Sleep -Milliseconds 300; Read-All | Out-Null
    $settings = Wait-Ok '$$'
    $spm = @(0, 1, 2) | ForEach-Object { [double]([regex]::Match($settings, "(?m)^\`$10$_=([0-9.]+)").Groups[1].Value) }
    Wait-Ok '$X' | Out-Null
    foreach ($line in ($GCode -split ";" | ForEach-Object { $_.Trim() } | Where-Object { $_ })) {
        $r = Wait-Ok $line
        if ($r -match "error:\d+") { throw "grblHAL rejected '$line': $r" }
    }

    # wait until idle (the simulator runs as fast as possible: -t 0)
    $status = ""
    for ($i = 0; $i -lt 600; $i++) {
        Send-Line "?"; Start-Sleep -Milliseconds 50; $status = Read-All
        if ($status -match "<Idle\|MPos:([-0-9.]+),([-0-9.]+),([-0-9.]+)") { break }
    }
    if ($status -notmatch "<Idle\|MPos:([-0-9.]+),([-0-9.]+),([-0-9.]+)") { throw "Machine did not return to idle: $status" }
    $mpos = @([double]$Matches[1], [double]$Matches[2], [double]$Matches[3])

    Send-Line ([string][char]6)            # Ctrl-F: clean shutdown, writes the last slice
    $client.Close()
    if (-not $p.WaitForExit(15000)) { throw "Simulator did not exit" }
}
finally {
    if (-not $p.HasExited) { Stop-Process $p -Force }
}

# --- slice file analysis
$rows = Get-Content $slices | Where-Object { $_ -notmatch "^#" } | ForEach-Object {
    $f = $_.Split(","); [pscustomobject]@{ i = [int]$f[0]; d = @([int]$f[2], [int]$f[3], [int]$f[4], [int]$f[5]) }
}
$sum = @(0, 0, 0, 0); $maxd = 0; $gaps = 0; $prev = $null
foreach ($r in $rows) {
    for ($a = 0; $a -lt 4; $a++) { $sum[$a] += $r.d[$a]; $maxd = [math]::Max($maxd, [math]::Abs($r.d[$a])) }
    if ($prev -ne $null -and $r.i -ne $prev + 1) { $gaps++ }
    $prev = $r.i
}
$fromMpos = @(0, 1, 2) | ForEach-Object { [math]::Round($mpos[$_] * $spm[$_]) }
$exp = $Expect.Split(",") | ForEach-Object { [int]$_ }

"Slices             : {0} (from {1} to {2}), restarts after idle: {3}" -f $rows.Count, $rows[0].i, $rows[-1].i, $gaps
"Total steps X,Y,Z,A: {0}" -f ($sum -join ",")
"grblHAL MPos       : {0} mm => {1} steps (steps/mm {2})" -f ($mpos -join " "), ($fromMpos -join ","), ($spm -join ",")
"Expected           : {0}" -f ($exp -join ",")
"Max steps/slice    : {0}" -f $maxd

$ok = $true
for ($a = 0; $a -lt 3; $a++) {
    if ($sum[$a] -ne $exp[$a]) { "FAIL: axis $a, total $($sum[$a]) instead of $($exp[$a])"; $ok = $false }
    if ($sum[$a] -ne $fromMpos[$a]) { "FAIL: axis $a, total $($sum[$a]) differs from MPos ($($fromMpos[$a]))"; $ok = $false }
}
if ($MaxStepsPerSlice -gt 0 -and $maxd -gt $MaxStepsPerSlice) { "FAIL: $maxd steps in one slice (max $MaxStepsPerSlice)"; $ok = $false }
if ($ok) { "RESULT: PASS"; exit 0 } else { "RESULT: FAIL"; exit 1 }
