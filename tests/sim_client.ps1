# sim_client.ps1 - helpers shared by the GRBL2MACH tests (dot-source this file)
#
# Start-Sim starts grbl2mach with the given extra arguments, connects over TCP and returns a
# session object. Invoke-GCode sends lines and waits for "ok"; Wait-Idle polls the status report;
# Stop-Sim shuts the simulator down cleanly (Ctrl-F) and waits for it to exit.

# GRBL2MACH_SIM may point to another build of the simulator (for example an instrumented one)
$script:SimExe = if ($env:GRBL2MACH_SIM) { $env:GRBL2MACH_SIM } else { Join-Path $PSScriptRoot "..\build\grbl2mach.exe" }

function Start-Sim([string]$SimArgs, [string]$OutDir, [int]$Port, [hashtable]$Env = @{}) {
    New-Item -ItemType Directory -Force $OutDir | Out-Null
    $eeprom = Join-Path $OutDir "test.eep"
    Remove-Item $eeprom -ErrorAction SilentlyContinue
    foreach ($k in $Env.Keys) { [Environment]::SetEnvironmentVariable($k, $Env[$k]) }
    $p = Start-Process $script:SimExe -ArgumentList "-n -e `"$eeprom`" -b NUL -s NUL -p $Port $SimArgs" -PassThru -WindowStyle Hidden `
            -RedirectStandardOutput (Join-Path $OutDir "sim_stdout.txt") -RedirectStandardError (Join-Path $OutDir "sim_stderr.txt")
    foreach ($k in $Env.Keys) { [Environment]::SetEnvironmentVariable($k, $null) }

    $client = $null
    for ($i = 0; $i -lt 50 -and -not $client; $i++) {
        try { $client = New-Object Net.Sockets.TcpClient("127.0.0.1", $Port) } catch { Start-Sleep -Milliseconds 100 }
    }
    if (-not $client) { Stop-Process $p -Force -ErrorAction SilentlyContinue; throw "Simulator not reachable on port $Port" }
    $stream = $client.GetStream(); $stream.ReadTimeout = 200
    $session = [pscustomobject]@{ Process = $p; Client = $client; Stream = $stream; Buffer = (New-Object byte[] 65536) }
    Start-Sleep -Milliseconds 300; Read-Sim $session | Out-Null
    return $session
}

function Read-Sim($s) {
    $o = ""
    try { while ($true) { $n = $s.Stream.Read($s.Buffer, 0, $s.Buffer.Length); if ($n -le 0) { break }; $o += [Text.Encoding]::ASCII.GetString($s.Buffer, 0, $n) } } catch {}
    return $o
}

function Send-Sim($s, [string]$text) {
    $b = [Text.Encoding]::ASCII.GetBytes($text); $s.Stream.Write($b, 0, $b.Length)
}

# Sends one line and returns everything received up to its "ok" or "error:n".
function Invoke-Line($s, [string]$line) {
    Send-Sim $s "$line`n"; $acc = ""
    for ($i = 0; $i -lt 300; $i++) { $acc += Read-Sim $s; if ($acc -match "(?m)^(ok|error:\d+)") { return $acc } }
    throw "No response to '$line'"
}

# Sends ";"-separated G-code lines, throws on the first error.
function Invoke-GCode($s, [string]$gcode) {
    foreach ($line in ($gcode -split ";" | ForEach-Object { $_.Trim() } | Where-Object { $_ })) {
        $r = Invoke-Line $s $line
        if ($r -match "error:\d+") { throw "grblHAL rejected '$line': $r" }
    }
}

# Returns steps/mm for X, Y, Z from the $$ settings.
function Get-StepsPerMm($s) {
    $settings = Invoke-Line $s '$$'
    return @(0, 1, 2) | ForEach-Object { [double]([regex]::Match($settings, "(?m)^\`$10$_=([0-9.]+)").Groups[1].Value) }
}

# Polls the status report until the state matches $State (regex); returns the last report.
function Wait-State($s, [string]$State = "Idle", [int]$TimeoutMs = 30000) {
    $status = ""
    for ($t = 0; $t -lt $TimeoutMs; $t += 50) {
        Send-Sim $s "?"; Start-Sleep -Milliseconds 50; $status = Read-Sim $s
        if ($status -match "<($State)\|") { return $status }
    }
    throw "State '$State' not reached: $status"
}

function Get-MPos([string]$status) {
    if ($status -notmatch "MPos:([-0-9.]+),([-0-9.]+),([-0-9.]+)") { throw "No MPos in '$status'" }
    return @([double]$Matches[1], [double]$Matches[2], [double]$Matches[3])
}

function Stop-Sim($s, [int]$TimeoutMs = 20000) {
    try { Send-Sim $s ([string][char]6) } catch {}   # Ctrl-F: clean shutdown
    $s.Client.Close()
    if (-not $s.Process.WaitForExit($TimeoutMs)) { Stop-Process $s.Process -Force; throw "Simulator did not exit" }
}
