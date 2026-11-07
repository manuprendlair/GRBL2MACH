# run_fake_board.ps1 - RNR sink tests against the fake controller (GRBL2MACH, plan step C)
#
# Scenarios:
#   motion   : runs a G-code program, then checks the fake controller log: frame sequence
#              (connect, arm, run, end of run, disarm), total steps = expected = grblHAL MPos,
#              real-time pacing (controller buffer depth, no underrun), no protocol error.
#   watchdog : the fake controller stops reporting during a move: grblHAL must go to Alarm
#              and the bridge must send an e-stop.
#   reset    : a soft reset (Ctrl-X) during a long move: the controller must get an e-stop at once
#              and must not execute the slices that were still queued.
#
# Usage: powershell -File tests\run_fake_board.ps1 [-Scenario motion|watchdog|reset|all]
#                       [-GCode "G21 G91 G1 X10 F600"] [-Expect "2500,0,0"] [-OutDir folder] [-Immediate]

param(
    [ValidateSet("motion", "watchdog", "reset", "all")] [string]$Scenario = "all",
    [string]$GCode = "G21 G91 G1 X10 F600",
    [string]$Expect = "2500,0,0",
    [string]$OutDir = "$env:TEMP\grbl2mach_fake",
    [int]$Port = 23040,
    [switch]$Immediate     # reset scenario: unlock and move right after the reset, without waiting
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "sim_client.ps1")

$script:failures = 0
function Check([bool]$cond, [string]$what) {
    if ($cond) { "  ok   : $what" } else { "  FAIL : $what"; $script:failures++ }
}

# Parses the fake controller log: frames (time, op, seq, deltas) and the final statistics.
function Read-FakeLog([string]$path) {
    $frames = @(); $stats = @{}; $final = $null
    foreach ($l in Get-Content $path) {
        if ($l -match "^\s*([0-9.]+) op=0x([0-9A-F]{2}) seq=(\d+)(.*)$") {
            $f = [pscustomobject]@{ t = [double]$Matches[1]; op = [Convert]::ToInt32($Matches[2], 16); seq = [int]$Matches[3]; rest = $Matches[4]; d = $null }
            if ($f.rest -match " d=(-?\d+),(-?\d+),(-?\d+),(-?\d+)") { $f.d = @([int]$Matches[1], [int]$Matches[2], [int]$Matches[3], [int]$Matches[4]) }
            $frames += $f
        } elseif ($l -match "^# frames=") {
            foreach ($kv in ($l.Substring(2) -split " ")) { $p = $kv -split "="; $stats[$p[0]] = [int]$p[1] }
        } elseif ($l -match "^# final position X=(-?\d+) Y=(-?\d+) Z=(-?\d+) A=(-?\d+) estop=(\d)") {
            $final = @([int]$Matches[1], [int]$Matches[2], [int]$Matches[3], [int]$Matches[4], [int]$Matches[5])
        }
    }
    return [pscustomobject]@{ Frames = $frames; Stats = $stats; Final = $final }
}

function Test-Motion {
    "=== motion: $GCode"
    $dir = Join-Path $OutDir "motion"; $log = Join-Path $dir "fake.log"
    $s = Start-Sim "-f `"$log`"" $dir $Port
    try {
        $spm = Get-StepsPerMm $s
        Invoke-Line $s '$X' | Out-Null
        $t0 = Get-Date
        Invoke-GCode $s $GCode
        $mpos = Get-MPos (Wait-State $s "Idle")
    } finally { Stop-Sim $s }

    $r = Read-FakeLog $log
    $ops = $r.Frames | ForEach-Object { $_.op }
    $slices = @($r.Frames | Where-Object { $_.op -eq 5 })
    $sum = @(0, 0, 0, 0); foreach ($f in $slices) { for ($a = 0; $a -lt 4; $a++) { $sum[$a] += $f.d[$a] } }
    $exp = $Expect.Split(",") | ForEach-Object { [int]$_ }
    $fromMpos = @(0, 1, 2) | ForEach-Object { [math]::Round($mpos[$_] * $spm[$_]) }
    $span = $slices[-1].t - $slices[0].t
    $expectedSpan = ($slices.Count - 1) * 25

    "  slices sent: $($slices.Count), over $([math]::Round($span)) ms (real time: $expectedSpan ms), stats: " + (($r.Stats.GetEnumerator() | Sort-Object Name | ForEach-Object { "$($_.Name)=$($_.Value)" }) -join " ")
    Check (($ops[0..3] -join ",") -eq "15,1,3,9") "connect sequence Configure, EStop, SetOutputs, SetPositions"
    $firstSlice = [array]::IndexOf($ops, 5)
    Check (($ops[($firstSlice - 4)..($firstSlice - 1)] -join ",") -eq "7,2,15,10") "arm then run start before the first slice: PreReset, ClearEStop, Configure, StartJog"
    Check ($ops[-2] -eq 6 -and $ops[-1] -eq 1) "run closed with EndOfMotion, controller disarmed (EStop) at exit"
    for ($a = 0; $a -lt 3; $a++) {
        Check ($sum[$a] -eq $exp[$a] -and $sum[$a] -eq $fromMpos[$a]) "axis $a total $($sum[$a]) = expected $($exp[$a]) = MPos $($fromMpos[$a])"
    }
    Check ($r.Final[0] -eq $exp[0] -and $r.Final[1] -eq $exp[1] -and $r.Final[2] -eq $exp[2]) "controller executed every step (final position $($r.Final[0..3] -join ','))"
    Check ($r.Stats.max_depth -le 4) "real-time pacing: controller buffer depth $($r.Stats.max_depth) <= 4 slices"
    Check ([math]::Abs($span - $expectedSpan) -le [math]::Max(50, 0.05 * $expectedSpan)) "slices spread in real time ($([math]::Round($span)) ms for $expectedSpan ms)"
    Check ($r.Stats.starved -eq 0) "no underrun during the run"
    Check ($r.Stats.seq_errors -eq 0 -and $r.Stats.rejected_estop -eq 0 -and $r.Stats.overflows -eq 0) "no sequence error, no slice rejected"
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    Check ($err -match "position checks after runs: [1-9]\d* ok, 0 mismatch") "controller counters = steps sent after each run ($(([regex]::Match($err, 'position checks[^\r\n]*')).Value))"
    Check ($err -notmatch "resynced") "no resync needed in normal operation"
}

function Test-Watchdog {
    "=== watchdog: controller goes silent 300 ms into a move"
    $dir = Join-Path $OutDir "watchdog"; $log = Join-Path $dir "fake.log"
    $s = Start-Sim "-f `"$log`"" $dir ($Port + 1) @{ GRBL2MACH_FAKE_SILENT_MS = "300" }
    try {
        Invoke-Line $s '$X' | Out-Null
        Send-Sim $s "G21 G91 G1 X20 F600`n"
        $out = ""; for ($i = 0; $i -lt 40 -and $out -notmatch "ALARM:"; $i++) { Start-Sleep -Milliseconds 100; $out += Read-Sim $s }
        # The motor fault is latched: a reset must not clear it while the controller is lost
        Send-Sim $s ([string][char]0x18); Start-Sleep -Milliseconds 500; Read-Sim $s | Out-Null
        $afterReset = Wait-State $s "Alarm" 2000
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    $alarms = ([regex]::Matches($out, 'ALARM:\d+') | ForEach-Object { $_.Value }) -join ", "
    Check ($out -match "ALARM:17") "grblHAL raised ALARM:17 (motor fault); alarms seen: $alarms"
    Check ($afterReset -match "<Alarm") "still in Alarm after a reset (fault latched)"
    Check ($err -match "watchdog") "bridge reported the watchdog fault"
    $silentAt = ($r.Frames | Where-Object { $_.op -eq 5 } | Select-Object -First 1).t + 300
    $estop = $r.Frames | Where-Object { $_.op -eq 1 -and $_.t -gt $silentAt } | Select-Object -First 1
    Check ($null -ne $estop -and ($estop.t - $silentAt) -lt 800) "e-stop sent $([math]::Round($estop.t - $silentAt)) ms after the controller went silent"
}

function Test-Reset {
    "=== reset: Ctrl-X during a long move"
    $dir = Join-Path $OutDir "reset"; $log = Join-Path $dir "fake.log"
    $s = Start-Sim "-f `"$log`"" $dir ($Port + 2)
    try {
        $spm = Get-StepsPerMm $s
        Invoke-Line $s '$X' | Out-Null
        Invoke-GCode $s "G21 G91 G1 X50 F1200"
        Start-Sleep -Milliseconds 1500
        Send-Sim $s ([string][char]0x18)   # Ctrl-X: soft reset
        # Alarm 3 (position lost) is expected, but the upstream simulator sometimes misses it: its
        # mc_reset() runs on another thread than the core (race seen without GRBL2MACH code too).
        # What matters here is the position, checked below in both cases.
        $state = ([regex]::Match((Wait-State $s "Alarm|Idle" 5000), '<(\w+)\|')).Groups[1].Value
        if ($Immediate) {
            # Unlock and move at once: the move must still start from the resynced position
            Invoke-Line $s '$X' | Out-Null
            Invoke-GCode $s "G21 G91 G1 X1 F600"
            $afterReset = $null
        } else {
            Start-Sleep -Milliseconds 500   # resync task: waits for the controller's halted position
            $afterReset = Get-MPos (Wait-State $s "Alarm|Idle" 2000)
            Invoke-Line $s '$X' | Out-Null
            Invoke-GCode $s "G21 G91 G1 X1 F600"
        }
        $final = Get-MPos (Wait-State $s "Idle")
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    $estop = $r.Frames | Where-Object { $_.op -eq 1 -and $_.rest -match "halted pos=(-?\d+)" -and $_.t -gt 100 } | Select-Object -First 1
    $null = $estop.rest -match "halted pos=(-?\d+),(-?\d+),(-?\d+)"; $haltedX = [int]$Matches[1]
    $lastSliceBefore = $r.Frames | Where-Object { $_.op -eq 5 -and $_.t -lt $estop.t } | Select-Object -Last 1
    $one = [math]::Round(1 * $spm[0])
    Check ($null -ne $estop -and ($estop.t - $lastSliceBefore.t) -lt 50) "e-stop sent right after the last slice (+$([math]::Round($estop.t - $lastSliceBefore.t)) ms), controller halted at X=$haltedX"
    Check ($err -match "position resynced from the machine: X-\d+") "bridge resynced the position: $(([regex]::Match($err, 'resynced[^\r\n]*')).Value)"
    if ($afterReset) {
        Check ([math]::Round($afterReset[0] * $spm[0]) -eq $haltedX) "grblHAL position after reset = controller halted position ($([math]::Round($afterReset[0] * $spm[0])) / $haltedX steps)"
    }
    Check ($r.Final[0] -eq $haltedX + $one) "controller ran exactly the extra 1 mm from where it halted (final X=$($r.Final[0]))"
    Check ([math]::Round($final[0] * $spm[0]) -eq $r.Final[0]) "grblHAL and controller agree at the end ($([math]::Round($final[0] * $spm[0])) / $($r.Final[0]) steps)"
    "  info : state after the reset: $state"
    Check ($r.Final[0] -lt 50 * 250) "controller stopped before the end of the long move"
    Check ($r.Stats.still_buffered -eq 0 -and $r.Stats.rejected_estop -eq 0) "nothing left in the controller buffer, no slice sent in e-stop"
    Check ($err -match "position checks after runs: \d+ ok, 0 mismatch") "position check after the extra move"
}

if ($Scenario -in "motion", "all") { Test-Motion }
if ($Scenario -in "watchdog", "all") { Test-Watchdog }
if ($Scenario -in "reset", "all") { Test-Reset }

if ($script:failures -eq 0) { "RESULT: PASS"; exit 0 } else { "RESULT: FAIL ($script:failures)"; exit 1 }
