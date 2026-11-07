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
#                       [-GCode "G21 G91 G1 X10 F600"] [-Expect "2500,0,0"] [-OutDir folder]

param(
    [ValidateSet("motion", "watchdog", "reset", "all")] [string]$Scenario = "all",
    [string]$GCode = "G21 G91 G1 X10 F600",
    [string]$Expect = "2500,0,0",
    [string]$OutDir = "$env:TEMP\grbl2mach_fake",
    [int]$Port = 23040
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
}

function Test-Watchdog {
    "=== watchdog: controller goes silent 300 ms into a move"
    $dir = Join-Path $OutDir "watchdog"; $log = Join-Path $dir "fake.log"
    $s = Start-Sim "-f `"$log`"" $dir ($Port + 1) @{ GRBL2MACH_FAKE_SILENT_MS = "300" }
    try {
        Invoke-Line $s '$X' | Out-Null
        Invoke-GCode $s "G21 G91 G1 X20 F600"
        $status = Wait-State $s "Alarm" 5000
        # Alarm 17 (motor fault) is critical: grblHAL wants a reset before anything else
        Send-Sim $s ([string][char]0x18); Start-Sleep -Milliseconds 500; Read-Sim $s | Out-Null
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    Check ($status -match "ALARM:17" -and $status -match "<Alarm") "grblHAL raised ALARM:17 (motor fault)"
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
        Invoke-Line $s '$X' | Out-Null
        Invoke-GCode $s "G21 G91 G1 X50 F1200"
        Start-Sleep -Milliseconds 1500
        Send-Sim $s ([string][char]0x18)   # Ctrl-X: soft reset
        $status = Wait-State $s "Alarm|Idle" 5000
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    $lastSlice = $r.Frames | Where-Object { $_.op -eq 5 } | Select-Object -Last 1
    $estop = $r.Frames | Where-Object { $_.op -eq 1 -and $_.t -gt 100 } | Select-Object -First 1
    Check ($status -match "<Alarm") "grblHAL in Alarm after reset during motion: $($status.Trim())"
    Check ($null -ne $estop -and $estop.t -ge $lastSlice.t -and ($estop.t - $lastSlice.t) -lt 50) "e-stop sent right after the last slice (+$([math]::Round($estop.t - $lastSlice.t)) ms)"
    Check ($r.Final[0] -lt 50 * 250) "controller stopped before the end of the move (X=$($r.Final[0]) steps of $(50 * 250))"
    Check ($r.Stats.still_buffered -eq 0) "nothing left in the controller buffer"
}

if ($Scenario -in "motion", "all") { Test-Motion }
if ($Scenario -in "watchdog", "all") { Test-Watchdog }
if ($Scenario -in "reset", "all") { Test-Reset }

if ($script:failures -eq 0) { "RESULT: PASS"; exit 0 } else { "RESULT: FAIL ($script:failures)"; exit 1 }
