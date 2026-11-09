# run_fake_board.ps1 - RNR sink tests against the fake controller (GRBL2MACH, plan step C)
#
# Scenarios:
#   motion   : runs a G-code program, then checks the fake controller log: frame sequence
#              (connect, arm, run, end of run, disarm), total steps = expected = grblHAL MPos,
#              real-time pacing (controller buffer depth, no underrun), no protocol error.
#   watchdog : the fake controller stops reporting during a move: grblHAL must go to Alarm
#              and the bridge must send an e-stop.
#   reset    : a soft reset (Ctrl-X) during a long move: the controller must get a cancel at once,
#              must not execute the slices that were still queued and must stay armed.
#   estop    : the controller's own e-stop input during a move: grblHAL must raise ALARM:10, resync
#              to where the controller stopped and re-arm it for the next move after a reset.
#   outputs  : M3 / M8 / M5 M9 around two moves: spindle PWM and outputs switched in step with the
#              motion (before the first slice, between the runs, after the last one), slices carry them.
#   pins     : configuration frame input bytes computed from --rnr-in, compared with Mach3's frames.
#
# Usage: powershell -File tests\run_fake_board.ps1 [-Scenario motion|watchdog|reset|estop|outputs|pins|all]
#                       [-GCode "G21 G91 G1 X10 F600"] [-Expect "2500,0,0"] [-OutDir folder] [-Immediate]

param(
    [ValidateSet("motion", "watchdog", "reset", "estop", "outputs", "pins", "home", "probe", "mach3", "pace", "jogs", "unplug", "client", "stop", "long", "all")] [string]$Scenario = "all",
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
        } elseif ($l -match "^# final position X=(-?\d+) Y=(-?\d+) Z=(-?\d+) A=(-?\d+) state=(\d)") {
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
    $slices = @($r.Frames | Where-Object { $_.op -eq 4 })
    $sum = @(0, 0, 0, 0); foreach ($f in $slices) { for ($a = 0; $a -lt 4; $a++) { $sum[$a] += $f.d[$a] } }
    $exp = $Expect.Split(",") | ForEach-Object { [int]$_ }
    $fromMpos = @(0, 1, 2) | ForEach-Object { [math]::Round($mpos[$_] * $spm[$_]) }
    $span = $slices[-1].t - $slices[0].t
    $expectedSpan = ($slices.Count - 1) * 5

    "  slices sent: $($slices.Count), over $([math]::Round($span)) ms (real time: $expectedSpan ms), stats: " + (($r.Stats.GetEnumerator() | Sort-Object Name | ForEach-Object { "$($_.Name)=$($_.Value)" }) -join " ")
    Check (($ops[0..4] -join ",") -eq "15,1,14,3,9") "connect sequence Configure, EStop, SpindlePwm, SetOutputs, SetPositions"
    $firstSlice = [array]::IndexOf($ops, 4)
    Check (($ops[($firstSlice - 4)..($firstSlice - 1)] -join ",") -eq "7,2,9,15") "arm like Mach3 before the first slice: PreReset, ClearEStop, SetPositions, Configure"
    Check (@($r.Frames | Where-Object { $_.op -in 5, 10 }).Count -eq 0) "program run made of GcodeSlice frames only (no StartJog, no JogSlice)"
    Check (@($r.Frames | Where-Object { $_.rest -notmatch "flags=0x00" }).Count -eq 0) "flags byte 0 in every frame, like Mach3"
    Check ($ops[-2] -eq 6 -and $ops[-1] -eq 1) "run closed with EndOfMotion, controller disarmed (EStop) at exit"
    for ($a = 0; $a -lt 3; $a++) {
        Check ($sum[$a] -eq $exp[$a] -and $sum[$a] -eq $fromMpos[$a]) "axis $a total $($sum[$a]) = expected $($exp[$a]) = MPos $($fromMpos[$a])"
    }
    Check ($r.Final[0] -eq $exp[0] -and $r.Final[1] -eq $exp[1] -and $r.Final[2] -eq $exp[2]) "controller executed every step (final position $($r.Final[0..3] -join ','))"
    Check ($r.Stats.max_depth -le 20) "real-time pacing: controller buffer depth $($r.Stats.max_depth) <= 20 slices (100 ms)"
    Check ([math]::Abs($span - $expectedSpan) -le [math]::Max(50, 0.05 * $expectedSpan)) "slices spread in real time ($([math]::Round($span)) ms for $expectedSpan ms)"
    Check ($r.Stats.starved -eq 0) "no underrun during the run"
    Check ($r.Stats.seq_errors -eq 0 -and $r.Stats.rejected -eq 0 -and $r.Stats.overflows -eq 0) "no sequence error, no slice rejected"
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
    $silentAt = ($r.Frames | Where-Object { $_.op -eq 4 } | Select-Object -First 1).t + 300
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
    $cancel = $r.Frames | Where-Object { $_.op -eq 8 -and $_.rest -match "halted pos=(-?\d+)" } | Select-Object -First 1
    $null = $cancel.rest -match "halted pos=(-?\d+),(-?\d+),(-?\d+)"; $haltedX = [int]$Matches[1]
    $lastSliceBefore = $r.Frames | Where-Object { $_.op -eq 4 -and $_.t -lt $cancel.t } | Select-Object -Last 1
    $one = [math]::Round(1 * $spm[0])
    Check ($null -ne $cancel -and ($cancel.t - $lastSliceBefore.t) -lt 50) "cancel sent right after the last slice (+$([math]::Round($cancel.t - $lastSliceBefore.t)) ms), controller halted at X=$haltedX"
    $afterCancel = @($r.Frames | Where-Object { $_.t -gt $cancel.t } | ForEach-Object { $_.op })
    Check (($afterCancel -notcontains 7) -and ($afterCancel -notcontains 2)) "controller stayed armed after the cancel (no re-arming sequence)"
    Check ($err -match "position resynced from the machine: X-\d+") "bridge resynced the position: $(([regex]::Match($err, 'resynced[^\r\n]*')).Value)"
    if ($afterReset) {
        Check ([math]::Round($afterReset[0] * $spm[0]) -eq $haltedX) "grblHAL position after reset = controller halted position ($([math]::Round($afterReset[0] * $spm[0])) / $haltedX steps)"
    }
    Check ($r.Final[0] -eq $haltedX + $one) "controller ran exactly the extra 1 mm from where it halted (final X=$($r.Final[0]))"
    Check ([math]::Round($final[0] * $spm[0]) -eq $r.Final[0]) "grblHAL and controller agree at the end ($([math]::Round($final[0] * $spm[0])) / $($r.Final[0]) steps)"
    "  info : state after the reset: $state"
    Check ($r.Final[0] -lt 50 * 250) "controller stopped before the end of the long move"
    Check ($r.Stats.still_buffered -eq 0 -and $r.Stats.rejected -eq 0) "nothing left in the controller buffer, no slice rejected"
    Check ($err -match "position checks after runs: \d+ ok, 0 mismatch") "position check after the extra move"
}

function Test-EStop([bool]$Self) {
    $mode = if ($Self) { "e-stop declared in the configuration frame (default): the controller stops by itself" } else { "no input pins declared (--rnr-pins 0...): the controller only reports the input, the bridge stops it" }
    "=== estop: the e-stop input (IN1) is pressed 500 ms into a move, released 1 s later; $mode"
    $dir = Join-Path $OutDir $(if ($Self) { "estop_self" } else { "estop" }); $log = Join-Path $dir "fake.log"
    $simArgs = if ($Self) { "" } else { "--rnr-pins 00000000000000000000" }
    $s = Start-Sim "-f `"$log`" $simArgs" $dir ($Port + $(if ($Self) { 5 } else { 3 })) @{ GRBL2MACH_FAKE_ESTOP_MS = "500,1500" }
    try {
        $spm = Get-StepsPerMm $s
        Invoke-Line $s '$X' | Out-Null
        Send-Sim $s "G21 G91 G1 X50 F1200`n"
        $out = ""; for ($i = 0; $i -lt 30 -and $out -notmatch "ALARM:"; $i++) { Start-Sleep -Milliseconds 100; $out += Read-Sim $s }
        $whilePressed = Wait-State $s "Alarm" 2000
        Start-Sleep -Milliseconds 1500                     # input released meanwhile
        Send-Sim $s ([string][char]0x18); Start-Sleep -Milliseconds 500; Read-Sim $s | Out-Null
        $afterReset = Get-MPos (Wait-State $s "Alarm|Idle" 2000)
        Invoke-Line $s '$X' | Out-Null
        Invoke-GCode $s "G21 G91 G1 X1 F600"
        $final = Get-MPos (Wait-State $s "Idle")
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    $pressed = Select-String -Path $log -Pattern "E-STOP INPUT pressed" | Select-Object -First 1
    $tPress = [double]($pressed.Line.Trim() -split " ")[0]
    $estop = $r.Frames | Where-Object { $_.op -eq 1 -and $_.t -gt $tPress } | Select-Object -First 1
    $haltedX = [int]([regex]::Match($estop.rest, "halted pos=(-?\d+)").Groups[1].Value)
    $delay = [math]::Round($estop.t - $tPress)
    $one = [math]::Round(1 * $spm[0])
    $alarms = ([regex]::Matches($out, 'ALARM:\d+') | ForEach-Object { $_.Value }) -join ", "
    Check ($out -match "ALARM:10") "grblHAL raised ALARM:10 (e-stop); alarms seen: $alarms"
    Check ($whilePressed -match "<Alarm") "grblHAL in Alarm while the input is pressed"
    Check ($err -match "controller e-stop input at X=") "bridge saw the e-stop input"
    Check ($delay -le 120) "bridge sent EStop (0x01) $delay ms after the press, controller halted at X=$haltedX"
    $lateSlices = @($r.Frames | Where-Object { $_.op -eq 4 -and $_.t -gt $tPress -and $_.t -lt $tPress + 1500 }).Count
    Check ($lateSlices -le 20) "slices stopped after the e-stop ($lateSlices sent before the bridge saw it, rejected by the controller)"
    Check ([math]::Round($afterReset[0] * $spm[0]) -eq $haltedX) "grblHAL position after reset = where the controller stopped ($([math]::Round($afterReset[0] * $spm[0])) / $haltedX steps)"
    $rearm = $r.Frames | Where-Object { $_.t -gt $tPress + 1000 } | ForEach-Object { $_.op }
    $firstSlice = [array]::IndexOf(@($rearm), 4)
    Check ($firstSlice -ge 4 -and (($rearm[($firstSlice - 4)..($firstSlice - 1)]) -join ",") -eq "7,2,9,15") "controller re-armed (PreReset, ClearEStop, SetPositions, Configure) before the next move"
    Check ($r.Final[0] -eq $haltedX + $one) "controller ran exactly the extra 1 mm from where it stopped (final X=$($r.Final[0]))"
    Check ([math]::Round($final[0] * $spm[0]) -eq $r.Final[0]) "grblHAL and controller agree at the end ($([math]::Round($final[0] * $spm[0])) / $($r.Final[0]) steps)"
    Check ($err -match "position checks after runs: [1-9]\d* ok, 0 mismatch") "position check after the extra move"
}

function Test-Outputs {
    "=== outputs: M3 S12000, move, M8, move, M5 M9 (spindle on OUT1, flood on OUT2, `$30=24000)"
    $dir = Join-Path $OutDir "outputs"; $log = Join-Path $dir "fake.log"
    $s = Start-Sim "-f `"$log`" --rnr-out spindle=1,flood=2" $dir ($Port + 4)
    try {
        $spm = Get-StepsPerMm $s
        Invoke-Line $s '$30=24000' | Out-Null
        Invoke-Line $s '$X' | Out-Null
        Invoke-GCode $s "G21 G91;M3 S12000;G1 X5 F600;M8;G1 X5;M5 M9"
        Wait-State $s "Idle" | Out-Null
        Start-Sleep -Milliseconds 300       # outputs at rest are switched once the last move has run
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    $five = [math]::Round(5 * $spm[0])
    $ops = $r.Frames | ForEach-Object { $_.op }
    $changes = @(Select-String -Path $log -Pattern "OUTPUTS 0x([0-9A-F]{2}) -> 0x([0-9A-F]{2}) pwm=(\d+) pos=(-?\d+),.* buffered=(\d+)" | ForEach-Object {
        $g = $_.Matches[0].Groups; [pscustomobject]@{ from = $g[1].Value; to = $g[2].Value; pwm = [int]$g[3].Value; x = [int]$g[4].Value; buffered = [int]$g[5].Value } })
    $slices = @($r.Frames | Where-Object { $_.op -eq 4 })
    $firstSlice = [array]::IndexOf($ops, 4)
    $before = @($r.Frames[0..($firstSlice - 1)])
    $pwmOn = @($before | Where-Object { $_.op -eq 14 -and $_.rest -match "pwm=500/1000" })
    $outOn = @($before | Where-Object { $_.op -eq 3 -and $_.rest -match "outputs=0x01" })

    Check (($ops[0..4] -join ",") -eq "15,1,14,3,9") "connect sequence Configure, EStop, SpindlePwm(0), SetOutputs(0), SetPositions"
    Check ($pwmOn.Count -eq 1 -and $outOn.Count -eq 1 -and $pwmOn[0].seq -lt $outOn[0].seq) "M3 S12000: SpindlePwm 500/1000 then SetOutputs 0x01, before the first slice"
    $first = @($slices | Where-Object { $_.rest -match "out=0x01 " }); $second = @($slices | Where-Object { $_.rest -match "out=0x03 " })
    Check ($first.Count + $second.Count -eq $slices.Count) "every slice carries the outputs ($($first.Count) with 0x01, $($second.Count) with 0x03, $($slices.Count) slices)"
    $sum1 = ($first | ForEach-Object { $_.d[0] } | Measure-Object -Sum).Sum; $sum2 = ($second | ForEach-Object { $_.d[0] } | Measure-Object -Sum).Sum
    Check ($sum1 -eq $five -and $sum2 -eq $five) "first move ran with the spindle only, second one with spindle and flood ($sum1 / $sum2 steps)"
    $m8 = @($changes | Where-Object { $_.from -eq "01" -and $_.to -eq "03" })
    Check ($m8.Count -eq 1 -and $m8[0].x -eq $five -and $m8[0].buffered -eq 0) "M8 switched once the first move had run (X=$($m8[0].x), $($m8[0].buffered) slices buffered)"
    $iM8 = [array]::IndexOf(@($r.Frames | ForEach-Object { "$($_.op)$($_.rest)" -match "^3 .*outputs=0x03" }), $true)
    Check ($iM8 -gt 0 -and $r.Frames[$iM8 - 1].op -eq 6) "the run was closed (EndOfMotion) before SetOutputs 0x03"
    $off = $changes[-1]
    Check ($off.to -eq "00" -and $off.x -eq 2 * $five -and $off.buffered -eq 0 -and $off.pwm -eq 0) "M5 M9: spindle PWM 0 and outputs off once the last move had run (X=$($off.x), pwm=$($off.pwm))"
    Check ($changes[0].from -eq "00" -and $changes[0].to -eq "01" -and $changes[0].x -eq 0) "spindle started before any motion"
    Check ($r.Stats.starved -eq 0 -and $r.Stats.rejected -eq 0 -and $r.Stats.seq_errors -eq 0) "no underrun, no rejected slice, no sequence error"
    Check ($err -match "position checks after runs: 2 ok, 0 mismatch") "position check after each of the two runs"
}

function Test-Pins {
    "=== pins: configuration frame input bytes computed from --rnr-in = Mach3's frames (USB captures 80, 81, 82)"
    # Reference machine as Mach3 declares it: limits not used but ticked active low, homes X/Y/Z on
    # IN3 active low, home A not used, probe on IN2, e-stop on IN1
    $ref = "x+=0low,x-=0low,xhome=3low,y+=0low,y-=0low,yhome=3low,z+=0low,z-=0low,zhome=3low,a+=0low,a-=0low,ahome=0low,probe=2"
    $cases = @(
        @("default (e-stop on IN1 only)", "", "00000000000000001000"),
        @("reference profile", "--rnr-in $ref", "00730073007300701200"),
        @("probe active low", "--rnr-in $ref,probe=2low", "00730073007300701210"),
        @("X home active high", "--rnr-in $ref,xhome=3", "00330073007300701200"),
        @("X home on IN4", "--rnr-in $ref,xhome=4low", "00740073007300701200"),
        @("A home on IN3", "--rnr-in $ref,ahome=3low", "00730073007300731200"),
        @("X++ on IN4", "--rnr-in $ref,x+=4low", "04730073007300701200"),
        @("X-- on IN4", "--rnr-in $ref,x-=4low", "40730073007300701200"),
        @("X++ and X-- on IN4", "--rnr-in $ref,x+=4low,x-=4low", "44730073007300701200"),
        @("e-stop on IN4", "--rnr-in $ref,estop=4", "00730073007300704200"),
        @("X++ active high", "--rnr-in $ref,x+=0", "00630073007300701200"),
        @("e-stop active low", "--rnr-in $ref --rnr-estop-in 1low", "00730073007300701220"),
        @("Y home active high", "--rnr-in $ref,yhome=3", "00730033007300701200"),
        @("Z home active high", "--rnr-in $ref,zhome=3", "00730073003300701200"),
        @("X-- active high", "--rnr-in $ref,x-=0", "00530073007300701200"),
        @("raw bytes", "--rnr-pins 0123456789ABCDEF0011", "0123456789ABCDEF0011"))
    $i = 0
    foreach ($c in $cases) {
        $dir = Join-Path $OutDir "pins$i"; $log = Join-Path $dir "fake.log"
        $s = Start-Sim "-f `"$log`" $($c[1])" $dir ($Port + 6)
        try { Invoke-Line $s '$I' | Out-Null } finally { Stop-Sim $s }
        $got = ([regex]::Match((Get-Content $log -Raw), "op=0x0F .*pins=([0-9A-F]{20})")).Groups[1].Value
        Check ($got -eq $c[2]) "$($c[0]): $got (expected $($c[2]))"
        $i++
    }
}

function Test-Home {
    "=== home: `$H with one axis per cycle (Z, X, Y), home switches on one input (IN3, active low) with 0.4 mm hysteresis, pull-off 0.5 mm"
    $dir = Join-Path $OutDir "home"; $log = Join-Path $dir "fake.log"
    $sw = @(2000, 1500, 1000)          # switch positions, steps (homes at the positive end)
    $s = Start-Sim "-f `"$log`" --rnr-in xhome=3low,yhome=3low,zhome=3low,probe=2" $dir ($Port + 7) @{ GRBL2MACH_FAKE_HOME = "$($sw[0]),$($sw[1]),$($sw[2]),99999999"; GRBL2MACH_FAKE_HOME_HYST = "120" }
    try {
        $spm = Get-StepsPerMm $s
        foreach ($l in '$22=1', '$23=0', '$44=4', '$45=1', '$46=2', '$24=50', '$25=300', '$27=0.5') { Invoke-Line $s $l | Out-Null }
        $out = Invoke-Line $s '$H'
        $homed = Get-MPos (Wait-State $s "Idle" 60000)
        Invoke-GCode $s "G21 G91 G1 X-1 Y-2 Z-3 F300"
        $moved = Get-MPos (Wait-State $s "Idle")
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    $pull = @(0, 1, 2) | ForEach-Object { [math]::Round(0.5 * $spm[$_]) }
    $arms = @($r.Frames | Where-Object { $_.op -eq 13 } | ForEach-Object { [int]([regex]::Match($_.rest, "axis=(\d)").Groups[1].Value) })
    $jogs = @($r.Frames | Where-Object { $_.op -eq 10 }).Count
    $switches = @(Select-String -Path $log -Pattern "HOME SWITCH").Count
    $homeFrames = $r.Frames | Where-Object { $_.t -ge ($r.Frames | Where-Object { $_.op -eq 13 } | Select-Object -First 1).t -and $_.t -le ($r.Frames | Where-Object { $_.op -eq 10 } | Select-Object -Last 1).t }
    "  `$H answered: $(($out -replace '\s+', ' ').Trim()); ArmHome axes: $($arms -join ','); stats: " + (($r.Stats.GetEnumerator() | Sort-Object Name | ForEach-Object { "$($_.Name)=$($_.Value)" }) -join " ")
    Check ($out -match "(?m)^ok") "`$H completed"
    Check (($arms -join ",") -eq "2,2,0,0,1,1") "ArmHome (0x0D) for seek and locate, one axis at a time, Z then X then Y"
    Check ($jogs -eq 6) "pull-offs as jogs (StartJog 0x0A): $jogs"
    Check ($switches -eq 6) "controller stopped by itself on the switch 6 times (seek and locate per axis): $switches"
    Check (@($homeFrames | Where-Object { $_.op -eq 4 }).Count -eq 0) "no program slice (0x04) during homing"
    $exp = @(0, 1, 2) | ForEach-Object { $sw[$_] - $pull[$_] }
    $fin = @(0, 1, 2) | ForEach-Object { $r.Final[$_] - [math]::Round(@(-1, -2, -3)[$_] * $spm[$_]) }
    Check (($fin -join ",") -eq ($exp -join ",")) "controller ended homing exactly at switch - pull-off on every axis ($($fin -join ',') steps)"
    $delta = @(0, 1, 2) | ForEach-Object { [math]::Round(($moved[$_] - $homed[$_]) * $spm[$_]) }
    $expDelta = (@(0, 1, 2) | ForEach-Object { [math]::Round(@(-1, -2, -3)[$_] * $spm[$_]) }) -join ","
    Check (($delta -join ",") -eq $expDelta) "move after homing: grblHAL and controller agree ($($delta -join ',') steps)"
    Check ($r.Stats.rejected -eq 0 -and $r.Stats.seq_errors -eq 0 -and $r.Stats.overflows -eq 0) "no rejected slice, no sequence error"
    Check ($err -match "mismatch" -and $err -notmatch "POSITION MISMATCH") "no position mismatch ($(([regex]::Match($err, 'position checks[^\r\n]*')).Value))"
}

function Test-Probe {
    "=== probe: G38.2 with contact (probe at Z = -2 mm), then without contact (ALARM:5)"
    $dir = Join-Path $OutDir "probe"; $log = Join-Path $dir "fake.log"
    $s = Start-Sim "-f `"$log`" --rnr-in probe=2" $dir ($Port + 8) @{ GRBL2MACH_FAKE_PROBE_Z = "-500" }
    try {
        $spm = Get-StepsPerMm $s
        Invoke-Line $s '$X' | Out-Null
        $prb = Invoke-Line $s "G21 G91 G38.2 Z-10 F100"
        $atContact = Get-MPos (Wait-State $s "Idle")
        Invoke-GCode $s "G91 G0 Z5"
        $up = Get-MPos (Wait-State $s "Idle")
        Send-Sim $s "G91 G38.2 Z-1 F100`n"
        $out = ""; for ($i = 0; $i -lt 50 -and $out -notmatch "ALARM:"; $i++) { Start-Sleep -Milliseconds 100; $out += Read-Sim $s }
        $noContact = Get-MPos (Wait-State $s "Alarm" 5000)
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    $zc = [regex]::Match($prb, "\[PRB:[-0-9.]+,[-0-9.]+,([-0-9.]+)(,[-0-9.]+)*:(\d)\]")
    $contacts = @(Select-String -Path $log -Pattern "PROBE CONTACT").Count
    "  G38.2 answered: $(($prb -replace '\s+', ' ').Trim()); stats: " + (($r.Stats.GetEnumerator() | Sort-Object Name | ForEach-Object { "$($_.Name)=$($_.Value)" }) -join " ")
    Check (@($r.Frames | Where-Object { $_.op -eq 12 }).Count -eq 2) "ArmProbe (0x0C) sent for each G38.2"
    Check ($contacts -eq 1) "controller stopped by itself on contact"
    Check ($zc.Success -and $zc.Groups[3].Value -eq "1" -and [math]::Round([double]$zc.Groups[1].Value * $spm[2]) -eq -500) "probe position from the controller's latch, to the step (PRB Z=$($zc.Groups[1].Value))"
    Check ([math]::Round($atContact[2] * $spm[2]) -eq -500) "grblHAL position after probing = where the controller stopped (Z=$($atContact[2]))"
    Check ([math]::Round(($up[2] - $atContact[2]) * $spm[2]) -eq [math]::Round(5 * $spm[2])) "move after probing starts from there"
    Check ($out -match "ALARM:5") "no contact: ALARM:5 (probe fail, no contact)"
    Check ([math]::Round($noContact[2] * $spm[2]) -eq $r.Final[2]) "no contact: whole move run, grblHAL and controller agree (Z=$($r.Final[2]) steps)"
    Check ($r.Stats.rejected -eq 0 -and $r.Stats.seq_errors -eq 0) "no rejected slice, no sequence error"
    Check ($err -notmatch "POSITION MISMATCH") "no position mismatch ($(([regex]::Match($err, 'position checks[^\r\n]*')).Value))"
}

function Test-Mach3 {
    "=== mach3: machine profile -> grblHAL settings and controller configuration; settings changes -> profile"
    $dir = Join-Path $OutDir "mach3"
    New-Item -ItemType Directory -Force $dir | Out-Null
    Remove-Item (Join-Path $dir "MACH3.xml*") -ErrorAction SilentlyContinue
    $src = Join-Path $PSScriptRoot "mach3_test.xml"; $xml = Join-Path $dir "MACH3.xml"; $log = Join-Path $dir "fake.log"
    Copy-Item $src $xml
    foreach ($i in 1..11) { Set-Content (Join-Path $dir ("MACH3.xml.2000-01-{0:D2}_00-00-00.bak" -f $i)) "old" }   # only 10 must stay
    $tag = { param($name) [regex]::Match([IO.File]::ReadAllText($xml), "<$name>([^<]*)</$name>").Groups[1].Value }

    # 1. Start-up from the profile, then settings changed by the sender
    $s = Start-Sim "-f `"$log`" --mach3 `"$xml`"" $dir ($Port + 9)
    try {
        $set = Invoke-Line $s '$$'
        foreach ($l in '$100=802', '$110=900', '$27=1') { Invoke-Line $s $l | Out-Null }
        Start-Sleep -Seconds 4      # written 2 s after the last change
    } finally { Stop-Sim $s }
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    $val = { param($id) [regex]::Match($set, "(?m)^\`$$id=([-0-9.]+)").Groups[1].Value }
    $got = (@(100, 101, 102, 103, 110, 112, 120, 121, 130, 131, 132, 2, 3, 20, 22, 23, 27, 30, 190, 192) | ForEach-Object { "`$$_=$(& $val $_)" }) -join " "
    "  settings: $got"
    Check ([double](& $val 100) -eq 801.5 -and [double](& $val 101) -eq 806.5 -and [math]::Abs([double](& $val 102) - 803.9) -lt 0.001) "steps/mm from Steps0..2"
    Check ([double](& $val 110) -eq 799.8 -and [double](& $val 112) -eq 499.98 -and [double](& $val 121) -eq 35) "max rates from Vel0..2 (mm/s x 60), accelerations from Acc0..2"
    Check ([double](& $val 130) -eq 285 -and [double](& $val 131) -eq 376 -and [double](& $val 132) -eq 50) "max travel from the soft limits (Max - Min)"
    Check ((& $val 2) -eq "15" -and (& $val 3) -eq "8" -and (& $val 20) -eq "1" -and (& $val 22) -eq "515" -and (& $val 23) -eq "0") "step/dir inversion, soft limits, homing on (single axis, per-axis rates), homing direction"
    Check ([double](& $val 190) -eq 639.8 -and [double](& $val 192) -eq 400 -and [double](& $val 27) -eq 0.5 -and [double](& $val 30) -eq 24000) "homing seek = RefSpeed % of the max rate, pull-off from the plugin, spindle max from the pulley"
    $cfg = Select-String -Path $log -Pattern "op=0x0F .*" | Select-Object -First 1
    Check ($cfg.Line -match "khz=100 slice_ms=5 invert=0x8F pins=00730073007300701200") "configuration frame identical to Mach3's (invert 0x8F, 100 kHz, pins)"
    Check ((& $tag "Steps0") -eq "802." -and (& $tag "Vel0") -eq "15." -and (& $tag "XPullOff") -eq "1.000" -and (& $tag "RefSpeed0") -eq "71") "changed settings written back (Steps0=802. Vel0=15. XPullOff=1.000, RefSpeed0=71 keeps the homing speed)"
    $norm = { param($bytes) [regex]::Replace([Text.Encoding]::GetEncoding(28591).GetString($bytes), "<(Steps0|Vel0|RefSpeed0|XPullOff|YPullOff|ZPullOff)>[^<]*<", '<$1>#<') }
    Check ((& $norm ([IO.File]::ReadAllBytes($src))) -eq (& $norm ([IO.File]::ReadAllBytes($xml)))) "every other byte of the profile unchanged (Latin-1 accent, layout, other tags)"
    $baks = @(Get-ChildItem $dir -Filter "MACH3.xml.*.bak" | Sort-Object Name)
    Check ($baks.Count -eq 10 -and [Linq.Enumerable]::SequenceEqual([IO.File]::ReadAllBytes($baks[-1].FullName), [IO.File]::ReadAllBytes($src))) "one backup of the previous file, the 10 most recent kept ($($baks.Count))"
    Check (@([regex]::Matches($err, "profile updated")).Count -eq 1) "one write for the batch of 3 changes"

    # 2. Restart from the updated profile: the changed settings come back, nothing is written
    $s = Start-Sim "-f `"$log`" --mach3 `"$xml`"" $dir ($Port + 9)
    try {
        $set = Invoke-Line $s '$$'
        Start-Sleep -Seconds 3
        # 3. The file is changed on disk while the bridge runs: it must not be overwritten
        Add-Content $xml "<!-- edited by hand -->"
        Invoke-Line $s '$100=803' | Out-Null
        Start-Sleep -Seconds 4
    } finally { Stop-Sim $s }
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    Check ([double](& $val 100) -eq 802 -and [double](& $val 110) -eq 900 -and [double](& $val 27) -eq 1) "restart: settings read back from the updated profile (round trip)"
    Check ($err -match "changed on disk" -and (& $tag "Steps0") -eq "802.") "profile changed on disk meanwhile: not overwritten, reported"

    # 4. No profile: refused, with a message saying what to do
    $p = Start-Process $script:SimExe -ArgumentList "-n -f `"$log`" --mach3 `"$dir\missing.xml`"" -PassThru -Wait -WindowStyle Hidden `
            -RedirectStandardOutput (Join-Path $dir "missing_stdout.txt") -RedirectStandardError (Join-Path $dir "missing_stderr.txt")
    $msg = Get-Content (Join-Path $dir "missing_stdout.txt") -Raw
    Check ($p.ExitCode -ne 0 -and $msg -match "no Mach3 profile") "missing profile: refused ($(($msg -replace '\s+', ' ').Trim()))"
}

function Test-Pace {
    "=== pace: controller 1 % slower than its nominal 5 ms per slice (the real one: 0.4 %), 10 s move"
    $dir = Join-Path $OutDir "pace"; $log = Join-Path $dir "fake.log"
    $s = Start-Sim "-f `"$log`"" $dir ($Port + 10) @{ GRBL2MACH_FAKE_SLICE_US = "5050" }
    try {
        $spm = Get-StepsPerMm $s
        Invoke-Line $s '$X' | Out-Null
        Invoke-GCode $s "G21 G91 G1 X100 F600"
        $mpos = Get-MPos (Wait-State $s "Idle" 60000)
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    $steps = [math]::Round(100 * $spm[0])
    "  stats: " + (($r.Stats.GetEnumerator() | Sort-Object Name | ForEach-Object { "$($_.Name)=$($_.Value)" }) -join " ")
    Check ($r.Stats.max_depth -le 20) "controller buffer kept at the bridge's lead: at most $($r.Stats.max_depth) slices (open loop: ~35 after 10 s)"
    Check ($r.Stats.starved -eq 0) "no underrun"
    Check ($r.Final[0] -eq $steps -and [math]::Round($mpos[0] * $spm[0]) -eq $steps) "every step run, grblHAL and controller agree ($($r.Final[0]) steps)"
    Check ($err -match "position checks after runs: [1-9]\d* ok, 0 mismatch") "position check after the run"
}

function Test-Long {
    "=== long: one 6-minute run (more than 65536 frames: the 16-bit sequence numbers wrap), controller 0.4 % slow"
    $dir = Join-Path $OutDir "long"; $log = Join-Path $dir "fake.log"
    $s = Start-Sim "-f `"$log`"" $dir ($Port + 12) @{ GRBL2MACH_FAKE_SLICE_US = "5020" }
    try {
        $spm = Get-StepsPerMm $s
        Invoke-Line $s '$X' | Out-Null
        Invoke-GCode $s "G21 G91 G1 F600"
        for ($i = 0; $i -lt 18; $i++) {
            Invoke-GCode $s "X100"
            Invoke-GCode $s "X-100"
        }
        $mpos = Get-MPos (Wait-State $s "Idle" 450000)
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    "  stats: " + (($r.Stats.GetEnumerator() | Sort-Object Name | ForEach-Object { "$($_.Name)=$($_.Value)" }) -join " ")
    Check ($r.Stats.frames -gt 66000) "the run went past a sequence wrap ($($r.Stats.frames) frames)"
    Check ($r.Stats.max_depth -le 20) "controller buffer kept at the bridge's lead all along: at most $($r.Stats.max_depth) slices"
    Check ($r.Stats.starved -eq 0) "no underrun"
    Check ($r.Final[0] -eq 0 -and [math]::Round($mpos[0] * $spm[0]) -eq 0) "back to the start, grblHAL and controller agree ($($r.Final[0]) steps)"
    Check ($err -match "position checks after runs: [1-9]\d* ok, 0 mismatch") "position check after the run"
}

function Test-Unplug([bool]$PowerCycle) {
    $what = if ($PowerCycle) { "powered up again when plugged back (counters at 0)" } else { "kept powered (counters kept)" }
    "=== unplug: USB unplugged 300 ms into a move, plugged back 1.5 s later; controller $what"
    $name = if ($PowerCycle) { "unplug_reset" } else { "unplug" }
    $dir = Join-Path $OutDir $name; $log = Join-Path $dir "fake.log"
    $envs = @{ GRBL2MACH_FAKE_UNPLUG_MS = "300,1800" }
    if ($PowerCycle) { $envs.GRBL2MACH_FAKE_UNPLUG_RESET = "1" }
    $s = Start-Sim "-f `"$log`"" $dir ($Port + 13 + [int]$PowerCycle) $envs
    try {
        $spm = Get-StepsPerMm $s
        Invoke-Line $s '$X' | Out-Null
        Send-Sim $s "G21 G91 G1 X20 F600`n"
        $out = ""; for ($i = 0; $i -lt 40 -and $out -notmatch "ALARM:17"; $i++) { Start-Sleep -Milliseconds 100; $out += Read-Sim $s }
        $lostAlarm = $out -match "ALARM:17"
        # plugged back at 1.8 s: the bridge reconnects (next attempt within 1 s) and resets the core
        $out2 = ""; for ($i = 0; $i -lt 50 -and $out2 -notmatch "reconnected"; $i++) { Start-Sleep -Milliseconds 100; $out2 += Read-Sim $s }
        Start-Sleep -Milliseconds 300; $out2 += Read-Sim $s
        $stateAfter = Wait-State $s "Alarm" 3000
        Invoke-Line $s '$X' | Out-Null
        $before = Get-MPos (Wait-State $s "Idle" 3000)
        Invoke-GCode $s "G21 G91 G1 X5 F600"
        $mpos = Get-MPos (Wait-State $s "Idle" 10000)
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    $moved = [math]::Round(($mpos[0] - $before[0]) * $spm[0])
    Check $lostAlarm "unplugged: grblHAL raised ALARM:17 (motor fault)"
    Check ($err -match "controller reconnected") "bridge reconnected to the controller once plugged back"
    Check ($out2 -match "MSG:.*reconnected") "user told: $(([regex]::Match($out2, '\[MSG:[^\]]*reconnected[^\]]*\]')).Value)"
    Check ($stateAfter -match "<Alarm") "after the reconnection: Alarm (ALARM:17 cleared by a reset, unlock or home to go on)"
    Check ($moved -eq [math]::Round(5 * $spm[0])) "move after unlocking: grblHAL moved 5 mm ($moved steps)"
    Check ([math]::Round($mpos[0] * $spm[0]) -eq $r.Final[0]) "grblHAL and controller agree after the reconnection (controller X=$($r.Final[0]))"
    Check ($r.Stats.seq_errors -eq 0 -and $r.Stats.rejected -eq 0) "no sequence error, no rejected slice"
}

function Test-UnplugIdle {
    "=== unplug at rest: USB unplugged with the machine idle, plugged back 1.5 s later"
    $dir = Join-Path $OutDir "unplug_idle"; $log = Join-Path $dir "fake.log"
    # times count from the first slice: a 0.1 s move, then unplugged 1 s later
    $s = Start-Sim "-f `"$log`"" $dir ($Port + 15) @{ GRBL2MACH_FAKE_UNPLUG_MS = "1000,2500" }
    try {
        Invoke-Line $s '$X' | Out-Null
        Invoke-GCode $s "G21 G91 G1 X1 F600"
        Wait-State $s "Idle" | Out-Null
        $out = ""; for ($i = 0; $i -lt 30 -and $out -notmatch "ALARM:17"; $i++) { Start-Sleep -Milliseconds 100; $out += Read-Sim $s }
        $out2 = ""; for ($i = 0; $i -lt 40 -and $out2 -notmatch "reconnected"; $i++) { Start-Sleep -Milliseconds 100; $out2 += Read-Sim $s }
        Invoke-Line $s '$X' | Out-Null
        $mpos = Get-MPos (Wait-State $s "Idle" 3000)
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    Check ($out -match "ALARM:17") "unplugged at rest: grblHAL raised ALARM:17 at once (no motion needed to notice)"
    Check ($out2 -match "MSG:.*reconnected") "reconnected and user told"
    Check ($null -ne $mpos -and $r.Final[0] -gt 0) "unlocked: Idle again (controller X=$($r.Final[0]))"
}

# Steps of the slices of the first run (op 0x04), up to its end of motion (op 0x06): the ramp.
function Get-FirstRunSlices($frames) {
    $slices = @()
    foreach ($f in $frames) {
        if ($f.op -eq 6 -and $slices.Count) { break }
        if ($f.op -eq 4 -and $f.d) { $slices += $f.d[0] }
    }
    return $slices
}

# The motion must end on a ramp: small last slices, no sudden drop in speed
function Check-Ramp($slices) {
    $last = @($slices | Select-Object -Last 60)
    $maxDrop = 0
    for ($i = 1; $i -lt $last.Count; $i++) { $maxDrop = [math]::Max($maxDrop, $last[$i - 1] - $last[$i]) }
    Check ($slices.Count -gt 100 -and ($slices | Measure-Object -Maximum).Maximum -ge 8) "the move was at speed ($($slices.Count) slices)"
    Check ($last[-1] -le 3 -and $maxDrop -le 2) "stopped on a ramp: last slice $($last[-1]) steps, largest drop between two slices $maxDrop steps"
}

function Test-ClientLost([bool]$Abort) {
    $how = if ($Abort) { "crash (connection reset)" } else { "clean disconnection" }
    "=== client lost during a move, spindle on: $how"
    $name = if ($Abort) { "client_reset" } else { "client_close" }
    $dir = Join-Path $OutDir $name; $log = Join-Path $dir "fake.log"; $p = $Port + 16 + [int]$Abort
    $s = Start-Sim "-f `"$log`"" $dir $p
    $n = $s
    try {
        $spm = Get-StepsPerMm $s
        Invoke-Line $s '$X' | Out-Null
        Invoke-GCode $s "M3 S1000; G21 G91 G1 X100 F600"
        Start-Sleep -Milliseconds 2000
        Close-Client $s $Abort
        Start-Sleep -Seconds 3
        $alive = -not $s.Process.HasExited
        $n = Connect-Sim $s.Process $p
        Start-Sleep -Milliseconds 400
        $greeting = Read-Sim $n
        $status = Wait-State $n "Idle" 3000
        $at = Get-MPos $status
        Start-Sleep -Milliseconds 1500
        $later = Get-MPos (Wait-State $n "Idle" 3000)
        Invoke-GCode $n "G21 G91 G1 X5 F600"
        $moved = Get-MPos (Wait-State $n "Idle" 10000)
    } finally { Stop-Sim $n }
    $r = Read-FakeLog $log
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    $off = Select-String -Path $log -Pattern "OUTPUTS 0x01 -> 0x00" | Select-Object -First 1
    Check $alive "the bridge survives"
    Check ($err -match "\[tcp\] client gone") "the loss is logged"
    Check-Ramp (Get-FirstRunSlices $r.Frames)
    Check ($null -ne $off) "spindle switched off"
    Check ($greeting -match "Grbl") "the new client gets the welcome message"
    Check ($status -match "<Idle") "next client finds the machine Idle, no alarm"
    Check ($at[0] -gt 5 -and $at[0] -lt 95 -and $at[0] -eq $later[0]) "the move does not resume: X stays at $($at[0]) mm"
    Check ([math]::Round(($moved[0] - $at[0]) * $spm[0]) -eq [math]::Round(5 * $spm[0])) "a new move from there: +5 mm"
    Check ([math]::Round($moved[0] * $spm[0]) -eq $r.Final[0]) "grblHAL and controller agree (controller X=$($r.Final[0]))"
    Check ($r.Stats.seq_errors -eq 0 -and $r.Stats.rejected -eq 0) "no sequence error, no rejected slice"
}

function Test-ClientLostIdle {
    "=== client lost at rest: nothing to do with nothing running; spindle on: switched off"
    $dir = Join-Path $OutDir "client_idle"; $log = Join-Path $dir "fake.log"; $p = $Port + 18
    $s = Start-Sim "-f `"$log`"" $dir $p
    $n = $s
    try {
        Invoke-Line $s '$X' | Out-Null
        Invoke-GCode $s "G91"
        Close-Client $s $false
        Start-Sleep -Seconds 1
        $n = Connect-Sim $s.Process $p
        Start-Sleep -Milliseconds 400
        Read-Sim $n | Out-Null
        $modal = Invoke-Line $n '$G'
        Invoke-GCode $n "M3 S500"
        Start-Sleep -Milliseconds 300
        Close-Client $n $false
        Start-Sleep -Seconds 1
        $n = Connect-Sim $s.Process $p
        Start-Sleep -Milliseconds 400
        Read-Sim $n | Out-Null
        $status = Wait-State $n "Idle" 3000
    } finally { Stop-Sim $n }
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    Check ($modal -match "G91") "no reset when nothing was running: the modal state stays (G91)"
    Check ((Select-String -Path $log -Pattern "OUTPUTS 0x01 -> 0x00" | Measure-Object).Count -ge 1) "spindle on at rest: switched off"
    Check ($status -match "<Idle") "next client finds the machine Idle"
    Check ($err -notmatch "feed hold") "no feed hold at rest"
}

function Test-CtrlC {
    "=== program closed (Ctrl-C) during a move, spindle on"
    $dir = Join-Path $OutDir "ctrl_c"; $log = Join-Path $dir "fake.log"
    $s = Start-Sim "-f `"$log`"" $dir ($Port + 19)
    try {
        Invoke-Line $s '$X' | Out-Null
        Invoke-GCode $s "M3 S1000; G21 G91 G1 X100 F600"
        Start-Sleep -Milliseconds 2000
        $helper = Join-Path $PSScriptRoot "send_ctrl_c.ps1"
        $h = Start-Process powershell -ArgumentList "-NoProfile -ExecutionPolicy Bypass -File `"$helper`" -TargetPid $($s.Process.Id)" -PassThru -Wait -WindowStyle Hidden
        $exited = $s.Process.WaitForExit(8000)
    } finally {
        if (-not $s.Process.HasExited) { Stop-Process $s.Process -Force }
        $s.Client.Close()
    }
    $r = Read-FakeLog $log
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    Check ($h.ExitCode -eq 0 -and $exited) "Ctrl-C ends the program"
    Check-Ramp (Get-FirstRunSlices $r.Frames)
    Check ((Select-String -Path $log -Pattern "OUTPUTS 0x01 -> 0x00" | Measure-Object).Count -ge 1) "spindle switched off"
    Check ($err -match "controller disarmed") "controller disarmed before the end"
    Check (@($r.Frames | Where-Object { $_.op -eq 1 }).Count -ge 1) "e-stop frame sent to the controller"
}

function Test-StopCommand {
    "=== stop command (0x19, gSender's Stop button) during a move, spindle on"
    $dir = Join-Path $OutDir "stop_cmd"; $log = Join-Path $dir "fake.log"
    $s = Start-Sim "-f `"$log`"" $dir ($Port + 20)
    try {
        $spm = Get-StepsPerMm $s
        Invoke-Line $s '$X' | Out-Null
        Invoke-GCode $s "M3 S1000; G21 G91 G1 X100 F600"
        Start-Sleep -Milliseconds 2000
        Read-Sim $s | Out-Null
        $t = [Diagnostics.Stopwatch]::StartNew()
        Send-Sim $s ([string][char]0x19)
        $status = Wait-State $s "Idle" 5000
        $ms = $t.ElapsedMilliseconds
        $at = Get-MPos $status
        Start-Sleep -Milliseconds 1500
        $out = Read-Sim $s
        $later = Get-MPos (Wait-State $s "Idle" 3000)
        Invoke-GCode $s "G21 G91 G1 X5 F600"
        $moved = Get-MPos (Wait-State $s "Idle" 10000)
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    Check ($ms -lt 3000) "back to Idle in $ms ms"
    Check-Ramp (Get-FirstRunSlices $r.Frames)
    Check ((Select-String -Path $log -Pattern "OUTPUTS 0x01 -> 0x00" | Measure-Object).Count -ge 1) "spindle switched off"
    Check ($out -notmatch "ALARM") "no alarm"
    Check ($at[0] -eq $later[0]) "the motion does not resume: X stays at $($at[0]) mm"
    Check ([math]::Round(($moved[0] - $at[0]) * $spm[0]) -eq [math]::Round(5 * $spm[0])) "a new move from there: +5 mm"
    Check ([math]::Round($moved[0] * $spm[0]) -eq $r.Final[0]) "grblHAL and controller agree (controller X=$($r.Final[0]))"
}

function Test-InputFull {
    "=== input buffer full: realtime commands still get through, waiting lines dropped by a reset"
    $dir = Join-Path $OutDir "input_full"; $log = Join-Path $dir "fake.log"
    $s = Start-Sim "-f `"$log`"" $dir ($Port + 21)
    try {
        Invoke-Line $s '$X' | Out-Null
        Invoke-GCode $s "G21 G91 F30"
        # about 100 blocks for the planner, then more than the input buffer holds
        $text = ""; for ($i = 0; $i -lt 400; $i++) { $text += "G1 X0.10$($i % 10) Y0.1`n" }
        Send-Sim $s $text
        Start-Sleep -Milliseconds 1500
        $slowest = 0
        for ($i = 0; $i -lt 12; $i++) {
            Read-Sim $s | Out-Null
            $t = [Diagnostics.Stopwatch]::StartNew(); Send-Sim $s "?"; $reply = ""
            while ($reply -notmatch "<\w+\|" -and $t.ElapsedMilliseconds -lt 5000) { $reply += Read-Sim $s }
            $slowest = [math]::Max($slowest, $t.ElapsedMilliseconds)
            Start-Sleep -Milliseconds 250
        }
        Send-Sim $s ([string][char]0x18)
        Start-Sleep -Milliseconds 800
        Read-Sim $s | Out-Null
        Invoke-Line $s '$X' | Out-Null
        $at = Get-MPos (Wait-State $s "Idle" 3000)
        Start-Sleep -Milliseconds 1500
        $later = Get-MPos (Wait-State $s "Idle" 3000)
    } finally { Stop-Sim $s }
    Check ($slowest -lt 1000) "status report while the input is full: $slowest ms at worst"
    Check ($at[0] -eq $later[0]) "after a reset, the lines that were waiting do not run (X stays at $($at[0]) mm)"
}

function Test-Jogs {
    "=== jogs: short Z moves one after the other (step jogs), each new run starting while the last one ends"
    $dir = Join-Path $OutDir "jogs"; $log = Join-Path $dir "fake.log"
    $s = Start-Sim "-f `"$log`"" $dir ($Port + 11)
    try {
        $spm = Get-StepsPerMm $s
        Invoke-Line $s '$X' | Out-Null
        $gaps = @(0, 20, 40, 60, 80, 100, 130, 160, 200, 250)
        foreach ($g in $gaps) {
            Invoke-GCode $s "G21 G91 G1 Z-0.5 F600"
            Wait-State $s "Idle" | Out-Null
            Start-Sleep -Milliseconds $g
        }
        $mpos = Get-MPos (Wait-State $s "Idle")
    } finally { Stop-Sim $s }
    $r = Read-FakeLog $log
    $err = Get-Content (Join-Path $dir "sim_stderr.txt") -Raw
    $expect = -[math]::Round(0.5 * $gaps.Count * $spm[2])
    "  stats: " + (($r.Stats.GetEnumerator() | Sort-Object Name | ForEach-Object { "$($_.Name)=$($_.Value)" }) -join " ")
    Check ($r.Stats.rejected -eq 0) "no slice rejected (a new run is opened only once the controller is ready again)"
    Check ($r.Final[2] -eq $expect -and [math]::Round($mpos[2] * $spm[2]) -eq $expect) "every jog run: controller Z = grblHAL Z = $expect steps (controller: $($r.Final[2]))"
    Check ($err -notmatch "POSITION MISMATCH") "no position mismatch ($(([regex]::Match($err, 'position checks[^;\r\n]*')).Value))"
}

if ($Scenario -in "motion", "all") { Test-Motion }
if ($Scenario -in "watchdog", "all") { Test-Watchdog }
if ($Scenario -in "reset", "all") { Test-Reset }
if ($Scenario -in "estop", "all") { Test-EStop $false; Test-EStop $true }
if ($Scenario -in "outputs", "all") { Test-Outputs }
if ($Scenario -in "pins", "all") { Test-Pins }
if ($Scenario -in "home", "all") { Test-Home }
if ($Scenario -in "probe", "all") { Test-Probe }
if ($Scenario -in "mach3", "all") { Test-Mach3 }
if ($Scenario -in "pace", "all") { Test-Pace }
if ($Scenario -in "jogs", "all") { Test-Jogs }
if ($Scenario -in "unplug", "all") { Test-Unplug $false; Test-Unplug $true; Test-UnplugIdle }
if ($Scenario -in "client", "all") { Test-ClientLost $false; Test-ClientLost $true; Test-ClientLostIdle; Test-CtrlC }
if ($Scenario -in "stop", "all") { Test-StopCommand; Test-InputFull }
if ($Scenario -eq "long") { Test-Long }     # 6 minutes: not part of "all"

if ($script:failures -eq 0) { "RESULT: PASS"; exit 0 } else { "RESULT: FAIL ($script:failures)"; exit 1 }
