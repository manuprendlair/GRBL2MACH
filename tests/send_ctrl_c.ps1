# send_ctrl_c.ps1 - sends Ctrl-C to the console of another process (GRBL2MACH tests).
# Run it in a process of its own: it leaves its own console to attach to the target's.

param([Parameter(Mandatory = $true)] [int]$TargetPid)

Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
public static class ConsoleCtl {
    [DllImport("kernel32.dll")] public static extern bool FreeConsole();
    [DllImport("kernel32.dll")] public static extern bool AttachConsole(uint pid);
    [DllImport("kernel32.dll")] public static extern bool SetConsoleCtrlHandler(IntPtr handler, bool add);
    [DllImport("kernel32.dll")] public static extern bool GenerateConsoleCtrlEvent(uint evt, uint group);
}
"@

[void][ConsoleCtl]::FreeConsole()
if (-not [ConsoleCtl]::AttachConsole($TargetPid)) { exit 2 }
[void][ConsoleCtl]::SetConsoleCtrlHandler([IntPtr]::Zero, $true)    # this script ignores the event
if (-not [ConsoleCtl]::GenerateConsoleCtrlEvent(0, 0)) { exit 3 }
Start-Sleep -Milliseconds 500
