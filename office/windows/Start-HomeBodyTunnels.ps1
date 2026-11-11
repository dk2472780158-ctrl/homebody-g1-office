param(
    [Parameter(Mandatory)][string]$Workstation,
    [Parameter(Mandatory)][string]$RobotHost,
    [string]$HostKeyAlias,
    [int]$RobotRosbridgePort=9090,
    [int]$RobotControlPort=8099
)
$ErrorActionPreference='Stop'
$stateDir=Join-Path $env:LOCALAPPDATA 'HomeBodyG1Office'
New-Item -ItemType Directory -Force -Path $stateDir | Out-Null
$common=@('-N','-T','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes',
    '-o','ExitOnForwardFailure=yes','-o','ServerAliveInterval=15','-o','ServerAliveCountMax=3')
if ($HostKeyAlias) {$common+=@('-o',"HostKeyAlias=$HostKeyAlias")}
$channels=@(
    @{name='control';args=@('-R',"127.0.0.1:18098:${RobotHost}:$RobotControlPort",'-R',"127.0.0.1:18099:${RobotHost}:$RobotControlPort")},
    @{name='cloud';args=@('-R',"127.0.0.1:19090:${RobotHost}:$RobotRosbridgePort")},
    @{name='feedback';args=@('-R',"127.0.0.1:19091:${RobotHost}:$RobotRosbridgePort")},
    @{name='pages';args=@('-L','127.0.0.1:8092:127.0.0.1:8092','-L','127.0.0.1:8093:127.0.0.1:8093')}
)
$launched=@()
try {
    foreach ($channel in $channels) {
        $pidPath=Join-Path $stateDir ($channel.name+'.pid')
        if (Test-Path -LiteralPath $pidPath) {
            $savedPid=[int](Get-Content -LiteralPath $pidPath -Raw)
            $previous=Get-CimInstance Win32_Process -Filter "ProcessId=$savedPid"
            if ($previous -and $previous.Name -eq 'ssh.exe') {throw "Tunnel $($channel.name) already recorded; stop the existing tunnels first."}
        }
        $process=Start-Process -FilePath 'ssh.exe' -ArgumentList ($common+$channel.args+@($Workstation)) `
            -WindowStyle Hidden -PassThru `
            -RedirectStandardOutput (Join-Path $stateDir ($channel.name+'.stdout.log')) `
            -RedirectStandardError (Join-Path $stateDir ($channel.name+'.stderr.log'))
        $launched+=$process
        Start-Sleep -Milliseconds 800
        $process.Refresh()
        if ($process.HasExited) {throw "Tunnel $($channel.name) failed. Inspect $stateDir."}
        $process.Id | Set-Content -LiteralPath $pidPath
    }
} catch {
    foreach ($process in $launched) {if (-not $process.HasExited) {Stop-Process -Id $process.Id}}
    throw
}
Write-Output 'Tunnels started. No robot commands were sent.'
