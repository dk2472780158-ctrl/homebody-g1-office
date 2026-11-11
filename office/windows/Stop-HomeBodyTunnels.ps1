$ErrorActionPreference='Stop'
$stateDir=Join-Path $env:LOCALAPPDATA 'HomeBodyG1Office'
if (-not (Test-Path -LiteralPath $stateDir)) {return}
foreach ($name in @('control','cloud','feedback','pages')) {
    $path=Join-Path $stateDir ($name+'.pid')
    if (-not (Test-Path -LiteralPath $path)) {continue}
    $savedPid=[int](Get-Content -LiteralPath $path -Raw)
    $process=Get-CimInstance Win32_Process -Filter "ProcessId=$savedPid"
    if ($process -and $process.Name -eq 'ssh.exe' -and $process.CommandLine -like '*ServerAliveInterval=15*' -and
        ($process.CommandLine -like '*127.0.0.1:18098:*' -or $process.CommandLine -like '*127.0.0.1:19090:*' -or
         $process.CommandLine -like '*127.0.0.1:19091:*' -or $process.CommandLine -like '*127.0.0.1:8092:*')) {
        Stop-Process -Id $savedPid
        Remove-Item -LiteralPath $path
    }
}
Write-Output 'Recorded tunnels stopped. This script does not send a robot stop command.'
