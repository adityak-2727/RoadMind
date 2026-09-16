param([switch]$SkipFreshStarts)
$ErrorActionPreference = 'Stop'
$projectDir = 'C:\Users\ADITYA\sih-autonomous-india'
$carlaExe = 'C:\Users\ADITYA\CARLA_0.9.16\CarlaUE4.exe'
$carlaShipping = 'C:\Users\ADITYA\CARLA_0.9.16\CarlaUE4\Binaries\Win64\CarlaUE4-Win64-Shipping.exe'
$matlabExe = 'C:\Program Files\MATLAB\R2026a\bin\matlab.exe'
Set-Location -LiteralPath $projectDir
$ledgerPath = Join-Path $projectDir 'results\phase15\fresh_start_processes.json'
$attempts = @()
if (-not $SkipFreshStarts) {
    foreach ($runIndex in 1..3) {
        $running = Get-CimInstance Win32_Process -Filter "Name='CarlaUE4-Win64-Shipping.exe'"
        foreach ($process in $running) {
            if ($process.ExecutablePath -ne $carlaShipping) {
                throw "Refusing to stop CARLA outside the explicitly configured installation: $($process.ExecutablePath)"
            }
            Stop-Process -Id $process.ProcessId
            Wait-Process -Id $process.ProcessId -ErrorAction SilentlyContinue
        }
        $server = Start-Process -FilePath $carlaExe -ArgumentList '-RenderOffScreen','-carla-server','-nosound' -WindowStyle Hidden -PassThru
        Start-Sleep -Seconds 35
        $started = Get-Date -Format o
        & $matlabExe -batch "addpath(genpath(pwd)); try; out=runCarlaIndianHeroDemo('fresh$runIndex',1000); disp(out); catch e; disp(getReport(e,'extended')); exit(1); end;"
        $attempts += [pscustomobject]@{run=$runIndex;launcherPid=$server.Id;started=$started;exitCode=$LASTEXITCODE}
        $attempts | ConvertTo-Json | Set-Content -LiteralPath $ledgerPath
    }
}
& $matlabExe -batch "addpath(genpath(pwd)); runPhase15Regression();"
