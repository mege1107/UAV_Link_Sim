$ErrorActionPreference = "Continue"
$PSNativeCommandUseErrorActionPreference = $false

$Root = "D:\linksim\UAV_Link_Sim"
$LogDir = Join-Path $Root "logs"
$BuildDir = Join-Path $Root "x64\Release"
$Exe = Join-Path $BuildDir "UAV_Link_Sim.exe"
$Solution = Join-Path $Root "UAV_Link_Sim.sln"

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

$DateTag = Get-Date -Format "yyyyMMdd_HHmmss"

# ===== 可改参数 =====
$Frames = 200
$Fc = 2.45e9
$SnrList = @(-5, 0, 5, 10, 15, 20)
$ModList = @("bpsk", "qpsk", "qam", "fsk", "ook", "msk")
# ===================

Write-Host "======================================"
Write-Host "Building solution..."
Write-Host "Solution: $Solution"
Write-Host "======================================"

msbuild $Solution /t:Build /p:Configuration=Debug /p:Platform=x64

if (-not (Test-Path $Exe)) {
    throw "Executable not found: $Exe"
}

Write-Host "Using executable: $Exe"
Get-Item $Exe | Format-List FullName, LastWriteTime, Length

Write-Host "======================================"
Write-Host "Starting full test sweep..."
Write-Host "Logs directory: $LogDir"
Write-Host "Frames per test: $Frames"
Write-Host "Center frequency: $Fc"
Write-Host "Modulations: $($ModList -join ', ')"
Write-Host "SNR list: $($SnrList -join ', ')"
Write-Host "======================================"

foreach ($mod in $ModList) {
    Write-Host ""
    Write-Host "######################################"
    Write-Host "Testing modulation: $mod"
    Write-Host "######################################"

    foreach ($snr in $SnrList) {
        $LogFile = Join-Path $LogDir "${DateTag}_AWGN_${mod}_SNR${snr}dB_${Frames}frames.log"

        Write-Host "--------------------------------------"
        Write-Host "Running AWGN | MOD=$mod | SNR=$snr dB"
        Write-Host "Log: $LogFile"
        Write-Host "--------------------------------------"

        & $Exe --mode awgn --snr $snr --frames $Frames --mod $mod --fc $Fc 2>&1 |
            Tee-Object -FilePath $LogFile
    }

    $UsrpLog = Join-Path $LogDir "${DateTag}_USRP_${mod}_${Frames}frames.log"

    Write-Host "--------------------------------------"
    Write-Host "Running USRP | MOD=$mod"
    Write-Host "Log: $UsrpLog"
    Write-Host "--------------------------------------"

    & $Exe --mode usrp --frames $Frames --mod $mod --fc $Fc 2>&1 |
        Tee-Object -FilePath $UsrpLog
}

Write-Host ""
Write-Host "======================================"
Write-Host "All tests finished."
Write-Host "Logs saved in: $LogDir"
Write-Host "======================================"