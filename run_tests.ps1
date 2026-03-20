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

# 所有调制（加入 FM）
$ModList = @("bpsk", "qpsk", "qam", "fsk", "ook", "msk", "fm")

# 所有允许速率（按你的程序实际支持情况修改）
# 这里先按常见档位给一版
$RateList = @(
    4e3,
    8e3,
    16e3,
    32e3,
    64e3,
    128e3,
    200e3
)

# AWGN 扫描范围
$SnrList = @(-15, -10, -5, 0, 5, 10, 15, 20)

# 如果你的程序命令行参数里速率不是 --rb，而是别的名字，
# 这里改成实际参数名，比如 --bitrate 或 --rate
$RateArgName = "--rb"
# ===================

Write-Host "======================================"
Write-Host "Building solution..."
Write-Host "Solution: $Solution"
Write-Host "======================================"

msbuild $Solution /t:Build /p:Configuration=Release /p:Platform=x64

if (-not (Test-Path $Exe)) {
    throw "Executable not found: $Exe"
}

Write-Host "Using executable: $Exe"
Get-Item $Exe | Format-List FullName, LastWriteTime, Length

Write-Host "======================================"
Write-Host "Starting AWGN full sweep..."
Write-Host "Logs directory: $LogDir"
Write-Host "Frames per test: $Frames"
Write-Host "Center frequency: $Fc"
Write-Host "Modulations: $($ModList -join ', ')"
Write-Host "Rates: $($RateList -join ', ')"
Write-Host "SNR list: $($SnrList -join ', ')"
Write-Host "======================================"

foreach ($mod in $ModList) {
    Write-Host ""
    Write-Host "######################################"
    Write-Host "Testing modulation: $mod"
    Write-Host "######################################"

    foreach ($rate in $RateList) {
        Write-Host ""
        Write-Host "======================================"
        Write-Host "Testing rate: $rate bps"
        Write-Host "======================================"

        foreach ($snr in $SnrList) {
            $rateText = [string]([int]$rate)
            $snrText = if ($snr -lt 0) { "m$([math]::Abs($snr))" } else { "$snr" }

            $LogFile = Join-Path $LogDir "${DateTag}_AWGN_${mod}_Rb${rateText}_SNR${snrText}dB_${Frames}frames.log"

            Write-Host "--------------------------------------"
            Write-Host "Running AWGN | MOD=$mod | Rb=$rate | SNR=$snr dB"
            Write-Host "Log: $LogFile"
            Write-Host "--------------------------------------"

            & $Exe `
                --mode awgn `
                --frames $Frames `
                --mod $mod `
                $RateArgName $rate `
                --snr $snr `
                --fc $Fc 2>&1 | Tee-Object -FilePath $LogFile
        }
    }
}

Write-Host ""
Write-Host "======================================"
Write-Host "All AWGN sweeps finished."
Write-Host "Logs saved in: $LogDir"
Write-Host "======================================"