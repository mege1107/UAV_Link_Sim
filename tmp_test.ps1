$path = 'D:\linksim\UAV_Link_Sim\UAV_Link_Sim\ofdm_link.cpp'
$content = Get-Content -Raw -Path $path
Write-Output ($content.Length)
