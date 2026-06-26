# fire.ps1
# Usage: .\fire.ps1

$ProjectDir = (Get-Location).Path
$ProjectName = Split-Path -Leaf $ProjectDir
$Firmware = "build/Debug/$ProjectName.bin"

Write-Host "Flashing $Firmware ..."

if (-not (Test-Path $Firmware)) {
    Write-Host "ERROR: $Firmware not found. Build first!" -ForegroundColor Red
    exit 1
}

taskkill /f /im openocd.exe 2>$null
Start-Sleep -Seconds 1

$retry = 0
while ($retry -lt 3) {
    $retry++
    Write-Host "Attempt $retry/3 ..."

    & openocd.exe -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "transport select swd" -c "adapter speed 2000" -c "init" -c "reset halt" -c "flash write_image erase $Firmware 0x08000000" -c "verify_image $Firmware 0x08000000" -c "reset run" -c "shutdown"

    if ($LASTEXITCODE -eq 0) {
        Write-Host "Success!" -ForegroundColor Green
        exit 0
    }

    Write-Host "Failed, retrying..." -ForegroundColor Yellow
    taskkill /f /im openocd.exe 2>$null
    Start-Sleep -Seconds 3
}

Write-Host "All attempts failed!" -ForegroundColor Red
exit 1
