# build.ps1
# PowerShell 脚本：配置 → 编译 → 烧录 (一键完成)
# 用法: .\build.ps1

$Preset = "Debug"

# ---- 1. CMake 配置 ----
Write-Host "=== 1/3: CMake Configure ($Preset) ===" -ForegroundColor Cyan
cmake --preset $Preset
if ($LASTEXITCODE -ne 0) {
    Write-Host "CMake 配置失败！" -ForegroundColor Red
    exit 1
}

# ---- 2. CMake 编译 ----
Write-Host "=== 2/3: CMake Build ($Preset) ===" -ForegroundColor Cyan
cmake --build ".\build\$Preset"
if ($LASTEXITCODE -ne 0) {
    Write-Host "编译失败！" -ForegroundColor Red
    exit 1
}

# ---- 3. 烧录 ----
Write-Host "=== 3/3: Flash ===" -ForegroundColor Cyan
& .\fire.ps1
