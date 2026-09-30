$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path -replace '\\', '/'
$uv = Get-Content (Join-Path $root "MDK-ARM/fY_270.uvprojx") -Raw -Encoding UTF8
$matches = [regex]::Matches($uv, '<FilePath>([^<]+\.c)</FilePath>')
$inc = @(
    '-ICore/Inc',
    '-IDrivers/STM32F4xx_HAL_Driver/Inc',
    '-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy',
    '-IDrivers/CMSIS/Device/ST/STM32F4xx/Include',
    '-IDrivers/CMSIS/Include',
    '-IUserFiles',
    '-IUserFiles/APP/app',
    '-IUserFiles/APP/servo',
    '-ICore/Src/servo'
)
$defs = @('-std=c11', '--target=arm-none-eabi', '-DUSE_HAL_DRIVER', '-DSTM32F405xx')
$entries = New-Object System.Collections.Generic.List[object]
$seen = @{}
foreach ($m in $matches) {
    $p = $m.Groups[1].Value -replace '\\', '/'
    if ($p.StartsWith('../')) { $rel = $p.Substring(3) } else { $rel = $p }
    if ($seen.ContainsKey($rel)) { continue }
    $seen[$rel] = $true
    $full = (Join-Path $root $rel) -replace '\\', '/'
    if (-not (Test-Path $full)) { continue }
    $clangArgs = @('clang', '-c') + $defs + $inc + @($rel)
    $entries.Add([ordered]@{
            directory  = $root
            file       = $full
            arguments  = $clangArgs
        })
}
$out = Join-Path $root "compile_commands.json"
$entries | ConvertTo-Json -Depth 6 | Set-Content -Path $out -Encoding UTF8
Write-Host "Wrote $($entries.Count) entries to $out"
