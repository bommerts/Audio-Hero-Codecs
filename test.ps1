# Builds the codec and runs every test.
#   .\test.ps1          release build and tests (ffmpeg on the PATH adds the reference checks)
#   .\test.ps1 -Asan    the same under AddressSanitizer (memory-error checker)
param([switch]$Asan)
$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot

function Invoke-Checked([string]$what, [scriptblock]$run) {
    & $run
    if ($LASTEXITCODE -ne 0) { throw "$what failed (exit $LASTEXITCODE)" }
}

if ($Asan) {
    # AddressSanitizer ships with the 14.44 toolset on this machine, so that
    # toolset's environment is used with NMake.
    $vs = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath
    $toolset = (Get-ChildItem "$vs\VC\Tools\MSVC" -Directory |
        Where-Object { Test-Path "$($_.FullName)\bin\Hostx64\x64\clang_rt.asan_dynamic-x86_64.dll" } |
        Sort-Object Name | Select-Object -Last 1)
    if (-not $toolset) { throw 'No MSVC toolset with AddressSanitizer is installed.' }
    $ver = ($toolset.Name -split '\.')[0..1] -join '.'
    $build = "$root\build-asan"
    Invoke-Checked 'asan build' {
        cmd /c "`"$vs\Common7\Tools\VsDevCmd.bat`" -arch=x64 -host_arch=x64 -vcvars_ver=$ver >nul 2>&1 && cmake -S `"$root`" -B `"$build`" -G `"NMake Makefiles`" -DCMAKE_BUILD_TYPE=RelWithDebInfo -DAHMP3_ASAN=ON >nul && cmake --build `"$build`""
    }
    $env:PATH = "$($toolset.FullName)\bin\Hostx64\x64;$env:PATH"
    $bin = $build
    Remove-Item "$build\asan-report*" -ErrorAction SilentlyContinue
    Push-Location $root
    $env:ASAN_OPTIONS = 'log_path=build-asan\asan-report'
} else {
    $build = "$root\build"
    Invoke-Checked 'configure' { cmake -S $root -B $build -A x64 | Out-Null }
    Invoke-Checked 'build' { cmake --build $build --config Release -- /v:m /nologo }
    $bin = "$build\Release"
    Push-Location $root
}

try {
    Remove-Item "$build\test-data" -Recurse -Force -ErrorAction SilentlyContinue
    New-Item -ItemType Directory -Force "$build\test-data" | Out-Null
    Invoke-Checked 'tests' { & "$bin\ahmp3_tests.exe" "$build\test-data" }
    if ($Asan) {
        $reports = @(Get-ChildItem "$build\asan-report*" -ErrorAction SilentlyContinue)
        if ($reports) { Get-Content $reports[0].FullName -TotalCount 60; throw 'AddressSanitizer reported a memory error.' }
        'AddressSanitizer: no memory errors.'
    }
    'All tests passed.'
} finally {
    Pop-Location
    Remove-Item Env:\ASAN_OPTIONS -ErrorAction SilentlyContinue
}
