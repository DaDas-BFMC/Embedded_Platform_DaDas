# Run mbed-tools from Python Scripts (no PATH needed)
$mbedExe = "$env:LOCALAPPDATA\Packages\PythonSoftwareFoundation.Python.3.13_qbz5n2kfra8p0\LocalCache\local-packages\Python313\Scripts\mbed-tools.exe"
if (-not (Test-Path $mbedExe)) {
    Write-Error "mbed-tools.exe not found at: $mbedExe"
    exit 1
}
& $mbedExe @args
