param (
  [string]$InstallPrefix
)

$env:PATH = ""
$env:CMAKE_PREFIX_PATH = ""

$setup_file = "$InstallPrefix/share/ament_cmake_vendor_package_test/local_setup.ps1"
if (-not (Test-Path $setup_file)) {
  Write-Error "Could not find local_setup.ps1 at $setup_file"
  exit 1
}

# Source the setup file
. $setup_file

Write-Host "PATH: $env:PATH"
Write-Host "CMAKE_PREFIX_PATH: $env:CMAKE_PREFIX_PATH"

if (-not "$env:PATH".Contains("ament_cmake_vendor_package_test")) {
  Write-Error "PATH was not updated by local_setup.ps1"
  exit 1
}

if (-not "$env:CMAKE_PREFIX_PATH".Contains("ament_cmake_vendor_package_test")) {
  Write-Error "CMAKE_PREFIX_PATH was not updated by local_setup.ps1"
  exit 1
}

Write-Host "PowerShell local_setup.ps1 verified successfully!"
exit 0
