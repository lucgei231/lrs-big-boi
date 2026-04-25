@echo off
setlocal

set REPO_DIR=%~dp0
set TOOLS_DIR=%REPO_DIR%tools
set CLI_EXE=%TOOLS_DIR%\arduino-cli.exe
set ZIP_TEMP=%REPO_DIR%\arduino-cli.zip
set FQBN=esp32:esp32:esp32
set SKETCH=hw870\hw870.ino
set ADDITIONAL_URLS=https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

echo Repo dir: %REPO_DIR%
pushd "%REPO_DIR%"

if exist "%CLI_EXE%" (
  echo Found arduino-cli at "%CLI_EXE%"
) else (
  goto :download_cli
)

goto :after_download

:download_cli
echo Downloading arduino-cli
powershell -NoProfile -Command " $r=Invoke-RestMethod -Uri 'https://api.github.com/repos/arduino/arduino-cli/releases/latest'; $asset=$r.assets | Where-Object { $_.name -match 'Windows' -and $_.name -match '64' -and $_.name -match '\.zip$' } | Select-Object -First 1; if(-not $asset){ Write-Error 'No suitable arduino-cli Windows asset found'; exit 2 }; Invoke-WebRequest -Uri $asset.browser_download_url -OutFile '%ZIP_TEMP%'; "
if errorlevel 1 (
  echo Download failed
  pause
)
if not exist "%TOOLS_DIR%" mkdir "%TOOLS_DIR%"
echo Extracting archive...
powershell -NoProfile -Command "Expand-Archive -Path '%ZIP_TEMP%' -DestinationPath '%TOOLS_DIR%' -Force"
if errorlevel 1 (
  echo Extraction failed
  pause
)
del "%ZIP_TEMP%" 2>nul

:after_download

set PATH=%TOOLS_DIR%;%PATH%

echo Updating Arduino CLI core index...
arduino-cli core update-index
if errorlevel 1 (
  echo core update-index failed
  pause
)

echo Installing ESP32 core (this may take a few minutes)...
arduino-cli core install esp32:esp32 --additional-urls "%ADDITIONAL_URLS%"
if errorlevel 1 (
  echo core install failed
  pause
)

echo Compiling %SKETCH% with FQBN %FQBN%...
arduino-cli compile --fqbn "%FQBN%" "%SKETCH%" --build-path ./build
if errorlevel 1 (
  echo compile failed
  pause
)

echo Listing build artifacts under hw870\build (if present):
if exist "hw870\build" (
  dir /s /b "hw870\build\*.bin" "hw870\build\*.elf" "hw870\build\*.hex" 2>nul || echo No artifacts found
) else (
  echo No build directory found
)

echo Done.
popd
endlocal
pause
