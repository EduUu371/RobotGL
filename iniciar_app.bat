@echo off
setlocal
cd /d "%~dp0"

echo [1/3] Verificando Python...
py -3 --version >nul 2>&1
if %errorlevel%==0 (
    set "PYTHON_CMD=py -3"
) else (
    python --version >nul 2>&1
    if errorlevel 1 (
        echo Python nao encontrado. Instale Python 3.12 ou superior e tente novamente.
        pause
        exit /b 1
    )
    set "PYTHON_CMD=python"
)

echo [2/3] Instalando dependencias...
%PYTHON_CMD% -m pip install -r requirements.txt
if errorlevel 1 (
    echo Falha ao instalar dependencias.
    pause
    exit /b 1
)

echo [3/3] Iniciando simulador...
%PYTHON_CMD% main.py

if errorlevel 1 (
    echo O aplicativo terminou com erro.
    pause
    exit /b 1
)

endlocal
