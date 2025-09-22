@echo off
echo ================================
echo  SISTEMA DE CONTROL DE MOTORES
echo ================================
echo.
echo Selecciona una opcion:
echo.
echo 1. Compilar y subir firmware
echo 2. Ejecutar interfaz grafica
echo 3. Test de conexion
echo 4. Diagnostico completo
echo 5. Salir
echo.
set /p choice="Ingresa tu opcion (1-5): "

if "%choice%"=="1" goto firmware
if "%choice%"=="2" goto gui
if "%choice%"=="3" goto test
if "%choice%"=="4" goto diagnostic
if "%choice%"=="5" goto exit
goto invalid

:firmware
echo.
echo Compilando y subiendo firmware...
pio run --target upload
pause
goto menu

:gui
echo.
echo Ejecutando interfaz grafica...
cd gui
python controller.py
cd ..
pause
goto menu

:test
echo.
echo Ejecutando test de conexion...
python test_connection.py
pause
goto menu

:diagnostic
echo.
echo Ejecutando diagnostico completo...
python diagnostic_speed.py
pause
goto menu

:invalid
echo.
echo Opcion invalida. Intenta de nuevo.
pause

:menu
cls
goto start

:exit
echo.
echo Saliendo...
exit

:start
cls
goto :EOF