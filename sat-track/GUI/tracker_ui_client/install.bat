@echo off
echo ========================================
echo Unified Satellite & LiDAR Tracker Setup
echo ========================================
echo.

echo Installing frontend dependencies...
npm install

echo.
echo ========================================
echo Installation Complete!
echo ========================================
echo.
echo To start the application:
echo 1. Start the backend: python main_unified.py --simulate
echo 2. Start the frontend: npm run dev
echo.
echo The application will be available at: http://localhost:3000
echo.
pause
