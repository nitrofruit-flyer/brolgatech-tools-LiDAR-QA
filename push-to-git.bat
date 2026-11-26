@echo off
setlocal enabledelayedexpansion
chcp 65001 >nul
cd /d "%~dp0"
title Push to GitHub - brolgatech-tools-LiDAR-QA

set "REMOTE=https://github.com/nitrofruit-flyer/brolgatech-tools-LiDAR-QA.git"

rem Clean any stale git lock files (left behind by cloud-sync/sandbox tools)
if exist ".git\index.lock" del /f /q ".git\index.lock" >nul 2>&1
if exist ".git\HEAD.lock" del /f /q ".git\HEAD.lock" >nul 2>&1
if exist ".git\refs\heads\*.lock" del /f /q ".git\refs\heads\*.lock" >nul 2>&1
if exist ".git\refs\remotes\origin\*.lock" del /f /q ".git\refs\remotes\origin\*.lock" >nul 2>&1

echo ============================================
echo   LiDAR QA tools - push to GitHub
echo ============================================
echo.

rem ---------- First run: turn this folder into a git repo ----------
if not exist ".git" (
  echo First run - setting up git in this folder...
  git init -b main
  if errorlevel 1 goto initfail
  git remote add origin "%REMOTE%"
  git fetch origin
  rem If the GitHub repo was created with a README, bring it in first.
  git rev-parse --verify --quiet origin/main >nul 2>&1
  if not errorlevel 1 (
    git pull origin main --allow-unrelated-histories --no-edit
    if errorlevel 1 goto pullfail
  )
  git add -A
  set "msg=Initial import of the LiDAR QA tools"
  if exist "commit-message.txt" (
    git commit -F "commit-message.txt"
    if not errorlevel 1 del /f /q "commit-message.txt" >nul 2>&1
  ) else (
    git commit -m "!msg!"
  )
  git push -u origin main
  if errorlevel 1 goto pushfail
  goto done
)

echo Pulling latest from GitHub first...
git pull --rebase --autostash
if errorlevel 1 goto pullfail
echo.

set "dirty="
for /f "delims=" %%A in ('git status --porcelain') do set "dirty=1"
if not defined dirty goto clean

echo Changed files:
echo --------------------------------------------
git status --short
echo --------------------------------------------
echo.

rem Claude leaves commit-message.txt (whole file = message). It is consumed
rem here and never committed (it is in .gitignore).
if exist "commit-message.txt" (
  echo Commit message from commit-message.txt:
  echo --------------------------------------------
  type "commit-message.txt"
  echo.
  echo --------------------------------------------
  git add -A
  git commit -F "commit-message.txt"
  if errorlevel 1 goto commitfail
  del /f /q "commit-message.txt" >nul 2>&1
  goto push
)
set "msg="
set /p msg="Commit message - or press Enter for an auto message: "
if "!msg!"=="" set "msg=Update %date% %time:~0,5%"
git add -A
git commit -m "!msg!"
if errorlevel 1 goto commitfail

:push
git push
if errorlevel 1 goto pushfail
goto done

:clean
set "ahead="
for /f "delims=" %%A in ('git log --oneline @{u}..HEAD 2^>nul') do set "ahead=1"
if not defined ahead (
  echo Nothing to commit - working folder matches GitHub.
  pause
  exit /b 0
)
echo Nothing new to commit, but these commits have not been pushed yet:
git log --oneline @{u}..HEAD
git push
if errorlevel 1 goto pushfail
goto done

:done
echo.
echo ============================================
echo   Pushed to GitHub.
echo ============================================
pause
exit /b 0

:initfail
echo.
echo *** git init failed - is git installed? ***
pause
exit /b 1

:pullfail
echo.
echo *** Pull failed - usually a merge conflict. Fix the above before pushing. ***
pause
exit /b 1

:commitfail
echo.
echo *** Commit failed - see the message above. ***
pause
exit /b 1

:pushfail
echo.
echo *** Push failed. Check the repo exists on GitHub
echo     (%REMOTE%), your internet and GitHub sign-in, then run this again. ***
pause
exit /b 1
