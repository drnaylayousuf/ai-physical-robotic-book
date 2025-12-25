---
id: 12
title: "Direct Python startup script to bypass command-line issues"
stage: "red"
date_iso: "2025-12-25"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "direct-python-startup-fix"
labels: ["docker", "railway", "environment-variables", "startup", "command-line"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "Dockerfile"
  - "start_app.py"
tests_yaml: []
---

# Direct Python startup script to bypass command-line issues

## Context
Despite multiple attempts, Railway continued to show the error "Error: Invalid value for '--port': '$PORT' is not a valid integer." This indicates that uvicorn was still being called with the literal string "$PORT" instead of the resolved value. The issue might be that some component is still parsing command-line arguments that contain "$PORT" as a literal string. The solution is to use a direct Python script that bypasses all command-line argument processing.

## Changes Made

### 1. Created direct Python startup script
- Created `start_app.py` that directly imports and runs the application
- No command-line arguments are passed to uvicorn
- Environment variables are read and processed directly in Python
- Proper error handling and logging included

### 2. Updated Dockerfile to use direct execution
- Changed CMD to execute the direct Python script: `["python", "start_app.py"]`
- Eliminates all command-line argument processing
- Bypasses any potential shell/command parsing issues

### 3. Maintained all previous fixes
- All optimizations preserved
- Proper environment variable handling maintained
- Error fallbacks maintained

## Result
The direct Python execution bypasses all command-line argument processing that might be causing the "$PORT" literal string issue, ensuring proper environment variable resolution.

## Files Modified
- Dockerfile: Updated to use direct Python execution
- start_app.py: Created direct startup script

## Verification
The application should now start successfully without any command-line argument parsing issues.