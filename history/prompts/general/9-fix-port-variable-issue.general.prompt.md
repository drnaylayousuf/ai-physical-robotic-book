---
id: 9
title: "Fix PORT environment variable expansion issue"
stage: "red"
date_iso: "2025-12-25"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "fix-port-variable-issue"
labels: ["docker", "environment-variables", "port", "runtime"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "backend/run_server.py"
tests_yaml: []
---

# Fix PORT environment variable expansion issue

## Context
After fixing the uvicorn installation issue, a new problem emerged where the `$PORT` environment variable was not being properly expanded when passed to uvicorn, resulting in the error "Error: Invalid value for '--port': '$PORT' is not a valid integer."

## Changes Made

### 1. Updated Python script to handle environment variables properly
- Modified `backend/run_server.py` to read PORT environment variable in Python
- Added proper integer conversion with error handling
- Added debugging output to show environment values
- Added fallback to port 8000 if PORT is invalid

### 2. Proper environment variable handling
- Read environment variables using `os.environ.get()`
- Convert string to integer with try/catch error handling
- Provide clear error messages for debugging

## Result
The application should now properly read the PORT environment variable and start on the correct port, while maintaining all other functionality.

## Files Modified
- backend/run_server.py: Updated to properly handle PORT environment variable

## Verification
The application should now start successfully on the correct port with all functionality preserved.