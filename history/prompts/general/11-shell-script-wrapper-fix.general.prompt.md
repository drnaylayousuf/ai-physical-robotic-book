---
id: 11
title: "Shell script wrapper to handle PORT environment variable"
stage: "red"
date_iso: "2025-12-25"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "shell-script-wrapper-fix"
labels: ["docker", "railway", "environment-variables", "port", "shell-script"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "Dockerfile"
  - "start_server.sh"
tests_yaml: []
---

# Shell script wrapper to handle PORT environment variable

## Context
Despite multiple attempts, Railway was still showing the error "Error: Invalid value for '--port': '$PORT' is not a valid integer." This indicates that Railway might be overriding the Dockerfile CMD or there's an issue with how environment variables are being processed. The solution is to use a shell script wrapper that properly handles environment variable substitution before executing the Python application.

## Changes Made

### 1. Created shell script wrapper
- Created `start_server.sh` shell script to handle environment variables
- Added proper PORT validation and default value handling
- Uses shell variable substitution to resolve $PORT before any processing

### 2. Updated Dockerfile to use shell script
- Changed CMD to execute the shell script: `["sh", "start_server.sh"]`
- Shell script handles proper environment variable processing
- Added shell script to the container

### 3. Maintained all previous fixes
- Python script approach still used for main application execution
- All optimizations preserved
- Proper error handling maintained

## Result
The shell script will properly resolve the PORT environment variable before executing the Python application, preventing the "$PORT" string literal from being passed to uvicorn.

## Files Modified
- Dockerfile: Updated to use shell script wrapper
- start_server.sh: Created shell script for environment handling

## Verification
The application should now properly resolve the PORT environment variable and start successfully on Railway.