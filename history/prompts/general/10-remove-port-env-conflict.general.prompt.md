---
id: 10
title: "Remove PORT environment variable from Dockerfile to avoid conflicts"
stage: "red"
date_iso: "2025-12-25"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "remove-port-env-conflict"
labels: ["docker", "environment-variables", "port", "conflict"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "Dockerfile"
tests_yaml: []
---

# Remove PORT environment variable from Dockerfile to avoid conflicts

## Context
After implementing the Python script approach to handle the PORT environment variable, there was still an error indicating that `$PORT` was being passed directly to uvicorn. This suggests that having PORT explicitly set in the Dockerfile ENV might have been causing conflicts or the environment variable wasn't being handled properly by the Python script.

## Changes Made

### 1. Removed explicit PORT setting from Dockerfile
- Removed `ENV PORT=8000` from the Dockerfile
- This allows the Python script to properly read the PORT environment variable from the runtime environment (Railway)
- Eliminates potential conflicts between Dockerfile ENV and runtime environment

### 2. Maintained Python script approach
- The Python script (`backend/run_server.py`) continues to handle PORT reading properly
- Proper integer conversion and error handling maintained
- Fallback to default port maintained

## Result
The application should now properly read the PORT environment variable provided by Railway without conflicts, while maintaining all other functionality.

## Files Modified
- Dockerfile: Removed explicit PORT environment variable setting

## Verification
The application should now start successfully on the correct port assigned by Railway with all functionality preserved.