---
id: 3
title: "Fix uvicorn not found error in optimized Dockerfile"
stage: "red"
date_iso: "2025-12-25"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "fix-uvicorn-error"
labels: ["docker", "optimization", "runtime-error", "uvicorn"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "Dockerfile"
tests_yaml: []
---

# Fix uvicorn not found error in optimized Dockerfile

## Context
After optimizing the Dockerfile to reduce image size, the deployment failed with "The executable `uvicorn` could not be found" error. This occurred because the multi-stage build process may not have properly copied all necessary dependencies.

## Changes Made

### 1. Enhanced build process
- Added gcc to build dependencies for proper compilation
- Added verification step (`pip show uvicorn`) to ensure uvicorn is installed
- Upgraded setuptools to ensure proper package installation

### 2. Fixed runtime command
- Changed from direct uvicorn command to shell command to ensure PATH is properly set
- Updated CMD to use `sh -c` to execute uvicorn through the shell

## Result
The Docker image should now build successfully with uvicorn available for execution while maintaining the reduced size for Railway deployment.

## Files Modified
- Dockerfile: Enhanced build process and fixed runtime command

## Verification
The application should now start successfully while maintaining all functionality and reduced image size.