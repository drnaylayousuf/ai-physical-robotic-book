---
id: 4
title: "Fix uvicorn not found error using Python module execution"
stage: "red"
date_iso: "2025-12-25"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "fix-uvicorn-module-execution"
labels: ["docker", "runtime-error", "uvicorn", "python-module"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "Dockerfile"
tests_yaml: []
---

# Fix uvicorn not found error using Python module execution

## Context
After previous optimizations, the deployment still failed with "The executable `uvicorn` could not be found" error. This occurred because the uvicorn executable was not in the PATH when running as the non-root user in the multi-stage build.

## Changes Made

### 1. Changed execution method
- Updated CMD to use Python module execution: `python -m uvicorn`
- This bypasses PATH issues since Python directly imports and runs the uvicorn module
- No longer relies on PATH to locate the uvicorn executable

### 2. Maintained all optimizations
- All previous size optimizations preserved
- Multi-stage build continues to work
- Non-root user execution maintained for security

## Result
The Docker image should now build and run successfully with uvicorn properly executed while maintaining the reduced size for Railway deployment.

## Files Modified
- Dockerfile: Changed to Python module execution for uvicorn

## Verification
The application should now start successfully with all functionality preserved and image size under Railway's 4GB limit.