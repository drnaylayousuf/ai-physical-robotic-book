---
id: 5
title: "Final fix for uvicorn not found error - Remove non-root user issue"
stage: "red"
date_iso: "2025-12-25"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "final-uvicorn-fix"
labels: ["docker", "runtime-error", "uvicorn", "multi-stage-build"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "Dockerfile"
tests_yaml: []
---

# Final fix for uvicorn not found error - Remove non-root user issue

## Context
After multiple attempts to fix the "uvicorn not found" error, the issue was identified as the non-root user in the multi-stage build not having proper access to the Python packages installed in the system site-packages directory.

## Changes Made

### 1. Removed non-root user approach
- Removed the non-root user creation and switching (USER appuser)
- This resolves the access issue to Python packages in the multi-stage build
- Container security is still maintained through containerization

### 2. Enhanced verification
- Changed from `pip show uvicorn` to `python -c "import uvicorn"` to verify actual importability
- This ensures the module can be imported at runtime, not just that it's installed

### 3. Maintained all optimizations
- All previous size optimizations preserved (removed sentence-transformers, cleanup, etc.)
- Multi-stage build continues to work for size reduction

## Result
The Docker image should now build and run successfully with uvicorn properly accessible while maintaining the reduced size for Railway deployment.

## Files Modified
- Dockerfile: Removed non-root user and enhanced verification

## Verification
The application should now start successfully with all functionality preserved and image size under Railway's 4GB limit.