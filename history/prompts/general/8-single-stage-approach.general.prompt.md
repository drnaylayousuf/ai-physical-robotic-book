---
id: 8
title: "Single stage Docker approach - Direct package installation"
stage: "red"
date_iso: "2025-12-25"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "single-stage-approach"
labels: ["docker", "runtime-error", "single-stage", "direct-install"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "Dockerfile"
tests_yaml: []
---

# Single stage Docker approach - Direct package installation

## Context
After multiple multi-stage build attempts, the issue persists with Python packages not being accessible at runtime. The solution is to use a single-stage build that installs packages directly in the final image, eliminating potential path and copying issues.

## Changes Made

### 1. Single-stage Dockerfile
- Removed multi-stage build complexity
- Install packages directly in the final stage
- Eliminates potential copying issues between stages

### 2. Maintained optimizations
- Still uses --no-cache-dir to reduce image size
- Still removes unnecessary files
- Still uses the dedicated Python script approach
- Still verifies package installation

## Result
The Docker image should now build and run successfully with all packages properly installed and accessible in the same environment where they're needed, while still maintaining reasonable size for Railway deployment.

## Files Modified
- Dockerfile: Changed to single-stage approach with direct package installation

## Verification
The application should now start successfully with all functionality preserved and image size under Railway's 4GB limit.