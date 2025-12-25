---
id: 13
title: "Railway configuration to override auto-detected start command"
stage: "red"
date_iso: "2025-12-25"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "railway-config-fix"
labels: ["railway", "configuration", "start-command", "auto-detection"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "railway.toml"
tests_yaml: []
---

# Railway configuration to override auto-detected start command

## Context
Despite all Dockerfile modifications, Railway continued to show the error "Error: Invalid value for '--port': '$PORT' is not a valid integer." This occurred because Railway auto-detected the Python/FastAPI project and used its own default start command (likely something like `uvicorn backend.main:app --port $PORT`) instead of respecting the Dockerfile CMD instruction.

## Changes Made

### 1. Created Railway configuration file
- Created `railway.toml` configuration file
- Explicitly set the builder to "dockerfile"
- Explicitly specified the start command as "python start_app.py"
- This overrides Railway's auto-detection behavior

### 2. Ensured compatibility with existing solution
- Uses the direct Python startup script already created
- Maintains all previous optimizations and fixes
- Proper environment variable handling preserved

## Result
Railway will now use the explicitly defined start command instead of its auto-detected default, allowing our properly configured Python script to handle the application startup.

## Files Modified
- railway.toml: Created Railway configuration file

## Verification
Railway should now use the specified start command and avoid the default uvicorn command that was causing the $PORT parsing issue.