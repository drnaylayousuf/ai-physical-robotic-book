# Multiple Gemini Model Quota Management

## Overview
This implementation enables your application to use multiple Gemini models with automatic rotation to manage quota limitations. When one model hits its daily quota limit, the system automatically switches to the next available model, effectively multiplying your total request capacity.

## Configuration

### Environment Variables
The following environment variables have been added to your `.env` file:

```env
GEMINI_MODELS=gemini-2.5-flash
```

### Model List
The system is currently configured with this model:
1. `gemini-2.5-flash` (primary model)

Multiple models can be configured for rotation, but currently using single model for stability.

## How It Works

### 1. Model Initialization
- All configured models are initialized at startup using the same API key
- The system maintains a list of available model instances
- The first model in the list is used as the primary model

### 2. Automatic Rotation
- When a quota exceeded error (429) is detected, the system automatically rotates to the next model
- Rotation happens seamlessly without interrupting the user experience
- Models rotate in a circular fashion (when the last model is reached, it cycles back to the first)

### 3. Error Handling
- If all models have reached their quota limits, the system returns a graceful fallback message
- The system preserves all existing functionality for non-quota-related errors

## Benefits

1. **Increased Quota Capacity**: Each model has its own daily quota, effectively tripling your request capacity
2. **Automatic Failover**: No manual intervention required when quotas are exceeded
3. **Seamless Operation**: Users experience minimal disruption when model switching occurs
4. **Single API Key**: All models use the same API key, simplifying configuration

## Model Rotation Sequence
The system rotates through models if multiple models are configured:
`gemini-2.5-flash` (single model - no rotation)

Currently configured with single model for stability: `gemini-2.5-flash`

## Fallback Behavior
If all configured models reach their quota limits, the system returns:
> "I've reached my response limit for now. Please try again later, or try rephrasing your question."

## Files Modified
- `backend/config/settings.py` - Added GEMINI_MODELS configuration
- `backend/models/rag.py` - Implemented multiple model support and rotation logic
- `backend/api/chat.py` - Updated exception handling to preserve HTTP status codes
- `.env` - Added GEMINI_MODELS environment variable

## Testing
A test script (`test_multi_model.py`) is provided to verify the configuration is working correctly.

## Usage
No changes are required to your existing code or API calls. The quota management happens automatically in the background.