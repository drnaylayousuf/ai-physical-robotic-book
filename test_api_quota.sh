#!/bin/bash
# Test script to verify the API returns proper error codes for quota exceeded scenarios
echo "Testing API endpoint for quota exceeded error handling..."

# Test with a query that should trigger the quota exceeded error
curl -X POST "http://localhost:8000/api/ask" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2 fundamentals?",
    "mode": "full_book",
    "selected_text": null
  }' \
  -w "\nHTTP Status: %{http_code}\n" \
  -o response.json

echo -e "\nResponse saved to response.json"
cat response.json
echo -e "\n\nTest completed."