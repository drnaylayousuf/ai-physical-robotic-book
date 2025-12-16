#!/bin/bash

# cURL examples for RAG Chatbot API testing

# Configuration
BASE_URL="http://localhost:8000"
TOKEN=""  # Will be set after login

echo "=== RAG Chatbot API cURL Examples ==="

# 1. Health Check
echo -e "\n1. Health Check:"
curl -X GET "$BASE_URL/health" \
  -H "Content-Type: application/json" \
  | jq '.'

# 2. Get Book Metadata
echo -e "\n2. Get Book Metadata:"
curl -X GET "$BASE_URL/metadata" \
  -H "Content-Type: application/json" \
  | jq '.'

# 3. User Registration
echo -e "\n3. User Registration:"
curl -X POST "$BASE_URL/auth/register" \
  -H "Content-Type: application/json" \
  -d '{
    "username": "testuser",
    "email": "test@example.com",
    "password": "securepassword123"
  }' | jq '.'

# 4. User Login
echo -e "\n4. User Login:"
LOGIN_RESPONSE=$(curl -X POST "$BASE_URL/auth/login" \
  -H "Content-Type: application/json" \
  -d '{
    "username": "testuser",
    "password": "securepassword123"
  }')

TOKEN=$(echo $LOGIN_RESPONSE | jq -r '.access_token')
echo "Access token retrieved: $TOKEN"

# 5. Get User Profile (with authentication)
echo -e "\n5. Get User Profile:"
curl -X GET "$BASE_URL/auth/profile" \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  | jq '.'

# 6. Ask a question (Full Book Mode)
echo -e "\n6. Ask a question (Full Book Mode):"
curl -X POST "$BASE_URL/ask" \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is Physical AI?",
    "mode": "full_book"
  }' | jq '.'

# 7. Ask a question (Selected Text Mode)
echo -e "\n7. Ask a question (Selected Text Mode):"
curl -X POST "$BASE_URL/ask" \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What does this text explain?",
    "mode": "selected_text",
    "selected_text": "Physical AI refers to artificial intelligence systems that interact with the physical world through sensors and actuators."
  }' | jq '.'

# 8. Admin Dashboard (requires admin token)
echo -e "\n8. Admin Dashboard (requires admin token):"
curl -X GET "$BASE_URL/admin" \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  | jq '.'

# 9. Ingest Content (Admin Only - would require admin token in real scenario)
echo -e "\n9. Ingest Content (Admin Only):"
curl -X POST "$BASE_URL/ingest" \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "source_path": "./doc",
    "collection_name": "book_chunks"
  }' | jq '.'

echo -e "\n=== End of cURL Examples ==="

# Additional utility functions
echo -e "\n=== Utility Commands ==="

# Check if the server is running
echo -e "\nCheck server status:"
curl -I "$BASE_URL/health" 2>/dev/null | head -n 1

# Test with different HTTP methods
echo -e "\nTest unsupported method (should return 405):"
curl -X PUT "$BASE_URL/health" -v 2>&1 | grep "< HTTP"

echo -e "\n=== cURL Examples Complete ==="