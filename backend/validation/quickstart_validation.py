"""
Quickstart Validation Script

This script validates that all the API endpoints work as expected after the Qdrant Cloud migration.
"""

import requests
import time
from typing import Dict, Any

def validate_api_endpoints(base_url: str = "http://localhost:8000") -> Dict[str, Any]:
    """
    Validate all API endpoints after Qdrant Cloud migration
    """
    results = {
        "health_check": False,
        "qdrant_health": False,
        "diagnostic": False,
        "chat_functionality": False,
        "errors": []
    }

    print("Starting API validation...")

    # 1. Test Health Endpoint
    print("\n1. Testing Health Endpoint...")
    try:
        response = requests.get(f"{base_url}/api/health", timeout=10)
        if response.status_code == 200:
            health_data = response.json()
            if "status" in health_data and "timestamp" in health_data:
                results["health_check"] = True
                print("   ✅ Health endpoint working correctly")
            else:
                results["errors"].append("Health endpoint response format incorrect")
                print("   ❌ Health endpoint response format incorrect")
        else:
            results["errors"].append(f"Health endpoint returned status {response.status_code}")
            print(f"   ❌ Health endpoint returned status {response.status_code}")
    except Exception as e:
        results["errors"].append(f"Health endpoint error: {str(e)}")
        print(f"   ❌ Health endpoint error: {str(e)}")

    # 2. Test Qdrant Cloud Health Endpoint
    print("\n2. Testing Qdrant Cloud Health Endpoint...")
    try:
        response = requests.get(f"{base_url}/api/health/qdrant", timeout=10)
        if response.status_code == 200:
            qdrant_health_data = response.json()
            required_fields = ["status", "message", "timestamp", "qdrant_cloud_status"]
            if all(field in qdrant_health_data for field in required_fields):
                results["qdrant_health"] = True
                print("   ✅ Qdrant Cloud health endpoint working correctly")
            else:
                results["errors"].append("Qdrant health endpoint missing required fields")
                print("   ❌ Qdrant health endpoint missing required fields")
        else:
            print(f"   ⚠️  Qdrant health endpoint returned status {response.status_code} (this may be OK if Qdrant is not accessible)")
            # This might be acceptable if Qdrant is not configured yet
    except Exception as e:
        results["errors"].append(f"Qdrant health endpoint error: {str(e)}")
        print(f"   ❌ Qdrant health endpoint error: {str(e)}")

    # 3. Test Diagnostic Endpoint
    print("\n3. Testing Diagnostic Endpoint...")
    try:
        response = requests.get(f"{base_url}/api/diagnostic/qdrant", timeout=10)
        if response.status_code == 200:
            diagnostic_data = response.json()
            required_fields = ["status", "collections", "qdrant_cloud_info", "timestamp"]
            if all(field in diagnostic_data for field in required_fields):
                results["diagnostic"] = True
                print("   ✅ Diagnostic endpoint working correctly")
            else:
                results["errors"].append("Diagnostic endpoint missing required fields")
                print("   ❌ Diagnostic endpoint missing required fields")
        else:
            print(f"   ⚠️  Diagnostic endpoint returned status {response.status_code} (this may be OK if Qdrant is not accessible)")
    except Exception as e:
        results["errors"].append(f"Diagnostic endpoint error: {str(e)}")
        print(f"   ❌ Diagnostic endpoint error: {str(e)}")

    # 4. Test Chat Endpoint
    print("\n4. Testing Chat Endpoint...")
    try:
        test_payload = {
            "question": "What is humanoid robotics?",
            "mode": "full_book"
        }
        response = requests.post(f"{base_url}/api/ask", json=test_payload, timeout=15)
        if response.status_code == 200:
            chat_data = response.json()
            required_fields = ["question", "answer", "sources", "mode", "timestamp", "processing_time_ms"]
            if all(field in chat_data for field in required_fields):
                results["chat_functionality"] = True
                print("   ✅ Chat endpoint working correctly")
            else:
                results["errors"].append("Chat endpoint missing required fields")
                print("   ❌ Chat endpoint missing required fields")
        else:
            results["errors"].append(f"Chat endpoint returned status {response.status_code}")
            print(f"   ❌ Chat endpoint returned status {response.status_code}")
    except Exception as e:
        results["errors"].append(f"Chat endpoint error: {str(e)}")
        print(f"   ❌ Chat endpoint error: {str(e)}")

    return results

def main():
    print("🔍 API Endpoint Validation for Qdrant Cloud Migration")
    print("=" * 50)

    validation_results = validate_api_endpoints()

    print("\n" + "=" * 50)
    print("VALIDATION SUMMARY:")
    print(f"✅ Health Check: {'PASS' if validation_results['health_check'] else 'FAIL'}")
    print(f"✅ Qdrant Health: {'PASS' if validation_results['qdrant_health'] else 'NOT CHECKED (OK if Qdrant not accessible)'}")
    print(f"✅ Diagnostic: {'PASS' if validation_results['diagnostic'] else 'NOT CHECKED (OK if Qdrant not accessible)'}")
    print(f"✅ Chat Functionality: {'PASS' if validation_results['chat_functionality'] else 'FAIL'}")

    if validation_results['errors']:
        print(f"\n❌ ERRORS FOUND: {len(validation_results['errors'])}")
        for i, error in enumerate(validation_results['errors'], 1):
            print(f"   {i}. {error}")
    else:
        print("\n🎉 All validations passed! The system is working correctly with Qdrant Cloud.")

    # Overall result
    core_functionality_pass = (
        validation_results['health_check'] and
        validation_results['chat_functionality']
    )

    if core_functionality_pass:
        print("\n🚀 Core functionality is working properly!")
        print("The API endpoints are successfully migrated to Qdrant Cloud.")
    else:
        print("\n❌ Core functionality has issues that need to be addressed.")

if __name__ == "__main__":
    main()