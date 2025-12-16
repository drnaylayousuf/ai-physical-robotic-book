#!/usr/bin/env python3
"""
Verification script to test that the RAG validation bug fix is working correctly.
This script tests the specific issue where requests with mode="selected_text"
and null/empty selected_text were incorrectly processed.
"""
import json
import requests
import time

def test_rag_validation_fix():
    """
    Test that the RAG validation bug fix is working correctly.
    The fix should ensure that when mode="selected_text" but selected_text is null/empty,
    the system properly falls back to full_book mode.
    """
    print("Testing RAG Validation Bug Fix...")
    print("="*50)

    # Base URL for the backend API
    base_url = "http://localhost:8000"

    # Test case 1: mode="selected_text" with null selected_text should fallback to full_book mode
    print("\nTest 1: mode='selected_text' with null selected_text")
    print("- Should fallback to full_book mode")

    payload1 = {
        "question": "What are humanoid robots used for?",
        "mode": "selected_text",
        "selected_text": None
    }

    try:
        response1 = requests.post(f"{base_url}/api/ask",
                                  json=payload1,
                                  headers={"Content-Type": "application/json"})

        if response1.status_code == 200:
            data1 = response1.json()
            print(f"[PASS] Status: {response1.status_code}")
            print(f"[PASS] Response length: {len(data1.get('response', ''))} characters")
            print(f"[PASS] Sources returned: {len(data1.get('sources', []))}")

            # Check if the response contains meaningful content (not just "book doesn't provide details")
            response_text = data1.get('response', '').lower()
            if "not provide details" in response_text or "does not provide details" in response_text:
                print("[WARN] Response indicates insufficient context - may not have fallen back properly")
            else:
                print("[PASS] Response contains meaningful content - fallback likely worked correctly")
        else:
            print(f"[FAIL] Failed with status: {response1.status_code}")
            print(f"[FAIL] Response: {response1.text}")
    except Exception as e:
        print(f"[FAIL] Error during request: {str(e)}")
        print("  Note: This might be expected if the server isn't running or has hit API quota limits")

    # Test case 2: mode="selected_text" with empty string selected_text should fallback
    print("\nTest 2: mode='selected_text' with empty string selected_text")
    print("- Should fallback to full_book mode")

    payload2 = {
        "question": "What is ROS 2?",
        "mode": "selected_text",
        "selected_text": ""
    }

    try:
        response2 = requests.post(f"{base_url}/api/ask",
                                  json=payload2,
                                  headers={"Content-Type": "application/json"})

        if response2.status_code == 200:
            data2 = response2.json()
            print(f"[PASS] Status: {response2.status_code}")
            print(f"[PASS] Response length: {len(data2.get('response', ''))} characters")
            print(f"[PASS] Sources returned: {len(data2.get('sources', []))}")

            response_text = data2.get('response', '').lower()
            if "not provide details" in response_text or "does not provide details" in response_text:
                print("[WARN] Response indicates insufficient context - may not have fallen back properly")
            else:
                print("[PASS] Response contains meaningful content - fallback likely worked correctly")
        else:
            print(f"[FAIL] Failed with status: {response2.status_code}")
            print(f"[FAIL] Response: {response2.text}")
    except Exception as e:
        print(f"[FAIL] Error during request: {str(e)}")
        print("  Note: This might be expected if the server isn't running or has hit API quota limits")

    # Test case 3: mode="selected_text" with whitespace-only selected_text should fallback
    print("\nTest 3: mode='selected_text' with whitespace-only selected_text")
    print("- Should fallback to full_book mode")

    payload3 = {
        "question": "What is Physical AI?",
        "mode": "selected_text",
        "selected_text": "   \t\n  "
    }

    try:
        response3 = requests.post(f"{base_url}/api/ask",
                                  json=payload3,
                                  headers={"Content-Type": "application/json"})

        if response3.status_code == 200:
            data3 = response3.json()
            print(f"[PASS] Status: {response3.status_code}")
            print(f"[PASS] Response length: {len(data3.get('response', ''))} characters")
            print(f"[PASS] Sources returned: {len(data3.get('sources', []))}")

            response_text = data3.get('response', '').lower()
            if "not provide details" in response_text or "does not provide details" in response_text:
                print("[WARN] Response indicates insufficient context - may not have fallen back properly")
            else:
                print("[PASS] Response contains meaningful content - fallback likely worked correctly")
        else:
            print(f"[FAIL] Failed with status: {response3.status_code}")
            print(f"[FAIL] Response: {response3.text}")
    except Exception as e:
        print(f"[FAIL] Error during request: {str(e)}")
        print("  Note: This might be expected if the server isn't running or has hit API quota limits")

    # Test case 4: mode="full_book" should continue to work normally
    print("\nTest 4: mode='full_book' with null selected_text")
    print("- Should work normally (control test)")

    payload4 = {
        "question": "What are the key components of humanoid robots?",
        "mode": "full_book",
        "selected_text": None
    }

    try:
        response4 = requests.post(f"{base_url}/api/ask",
                                  json=payload4,
                                  headers={"Content-Type": "application/json"})

        if response4.status_code == 200:
            data4 = response4.json()
            print(f"[PASS] Status: {response4.status_code}")
            print(f"[PASS] Response length: {len(data4.get('response', ''))} characters")
            print(f"[PASS] Sources returned: {len(data4.get('sources', []))}")

            print("[PASS] Control test passed - full_book mode works normally")
        else:
            print(f"[FAIL] Failed with status: {response4.status_code}")
            print(f"[FAIL] Response: {response4.text}")
    except Exception as e:
        print(f"[FAIL] Error during request: {str(e)}")
        print("  Note: This might be expected if the server isn't running or has hit API quota limits")

    # Test case 5: mode="selected_text" with valid selected_text should work normally
    print("\nTest 5: mode='selected_text' with valid selected_text")
    print("- Should work normally with selected text context")

    payload5 = {
        "question": "Based on this text, what are actuators?",
        "mode": "selected_text",
        "selected_text": "Actuators are the motors and mechanical systems that enable movement in humanoid robots. High-torque actuators are essential for achieving human-like motion and maintaining balance."
    }

    try:
        response5 = requests.post(f"{base_url}/api/ask",
                                  json=payload5,
                                  headers={"Content-Type": "application/json"})

        if response5.status_code == 200:
            data5 = response5.json()
            print(f"[PASS] Status: {response5.status_code}")
            print(f"[PASS] Response length: {len(data5.get('response', ''))} characters")
            print(f"[PASS] Sources returned: {len(data5.get('sources', []))}")

            print("[PASS] Valid selected_text mode works normally")
        else:
            print(f"[FAIL] Failed with status: {response5.status_code}")
            print(f"[FAIL] Response: {response5.text}")
    except Exception as e:
        print(f"[FAIL] Error during request: {str(e)}")
        print("  Note: This might be expected if the server isn't running or has hit API quota limits")

    print("\n" + "="*50)
    print("Test Summary:")
    print("- Tests 1-3 verify the fix for the validation bug (fallback behavior)")
    print("- Tests 4-5 verify normal operation still works (regression check)")
    print("- Some tests may fail if server isn't running or API quota is exceeded")
    print("\nThe fix is working correctly if:")
    print("  - Tests 1-3 return meaningful responses (not 'insufficient context')")
    print("  - Tests 4-5 continue to work normally")


if __name__ == "__main__":
    test_rag_validation_fix()