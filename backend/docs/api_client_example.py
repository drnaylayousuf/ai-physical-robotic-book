"""
API Client Examples for Frontend Integration

This file provides examples of how to call the API endpoints from a frontend application.
"""

import requests
import json
from typing import Dict, Any, Optional

class HumanoidRoboticsAPIClient:
    """
    Example API client for the Humanoid Robotics RAG system
    """

    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url.rstrip('/')
        self.session = requests.Session()

    def health_check(self) -> Dict[str, Any]:
        """
        Check the health of the API
        """
        try:
            response = self.session.get(f"{self.base_url}/api/health")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Health check failed: {e}")
            return {"error": str(e)}

    def qdrant_health_check(self) -> Dict[str, Any]:
        """
        Check the health of the Qdrant Cloud connection
        """
        try:
            response = self.session.get(f"{self.base_url}/api/health/qdrant")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Qdrant health check failed: {e}")
            return {"error": str(e)}

    def get_diagnostics(self) -> Dict[str, Any]:
        """
        Get diagnostic information about Qdrant Cloud collections
        """
        try:
            response = self.session.get(f"{self.base_url}/api/diagnostic/qdrant")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Diagnostics request failed: {e}")
            return {"error": str(e)}

    def ask_question(self, question: str, mode: str = "full_book", user_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Ask a question to the RAG system

        Args:
            question: The question to ask
            mode: The mode to use (currently only "full_book" is supported)
            user_id: Optional user identifier

        Returns:
            Response from the RAG system
        """
        try:
            payload = {
                "question": question,
                "mode": mode
            }

            if user_id:
                payload["user_id"] = user_id

            response = self.session.post(
                f"{self.base_url}/api/ask",
                json=payload,
                headers={"Content-Type": "application/json"}
            )
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Ask question request failed: {e}")
            return {"error": str(e)}

# Example usage
if __name__ == "__main__":
    # Create API client instance
    client = HumanoidRoboticsAPIClient()

    # Example 1: Health check
    print("=== Health Check ===")
    health = client.health_check()
    print(json.dumps(health, indent=2))

    # Example 2: Qdrant health check
    print("\n=== Qdrant Health Check ===")
    qdrant_health = client.qdrant_health_check()
    print(json.dumps(qdrant_health, indent=2))

    # Example 3: Diagnostic information
    print("\n=== Diagnostic Information ===")
    diagnostics = client.get_diagnostics()
    print(json.dumps(diagnostics, indent=2))

    # Example 4: Ask a question
    print("\n=== Ask Question ===")
    question_response = client.ask_question("What is humanoid robotics?")
    print(json.dumps(question_response, indent=2))

    # Example 5: Ask another question with different parameters
    print("\n=== Another Question ===")
    another_response = client.ask_question(
        question="What are the main components of a humanoid robot?",
        mode="full_book"
    )
    print(json.dumps(another_response, indent=2))