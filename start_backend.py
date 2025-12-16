#!/usr/bin/env python3
"""
Script to start the backend server
"""
import subprocess
import sys
import os
from pathlib import Path

def start_backend():
    """Start the backend server"""
    print("Starting the RAG Chatbot Backend Server...")
    print("This will start the server on http://localhost:8000")
    print()

    # Change to the project directory
    project_dir = Path(__file__).parent
    os.chdir(project_dir)

    # Set environment to load .env file
    env = os.environ.copy()
    env['PYTHONPATH'] = str(project_dir) + os.pathsep + env.get('PYTHONPATH', '')

    try:
        # Run uvicorn to start the server
        cmd = [
            sys.executable, "-m", "uvicorn",
            "backend.main:app",
            "--host", "0.0.0.0",
            "--port", "8000",
            "--reload"
        ]

        print(f"Running command: {' '.join(cmd)}")
        print("Server starting... Please wait for startup message...")
        print()

        # Start the server
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            env=env
        )

        # Print the server output
        while True:
            output = process.stdout.readline()
            if output == '' and process.poll() is not None:
                break
            if output:
                print(output.strip())
                # Check if server is ready
                if "Uvicorn running on" in output:
                    print()
                    print("Server is running!")
                    print("The chatbot should now be able to connect to Qdrant!")
                    print("You can now ask questions about the book in the chatbot.")
                    print()
                    print("Server is accessible at: http://localhost:8000")

        return_code = process.poll()
        return return_code == 0

    except KeyboardInterrupt:
        print("\nServer stopped by user.")
        return True
    except Exception as e:
        print(f"Error starting server: {e}")
        return False

if __name__ == "__main__":
    print("RAG Chatbot Backend Server Starter")
    print("=" * 40)

    success = start_backend()

    if success:
        print("\nBackend server started successfully!")
        print("The 'Failed to fetch' error should now be resolved.")
    else:
        print("\nError starting backend server.")
        print("Make sure all dependencies are installed and .env file is configured.")