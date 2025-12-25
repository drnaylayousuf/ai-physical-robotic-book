#!/usr/bin/env python3
"""
Script to run the backend server using uvicorn
This ensures that Python can find the installed packages and handles environment variables properly
"""
import os
import sys
from pathlib import Path

# Add the app directory to Python path
app_dir = Path(__file__).parent.parent
sys.path.insert(0, str(app_dir))

def run_server():
    """Run the FastAPI application using uvicorn"""
    try:
        import uvicorn
        from backend.main import app

        # Get host and port from environment variables
        host = os.environ.get('HOST', '0.0.0.0')
        port_str = os.environ.get('PORT', '8000')

        # Convert port to integer with error handling
        try:
            port = int(port_str)
        except ValueError:
            print(f"Error: PORT environment variable '{port_str}' is not a valid integer. Defaulting to 8000.")
            port = 8000

        print(f"Starting server on {host}:{port}")
        print(f"Environment: PORT={port_str}, HOST={host}")

        uvicorn.run(
            app,
            host=host,
            port=port,
            reload=False
        )
    except ImportError as e:
        print(f"Import error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Error running server: {e}")
        sys.exit(1)

if __name__ == "__main__":
    run_server()