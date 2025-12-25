#!/usr/bin/env python3
"""
Direct startup script for the backend server
This bypasses any command-line argument issues with uvicorn
"""
import os
import sys
from pathlib import Path

# Add the app directory to Python path
app_dir = Path(__file__).parent
backend_dir = app_dir / "backend"
sys.path.insert(0, str(app_dir))
sys.path.insert(0, str(backend_dir))

def main():
    """Main entry point that runs the server directly"""
    try:
        # Set up environment
        host = os.environ.get('HOST', '0.0.0.0')
        port_str = os.environ.get('PORT', '8000')

        # Validate and convert port
        try:
            port = int(port_str)
        except ValueError:
            print(f"Warning: PORT '{port_str}' is not a valid integer, using default 8000")
            port = 8000

        print(f"Starting server on {host}:{port}")
        print(f"Environment: PORT={port_str}, HOST={host}")

        # Import and run the app directly with uvicorn
        import uvicorn
        from backend.main import app

        # Run the application
        uvicorn.run(
            app,
            host=host,
            port=port,
            reload=False,
            log_level="info"
        )
    except ImportError as e:
        print(f"Failed to import required modules: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Failed to start server: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()