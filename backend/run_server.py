#!/usr/bin/env python3
"""
Script to run the backend server using uvicorn
This ensures that Python can find the installed packages
"""
import os
import sys
from pathlib import Path

# Add the app directory to Python path
app_dir = Path(__file__).parent.parent
sys.path.insert(0, str(app_dir))

# Set environment variables
os.environ.setdefault('PYTHONPATH', str(app_dir))
os.environ.setdefault('HOST', '0.0.0.0')
os.environ.setdefault('PORT', os.environ.get('PORT', '8000'))

def run_server():
    """Run the FastAPI application using uvicorn"""
    try:
        import uvicorn
        from backend.main import app

        host = os.environ.get('HOST', '0.0.0.0')
        port = int(os.environ.get('PORT', '8000'))

        print(f"Starting server on {host}:{port}")
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