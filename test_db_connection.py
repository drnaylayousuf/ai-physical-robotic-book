#!/usr/bin/env python3
"""
Test script to check database connection
"""
import os
from dotenv import load_dotenv
from sqlalchemy import create_engine, text

# Load environment variables
load_dotenv()

def test_db_connection():
    """Test if the database connection works"""
    database_url = os.getenv("DATABASE_URL")

    if not database_url:
        print("DATABASE_URL not found in .env file")
        return False

    print(f"Testing database connection to: {database_url}")

    try:
        # Create engine and test connection
        engine = create_engine(database_url, pool_recycle=300)

        with engine.connect() as connection:
            result = connection.execute(text("SELECT 1"))
            print("Database connection successful!")
            print(f"   Result: {result.fetchone()}")
            return True

    except Exception as e:
        print(f"Database connection failed: {e}")
        print("This might be why your API routes aren't loading")
        print("The backend server might be failing to start properly due to database issues")
        return False

if __name__ == "__main__":
    print("Database Connection Test")
    print("=" * 30)

    success = test_db_connection()

    if success:
        print("\nDATABASE CONNECTION: SUCCESSFUL")
        print("Your API routes should be loading properly")
    else:
        print("\nDATABASE CONNECTION: FAILED")
        print("This is likely why you're getting '404 Not Found' errors")
        print("The server might be starting but API routes aren't loading due to database issues")