#!/usr/bin/env python3
"""
Script to create an admin user in the database for content ingestion.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.models.database import SessionLocal
from backend.models.user import User
from datetime import datetime
from sqlalchemy import text

def create_admin_user():
    """Create an admin user in the database."""
    db = SessionLocal()
    try:
        # Check if an admin user already exists
        existing_admin = db.query(User).filter(User.username == "admin").first()
        if existing_admin:
            print(f"Admin user with username 'admin' already exists (ID: {existing_admin.id}).")
            return existing_admin

        # Check for the next available ID
        max_id_result = db.execute(text("SELECT MAX(id) FROM users")).fetchone()
        next_id = 1
        if max_id_result[0] is not None:
            next_id = max_id_result[0] + 1

        # Create an admin user with the next available ID
        admin_user = User(
            id=next_id,
            username="admin",
            email="admin@example.com",
            password_hash="$2b$12$C8EJ4KlQ9Kz7Z7Z7Z7Z7Z.Z7Z7Z7Z7Z7Z7Z7Z7Z7Z7Z7Z7Z7Z7Z7Z",  # Placeholder hash
            role="admin",
            created_at=datetime.utcnow()
        )

        db.add(admin_user)
        db.commit()
        db.refresh(admin_user)

        print(f"Created admin user with ID: {admin_user.id}")
        print("Admin user is ready for content ingestion.")

        return admin_user

    except Exception as e:
        print(f"Error creating admin user: {e}")
        db.rollback()
        raise
    finally:
        db.close()

if __name__ == "__main__":
    create_admin_user()