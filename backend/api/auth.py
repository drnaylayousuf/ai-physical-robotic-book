from fastapi import APIRouter, Depends, HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel, EmailStr
from datetime import timedelta
from typing import Optional, Dict, Any
from sqlalchemy.orm import Session
from sqlalchemy import create_engine, Column, Integer, String, Boolean, DateTime, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
from ..models.user import User, UserCreate, UserPublic
from ..models.database import get_db
from ..utils.auth import (
    verify_password,
    get_password_hash,
    create_access_token,
    decode_access_token,
    create_token_data
)
from ..config.settings import settings
import json
import uuid
from datetime import datetime

# Create two separate routers
router = APIRouter()  # For legacy /api/auth endpoints
better_auth_router = APIRouter()  # For Better Auth compatible endpoints

# Define database models to match Prisma schema
Base = declarative_base()

class DBUser(Base):
    __tablename__ = "User"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    email = Column(String, unique=True, nullable=False)
    name = Column(String)
    emailVerified = Column(DateTime)
    createdAt = Column(DateTime, default=func.now())
    updatedAt = Column(DateTime, default=func.now(), onupdate=func.now())

class DBUserProfile(Base):
    __tablename__ = "UserProfile"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    userId = Column(String, unique=True, nullable=False)
    pythonProficiency = Column(String, nullable=False)
    operatingSystem = Column(String, nullable=False)
    preferredEnvironment = Column(String, nullable=False)
    onboardingComplete = Column(Boolean, default=False)
    createdAt = Column(DateTime, default=func.now())
    updatedAt = Column(DateTime, default=func.now(), onupdate=func.now())

# Request/Response models
class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    username: Optional[str] = None
    role: Optional[str] = None

class UserLogin(BaseModel):
    username: str
    password: str

class UserProfile(BaseModel):
    id: Optional[str] = None
    userId: Optional[str] = None
    technicalBackground: Optional[str] = None
    experienceLevel: Optional[str] = None
    interests: Optional[str] = None
    goals: Optional[str] = None
    onboardingComplete: bool = False

class BetterAuthUser(BaseModel):
    id: str
    email: str
    name: Optional[str] = None
    image: Optional[str] = None
    emailVerified: Optional[bool] = None

class BetterAuthSession(BaseModel):
    id: str
    userId: str
    expiresAt: str
    token: str

class BetterAuthResponse(BaseModel):
    user: Optional[BetterAuthUser] = None
    session: Optional[BetterAuthSession] = None
    profile: Optional[UserProfile] = None
    onboardingComplete: bool = False

# Better Auth compatible endpoints
@better_auth_router.post("/sign-in/email")
async def better_auth_signin_email(request: Request, db: Session = Depends(get_db)):
    """
    Better Auth compatible sign-in endpoint
    """
    try:
        body = await request.json()
        email = body.get('email')
        password = body.get('password')

        # Find user by email in the database
        db_user = db.query(DBUser).filter(DBUser.email == email).first()

        if not db_user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect email or password",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # In the current setup, password is stored separately from the User model
        # We need to check the password hash in the existing User table
        existing_user = db.query(User).filter(User.email == email).first()
        if not existing_user or not verify_password(password, existing_user.password_hash):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect email or password",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Create token data
        token_data = create_token_data(existing_user.username, existing_user.role)

        # Create access token
        access_token_expires = timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
        access_token = create_access_token(
            data=token_data, expires_delta=access_token_expires
        )

        # Create BetterAuth compatible response
        better_user = BetterAuthUser(
            id=str(existing_user.id),
            email=existing_user.email,
            name=existing_user.username
        )

        better_session = BetterAuthSession(
            id=f"sess_{existing_user.id}",
            userId=str(existing_user.id),
            expiresAt=(datetime.now() + access_token_expires).isoformat(),
            token=access_token
        )

        # Get user profile if exists
        db_user_profile = db.query(DBUserProfile).filter(DBUserProfile.userId == str(existing_user.id)).first()
        if db_user_profile:
            user_profile = UserProfile(
                id=db_user_profile.id,
                userId=db_user_profile.userId,
                technicalBackground=db_user_profile.pythonProficiency,  # Map to appropriate field
                experienceLevel=db_user_profile.operatingSystem,  # Map to appropriate field
                interests=db_user_profile.preferredEnvironment,  # Map to appropriate field
                goals="",  # No direct mapping
                onboardingComplete=db_user_profile.onboardingComplete
            )
        else:
            user_profile = UserProfile(userId=str(existing_user.id), onboardingComplete=False)

        return BetterAuthResponse(
            user=better_user,
            session=better_session,
            profile=user_profile,
            onboardingComplete=user_profile.onboardingComplete
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=f"Sign in failed: {str(e)}",
        )


@better_auth_router.post("/sign-up/email")
async def better_auth_signup_email(request: Request, db: Session = Depends(get_db)):
    """
    Better Auth compatible sign-up endpoint
    """
    try:
        body = await request.json()
        email = body.get('email')
        password = body.get('password')
        name = body.get('name', email.split('@')[0])  # Use part of email as name if not provided

        # Check if user already exists in the existing User table
        existing_user = db.query(User).filter(User.email == email).first()
        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email already registered"
            )

        # Create new user in the existing User table
        hashed_password = get_password_hash(password)
        new_user = User(
            username=name,
            email=email,
            password_hash=hashed_password,
            role="user"  # Default role
        )
        db.add(new_user)
        db.commit()
        db.refresh(new_user)

        # Also create a corresponding entry in the DBUser table (for Prisma schema compatibility)
        db_user = DBUser(
            id=str(new_user.id),  # Use the same ID
            email=email,
            name=name
        )
        db.add(db_user)
        db.commit()

        # Create token data
        token_data = create_token_data(new_user.username, new_user.role)

        # Create access token
        access_token_expires = timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
        access_token = create_access_token(
            data=token_data, expires_delta=access_token_expires
        )

        # Create BetterAuth compatible response
        better_user = BetterAuthUser(
            id=str(new_user.id),
            email=new_user.email,
            name=new_user.username
        )

        better_session = BetterAuthSession(
            id=f"sess_{new_user.id}",
            userId=str(new_user.id),
            expiresAt=(datetime.now() + access_token_expires).isoformat(),
            token=access_token
        )

        # Create a default user profile
        user_profile = UserProfile(
            userId=str(new_user.id),
            onboardingComplete=False
        )

        return BetterAuthResponse(
            user=better_user,
            session=better_session,
            profile=user_profile,
            onboardingComplete=False
        )
    except HTTPException:
        db.rollback()
        raise
    except Exception as e:
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Sign up failed: {str(e)}",
        )


@better_auth_router.post("/sign-out")
async def better_auth_signout(credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer())):
    """
    Better Auth compatible sign-out endpoint
    """
    try:
        # Decode token to verify it's valid
        payload = decode_access_token(credentials.credentials)
        if payload is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )

        return {"success": True}
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=f"Sign out failed: {str(e)}",
        )


@better_auth_router.get("/session")
async def better_auth_get_session(credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer()), db: Session = Depends(get_db)):
    """
    Better Auth compatible get session endpoint
    """
    try:
        # Decode the token to get user info
        payload = decode_access_token(credentials.credentials)
        if payload is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )

        username = payload.get("sub")
        if username is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Get user from database
        user = db.query(User).filter(User.username == username).first()
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found",
            )

        # Create BetterAuth compatible response
        better_user = BetterAuthUser(
            id=str(user.id),
            email=user.email,
            name=user.username
        )

        better_session = BetterAuthSession(
            id=f"sess_{user.id}",
            userId=str(user.id),
            expiresAt="",  # Will be set properly in a real implementation
            token=credentials.credentials
        )

        # Get user profile if exists
        db_user_profile = db.query(DBUserProfile).filter(DBUserProfile.userId == str(user.id)).first()
        if db_user_profile:
            user_profile = UserProfile(
                id=db_user_profile.id,
                userId=db_user_profile.userId,
                technicalBackground=db_user_profile.pythonProficiency,  # Map to appropriate field
                experienceLevel=db_user_profile.operatingSystem,  # Map to appropriate field
                interests=db_user_profile.preferredEnvironment,  # Map to appropriate field
                goals="",  # No direct mapping
                onboardingComplete=db_user_profile.onboardingComplete
            )
        else:
            user_profile = UserProfile(userId=str(user.id), onboardingComplete=False)

        return BetterAuthResponse(
            user=better_user,
            session=better_session,
            profile=user_profile,
            onboardingComplete=user_profile.onboardingComplete
        )
    except Exception as e:
        # Return a response indicating no session (user is not authenticated)
        return BetterAuthResponse(
            onboardingComplete=False
        )


@better_auth_router.post("/onboarding")
async def better_auth_onboarding(request: Request, credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer()), db: Session = Depends(get_db)):
    """
    Better Auth compatible onboarding endpoint
    """
    try:
        # Decode the token to get user info
        payload = decode_access_token(credentials.credentials)
        if payload is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )

        username = payload.get("sub")
        if username is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Get user from database
        user = db.query(User).filter(User.username == username).first()
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found",
            )

        body = await request.json()

        # Get or create user profile in the database
        db_user_profile = db.query(DBUserProfile).filter(DBUserProfile.userId == str(user.id)).first()

        if db_user_profile:
            # Update existing profile
            db_user_profile.pythonProficiency = body.get("technicalBackground", "BEGINNER")  # Map to appropriate field
            db_user_profile.operatingSystem = body.get("experienceLevel", "OTHER")  # Map to appropriate field
            db_user_profile.preferredEnvironment = body.get("interests", "LOCAL_MACHINE")  # Map to appropriate field
            db_user_profile.onboardingComplete = True
        else:
            # Create new profile
            db_user_profile = DBUserProfile(
                userId=str(user.id),
                pythonProficiency=body.get("technicalBackground", "BEGINNER"),  # Map to appropriate field
                operatingSystem=body.get("experienceLevel", "OTHER"),  # Map to appropriate field
                preferredEnvironment=body.get("interests", "LOCAL_MACHINE"),  # Map to appropriate field
                onboardingComplete=True
            )
            db.add(db_user_profile)

        db.commit()
        if db_user_profile:
            db.refresh(db_user_profile)

        # Create response profile
        user_profile = UserProfile(
            id=db_user_profile.id if db_user_profile else None,
            userId=str(user.id),
            technicalBackground=db_user_profile.pythonProficiency if db_user_profile else body.get("technicalBackground", "BEGINNER"),
            experienceLevel=db_user_profile.operatingSystem if db_user_profile else body.get("experienceLevel", "OTHER"),
            interests=db_user_profile.preferredEnvironment if db_user_profile else body.get("interests", "LOCAL_MACHINE"),
            goals=body.get("goals", ""),
            onboardingComplete=True
        )

        return {
            "profile": user_profile,
            "onboardingComplete": True
        }
    except HTTPException:
        db.rollback()
        raise
    except Exception as e:
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Onboarding failed: {str(e)}",
        )


# Legacy endpoints for compatibility
@router.post("/auth/register", response_model=UserPublic)
async def register_user(user: UserCreate, db: Session = Depends(get_db)):
    """
    Register a new user account
    """
    # Check if user already exists in the database
    existing_user = db.query(User).filter((User.username == user.username) | (User.email == user.email)).first()
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Username or email already registered"
        )

    # Create new user
    hashed_password = get_password_hash(user.password)
    new_user = User(
        username=user.username,
        email=user.email,
        password_hash=hashed_password,
        role="user",  # Default role
    )

    db.add(new_user)
    db.commit()
    db.refresh(new_user)

    # Also create a corresponding entry in the DBUser table (for Prisma schema compatibility)
    db_user = DBUser(
        id=str(new_user.id),  # Use the same ID
        email=user.email,
        name=user.username
    )
    db.add(db_user)
    db.commit()

    # Return user data (excluding password hash)
    return UserPublic(
        id=new_user.id,
        username=new_user.username,
        email=new_user.email,
        role=new_user.role,
        created_at=new_user.created_at  # Now comes from DB
    )

@router.post("/auth/login", response_model=Token)
async def login_user(form_data: UserLogin, db: Session = Depends(get_db)):
    """
    Authenticate user and return access token
    """
    # Retrieve user from database
    user = db.query(User).filter(User.username == form_data.username).first()
    if not user or not verify_password(form_data.password, user.password_hash):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Create token data
    token_data = create_token_data(user.username, user.role)

    # Create access token
    access_token_expires = timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data=token_data, expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}

@router.get("/auth/profile", response_model=UserPublic)
async def get_user_profile(token: HTTPAuthorizationCredentials = Depends(HTTPBearer()), db: Session = Depends(get_db)):
    """
    Get authenticated user's profile information
    """
    # Decode the token to get user info
    payload = decode_access_token(token.credentials)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    username = payload.get("sub")
    if username is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Get user from database
    user = db.query(User).filter(User.username == username).first()
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found",
        )

    # Return user data (excluding password hash)
    return UserPublic(
        id=user.id,
        username=user.username,
        email=user.email,
        role=user.role,
        created_at=user.created_at  # Now comes from DB
    )