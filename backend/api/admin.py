from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer
from pydantic import BaseModel
from typing import List, Dict, Any
from datetime import datetime
import logging

from ..utils.auth import decode_access_token, get_role_from_token
from ..utils.metrics import get_metrics_summary
from ..models.user import User
from ..models.user_query import UserQuery
from ..models.database import get_db
from sqlalchemy.orm import Session

router = APIRouter()
logger = logging.getLogger(__name__)

# Response models
class AdminStatsResponse(BaseModel):
    total_users: int
    total_queries: int
    system_metrics: Dict[str, Any]
    timestamp: datetime

class SystemHealthResponse(BaseModel):
    status: str
    timestamp: datetime
    metrics: Dict[str, Any]

@router.get("/admin", response_model=SystemHealthResponse)
async def get_admin_dashboard(
    token: str = Depends(HTTPBearer())
):
    """
    Administrative functions endpoint for admin users
    """
    # Decode token to get user info
    payload = decode_access_token(token.credentials)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    username = payload.get("sub")
    user_role = get_role_from_token(token.credentials) or "user"

    # Check if user has admin privileges
    if user_role != "admin":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Only admin users can access this endpoint"
        )

    try:
        # Get system metrics
        metrics = get_metrics_summary()

        # Log admin access
        logger.info(f"Admin dashboard accessed by user: {username}")

        return SystemHealthResponse(
            status="healthy",
            timestamp=datetime.utcnow(),
            metrics=metrics
        )

    except Exception as e:
        logger.error(f"Error in admin dashboard: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error accessing admin dashboard"
        )

@router.get("/admin/stats", response_model=AdminStatsResponse)
async def get_admin_stats(
    token: str = Depends(HTTPBearer())
):
    """
    Get administrative statistics
    """
    # Decode token to get user info
    payload = decode_access_token(token.credentials)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    username = payload.get("sub")
    user_role = get_role_from_token(token.credentials) or "user"

    # Check if user has admin privileges
    if user_role != "admin":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Only admin users can access this endpoint"
        )

    try:
        # Get system metrics
        metrics = get_metrics_summary()

        # In a real implementation, you would fetch these from the database
        total_users = 0  # This would be fetched from DB
        total_queries = 0  # This would be fetched from DB

        return AdminStatsResponse(
            total_users=total_users,
            total_queries=total_queries,
            system_metrics=metrics,
            timestamp=datetime.utcnow()
        )

    except Exception as e:
        logger.error(f"Error in admin stats: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error fetching admin statistics"
        )

# Add other admin endpoints as needed
@router.get("/admin/users")
async def get_users_list(
    token: str = Depends(HTTPBearer()),
    db: Session = Depends(get_db)
):
    """
    Get list of all users (admin only)
    """
    # Implementation would go here
    pass

@router.get("/admin/queries")
async def get_recent_queries(
    token: str = Depends(HTTPBearer()),
    db: Session = Depends(get_db)
):
    """
    Get recent queries (admin only)
    """
    # Decode token to get user info
    payload = decode_access_token(token.credentials)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    user_role = get_role_from_token(token.credentials) or "user"

    # Check if user has admin or moderator privileges
    if user_role not in ["admin", "moderator"]:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Only admin and moderator users can access this endpoint"
        )

    try:
        # Get recent queries from the database
        recent_queries = db.query(UserQuery).order_by(UserQuery.timestamp.desc()).limit(50).all()

        queries_list = []
        for query in recent_queries:
            queries_list.append({
                "id": query.id,
                "user_id": query.user_id,
                "question": query.question,
                "response": query.response,
                "sources": query.sources,
                "timestamp": query.timestamp.isoformat() if query.timestamp else None
            })

        return {"queries": queries_list}

    except Exception as e:
        logger.error(f"Error fetching recent queries: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error fetching recent queries"
        )

@router.get("/moderate", response_model=Dict[str, Any])
async def get_content_moderation_dashboard(
    token: str = Depends(HTTPBearer()),
    db: Session = Depends(get_db)
):
    """
    Content moderation endpoint for moderators and admins
    """
    # Decode token to get user info
    payload = decode_access_token(token.credentials)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    user_role = get_role_from_token(token.credentials) or "user"

    # Check if user has admin or moderator privileges
    if user_role not in ["admin", "moderator"]:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Only admin and moderator users can access moderation functions"
        )

    try:
        # Get content that may need moderation (recent queries)
        recent_queries = db.query(UserQuery).order_by(UserQuery.timestamp.desc()).limit(100).all()

        # Count queries by time period
        import datetime
        from datetime import timezone
        now = datetime.datetime.now(timezone.utc)
        one_hour_ago = now - datetime.timedelta(hours=1)

        recent_hour_count = db.query(UserQuery).filter(UserQuery.timestamp > one_hour_ago).count()

        # Prepare moderation dashboard data
        dashboard_data = {
            "total_recent_queries": len(recent_queries),
            "queries_in_last_hour": recent_hour_count,
            "moderation_queue_size": len([q for q in recent_queries if q.sources is None or q.response is None]),  # Example criteria
            "recent_queries": [
                {
                    "id": q.id,
                    "user_id": q.user_id,
                    "question": q.question[:100] + "..." if len(q.question) > 100 else q.question,  # Truncate long questions
                    "timestamp": q.timestamp.isoformat() if q.timestamp else None,
                    "has_sources": bool(q.sources)
                }
                for q in recent_queries[:20]  # Limit to first 20 for the dashboard
            ],
            "moderation_needed_count": 0,  # This would be calculated based on specific criteria
            "last_updated": now.isoformat()
        }

        logger.info(f"Moderation dashboard accessed by user with role: {user_role}")

        return dashboard_data

    except Exception as e:
        logger.error(f"Error in content moderation dashboard: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error accessing content moderation dashboard"
        )

@router.post("/moderate/query/{query_id}/flag")
async def flag_query_for_moderation(
    query_id: int,
    token: str = Depends(HTTPBearer()),
    db: Session = Depends(get_db)
):
    """
    Flag a specific query for moderation review
    """
    # Decode token to get user info
    payload = decode_access_token(token.credentials)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    user_role = get_role_from_token(token.credentials) or "user"

    # Check if user has admin or moderator privileges
    if user_role not in ["admin", "moderator"]:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Only admin and moderator users can flag queries for moderation"
        )

    try:
        # Get the query from the database
        query = db.query(UserQuery).filter(UserQuery.id == query_id).first()
        if not query:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Query with ID {query_id} not found"
            )

        # In a real implementation, you might add a flag field or create a moderation queue
        # For now, we'll just log the action
        logger.info(f"Query {query_id} flagged for moderation by user with role: {user_role}")

        return {
            "message": f"Query {query_id} flagged for moderation review",
            "query_id": query_id,
            "status": "flagged"
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error flagging query {query_id} for moderation: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error flagging query {query_id} for moderation"
        )