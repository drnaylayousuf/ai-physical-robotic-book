from typing import Dict, Any, Optional
from sqlalchemy.orm import Session
from ..models.ingestion_status import IngestionStatus

class ProgressTracker:
    """
    Utility class for tracking ingestion progress in the database
    """

    def __init__(self, db: Session, ingestion_status_id: int):
        self.db = db
        self.ingestion_status_id = ingestion_status_id

    def update_progress(self,
                       status: Optional[str] = None,
                       processed_files: Optional[int] = None,
                       total_chunks: Optional[int] = None,
                       progress_percentage: Optional[int] = None,
                       error_message: Optional[str] = None):
        """
        Update the progress of the ingestion process
        """
        ingestion_status = self.db.query(IngestionStatus).filter(
            IngestionStatus.id == self.ingestion_status_id
        ).first()

        if not ingestion_status:
            raise ValueError(f"Ingestion status with ID {self.ingestion_status_id} not found")

        if status is not None:
            ingestion_status.status = status
        if processed_files is not None:
            ingestion_status.processed_files = processed_files
        if total_chunks is not None:
            ingestion_status.total_chunks = total_chunks
        if progress_percentage is not None:
            ingestion_status.progress_percentage = progress_percentage
        if error_message is not None:
            ingestion_status.error_message = error_message

        self.db.commit()
        self.db.refresh(ingestion_status)

        return ingestion_status