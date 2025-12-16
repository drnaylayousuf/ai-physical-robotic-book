from typing import Dict, Any, Optional
import time
import threading
from datetime import datetime, timedelta
from collections import defaultdict, deque
import logging

logger = logging.getLogger(__name__)

class MetricsCollector:
    """
    Metrics collection and monitoring utility
    """
    def __init__(self):
        self.request_count = 0
        self.error_count = 0
        self.response_times = deque(maxlen=1000)  # Keep last 1000 response times
        self.endpoint_stats = defaultdict(lambda: {
            'count': 0,
            'errors': 0,
            'total_time': 0.0,
            'avg_time': 0.0
        })
        self.lock = threading.Lock()

    def record_request(self, endpoint: str, response_time: float, is_error: bool = False):
        """
        Record a request with its response time and error status
        """
        with self.lock:
            self.request_count += 1
            if is_error:
                self.error_count += 1

            self.response_times.append(response_time)

            stats = self.endpoint_stats[endpoint]
            stats['count'] += 1
            if is_error:
                stats['errors'] += 1
            stats['total_time'] += response_time
            stats['avg_time'] = stats['total_time'] / stats['count']

    def get_metrics(self) -> Dict[str, Any]:
        """
        Get current metrics
        """
        with self.lock:
            if len(self.response_times) > 0:
                avg_response_time = sum(self.response_times) / len(self.response_times)
                p95_response_time = self._calculate_percentile(95)
                p99_response_time = self._calculate_percentile(99)
            else:
                avg_response_time = 0.0
                p95_response_time = 0.0
                p99_response_time = 0.0

            error_rate = self.error_count / self.request_count if self.request_count > 0 else 0.0

            return {
                "total_requests": self.request_count,
                "total_errors": self.error_count,
                "error_rate": error_rate,
                "avg_response_time": avg_response_time,
                "p95_response_time": p95_response_time,
                "p99_response_time": p99_response_time,
                "endpoint_stats": dict(self.endpoint_stats),
                "timestamp": datetime.utcnow().isoformat()
            }

    def _calculate_percentile(self, percentile: float) -> float:
        """
        Calculate percentile of response times
        """
        if not self.response_times:
            return 0.0

        sorted_times = sorted(self.response_times)
        index = int((percentile / 100.0) * len(sorted_times))
        return sorted_times[min(index, len(sorted_times) - 1)] if sorted_times else 0.0

    def get_throughput(self, window_minutes: int = 1) -> float:
        """
        Calculate requests per minute
        """
        # In a real implementation, we would track timestamps to calculate throughput
        # For now, this is a simplified version
        return self.request_count / window_minutes if window_minutes > 0 else 0.0

# Global metrics collector instance
metrics_collector = MetricsCollector()

class MetricsMiddleware:
    """
    Middleware to automatically collect metrics for requests
    """
    def __init__(self):
        self.metrics = metrics_collector

    async def record_request_metrics(self, endpoint: str, response_time: float, is_error: bool = False):
        """
        Record metrics for a request
        """
        self.metrics.record_request(endpoint, response_time, is_error)

def get_metrics_summary() -> Dict[str, Any]:
    """
    Get a summary of metrics for health checks and monitoring
    """
    metrics = metrics_collector.get_metrics()

    # Add health indicators
    response_time_ok = metrics["avg_response_time"] < 2.0  # Less than 2 seconds
    error_rate_ok = metrics["error_rate"] < 0.05  # Less than 5% error rate

    return {
        **metrics,
        "health_indicators": {
            "response_time_ok": response_time_ok,
            "error_rate_ok": error_rate_ok,
            "overall_healthy": response_time_ok and error_rate_ok
        }
    }