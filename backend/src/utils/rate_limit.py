import time
from typing import Dict
from fastapi import HTTPException, status
from collections import defaultdict
import threading

# In-memory rate limiter - for production, use Redis or similar
class RateLimiter:
    def __init__(self, max_requests: int = 60, window_size: int = 60):
        self.max_requests = max_requests
        self.window_size = window_size
        self.requests: Dict[str, list] = defaultdict(list)
        self.lock = threading.Lock()

    def is_allowed(self, user_id: str) -> bool:
        """
        Check if the user is allowed to make a request based on rate limits
        """
        with self.lock:
            now = time.time()
            # Clean old requests outside the window
            self.requests[user_id] = [
                req_time for req_time in self.requests[user_id]
                if now - req_time < self.window_size
            ]

            # Check if user has exceeded the limit
            if len(self.requests[user_id]) >= self.max_requests:
                return False

            # Add current request
            self.requests[user_id].append(now)
            return True

# Create a global rate limiter instance
rate_limiter = RateLimiter()

def rate_limit_middleware(user_id: str):
    """
    Rate limiting function that raises HTTPException if rate limit is exceeded
    """
    if not rate_limiter.is_allowed(user_id):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Rate limit exceeded"
        )