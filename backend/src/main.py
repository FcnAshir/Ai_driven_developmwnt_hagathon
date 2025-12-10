import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from contextlib import asynccontextmanager
import logging
import sys
import os
# Add the backend directory to the Python path to allow relative imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import settings
from src.middleware.error_handler import (
    http_exception_handler,
    validation_exception_handler,
    general_exception_handler
)
from starlette.exceptions import HTTPException as StarletteHTTPException
from fastapi.exceptions import RequestValidationError
from starlette.middleware import Middleware
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response
import time

# Configure logging
logging.basicConfig(
    level=logging.INFO if not settings.debug else logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """
    Middleware to add security headers to responses
    """
    async def dispatch(self, request, call_next):
        response = await call_next(request)

        # Add security headers
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"
        response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"

        return response

class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Basic rate limiting middleware
    """
    def __init__(self, app, max_requests=60, window_size=60):
        super().__init__(app)
        self.max_requests = max_requests
        self.window_size = window_size
        self.requests = {}

    async def dispatch(self, request, call_next):
        # Get client IP
        client_ip = request.client.host

        # Check rate limit
        current_time = time.time()
        if client_ip not in self.requests:
            self.requests[client_ip] = []

        # Clean old requests
        self.requests[client_ip] = [
            req_time for req_time in self.requests[client_ip]
            if current_time - req_time < self.window_size
        ]

        # Check if limit exceeded
        if len(self.requests[client_ip]) >= self.max_requests:
            return Response(
                content="Rate limit exceeded",
                status_code=429,
                headers={"Retry-After": str(self.window_size)}
            )

        # Add current request
        self.requests[client_ip].append(current_time)

        response = await call_next(request)
        return response

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handler for application startup and shutdown
    """
    logger.info("Application starting up...")
    # Startup logic here
    yield
    # Shutdown logic here
    logger.info("Application shutting down...")

# Create FastAPI app instance
app = FastAPI(
    title=settings.app_title,
    version=settings.app_version,
    lifespan=lifespan,
    debug=settings.debug
)

# Add security headers middleware first
app.add_middleware(SecurityHeadersMiddleware)

# Add rate limiting middleware
app.add_middleware(RateLimitMiddleware, max_requests=100, window_size=60)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure this properly
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add exception handlers
app.add_exception_handler(StarletteHTTPException, http_exception_handler)
app.add_exception_handler(RequestValidationError, validation_exception_handler)
app.add_exception_handler(Exception, general_exception_handler)

# Include API routes
from src.api.chatkit_endpoints import router as chatkit_router
app.include_router(chatkit_router)

@app.get("/")
async def root():
    return {"message": "ChatKit + Gemini + RAG Integration API"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "backend.src.main:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000)),
        reload=settings.debug
    )