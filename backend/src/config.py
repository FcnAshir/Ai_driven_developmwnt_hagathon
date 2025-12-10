try:
    from pydantic_settings import BaseSettings
except ImportError:
    from pydantic import BaseSettings
from typing import Optional
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class Settings(BaseSettings):
    """
    Application settings loaded from environment variables
    """
    # Qdrant Configuration
    qdrant_url: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")

    # Gemini Configuration
    gemini_api_key: str = os.getenv("GEMINI_API_KEY", "")
    gemini_model: str = os.getenv("GEMINI_MODEL", "gemini-pro")

    # JWT Configuration
    jwt_secret: str = os.getenv("JWT_SECRET", "your-default-secret-key-change-in-production")
    jwt_algorithm: str = os.getenv("JWT_ALGORITHM", "HS256")
    access_token_expire_minutes: int = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

    # Database Configuration
    database_url: str = os.getenv("DATABASE_URL", "sqlite:///./chat_threads.db")

    # Application Configuration
    app_title: str = os.getenv("APP_TITLE", "ChatKit + Gemini + RAG Integration")
    app_version: str = os.getenv("APP_VERSION", "0.1.0")
    debug: bool = os.getenv("DEBUG", "False").lower() == "true"

    # Rate Limiting
    rate_limit_per_minute: int = int(os.getenv("RATE_LIMIT_PER_MINUTE", "60"))
    max_concurrent_llm_calls: int = int(os.getenv("MAX_CONCURRENT_LLM_CALLS", "10"))

    # RAG Configuration
    top_k: int = int(os.getenv("TOP_K", "5"))
    max_context_tokens: int = int(os.getenv("MAX_CONTEXT_TOKENS", "2500"))

    class Config:
        env_file = ".env"

# Create a single instance of settings
settings = Settings()