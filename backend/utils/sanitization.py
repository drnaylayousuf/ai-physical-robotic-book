import re
from typing import Union

def sanitize_input(text: str) -> str:
    """
    Sanitize user input by removing potentially harmful content
    """
    if not text:
        return text

    # Remove potential SQL injection patterns
    sql_patterns = [
        r"(?i)(union\s+select)",
        r"(?i)(drop\s+table)",
        r"(?i)(delete\s+from)",
        r"(?i)(insert\s+into)",
        r"(?i)(update\s+set)",
        r"(?i)(exec\s*\()",
        r"(?i)(execute\s*\()",
        r"(?i)(sp_)",
        r"(?i)(xp_)",
    ]

    sanitized = text
    for pattern in sql_patterns:
        sanitized = re.sub(pattern, "[REDACTED]", sanitized)

    # Remove potential XSS patterns (basic)
    xss_patterns = [
        r"(?i)(<script[^>]*>.*?</script>)",
        r"(?i)(javascript:)",
        r"(?i)(on\w+\s*=)",
        r"(?i)(<iframe[^>]*>.*?</iframe>)",
        r"(?i)(<object[^>]*>.*?</object>)",
        r"(?i)(<embed[^>]*>.*?</embed>)",
    ]

    for pattern in xss_patterns:
        sanitized = re.sub(pattern, "[REDACTED]", sanitized, flags=re.IGNORECASE)

    # Remove excessive whitespace but keep meaningful spacing
    sanitized = re.sub(r'\s+', ' ', sanitized).strip()

    return sanitized

def validate_question(question: str) -> bool:
    """
    Validate a question to ensure it meets basic requirements
    """
    if not question or not question.strip():
        return False

    # Check length
    if len(question.strip()) < 3:
        return False

    if len(question) > 1000:  # Reasonable limit for questions
        return False

    # Check for basic structure (at least one letter or number)
    if not re.search(r'[a-zA-Z0-9]', question):
        return False

    return True

def validate_mode(mode: str) -> bool:
    """
    Validate the mode parameter
    """
    return mode in ["full_book", "selected_text"]

def sanitize_selected_text(text: str) -> str:
    """
    Sanitize selected text input
    """
    if not text:
        return text

    # Apply general sanitization
    sanitized = sanitize_input(text)

    # Limit length to prevent abuse
    if len(sanitized) > 10000:  # 10KB limit
        sanitized = sanitized[:10000]

    return sanitized