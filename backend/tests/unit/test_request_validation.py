"""
Unit tests for request validation.
"""
import pytest
from pydantic import ValidationError
from src.models.request import RAGRequest, RAGMode


def test_rag_request_valid():
    """Test that a valid RAG request is accepted."""
    request = RAGRequest(
        question="What are humanoid robots used for?",
        mode=RAGMode.FULL_BOOK,
        selected_text=None
    )

    assert request.question == "What are humanoid robots used for?"
    assert request.mode == RAGMode.FULL_BOOK
    assert request.selected_text is None


def test_rag_request_selected_text_mode():
    """Test RAG request with selected text mode."""
    request = RAGRequest(
        question="What does this text say?",
        mode=RAGMode.SELECTED_TEXT,
        selected_text="This is the selected text."
    )

    assert request.question == "What does this text say?"
    assert request.mode == RAGMode.SELECTED_TEXT
    assert request.selected_text == "This is the selected text."


def test_rag_request_empty_question():
    """Test that an empty question raises validation error."""
    with pytest.raises(ValidationError):
        RAGRequest(
            question="",
            mode=RAGMode.FULL_BOOK,
            selected_text=None
        )


def test_rag_request_whitespace_question():
    """Test that a whitespace-only question raises validation error."""
    with pytest.raises(ValidationError):
        RAGRequest(
            question="   \n\t   ",
            mode=RAGMode.FULL_BOOK,
            selected_text=None
        )


def test_rag_request_none_question():
    """Test that a None question raises validation error."""
    with pytest.raises(ValidationError):
        RAGRequest(
            question=None,
            mode=RAGMode.FULL_BOOK,
            selected_text=None
        )


def test_rag_request_valid_modes():
    """Test that both valid modes are accepted."""
    # Test full_book mode
    request1 = RAGRequest(
        question="Test question",
        mode=RAGMode.FULL_BOOK,
        selected_text=None
    )
    assert request1.mode == RAGMode.FULL_BOOK

    # Test selected_text mode
    request2 = RAGRequest(
        question="Test question",
        mode=RAGMode.SELECTED_TEXT,
        selected_text="Some text"
    )
    assert request2.mode == RAGMode.SELECTED_TEXT


def test_rag_request_null_selected_text():
    """Test that null selected_text is handled properly."""
    request = RAGRequest(
        question="Test question",
        mode=RAGMode.SELECTED_TEXT,
        selected_text=None
    )

    assert request.selected_text is None


def test_rag_request_empty_selected_text():
    """Test that empty selected_text is converted to None."""
    request = RAGRequest(
        question="Test question",
        mode=RAGMode.SELECTED_TEXT,
        selected_text=""
    )

    assert request.selected_text is None


def test_rag_request_whitespace_selected_text():
    """Test that whitespace-only selected_text is converted to None."""
    request = RAGRequest(
        question="Test question",
        mode=RAGMode.SELECTED_TEXT,
        selected_text="   \n\t   "
    )

    assert request.selected_text is None


def test_rag_request_preserves_valid_selected_text():
    """Test that valid selected_text is preserved."""
    original_text = "This is valid selected text."
    request = RAGRequest(
        question="Test question",
        mode=RAGMode.SELECTED_TEXT,
        selected_text=original_text
    )

    assert request.selected_text == original_text


def test_rag_request_trims_selected_text():
    """Test that selected_text is trimmed of leading/trailing whitespace."""
    original_text = "   This text has spaces   "
    expected_text = "This text has spaces"
    request = RAGRequest(
        question="Test question",
        mode=RAGMode.SELECTED_TEXT,
        selected_text=original_text
    )

    assert request.selected_text == expected_text


def test_question_validation_removes_whitespace():
    """Test that question is properly trimmed."""
    original_question = "   What are robots?   "
    expected_question = "What are robots?"
    request = RAGRequest(
        question=original_question,
        mode=RAGMode.FULL_BOOK,
        selected_text=None
    )

    assert request.question == expected_question