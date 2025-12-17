import os
import google.generativeai as genai
from dotenv import load_dotenv

load_dotenv()

class EmbeddingService:
    def __init__(self):
        self.api_key = os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY environment variable not set.")
        genai.configure(api_key=self.api_key)
        self.model_name = os.getenv("EMBEDDING_MODEL", "models/text-embedding-004")

    async def embed_text(self, text: str) -> list[float]:
        """
        Embeds a given text using the configured Gemini embedding model.
        """
        try:
            response = genai.embed_content(
                model=self.model_name,
                content=text,
                task_type="RETRIEVAL_QUERY"
            )
            return response['embedding']
        except Exception as e:
            print(f"Error embedding text: {e}")
            raise

embedding_service = EmbeddingService()
