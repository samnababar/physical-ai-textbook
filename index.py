from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import google.generativeai as genai
import os
from dotenv import load_dotenv

from api.services.embedding_service import embedding_service
from api.services.qdrant_client import qdrant_service

load_dotenv()
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
generation_model = genai.GenerativeModel(os.getenv("GENERATION_MODEL", "gemini-1.5-flash"))

app = FastAPI(
    title="Textbook RAG Chatbot",
    description="A RAG-based chatbot for the 'Physical AI & Humanoid Robotics' textbook.",
    version="0.1.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Allow requests from your Docusaurus frontend
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ChatQuery(BaseModel):
    query: str

class ChatResponse(BaseModel):
    answer: str

@app.get("/")
def read_root():
    return {"message": "Welcome to the RAG Chatbot API"}

@app.post("/chat", response_model=ChatResponse)
async def chat(chat_query: ChatQuery):
    """
    Accepts a user's query and returns a mock answer.
    """
    print(f"Received query: {chat_query.query}")

    # 1. Embed the user's query.
    query_vector = await embedding_service.embed_text(chat_query.query)

    # 2. Query the Qdrant vector store to find relevant text chunks.
    relevant_chunks = qdrant_service.search(query_vector)
    
    # 3. Use the retrieved chunks to build a context for an LLM.
    context = " ".join([chunk["payload"]["text"] for chunk in relevant_chunks])
    
    # If no relevant chunks are found, inform the user or handle appropriately
    if not context.strip():
        return {"answer": "I couldn't find any relevant information for your query in the textbook."}

    # 4. Construct the LLM prompt.
    prompt = f"""You are a helpful assistant that answers questions based on the provided textbook content.
    
    Here is the relevant textbook content:
    ---
    {context}
    ---
    
    Based on the above content, please answer the following question:
    {chat_query.query}
    
    If the answer cannot be found in the provided content, respond with "I cannot answer this question based on the provided textbook content."
    """

    # 5. Call the LLM with the context and query to get an answer.
    try:
        response = await generation_model.generate_content(prompt)
        llm_answer = response.text
    except Exception as e:
        print(f"Error generating content with LLM: {e}")
        llm_answer = "An error occurred while trying to generate a response."

    return {"answer": llm_answer}
