import os
import glob
import re
import uuid
from dotenv import load_dotenv

from dotenv import load_dotenv

from api.services.embedding_service import embedding_service
from api.services.qdrant_client import qdrant_service

load_dotenv()

TEXTBOOK_DOCS_PATH = "textbook/docs"
CHUNK_SIZE = 500  # characters
CHUNK_OVERLAP = 50 # characters

def clean_text(text: str) -> str:
    """Removes markdown and other non-content elements from the text."""
    # Remove Docusaurus-specific components like <Tabs>, <TabItem>
    text = re.sub(r'<Tabs>.*?</Tabs>', '', text, flags=re.DOTALL)
    text = re.sub(r'<TabItem.*?>', '', text)
    text = re.sub(r'</TabItem>', '', text)
    
    # Remove front matter (YAML at the beginning of the file)
    text = re.sub(r'---\n(.*?)\n---', '', text, flags=re.DOTALL)
    
    # Remove images and links
    text = re.sub(r'!\\[.*?\\]\(.*?\)','',text) # Images
    text = re.sub(r'\[.*?\\]\(.*?\)','',text) # Links

    # Remove markdown headings, bold, italics, etc.
    text = re.sub(r'#+\s*', '', text) # Headings
    text = re.sub(r'\*\*|__|\*|_', '', text) # Bold and italics
    text = re.sub(r'`{1,3}.*?`{1,3}', '', text) # Inline code
    
    # Remove multiple newlines and extra spaces
    text = re.sub(r'\n\s*\n', '\n', text)
    text = re.sub(r' +', ' ', text)
    
    return text.strip()

def chunk_text(text: str) -> list[str]:
    """Splits text into chunks of CHUNK_SIZE with CHUNK_OVERLAP."""
    chunks = []
    if not text:
        return chunks

    start = 0
    while start < len(text):
        end = start + CHUNK_SIZE
        chunk = text[start:end]
        chunks.append(chunk)
        if end >= len(text):
            break
        start += CHUNK_SIZE - CHUNK_OVERLAP
    return chunks

async def index_textbook_content():
    print(f"Starting to index content from {TEXTBOOK_DOCS_PATH}...")
    
    # 1. Identify all markdown files
    md_files = glob.glob(os.path.join(TEXTBOOK_DOCS_PATH, "**/*.md"), recursive=True)
    md_files.extend(glob.glob(os.path.join(TEXTBOOK_DOCS_PATH, "**/*.mdx"), recursive=True))

    if not md_files:
        print(f"No markdown files found in {TEXTBOOK_DOCS_PATH}")
        return

    documents_to_index = [] # List of dictionaries for Qdrant batch upsert
    for file_path in md_files:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Extract title from the file path or content (e.g., heading or front matter)
        # For simplicity, let's use the file name as title for now
        file_name = os.path.basename(file_path)
        doc_id = os.path.splitext(file_name)[0] # Use filename without extension as a simple ID
        
        cleaned_content = clean_text(content)
        chunks = chunk_text(cleaned_content)

        print(f"Processing {file_path} -> {len(chunks)} chunks.")
        
        for i, chunk in enumerate(chunks):
            # Create a unique ID for each chunk (UUID for Qdrant)
            point_id = str(uuid.uuid4())
            original_chunk_identifier = f"{doc_id}-{i}" # For traceability
            
            # Embed the chunk
            vector = await embedding_service.embed_text(chunk)

            # Prepare payload for Qdrant
            payload = {
                "text": chunk,
                "source_file": file_path,
                "original_chunk_id": original_chunk_identifier,
                # Add more metadata if available, e.g., title, chapter, URL
            }
            
            documents_to_index.append({
                "id": point_id,
                "vector": vector,
                "payload": payload
            })

    # Qdrant batch upsert
    if documents_to_index:
        qdrant_service.upsert_vectors(documents_to_index)
        print(f"Successfully indexed {len(documents_to_index)} documents into Qdrant.")
    else:
        print("No documents to index.")


if __name__ == "__main__":
    import asyncio
    asyncio.run(index_textbook_content())
