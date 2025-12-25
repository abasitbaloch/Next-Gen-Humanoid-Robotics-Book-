import os
import requests
from typing import List
from pydantic import BaseModel
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastembed import TextEmbedding
from openai import OpenAI
from dotenv import load_dotenv

# 1. Load Secrets
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), '.env'))

OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# 2. Initialize App
app = FastAPI(title="RAG Chatbot Backend")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 3. Initialize Brain
embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")

# 4. Initialize Chat AI
ai_client = OpenAI(
    base_url="https://openrouter.ai/api/v1",
    api_key=OPENROUTER_API_KEY,
)

# 5. Data Models
class ChatRequest(BaseModel):
    message: str
    session_id: str

class ChatResponse(BaseModel):
    response: str
    sources: List[dict]
    session_id: str

@app.get("/")
def home():
    return {"message": "✅ Backend is RUNNING!"}

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    print(f"📩 Received: {request.message}")

    # --- STEP A: SEARCH ---
    try:
        query_vector = list(embedding_model.embed([request.message]))[0].tolist()
        
        search_url = f"{QDRANT_URL}/collections/book_knowledge/points/search"
        headers = {"api-key": QDRANT_API_KEY, "Content-Type": "application/json"}
        payload = {"vector": query_vector, "limit": 3, "with_payload": True}

        response = requests.post(search_url, headers=headers, json=payload)
        response.raise_for_status()
        search_results = response.json().get("result", [])
        
        context_text = ""
        sources = []
        for res in search_results:
            data = res.get("payload", {})
            text = data.get("text", "")
            url = data.get("source_url", "")
            if text:
                context_text += f"{text}\n---\n"
                sources.append({"text": text[:100] + "...", "url": url})

    except Exception as e:
        print(f"⚠️ Search Error: {e}")
        context_text = ""
        sources = []

    # --- STEP B: ASK AI (WITH BACKUP) ---
    system_prompt = """You are a helpful expert on Humanoid Robotics. 
    Use the provided Context to answer. If the answer is not in the context, say you don't know."""
    
    user_prompt = f"Context:\n{context_text}\n\nQuestion:\n{request.message}"

    # LIST OF BRAINS TO TRY (If one fails, try the next)
    models_to_try = [
        "google/gemini-2.0-flash-exp:free",     # 1. Smartest
        "meta-llama/llama-3.2-3b-instruct:free", # 2. Fast Backup
        "microsoft/phi-3-mini-128k-instruct:free" # 3. Deep Backup
    ]

    ai_answer = "I'm having trouble connecting to my brain right now. Please try again."

    for model in models_to_try:
        try:
            print(f"🤖 Trying model: {model}...")
            completion = ai_client.chat.completions.create(
                model=model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt},
                ]
            )
            ai_answer = completion.choices[0].message.content
            print(f"✅ Success with {model}!")
            break # Stop loop if successful
        except Exception as e:
            print(f"❌ {model} Failed: {e}")
            continue # Try next model

    return ChatResponse(
        response=ai_answer,
        sources=sources,
        session_id=request.session_id
    )