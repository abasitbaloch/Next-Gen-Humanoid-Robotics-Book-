import os
import requests
from fastembed import TextEmbedding
from dotenv import load_dotenv

# 1. Load Secrets
load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

print("🔌 Connecting to Database via HTTP (Bypassing Python Library)...")

# 2. Prepare the Question
query = "What is physical AI?"
print(f"🧠 Converting question to numbers: '{query}'")

embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")
query_vector = list(embedding_model.embed([query]))[0].tolist()

# 3. Send Search Request Directly to the Cloud
search_url = f"{QDRANT_URL}/collections/book_knowledge/points/search"

headers = {
    "api-key": QDRANT_API_KEY,
    "Content-Type": "application/json"
}

payload = {
    "vector": query_vector,
    "limit": 3,
    "with_payload": True
}

try:
    response = requests.post(search_url, headers=headers, json=payload)
    
    if response.status_code == 200:
        results = response.json().get("result", [])
        print(f"✅ SUCCESS! Found {len(results)} results.")
        
        for res in results:
            text = res.get("payload", {}).get("text", "")
            print(f"--- Found: {text[:100]}...")
    else:
        print(f"❌ Error from Server: {response.text}")

except Exception as e:
    print(f"❌ Connection Failed: {e}")