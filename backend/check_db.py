import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load secrets
load_dotenv()

# Setup Client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

collection_name = "book_knowledge"

try:
    # Get collection info
    info = qdrant_client.get_collection(collection_name)
    print("✅ CONNECTION SUCCESSFUL!")
    print(f"📚 Collection Name: {collection_name}")
    print(f"🔢 Total Vectors (Chunks of text): {info.points_count}")
    
    if info.points_count == 0:
        print("⚠️ WARNING: The database is EMPTY. You need to run ingest.py again.")
    else:
        print("🚀 SUCCESS: The database contains data! The issue is with the Chat Model.")

except Exception as e:
    print(f"❌ ERROR: Could not connect to Qdrant. Reason: {e}")