import os
from qdrant_client import QdrantClient
from fastembed import TextEmbedding
from dotenv import load_dotenv

load_dotenv()

print("🔌 Connecting...")
client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

print("\n--- CLIENT CAPABILITIES ---")
# This prints all available commands. Look for 'search' in the output!
commands = dir(client)
if "search" in commands:
    print("✅ GREAT NEWS: .search() exists!")
else:
    print("❌ BAD NEWS: .search() is missing.")
    print("Here is what DOES exist (first 20):")
    print([c for c in commands if not c.startswith("_")][:20])

print("\n🔎 Attempting Search...")
try:
    embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")
    query_vector = list(embedding_model.embed(["Physical AI"]))[0].tolist()
    
    results = client.search(
        collection_name="book_knowledge",
        query_vector=query_vector,
        limit=3
    )
    print(f"✅ SUCCESS! Found {len(results)} results.")
    for res in results:
        print(f" - Found: {res.payload.get('source_url', 'No URL')}")

except AttributeError:
    print("❌ FATAL: Client object is still broken.")
except Exception as e:
    print(f"❌ ERROR: {e}")