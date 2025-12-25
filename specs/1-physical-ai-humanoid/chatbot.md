# Project: RAG Chatbot Backend (Free Tier Edition)

## 1. Goal
Build a backend for a book chatbot. It must scrape the book, store it in a vector database, and answer questions using a free AI model via OpenRouter.

## 2. Tech Stack (Strict Requirements)
- **Language:** Python
- **Framework:** FastAPI
- **Package Manager:** uv
- **AI Provider:** OpenRouter (Base URL: https://openrouter.ai/api/v1)
- **Vector DB:** Qdrant Cloud
- **History DB:** Neon Serverless Postgres
- **Embeddings:** `fastembed` (Use "BAAI/bge-small-en-v1.5" running locally to avoid API costs).

## 3. Architecture & Features

### A. Environment Setup
The app must use a `.env` file for these keys:
- `OPENROUTER_API_KEY`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `NEON_DATABASE_URL`
- `SITE_URL` (The live link to the Docusaurus book)

### B. Ingestion Script (`backend/ingest.py`)
A script to "teach" the AI the book.
1. **Fetch:** Download `sitemap.xml` from `SITE_URL`.
2. **Scrape:** Visit every page link and extract text (use `BeautifulSoup`).
3. **Chunk:** Split text into chunks of 1000 characters.
4. **Embed:** Convert chunks to vectors using `fastembed`.
5. **Store:** Upload vectors to Qdrant collection named `book_knowledge`.

### C. Chat API (`backend/main.py`)
The server that answers questions.
1. **Endpoint:** `POST /chat`
2. **Logic:**
   - Receive user query.
   - Search Qdrant for top 3 matching chunks.
   - **Important:** Use OpenRouter model `google/gemini-2.0-flash-exp:free` (it is free and smart).
   - Send the User Question + Found Chunks to the AI.
   - Save the chat history to Neon DB.
   - Return the answer.

## 4. Instructions for Claude
- Use `uv` for all dependency management.
- Ensure the Qdrant client uses the API Key.
- Create a `backend` folder for all this code.