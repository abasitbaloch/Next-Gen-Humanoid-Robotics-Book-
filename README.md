# 🤖 Next-Gen Humanoid Robotics Book

<div align="center">

![Project Status](https://img.shields.io/badge/status-active-success)
![License](https://img.shields.io/badge/license-MIT-blue)
![Docusaurus](https://img.shields.io/badge/docusaurus-3.1.0-blue)
![FastAPI](https://img.shields.io/badge/fastapi-latest-green)
![React](https://img.shields.io/badge/react-19.2.0-blue)

**A comprehensive, interactive book on Physical AI & Humanoid Robotics with integrated RAG chatbot and multi-language translation**

[Features](#-features) • [Architecture](#-architecture) • [Quick Start](#-quick-start) • [Documentation](#-documentation)

</div>

---

## 📖 Overview

This project is an **interactive educational platform** that combines:
- 📚 **Docusaurus-based documentation** covering humanoid robotics, ROS 2, digital twins, AI systems, and VLA (Vision-Language-Action)
- 🤖 **RAG-powered chatbot** that answers questions using the book's content
- 🌍 **Multi-language translator** with text-to-speech capabilities
- 🔗 **Seamless integration** between frontend, backend, and documentation

The platform serves as a complete learning resource for building AI-driven humanoid robots, from ROS 2 basics to advanced VLA systems.

---

## ✨ Features

### 📚 Interactive Documentation
- **4 Core Modules + Capstone Project**
  - Module 1: ROS 2 Fundamentals
  - Module 2: Digital Twin & Simulation
  - Module 3: AI-Robot Brain (Perception, Navigation, Manipulation)
  - Module 4: Vision-Language-Action (VLA) Systems
  - Capstone: Autonomous Humanoid Integration
- **Code examples** with validation framework
- **Diagrams and visualizations** (Mermaid)
- **Exercise validation** system

### 🤖 RAG Chatbot
- **Intelligent Q&A** powered by the book's content
- **Vector search** using Qdrant for semantic retrieval
- **Multi-model fallback** via OpenRouter (Gemini, Llama, Phi-3)
- **Source citations** showing where answers come from
- **Markdown rendering** for rich responses
- **Text-to-speech** with Urdu/Hindi language support
- **Session persistence** via localStorage

### 🌍 Multi-Language Translator
- **Real-time translation** using MyMemory Translation API
- **7+ languages** supported (Spanish, French, German, Urdu, Chinese, Japanese, Russian)
- **Text-to-speech** with accurate language accents
- **Copy-to-clipboard** functionality
- **Clean, modern UI** with dark emerald theme

### 🎨 User Experience
- **Dark emerald theme** throughout
- **Responsive design** (mobile & desktop)
- **Smooth animations** and transitions
- **Iframe-based widgets** for seamless Docusaurus integration
- **PostMessage communication** between parent and widgets

---

## 🏗️ Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Docusaurus Documentation                   │
│  (Vercel Deployment)                                          │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  React Frontend (iframe)                             │   │
│  │  ├── Chatbot Widget                                  │   │
│  │  └── Translator Widget                               │   │
│  └──────────────────────────────────────────────────────┘   │
└───────────────────────┬─────────────────────────────────────┘
                        │ HTTP/HTTPS
                        ▼
┌─────────────────────────────────────────────────────────────┐
│              FastAPI Backend (Hugging Face Space)            │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  /chat endpoint                                       │   │
│  │  1. Embed query (fastembed)                          │   │
│  │  2. Search Qdrant (vector DB)                        │   │
│  │  3. Generate answer (OpenRouter)                      │   │
│  └──────────────────────────────────────────────────────┘   │
└───────────────────────┬─────────────────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        ▼               ▼               ▼
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│   Qdrant     │  │  OpenRouter  │  │  fastembed   │
│  (Vector DB) │  │  (LLM API)   │  │ (Embeddings) │
└──────────────┘  └──────────────┘  └──────────────┘
```

### Component Breakdown

#### 1. **Frontend** (`frontend/`)
- **Technology**: React 19.2.0, Vite, Tailwind CSS
- **Key Files**:
  - `src/App.jsx`: Main chatbot interface with dual-mode support
  - `src/components/TranslatorWidget.jsx`: Translation component
- **Features**:
  - Dual-mode operation (chat/translator) via URL params
  - PostMessage API for Docusaurus communication
  - Markdown rendering with `react-markdown`
  - Web Speech API for text-to-speech
  - LocalStorage for chat history
  - Axios for API calls

#### 2. **Backend** (`backend/`)
- **Technology**: FastAPI, Python
- **Key Files**:
  - `main.py`: Chat API endpoint with RAG pipeline
  - `ingest.py`: Book content ingestion script
  - `check_db.py`: Qdrant connection validator
- **Dependencies**:
  - `fastapi`: Web framework
  - `fastembed`: Local embedding model (BAAI/bge-small-en-v1.5)
  - `qdrant-client`: Vector database client
  - `openai`: OpenRouter API client
  - `beautifulsoup4`: Web scraping
  - `requests`: HTTP client

#### 3. **Documentation** (`docs/`)
- **Technology**: Docusaurus 3.1.0
- **Structure**:
  ```
  docs/
  ├── docs/              # Main documentation content
  │   ├── module-1-ros2/
  │   ├── module-2-digital-twin/
  │   ├── module-3-ai-brain/
  │   ├── module-4-vla/
  │   └── capstone/
  ├── static/            # Images, diagrams
  └── docusaurus.config.js
  ```

#### 4. **Vector Database: Qdrant**
- **Purpose**: Store embedded book content for semantic search
- **Collection**: `book_knowledge`
- **Vector Size**: 384 dimensions (BGE-small-en-v1.5)
- **Distance Metric**: Cosine similarity
- **Storage**: Qdrant Cloud (managed service)

#### 5. **LLM Provider: OpenRouter**
- **Primary Model**: `google/gemini-2.0-flash-exp:free`
- **Fallback Models**:
  - `meta-llama/llama-3.2-3b-instruct:free`
  - `microsoft/phi-3-mini-128k-instruct:free`
- **Strategy**: Automatic fallback on failure
- **API**: OpenAI-compatible endpoint

#### 6. **Embeddings: Hugging Face (fastembed)**
- **Model**: `BAAI/bge-small-en-v1.5`
- **Type**: Local execution (no API costs)
- **Dimensions**: 384
- **Library**: `fastembed` (optimized for speed)

#### 7. **Translation Service: MyMemory**
- **API**: `api.mymemory.translated.net`
- **Free tier**: 10,000 words/day
- **Languages**: 7+ supported languages

---

## 🔌 Connectivity & Integration

### Frontend ↔ Backend
- **Protocol**: HTTPS
- **Endpoint**: `https://janabkakarot-robotics-brain.hf.space/chat`
- **Method**: POST
- **Payload**:
  ```json
  {
    "message": "user question",
    "session_id": "session-1"
  }
  ```
- **Response**:
  ```json
  {
    "response": "AI answer",
    "sources": [{"text": "...", "url": "..."}],
    "session_id": "session-1"
  }
  ```

### Backend ↔ Qdrant
- **Protocol**: HTTPS
- **Connection**: Qdrant Cloud API
- **Authentication**: API Key
- **Operations**:
  - Collection creation/deletion
  - Vector upsert
  - Semantic search (top-k retrieval)

### Backend ↔ OpenRouter
- **Protocol**: HTTPS
- **Base URL**: `https://openrouter.ai/api/v1`
- **Authentication**: API Key
- **Format**: OpenAI-compatible chat completions
- **Fallback**: Automatic model switching

### Docusaurus ↔ Frontend Widgets
- **Method**: iframe embedding
- **Communication**: PostMessage API
- **Signals**:
  - `toggle-chat`: Open/close chatbot
  - `toggle-translator`: Open/close translator
  - `chat-opened/closed`: Status updates
  - `translator-opened/closed`: Status updates

### Ingestion Pipeline
1. **Fetch**: Download `sitemap.xml` from Docusaurus site
2. **Parse**: Extract all page URLs
3. **Scrape**: Use BeautifulSoup to extract text content
4. **Chunk**: Split into 1000-character segments
5. **Embed**: Convert to vectors using fastembed
6. **Store**: Upload to Qdrant with metadata (URL, text)

---

## 🚀 Quick Start

### Prerequisites
- **Node.js** 24.x
- **Python** 3.8+
- **uv** (Python package manager) - optional but recommended
- **Git**

### 1. Clone Repository
```bash
git clone https://github.com/abasitbaloch/Next-Gen-Humanoid-Robotics-Book.git
cd Next-Gen-Humanoid-Robotics-Book
```

### 2. Backend Setup

```bash
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Create .env file
cat > .env << EOF
OPENROUTER_API_KEY=your_openrouter_api_key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
SITE_URL=https://your-docusaurus-site.vercel.app
EOF

# Ingest book content
python ingest.py

# Verify Qdrant connection
python check_db.py

# Start server (local development)
uvicorn main:app --reload --port 8000
```

### 3. Frontend Setup

```bash
cd frontend

# Install dependencies
npm install

# Start development server
npm run dev
```

### 4. Docusaurus Setup

```bash
# Install dependencies
npm install

# Start development server
npm start

# Build for production
npm run build
```

### 5. Deploy Backend to Hugging Face Spaces

1. Create a new Space on [Hugging Face](https://huggingface.co/spaces)
2. Select **Docker** as SDK
3. Upload backend files
4. Add environment variables in Space settings
5. Deploy

### 6. Deploy Docusaurus to Vercel

1. Push code to GitHub
2. Import project on [Vercel](https://vercel.com)
3. Configure build settings:
   - Build Command: `npm run build`
   - Output Directory: `build`
4. Add frontend iframe URL to Docusaurus config
5. Deploy

---

## 📁 Project Structure

```
Next-Gen-Humanoid-Robotics-Book/
│
├── backend/                 # FastAPI backend
│   ├── main.py             # Chat API endpoint
│   ├── ingest.py           # Content ingestion script
│   ├── check_db.py         # Qdrant validator
│   ├── requirements.txt    # Python dependencies
│   └── .env                # Environment variables (not in git)
│
├── frontend/               # React frontend
│   ├── src/
│   │   ├── App.jsx         # Main chatbot component
│   │   └── components/
│   │       └── TranslatorWidget.jsx
│   ├── package.json
│   └── vite.config.js
│
├── docs/                   # Docusaurus documentation
│   ├── docs/               # Content files
│   │   ├── module-1-ros2/
│   │   ├── module-2-digital-twin/
│   │   ├── module-3-ai-brain/
│   │   ├── module-4-vla/
│   │   └── capstone/
│   ├── static/             # Images, diagrams
│   ├── docusaurus.config.js
│   └── sidebars.js
│
├── src/                    # Docusaurus source
│   ├── theme/
│   │   └── Root.js         # Custom theme components
│   ├── pages/
│   └── css/
│
├── specs/                  # Project specifications
├── tests/                  # Test suites
├── docusaurus.config.js    # Root Docusaurus config
├── package.json            # Root package.json
├── vercel.json             # Vercel deployment config
└── README.md               # This file
```

---

## 🔧 Configuration

### Environment Variables

#### Backend (`.env`)
```bash
# OpenRouter API (for LLM)
OPENROUTER_API_KEY=sk-or-v1-...

# Qdrant Vector Database
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=xxx

# Docusaurus Site URL (for ingestion)
SITE_URL=https://your-site.vercel.app
```

#### Frontend
Update `frontend/src/App.jsx` line 82 with your backend URL:
```javascript
const response = await axios.post('YOUR_BACKEND_URL/chat', ...)
```

#### Docusaurus
Update `docusaurus.config.js`:
```javascript
url: 'https://your-site.vercel.app',
baseUrl: '/',
```

---

## 🧪 Testing

### Test Backend Connection
```bash
cd backend
python check_db.py
```

### Test Chat API
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?", "session_id": "test-1"}'
```

### Test Frontend Locally
```bash
cd frontend
npm run dev
# Open http://localhost:5173?mode=chat
```

---

## 📊 Technology Stack Summary

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **Documentation** | Docusaurus 3.1.0 | Static site generator |
| **Frontend** | React 19.2.0, Vite, Tailwind CSS | UI components |
| **Backend** | FastAPI, Python | API server |
| **Vector DB** | Qdrant Cloud | Semantic search storage |
| **LLM** | OpenRouter (Gemini/Llama/Phi-3) | Text generation |
| **Embeddings** | fastembed (BGE-small-en-v1.5) | Text-to-vector conversion |
| **Translation** | MyMemory API | Multi-language support |
| **Deployment** | Vercel (Docusaurus), Hugging Face Spaces (Backend) | Hosting |
| **Build Tools** | npm, pip, uv | Package management |

---

## 🎯 Key Features Explained

### RAG (Retrieval-Augmented Generation) Pipeline

1. **User Query** → Frontend sends question
2. **Embedding** → Query converted to 384-dim vector
3. **Vector Search** → Qdrant finds top 3 similar chunks
4. **Context Assembly** → Relevant text chunks combined
5. **LLM Generation** → OpenRouter generates answer with context
6. **Response** → Answer + sources returned to user

### Multi-Model Fallback Strategy

The backend implements intelligent fallback:
```python
models_to_try = [
    "google/gemini-2.0-flash-exp:free",      # Primary (smartest)
    "meta-llama/llama-3.2-3b-instruct:free", # Backup (fast)
    "microsoft/phi-3-mini-128k-instruct:free" # Deep backup
]
```
If one model fails, the system automatically tries the next.

### Content Ingestion Flow

```
Docusaurus Site
    ↓ (sitemap.xml)
URL List
    ↓ (BeautifulSoup)
Page Content
    ↓ (1000-char chunks)
Text Chunks
    ↓ (fastembed)
Vectors (384-dim)
    ↓ (Qdrant API)
Vector Database
```

---

## 🌐 Deployment

### Vercel (Docusaurus)
- **Automatic** via GitHub integration
- **Build**: `npm run build`
- **Output**: `build/` directory
- **Custom Domain**: Supported

### Hugging Face Spaces (Backend)
- **Docker-based** deployment
- **Environment Variables**: Set in Space settings
- **Auto-redeploy**: On git push
- **Public URL**: `https://username-space-name.hf.space`

### Frontend Widgets
- **Embedded** in Docusaurus via iframe
- **URL Parameters**: `?mode=chat` or `?mode=translator`
- **Communication**: PostMessage API

---

## 🔍 Troubleshooting

### Backend Issues

**Qdrant Connection Failed**
```bash
# Check environment variables
python backend/check_db.py

# Verify API key and URL
echo $QDRANT_API_KEY
```

**OpenRouter API Errors**
- Check API key validity
- Verify free tier limits
- Check model availability

**Empty Vector Database**
```bash
# Re-run ingestion
cd backend
python ingest.py
```

### Frontend Issues

**Chatbot Not Loading**
- Verify backend URL in `App.jsx`
- Check CORS settings in backend
- Inspect browser console for errors

**Translator Not Working**
- Check MyMemory API status
- Verify network connectivity
- Check browser console

### Docusaurus Issues

**Build Fails**
- Check Node.js version (requires 24.x)
- Clear cache: `npm run clear`
- Verify all dependencies installed

**Iframe Not Communicating**
- Check PostMessage listeners
- Verify iframe src URL
- Check browser console for errors

---

## 📚 Documentation Modules

### Module 1: ROS 2 Fundamentals
- ROS 2 architecture
- Humanoid robot structures
- Perception-action loops
- Robot learning
- Sim-to-real transfer

### Module 2: Digital Twin
- Simulation environments
- Robot models (URDF)
- Sensors and physics
- Testing and validation
- Unity alternatives

### Module 3: AI-Robot Brain
- Perception systems
- Navigation and planning
- Manipulation control
- Synthetic data generation
- Sim-to-real validation

### Module 4: Vision-Language-Action (VLA)
- LLM integration
- Task decomposition
- Autonomous behaviors
- Human-robot interaction
- Voice recognition

### Capstone: Autonomous Humanoid
- End-to-end integration
- System validation
- Real-world deployment
- Performance optimization

---

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## 📝 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

## 🙏 Acknowledgments

- **Docusaurus** team for the excellent documentation framework
- **Qdrant** for vector database infrastructure
- **OpenRouter** for free LLM access
- **Hugging Face** for embedding models and Spaces hosting
- **MyMemory** for translation API
- **FastAPI** and **React** communities

---

## 📧 Contact & Support

- **GitHub**: [@abasitbaloch](https://github.com/abasitbaloch)
- **Repository**: [Next-Gen-Humanoid-Robotics-Book](https://github.com/abasitbaloch/Next-Gen-Humanoid-Robotics-Book-)
- **Issues**: [GitHub Issues](https://github.com/abasitbaloch/Next-Gen-Humanoid-Robotics-Book-/issues)

---

## 🗺️ Roadmap

- [ ] Add more languages to translator
- [ ] Implement chat history persistence (Neon DB)
- [ ] Add voice input for chatbot
- [ ] Enhance diagram rendering
- [ ] Add code execution sandbox
- [ ] Implement user authentication
- [ ] Add progress tracking
- [ ] Create mobile app version

---

<div align="center">

**Built with ❤️ for the robotics community**

⭐ Star this repo if you find it helpful!

</div>
