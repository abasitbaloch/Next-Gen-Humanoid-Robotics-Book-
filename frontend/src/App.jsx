import { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import { Send, Bot, Loader2, BookOpen, Volume2, StopCircle, Trash2, X, MessageSquare, Languages } from 'lucide-react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';

// Import your widget
import TranslatorWidget from './components/TranslatorWidget';

export default function App() {
  // --- STATE: Tracks WHICH widget is open ('chat', 'translator', or null) ---
  const [activeWidget, setActiveWidget] = useState(null); // <--- CHANGED THIS

  // Chatbot State
  const [isSpeaking, setIsSpeaking] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [input, setInput] = useState('');
  const messagesEndRef = useRef(null);

  // --- 1. MEMORY: Load saved messages ---
  const [messages, setMessages] = useState(() => {
    const saved = localStorage.getItem('robotics_chat_history');
    return saved ? JSON.parse(saved) : [
      {
        role: 'ai',
        text: "Hello! I am your Humanoid Robotics expert. Ask me anything about the book!",
        sources: []
      }
    ];
  });

  // --- 2. MEMORY: Save messages ---
  useEffect(() => {
    localStorage.setItem('robotics_chat_history', JSON.stringify(messages));
    if (activeWidget === 'chat') scrollToBottom();
  }, [messages, activeWidget]);

  const scrollToBottom = () => {
    setTimeout(() => {
      messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
    }, 100);
  };

  // --- 3. VOICE: Text-to-Speech ---
  const speak = (text) => {
    window.speechSynthesis.cancel();
    const utterance = new SpeechSynthesisUtterance(text);
    const voices = window.speechSynthesis.getVoices();
    const preferredVoice = voices.find(v => v.name.includes("Google US English")) || voices[0];
    if (preferredVoice) utterance.voice = preferredVoice;
    utterance.onstart = () => setIsSpeaking(true);
    utterance.onend = () => setIsSpeaking(false);
    window.speechSynthesis.speak(utterance);
  };

  const stopSpeaking = () => {
    window.speechSynthesis.cancel();
    setIsSpeaking(false);
  };

  const clearHistory = () => {
    if (window.confirm("Delete chat history?")) {
      const resetState = [{ role: 'ai', text: "History cleared!", sources: [] }];
      setMessages(resetState);
      stopSpeaking();
    }
  };

  // --- 4. TOGGLE LOGIC (Mutual Exclusion) ---
  const toggleWidget = (widgetName) => {
    if (activeWidget === widgetName) {
      // If clicking the already open one, close it
      setActiveWidget(null);
    } else {
      // If clicking a new one, open it (this auto-closes the other one)
      setActiveWidget(widgetName);
    }
    window.parent.postMessage('toggle-widget', '*');
  };

  // --- 5. SEND MESSAGE LOGIC ---
  const sendMessage = async (e) => {
    e.preventDefault();
    if (!input.trim()) return;

    const userMessage = input;
    setInput('');
    setMessages(prev => [...prev, { role: 'user', text: userMessage }]);
    setIsLoading(true);

    try {
      const response = await axios.post('https://janabkakarot-robotics-brain.hf.space/chat', {
        message: userMessage,
        session_id: "session-1"
      });

      setMessages(prev => [...prev, {
        role: 'ai',
        text: response.data.response,
        sources: response.data.sources || []
      }]);
    } catch (error) {
      setMessages(prev => [...prev, {
        role: 'ai',
        text: "⚠️ Connection Error: Is the backend running?",
        sources: []
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {/* Background Content (Optional) */}
      <div className="min-h-screen bg-gray-900 text-white p-10 flex flex-col items-center pt-40">
        <h1 className="text-4xl font-bold mb-4 text-blue-400">Capstone Dashboard</h1>
        <p className="text-gray-400">Select a tool from the dock in the bottom-right.</p>
      </div>

      {/* =========================================
          THE DOCK (Fixed Overlay)
         ========================================= */}
      <div className="fixed bottom-0 right-0 p-4 z-50 font-sans text-gray-100 flex flex-col items-end gap-3">

        {/* --- 1. THE WINDOW ZONE (Shared Space) --- */}
        {/* Only ONE of these two divs will render at a time */}

        {/* A. TRANSLATOR WINDOW */}
        {activeWidget === 'translator' && (
           <div className="mb-4 mr-2 animate-in slide-in-from-bottom-10 fade-in duration-300">
             {/* We wrap your widget in a nice dark card to match the theme */}
             <div className="bg-gray-800 p-2 rounded-2xl border border-gray-700 shadow-2xl w-[350px]">
                <div className="flex justify-between items-center p-2 border-b border-gray-700 mb-2">
                   <h3 className="font-bold text-gray-200 flex items-center gap-2">
                     <Languages size={18} /> Translator
                   </h3>
                   <button onClick={() => setActiveWidget(null)} className="text-gray-400 hover:text-white">
                     <X size={18} />
                   </button>
                </div>
                {/* Your Component */}
                <TranslatorWidget /> 
             </div>
           </div>
        )}

        {/* B. CHATBOT WINDOW */}
        <div className={`
          w-[90vw] md:w-[400px] h-[600px] max-h-[80vh] mb-4
          bg-gray-800 border border-gray-700 rounded-2xl shadow-2xl flex flex-col overflow-hidden
          transition-all duration-300 ease-in-out transform origin-bottom-right
          ${activeWidget === 'chat' ? 'scale-100 opacity-100' : 'scale-0 opacity-0 hidden'}
        `}>
          {/* Header */}
          <div className="flex items-center justify-between p-4 bg-gray-900/90 backdrop-blur border-b border-gray-700">
            <div className="flex items-center gap-3">
              <div className="p-2 bg-blue-600 rounded-lg shadow-lg shadow-blue-500/20">
                <Bot size={20} className="text-white" />
              </div>
              <div>
                <h1 className="font-bold text-sm text-blue-100">Robotics AI</h1>
                <p className="text-[10px] text-gray-400">Ask me anything</p>
              </div>
            </div>
            <div className="flex gap-2">
               {isSpeaking && (
                <button onClick={stopSpeaking} className="p-1.5 text-red-400 hover:bg-gray-700 rounded-md">
                  <StopCircle size={16} className="animate-pulse" />
                </button>
              )}
              <button onClick={clearHistory} className="p-1.5 text-gray-400 hover:text-red-400 hover:bg-gray-700 rounded-md">
                <Trash2 size={16} />
              </button>
              <button onClick={() => setActiveWidget(null)} className="p-1.5 text-gray-400 hover:text-white hover:bg-gray-700 rounded-md">
                <X size={20} />
              </button>
            </div>
          </div>

          {/* Messages */}
          <div className="flex-1 overflow-y-auto p-4 space-y-4 scroll-smooth bg-gray-800 custom-scrollbar">
            {messages.map((msg, index) => (
              <div key={index} className={`flex ${msg.role === 'user' ? 'justify-end' : 'justify-start'}`}>
                <div className={`max-w-[85%] rounded-2xl p-3 text-sm relative group ${
                  msg.role === 'user' 
                    ? 'bg-blue-600 text-white rounded-tr-none' 
                    : 'bg-gray-700 text-gray-200 rounded-tl-none border border-gray-600'
                }`}>
                    <div className="prose prose-invert prose-sm max-w-none">
                       <ReactMarkdown remarkPlugins={[remarkGfm]}>{msg.text}</ReactMarkdown>
                    </div>

                    {msg.role === 'ai' && (
                      <button 
                        onClick={() => speak(msg.text)}
                        className="absolute -right-6 top-0 p-1 text-gray-500 hover:text-blue-400 opacity-0 group-hover:opacity-100 transition-opacity"
                      >
                        <Volume2 size={14} />
                      </button>
                    )}
                    {msg.sources && msg.sources.length > 0 && (
                      <div className="mt-2 pt-2 border-t border-gray-600/50">
                        <p className="text-[10px] font-bold text-gray-500 mb-1 flex items-center gap-1">
                          <BookOpen size={10} /> SOURCES
                        </p>
                        <div className="flex flex-wrap gap-1">
                          {msg.sources.map((s, i) => (
                            <a key={i} href={s.url} target="_blank" className="text-[10px] bg-gray-900 px-1.5 py-0.5 rounded text-blue-300 hover:underline truncate max-w-[150px]">
                              {i+1}. {s.text.substring(0, 15)}...
                            </a>
                          ))}
                        </div>
                      </div>
                    )}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className="flex items-center gap-2 text-gray-500 text-xs ml-2">
                <Loader2 size={12} className="animate-spin" /> Thinking...
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <form onSubmit={sendMessage} className="p-3 bg-gray-900 border-t border-gray-700 flex gap-2">
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Type a question..."
              className="flex-1 bg-gray-800 text-white text-sm rounded-lg px-3 py-2 border border-gray-600 focus:outline-none focus:border-blue-500"
              disabled={isLoading}
            />
            <button 
              type="submit" 
              disabled={isLoading || !input.trim()}
              className="p-2 bg-blue-600 text-white rounded-lg hover:bg-blue-500 disabled:opacity-50"
            >
              <Send size={18} />
            </button>
          </form>
        </div>


        {/* --- 2. THE BUTTON STACK --- */}
        
        {/* Button A: TRANSLATOR */}
        <button
          onClick={() => toggleWidget('translator')}
          className={`
            p-3 rounded-full shadow-lg transition-all duration-300 hover:scale-110 active:scale-95
            ${activeWidget === 'translator' ? 'bg-green-600 text-white' : 'bg-gray-700 text-gray-300 hover:bg-gray-600'}
          `}
          title="Open Translator"
        >
          {activeWidget === 'translator' ? <X size={20} /> : <Languages size={20} />}
        </button>

        {/* Button B: CHATBOT */}
        <button
          onClick={() => toggleWidget('chat')}
          className={`
            p-4 rounded-full shadow-2xl transition-all duration-300 hover:scale-110 active:scale-95
            ${activeWidget === 'chat' ? 'bg-red-500 rotate-90' : 'bg-blue-600 hover:bg-blue-500'}
          `}
          title="Open Chatbot"
        >
          {activeWidget === 'chat' ? <X size={24} className="text-white" /> : <MessageSquare size={24} className="text-white fill-current" />}
        </button>

      </div>
    </>
  );
}