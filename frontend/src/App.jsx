import { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import { Send, Bot, Loader2, BookOpen, Volume2, StopCircle, Trash2, X, MessageSquare, Languages } from 'lucide-react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';

import TranslatorWidget from './components/TranslatorWidget';

export default function App() {
  const [activeWidget, setActiveWidget] = useState(null); 
  const [isSpeaking, setIsSpeaking] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [input, setInput] = useState('');
  const messagesEndRef = useRef(null);

  // --- MEMORY & SPEECH LOGIC (Kept same) ---
  const [messages, setMessages] = useState(() => {
    const saved = localStorage.getItem('robotics_chat_history');
    return saved ? JSON.parse(saved) : [{ role: 'ai', text: "Hello! I am your Humanoid Robotics expert.", sources: [] }];
  });

  useEffect(() => {
    localStorage.setItem('robotics_chat_history', JSON.stringify(messages));
    if (activeWidget === 'chat') scrollToBottom();
  }, [messages, activeWidget]);

  const scrollToBottom = () => {
    setTimeout(() => {
      messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
    }, 100);
  };

  const speak = (text) => {
    window.speechSynthesis.cancel();
    const u = new SpeechSynthesisUtterance(text);
    u.onstart = () => setIsSpeaking(true);
    u.onend = () => setIsSpeaking(false);
    window.speechSynthesis.speak(u);
  };

  const stopSpeaking = () => {
    window.speechSynthesis.cancel();
    setIsSpeaking(false);
  };

  const clearHistory = () => {
    if (window.confirm("Delete chat history?")) {
      setMessages([{ role: 'ai', text: "History cleared!", sources: [] }]);
      stopSpeaking();
    }
  };

  const toggleWidget = (widgetName) => {
    if (activeWidget === widgetName) {
      setActiveWidget(null); // Close if same
    } else {
      setActiveWidget(widgetName); // Open new
    }
    // Signal to parent iframe if needed
    window.parent.postMessage('toggle-widget', '*');
  };

  const sendMessage = async (e) => {
    e.preventDefault();
    if (!input.trim()) return;
    const userMessage = input;
    setInput('');
    setMessages(prev => [...prev, { role: 'user', text: userMessage }]);
    setIsLoading(true);

    try {
      const response = await axios.post('https://janabkakarot-robotics-brain.hf.space/chat', {
        message: userMessage, session_id: "session-1"
      });
      setMessages(prev => [...prev, { role: 'ai', text: response.data.response, sources: response.data.sources || [] }]);
    } catch (error) {
      setMessages(prev => [...prev, { role: 'ai', text: "⚠️ Connection Error.", sources: [] }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {/* -------------------------------------------
          1. MAIN PAGE BACKGROUND
          This is the content BEHIND the widgets.
         ------------------------------------------- */}
      <div className="min-h-screen bg-gray-900 text-white flex flex-col items-center justify-center p-4">
        <div className="text-center max-w-2xl">
          <h1 className="text-5xl font-bold mb-6 text-blue-500 tracking-tight">
            Capstone Dashboard
          </h1>
          <p className="text-xl text-gray-400 mb-8">
            Welcome to the Robotics Integration Hub.
          </p>
          
          <div className="p-6 bg-gray-800 rounded-xl border border-gray-700 shadow-xl">
             <h2 className="text-lg font-semibold text-white mb-2">System Status</h2>
             <div className="flex items-center justify-center gap-4 text-sm text-gray-400">
                <span className="flex items-center gap-2"><span className="w-2 h-2 bg-green-500 rounded-full"></span> Translator Ready</span>
                <span className="flex items-center gap-2"><span className="w-2 h-2 bg-green-500 rounded-full"></span> Chatbot Online</span>
             </div>
          </div>
          
          <p className="mt-12 text-gray-500 text-sm">
            ↙ Use the Dock in the bottom-right corner to open tools.
          </p>
        </div>
      </div>

      {/* -------------------------------------------
          2. WIDGET WINDOWS (The Popups)
          Positioned Fixed, Above the buttons
         ------------------------------------------- */}
      <div className="fixed bottom-24 right-6 z-40 flex flex-col items-end gap-4 pointer-events-none">
        
        {/* A. TRANSLATOR WINDOW */}
        {activeWidget === 'translator' && (
           <div className="pointer-events-auto bg-gray-800 p-1 rounded-2xl border border-gray-700 shadow-2xl w-[350px] animate-in slide-in-from-bottom-10 fade-in duration-200">
              <div className="flex justify-between items-center p-3 border-b border-gray-700 bg-gray-900/50 rounded-t-xl">
                 <h3 className="font-bold text-gray-200 flex items-center gap-2 text-sm">
                   <Languages size={16} className="text-blue-400"/> Quick Translator
                 </h3>
                 <button onClick={() => setActiveWidget(null)} className="text-gray-400 hover:text-white">
                   <X size={18} />
                 </button>
              </div>
              <div className="p-3">
                <TranslatorWidget /> 
              </div>
           </div>
        )}

        {/* B. CHATBOT WINDOW */}
        <div className={`
          pointer-events-auto
          w-[90vw] md:w-[400px] h-[600px] max-h-[70vh]
          bg-gray-800 border border-gray-700 rounded-2xl shadow-2xl flex flex-col overflow-hidden
          transition-all duration-300 ease-in-out origin-bottom-right
          ${activeWidget === 'chat' ? 'scale-100 opacity-100' : 'scale-0 opacity-0 absolute'}
        `}>
          {/* Chat Header */}
          <div className="flex items-center justify-between p-4 bg-gray-900 border-b border-gray-700">
            <div className="flex items-center gap-3">
              <div className="p-2 bg-blue-600 rounded-lg shadow-lg">
                <Bot size={20} className="text-white" />
              </div>
              <div>
                <h1 className="font-bold text-sm text-blue-100">Robotics AI</h1>
                <p className="text-[10px] text-gray-400">Online</p>
              </div>
            </div>
            <div className="flex gap-1">
               {isSpeaking && <button onClick={stopSpeaking} className="p-2 text-red-400 hover:bg-gray-700 rounded"><StopCircle size={16}/></button>}
               <button onClick={clearHistory} className="p-2 text-gray-400 hover:text-red-400 hover:bg-gray-700 rounded"><Trash2 size={16}/></button>
               <button onClick={() => setActiveWidget(null)} className="p-2 text-gray-400 hover:text-white hover:bg-gray-700 rounded"><X size={18}/></button>
            </div>
          </div>

          {/* Chat Messages */}
          <div className="flex-1 overflow-y-auto p-4 space-y-4 bg-gray-800 custom-scrollbar">
            {messages.map((msg, index) => (
              <div key={index} className={`flex ${msg.role === 'user' ? 'justify-end' : 'justify-start'}`}>
                <div className={`max-w-[85%] rounded-2xl p-3 text-sm group relative ${msg.role === 'user' ? 'bg-blue-600 text-white' : 'bg-gray-700 text-gray-200 border border-gray-600'}`}>
                    <div className="prose prose-invert prose-sm"><ReactMarkdown remarkPlugins={[remarkGfm]}>{msg.text}</ReactMarkdown></div>
                    {msg.role === 'ai' && <button onClick={() => speak(msg.text)} className="absolute -right-6 top-0 p-1 text-gray-500 hover:text-blue-400 opacity-0 group-hover:opacity-100"><Volume2 size={14} /></button>}
                </div>
              </div>
            ))}
            {isLoading && <div className="text-gray-500 text-xs ml-2 flex gap-2"><Loader2 size={12} className="animate-spin" /> Thinking...</div>}
            <div ref={messagesEndRef} />
          </div>

          {/* Chat Input */}
          <form onSubmit={sendMessage} className="p-3 bg-gray-900 border-t border-gray-700 flex gap-2">
            <input type="text" value={input} onChange={(e) => setInput(e.target.value)} placeholder="Ask a question..." className="flex-1 bg-gray-800 text-white text-sm rounded-lg px-3 py-2 border border-gray-600 focus:border-blue-500 outline-none" disabled={isLoading}/>
            <button type="submit" disabled={isLoading || !input.trim()} className="p-2 bg-blue-600 text-white rounded-lg hover:bg-blue-500 disabled:opacity-50"><Send size={18} /></button>
          </form>
        </div>
      </div>

      {/* -------------------------------------------
          3. THE DOCK (Buttons Only)
          Strictly fixed to bottom-right
         ------------------------------------------- */}
      <div className="fixed bottom-6 right-6 z-50 flex flex-col gap-3">
        
        {/* Translator Toggle */}
        <button
          onClick={() => toggleWidget('translator')}
          className={`
            p-3 rounded-full shadow-lg transition-all duration-200 hover:scale-105 border border-gray-700
            ${activeWidget === 'translator' ? 'bg-gray-200 text-gray-900' : 'bg-gray-800 text-gray-400 hover:text-white hover:bg-gray-700'}
          `}
          title="Open Translator"
        >
          <Languages size={24} />
        </button>

        {/* Chatbot Toggle */}
        <button
          onClick={() => toggleWidget('chat')}
          className={`
            p-4 rounded-full shadow-xl transition-all duration-300 hover:scale-105
            ${activeWidget === 'chat' ? 'bg-red-500 rotate-90 text-white' : 'bg-blue-600 text-white hover:bg-blue-500'}
          `}
          title="Open Chatbot"
        >
          {activeWidget === 'chat' ? <X size={24} /> : <MessageSquare size={24} className="fill-current" />}
        </button>
      </div>

    </>
  );
}