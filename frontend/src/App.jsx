import { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import { Send, Bot, Loader2, BookOpen, Volume2, StopCircle, Trash2, X, Sparkles, Languages } from 'lucide-react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import TranslatorWidget from './components/TranslatorWidget';

export default function App() {
  const [activeWidget, setActiveWidget] = useState(null); // 'chat' | 'translator' | null
  
  // Chatbot State
  const [isSpeaking, setIsSpeaking] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [input, setInput] = useState('');
  const [messages, setMessages] = useState(() => {
    const saved = localStorage.getItem('robotics_chat_history');
    return saved ? JSON.parse(saved) : [{ role: 'ai', text: "Greetings. I am the **Next-Gen Robotics AI**.", sources: [] }];
  });
  const messagesEndRef = useRef(null);

  // Listener for Navbar Signals
  useEffect(() => {
    const handleMessage = (event) => {
      if (event.data === 'open-translator') toggleWidget('translator');
      if (event.data === 'open-chat') toggleWidget('chat');
    };
    window.addEventListener('message', handleMessage);
    return () => window.removeEventListener('message', handleMessage);
  }, []);

  useEffect(() => {
    localStorage.setItem('robotics_chat_history', JSON.stringify(messages));
    if (activeWidget === 'chat') scrollToBottom();
  }, [messages, activeWidget]);

  const scrollToBottom = () => setTimeout(() => messagesEndRef.current?.scrollIntoView({ behavior: "smooth" }), 100);

  // Chatbot Helper Functions
  const speak = (text) => { /* (Use previous speak logic) */ window.speechSynthesis.cancel(); const u = new SpeechSynthesisUtterance(text); u.onstart=()=>setIsSpeaking(true); u.onend=()=>setIsSpeaking(false); window.speechSynthesis.speak(u); };
  const stopSpeaking = () => { window.speechSynthesis.cancel(); setIsSpeaking(false); };
  const clearHistory = () => { if(window.confirm("Clear?")) { setMessages([{ role: 'ai', text: "Reset.", sources: [] }]); stopSpeaking(); }};

  const toggleWidget = (widgetName) => {
    const newWidget = (activeWidget === widgetName) ? null : widgetName;
    setActiveWidget(newWidget);
    // Tell Parent to Expand/Collapse Iframe
    window.parent.postMessage(newWidget ? 'open-widget' : 'close-widget', '*');
  };

  const sendMessage = async (e) => {
    e.preventDefault();
    if (!input.trim()) return;
    const userMessage = input;
    setInput('');
    setMessages(prev => [...prev, { role: 'user', text: userMessage }]);
    setIsLoading(true);
    try {
      const response = await axios.post('https://janabkakarot-robotics-brain.hf.space/chat', { message: userMessage, session_id: "session-1" });
      setMessages(prev => [...prev, { role: 'ai', text: response.data.response, sources: response.data.sources || [] }]);
    } catch (error) { setMessages(prev => [...prev, { role: 'ai', text: "⚠️ Connection error.", sources: [] }]); } 
    finally { setIsLoading(false); }
  };

  // --- DYNAMIC POSITIONING CLASS ---
  // If Translator -> Top Right (Dropdown style)
  // If Chatbot -> Bottom Right (Chat style)
  const positionClass = activeWidget === 'translator' 
    ? 'top-16 right-4 max-h-[500px]' 
    : 'bottom-20 right-4 max-h-[75vh]';

  return (
    <>
      {/* 1. THE SHARED WINDOW (Moves based on active widget) */}
      <div className={`
        fixed ${positionClass} z-50 
        w-[90vw] md:w-[380px] flex flex-col
        transition-all duration-300 ease-out origin-right
        ${activeWidget ? 'scale-100 opacity-100 translate-y-0' : 'scale-90 opacity-0 pointer-events-none'}
      `}>
        
        {/* === TRANSLATOR UI === */}
        {activeWidget === 'translator' && (
          <div className="bg-gray-900/95 backdrop-blur-xl border border-white/10 rounded-2xl shadow-2xl p-1">
             <div className="flex justify-between items-center px-4 py-3 border-b border-white/5 bg-white/5 rounded-t-xl mb-1">
                 <h3 className="font-bold text-gray-200 flex items-center gap-2 text-sm">
                   <Languages size={16} className="text-blue-400"/> Translator
                 </h3>
                 <button onClick={() => toggleWidget('translator')} className="text-gray-400 hover:text-white"><X size={18}/></button>
             </div>
             <div className="p-3">
               <TranslatorWidget />
             </div>
          </div>
        )}

        {/* === CHATBOT UI === */}
        {activeWidget === 'chat' && (
          <div className="bg-gray-900/95 backdrop-blur-xl border border-white/10 rounded-2xl shadow-2xl flex flex-col h-[600px]">
            {/* Header */}
            <div className="flex items-center justify-between px-5 py-4 bg-white/5 border-b border-white/5 rounded-t-2xl">
              <div className="flex items-center gap-3">
                <div className="p-2 bg-gradient-to-tr from-blue-600 to-indigo-600 rounded-xl shadow-lg"><Bot size={20} className="text-white" /></div>
                <div><h1 className="font-semibold text-sm text-white">Next-Gen AI</h1><p className="text-[10px] text-green-400">Online</p></div>
              </div>
              <div className="flex gap-1">
                  {isSpeaking && <button onClick={stopSpeaking} className="p-2 text-rose-400 hover:bg-white/10 rounded"><StopCircle size={16}/></button>}
                  <button onClick={clearHistory} className="p-2 text-gray-400 hover:text-white hover:bg-white/10 rounded"><Trash2 size={16}/></button>
                  <button onClick={() => toggleWidget('chat')} className="p-2 text-gray-400 hover:text-white hover:bg-white/10 rounded"><X size={18}/></button>
              </div>
            </div>
            {/* Messages */}
            <div className="flex-1 overflow-y-auto p-4 space-y-4 custom-scrollbar">
              {messages.map((msg, i) => (
                <div key={i} className={`flex ${msg.role === 'user' ? 'justify-end' : 'justify-start'}`}>
                  <div className={`max-w-[85%] rounded-2xl px-4 py-3 text-sm relative group border ${msg.role === 'user' ? 'bg-blue-600 border-blue-500 text-white' : 'bg-white/5 border-white/10 text-gray-100'}`}>
                      <ReactMarkdown remarkPlugins={[remarkGfm]}>{msg.text}</ReactMarkdown>
                      {msg.role === 'ai' && <button onClick={() => speak(msg.text)} className="absolute -right-6 top-1 text-gray-500 hover:text-blue-400 opacity-0 group-hover:opacity-100"><Volume2 size={14}/></button>}
                  </div>
                </div>
              ))}
              {isLoading && <div className="text-gray-400 text-xs ml-4 flex gap-2"><Loader2 size={12} className="animate-spin"/> Thinking...</div>}
              <div ref={messagesEndRef} />
            </div>
            {/* Input */}
            <form onSubmit={sendMessage} className="p-4 border-t border-white/5">
              <div className="flex items-center gap-2 bg-white/5 border border-white/10 rounded-xl px-2 py-1">
                <input type="text" value={input} onChange={(e) => setInput(e.target.value)} placeholder="Ask..." className="flex-1 bg-transparent text-white text-sm px-2 py-2 focus:outline-none" disabled={isLoading}/>
                <button type="submit" disabled={isLoading || !input.trim()} className="p-2 bg-blue-600 text-white rounded-lg"><Send size={16}/></button>
              </div>
            </form>
          </div>
        )}
      </div>

      {/* 2. CHAT BUTTON (Bottom Right) */}
      <div className="fixed bottom-3 right-3 z-50 group">
        <button onClick={() => toggleWidget('chat')} className={`relative flex items-center justify-center w-14 h-14 rounded-full shadow-xl border border-white/20 ring-1 ring-black/50 transition-all hover:scale-105 active:scale-95 backdrop-blur-sm ${activeWidget === 'chat' ? 'bg-neutral-800 rotate-90' : 'bg-gradient-to-br from-blue-600 to-indigo-700'}`}>
          {activeWidget === 'chat' ? <X size={24} className="text-gray-400" /> : <Sparkles size={22} className="text-white" />}
        </button>
      </div>
    </>
  );
}