import { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import { Send, Bot, Loader2, BookOpen, Volume2, StopCircle, Trash2, X, Sparkles, Languages } from 'lucide-react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import TranslatorWidget from './components/TranslatorWidget';

export default function App() {
  // 1. CHECK THE URL MODE
  const urlParams = new URLSearchParams(window.location.search);
  const appMode = urlParams.get('mode') || 'chat'; // Default to chat

  const [isOpen, setIsOpen] = useState(false);
  
  // NEW: State for Neon Green Theme
  const [isNeon, setIsNeon] = useState(false); 

  // Chatbot State
  const [isSpeaking, setIsSpeaking] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [input, setInput] = useState('');
  
  // NEW: Language State (needed for TTS to know which accent to use)
  const [ttsLang, setTtsLang] = useState('en'); 

  const [messages, setMessages] = useState(() => {
    if (appMode === 'chat') {
      const saved = localStorage.getItem('robotics_chat_history');
      return saved ? JSON.parse(saved) : [{ role: 'ai', text: "Greetings. I am the **Next-Gen Robotics AI**.", sources: [] }];
    }
    return [];
  });
  const messagesEndRef = useRef(null);

  // --- 2. LISTEN FOR PARENT SIGNALS (Navbar Buttons) ---
  useEffect(() => {
    const handleMessage = (event) => {
      // A. Standard Translator Open
      if (appMode === 'translator' && event.data === 'open-translator') {
        setIsOpen(true);
        setIsNeon(false); // Default theme
      }
      
      // B. NEW: World Button (Green Theme + Toggle)
      if (event.data === 'toggle-world-view') {
        setIsOpen(prev => !prev); // Toggle Open/Close
        setIsNeon(true);          // Force Neon Theme
      }
    };
    window.addEventListener('message', handleMessage);
    return () => window.removeEventListener('message', handleMessage);
  }, [appMode]);

  // --- 3. COMMUNICATE STATE TO PARENT ---
  useEffect(() => {
    const signal = appMode === 'chat' 
      ? (isOpen ? 'chat-opened' : 'chat-closed')
      : (isOpen ? 'translator-opened' : 'translator-closed');
      
    window.parent.postMessage(signal, '*');
  }, [isOpen, appMode]);

  // Chatbot Helpers
  useEffect(() => { if (appMode === 'chat' && isOpen) scrollToBottom(); }, [messages, isOpen, appMode]);
  const scrollToBottom = () => setTimeout(() => messagesEndRef.current?.scrollIntoView({ behavior: "smooth" }), 100);

  // --- 4. FIXED SPEAK FUNCTION (Urdu & Roman) ---
  const speak = (text) => { 
    window.speechSynthesis.cancel(); 
    const u = new SpeechSynthesisUtterance(text);
    
    // Logic: Detect language or use manual toggle
    // For now, we assume simple detection or default. 
    // Ideally, you pass the language code to this function.
    
    // Quick Check: If text contains Urdu chars, use Urdu voice
    const hasUrduChars = /[\u0600-\u06FF]/.test(text);

    if (hasUrduChars) {
        u.lang = 'ur-PK'; // Standard Urdu
    } else {
        // If it's English text, it might be "Roman Urdu" or just English
        // We use English voice for both
        u.lang = 'en-US'; 
    }

    u.onstart = () => setIsSpeaking(true); 
    u.onend = () => setIsSpeaking(false); 
    window.speechSynthesis.speak(u); 
  };

  const stopSpeaking = () => { window.speechSynthesis.cancel(); setIsSpeaking(false); };
  
  const clearHistory = () => { 
    if(window.confirm("Clear?")) { 
      setMessages([{ role: 'ai', text: "Reset.", sources: [] }]); 
      stopSpeaking(); 
    }
  };

  const sendMessage = async (e) => {
    e.preventDefault();
    if (!input.trim()) return;
    const userMessage = input;
    setInput('');
    setMessages(prev => [...prev, { role: 'user', text: userMessage }]);
    setIsLoading(true);
    try {
      // NOTE: You might need to update your backend to return "roman_urdu" if requested
      const response = await axios.post('https://janabkakarot-robotics-brain.hf.space/chat', { message: userMessage, session_id: "session-1" });
      setMessages(prev => [...prev, { role: 'ai', text: response.data.response, sources: response.data.sources || [] }]);
    } catch (error) { setMessages(prev => [...prev, { role: 'ai', text: "⚠️ Connection error.", sources: [] }]); } 
    finally { setIsLoading(false); }
  };

  // --- DYNAMIC THEME CLASSES ---
  // If isNeon is true, use Green (#39ff14). If false, use Blue/Indigo.
  const theme = {
    iconBg: isNeon ? 'bg-[#39ff14]' : 'bg-gradient-to-tr from-blue-600 to-indigo-600',
    iconColor: isNeon ? 'text-black' : 'text-white',
    userBubble: isNeon ? 'bg-[#39ff14] text-black border-[#39ff14]' : 'bg-blue-600 border-blue-500 text-white',
    sendBtn: isNeon ? 'bg-[#39ff14] text-black' : 'bg-blue-600 text-white',
    border: isNeon ? 'border-[#39ff14]' : 'border-white/10',
    shadow: isNeon ? 'shadow-[0_0_20px_rgba(57,255,20,0.5)]' : 'shadow-2xl',
    subText: isNeon ? 'text-[#39ff14]' : 'text-green-400'
  };

  // ==========================================
  // RENDER: TRANSLATOR MODE (Top Right)
  // ==========================================
  if (appMode === 'translator') {
    if (!isOpen) return null;
    return (
      <div className={`bg-gray-900/95 backdrop-blur-xl rounded-2xl p-1 w-full h-full overflow-hidden animate-in fade-in slide-in-from-top-5 border ${theme.border} ${theme.shadow}`}>
         <div className="flex justify-between items-center px-4 py-3 border-b border-white/5 bg-white/5 rounded-t-xl mb-1">
             <h3 className={`font-bold flex items-center gap-2 text-sm ${isNeon ? 'text-[#39ff14]' : 'text-gray-200'}`}>
                <Languages size={16} className={isNeon ? 'text-[#39ff14]' : 'text-blue-400'}/> Translator
             </h3>
             <button onClick={() => setIsOpen(false)} className="text-gray-400 hover:text-white"><X size={18}/></button>
         </div>
         <div className="p-3"><TranslatorWidget isNeon={isNeon} /></div>
      </div>
    );
  }

  // ==========================================
  // RENDER: CHAT MODE (Bottom Right)
  // ==========================================
  return (
    <>
      {/* Chat Window */}
      <div className={`
        fixed bottom-20 right-4 z-50 
        w-[90vw] md:w-[380px] h-[600px] max-h-[75vh] flex flex-col
        bg-gray-900/95 backdrop-blur-xl rounded-2xl
        transition-all duration-300 ease-out origin-bottom-right
        ${theme.border} ${theme.shadow}
        ${isOpen ? 'scale-100 opacity-100 translate-y-0' : 'scale-90 opacity-0 pointer-events-none translate-y-10'}
      `}>
          {/* Header */}
          <div className="flex items-center justify-between px-5 py-4 bg-white/5 border-b border-white/5 rounded-t-2xl">
            <div className="flex items-center gap-3">
              <div className={`p-2 rounded-xl shadow-lg ${theme.iconBg}`}>
                  <Bot size={20} className={theme.iconColor} />
              </div>
              <div>
                  <h1 className="font-semibold text-sm text-white">Next-Gen AI</h1>
                  <p className={`text-[10px] ${theme.subText}`}>Online</p>
              </div>
            </div>
            <div className="flex gap-1">
                {isSpeaking && <button onClick={stopSpeaking} className="p-2 text-rose-400 hover:bg-white/10 rounded"><StopCircle size={16}/></button>}
                <button onClick={clearHistory} className="p-2 text-gray-400 hover:text-white hover:bg-white/10 rounded"><Trash2 size={16}/></button>
                <button onClick={() => setIsOpen(!isOpen)} className="p-2 text-gray-400 hover:text-white hover:bg-white/10 rounded"><X size={18}/></button>
            </div>
          </div>

          {/* Messages */}
          <div className="flex-1 overflow-y-auto p-4 space-y-4 custom-scrollbar">
            {messages.map((msg, i) => (
              <div key={i} className={`flex ${msg.role === 'user' ? 'justify-end' : 'justify-start'}`}>
                <div className={`max-w-[85%] rounded-2xl px-4 py-3 text-sm relative group border ${msg.role === 'user' ? theme.userBubble : 'bg-white/5 border-white/10 text-gray-100'}`}>
                    <ReactMarkdown remarkPlugins={[remarkGfm]}>{msg.text}</ReactMarkdown>
                    {msg.role === 'ai' && <button onClick={() => speak(msg.text)} className={`absolute -right-6 top-1 opacity-0 group-hover:opacity-100 ${isNeon ? 'text-[#39ff14]' : 'text-gray-500 hover:text-blue-400'}`}><Volume2 size={14}/></button>}
                </div>
              </div>
            ))}
            {isLoading && <div className="text-gray-400 text-xs ml-4 flex gap-2"><Loader2 size={12} className={`animate-spin ${isNeon ? 'text-[#39ff14]' : ''}`}/> Thinking...</div>}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <form onSubmit={sendMessage} className="p-4 border-t border-white/5">
            <div className={`flex items-center gap-2 bg-white/5 rounded-xl px-2 py-1 border ${isNeon ? 'border-[#39ff14]' : 'border-white/10'}`}>
              <input type="text" value={input} onChange={(e) => setInput(e.target.value)} placeholder="Ask..." className="flex-1 bg-transparent text-white text-sm px-2 py-2 focus:outline-none" disabled={isLoading}/>
              <button type="submit" disabled={isLoading || !input.trim()} className={`p-2 rounded-lg ${theme.sendBtn}`}><Send size={16}/></button>
            </div>
          </form>
      </div>

      {/* Standard Chat Toggle Button (Bottom Right) */}
      <div className="fixed bottom-3 right-3 z-50 group">
        <button onClick={() => { setIsOpen(!isOpen); setIsNeon(false); }} className={`relative flex items-center justify-center w-14 h-14 rounded-full shadow-xl border border-white/20 ring-1 ring-black/50 transition-all hover:scale-105 active:scale-95 backdrop-blur-sm ${isOpen ? 'bg-neutral-800 rotate-90' : 'bg-gradient-to-br from-blue-600 to-indigo-700'}`}>
          <span className={`absolute inset-0 rounded-full bg-blue-400/20 blur-sm ${isOpen ? 'opacity-0' : 'opacity-100'}`}></span>
          {isOpen ? <X size={24} className="text-gray-400" /> : <Sparkles size={22} className="text-white" />}
        </button>
      </div>
    </>
  );
}