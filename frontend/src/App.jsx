import { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import { Send, Bot, Loader2, Volume2, StopCircle, Trash2, X, Sparkles, Languages } from 'lucide-react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import TranslatorWidget from './components/TranslatorWidget';

export default function App() {
  const urlParams = new URLSearchParams(window.location.search);
  const appMode = urlParams.get('mode') || 'chat';

  const [isOpen, setIsOpen] = useState(false);
  const [isNeon, setIsNeon] = useState(false); // Controls the Green Theme
  const [isSpeaking, setIsSpeaking] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [input, setInput] = useState('');
  
  // Chat History
  const [messages, setMessages] = useState(() => {
    if (appMode === 'chat') {
      const saved = localStorage.getItem('robotics_chat_history');
      return saved ? JSON.parse(saved) : [{ role: 'ai', text: "Greetings. I am the **Next-Gen Robotics AI**.", sources: [] }];
    }
    return [];
  });
  const messagesEndRef = useRef(null);

  // --- LISTEN FOR NAVBAR BUTTON ---
  useEffect(() => {
    const handleMessage = (event) => {
      // If we receive the 'toggle-world-view' signal from the Navbar
      if (event.data === 'toggle-world-view') {
        setIsOpen(prev => !prev); // Toggle Open/Close
        setIsNeon(true);          // Switch to Dark Green Theme
      }
    };
    window.addEventListener('message', handleMessage);
    return () => window.removeEventListener('message', handleMessage);
  }, []);

  // Communicate with Parent (Docusaurus)
  useEffect(() => {
    const signal = isOpen ? 'chat-opened' : 'chat-closed';
    window.parent.postMessage(signal, '*');
  }, [isOpen]);

  useEffect(() => { if (appMode === 'chat' && isOpen) scrollToBottom(); }, [messages, isOpen]);
  const scrollToBottom = () => setTimeout(() => messagesEndRef.current?.scrollIntoView({ behavior: "smooth" }), 100);

  // --- FIXED SPEECH FUNCTION (Urdu & Hindi Fallback) ---
  const speak = (text) => { 
    window.speechSynthesis.cancel(); 
    const u = new SpeechSynthesisUtterance(text);
    
    // 1. Detect if text is Urdu/Arabic Script
    const isUrduScript = /[\u0600-\u06FF]/.test(text);

    if (isUrduScript) {
        // Try to find an Urdu voice first
        const voices = window.speechSynthesis.getVoices();
        const urduVoice = voices.find(v => v.lang.includes('ur'));
        const hindiVoice = voices.find(v => v.lang.includes('hi')); // Fallback

        if (urduVoice) {
            u.voice = urduVoice;
            u.lang = 'ur-PK';
        } else if (hindiVoice) {
            // TRICK: If no Urdu voice, use Hindi (sounds the same for speaking)
            u.voice = hindiVoice; 
            u.lang = 'hi-IN';
        }
        // If neither, it will use default (which might be Arabic or silent, but we tried)
    } else {
        // English / Roman Urdu
        u.lang = 'en-US'; 
        u.rate = 0.9;
    }

    u.onstart = () => setIsSpeaking(true); 
    u.onend = () => setIsSpeaking(false); 
    u.onerror = (e) => console.error("Speech Error:", e);
    
    window.speechSynthesis.speak(u); 
  };

  const stopSpeaking = () => { window.speechSynthesis.cancel(); setIsSpeaking(false); };
  
  const clearHistory = () => { 
    if(window.confirm("Clear chat?")) { 
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
      const response = await axios.post('https://janabkakarot-robotics-brain.hf.space/chat', { message: userMessage, session_id: "session-1" });
      setMessages(prev => [...prev, { role: 'ai', text: response.data.response, sources: response.data.sources || [] }]);
    } catch (error) { setMessages(prev => [...prev, { role: 'ai', text: "⚠️ Connection error.", sources: [] }]); } 
    finally { setIsLoading(false); }
  };

  // --- THEME SETTINGS (Dark Green Logic) ---
  const theme = {
    // Outer Container
    containerBg: isNeon ? 'bg-black border-2 border-[#39ff14] shadow-[0_0_20px_rgba(57,255,20,0.3)]' : 'bg-gray-900/95 border border-white/10 shadow-2xl',
    
    // Header
    headerBg: isNeon ? 'bg-[#051a05] border-b border-[#39ff14]' : 'bg-white/5 border-b border-white/5',
    headerText: isNeon ? 'text-[#39ff14]' : 'text-white',
    onlineStatus: isNeon ? 'text-[#39ff14]' : 'text-green-400',
    
    // Icons
    iconBox: isNeon ? 'bg-black border border-[#39ff14] shadow-[0_0_10px_#39ff14]' : 'bg-gradient-to-tr from-blue-600 to-indigo-600 shadow-lg',
    iconColor: isNeon ? 'text-[#39ff14]' : 'text-white',

    // Bubbles
    userBubble: isNeon ? 'bg-[#0a2e0a] border border-[#39ff14] text-[#39ff14]' : 'bg-blue-600 border-blue-500 text-white',
    aiBubble: isNeon ? 'bg-black border border-[#39ff14]/50 text-[#39ff14]' : 'bg-white/5 border-white/10 text-gray-100',
    
    // Input Area
    inputBg: isNeon ? 'bg-black border border-[#39ff14] text-[#39ff14]' : 'bg-white/5 border border-white/10 text-white',
    sendBtn: isNeon ? 'bg-[#39ff14] text-black hover:bg-[#32cc12]' : 'bg-blue-600 text-white',
  };

  // ==========================================
  // RENDER: TRANSLATOR MODE
  // ==========================================
  if (appMode === 'translator') {
    if (!isOpen) return null;
    return (
      <div className={`backdrop-blur-xl rounded-2xl p-1 w-full h-full overflow-hidden animate-in fade-in slide-in-from-top-5 ${theme.containerBg}`}>
         <div className={`flex justify-between items-center px-4 py-3 rounded-t-xl mb-1 ${theme.headerBg}`}>
             <h3 className={`font-bold flex items-center gap-2 text-sm ${theme.headerText}`}>
                <Languages size={16} /> Translator
             </h3>
             <button onClick={() => setIsOpen(false)} className={`hover:text-white ${isNeon ? 'text-[#39ff14]' : 'text-gray-400'}`}><X size={18}/></button>
         </div>
         {/* Pass the theme prop to widget if needed, or just let it inherit styles */}
         <div className="p-3"><TranslatorWidget isNeon={isNeon} /></div>
      </div>
    );
  }

  // ==========================================
  // RENDER: CHAT MODE
  // ==========================================
  return (
    <>
      <div className={`
        fixed bottom-20 right-4 z-50 
        w-[90vw] md:w-[380px] h-[600px] max-h-[75vh] flex flex-col
        backdrop-blur-xl rounded-2xl
        transition-all duration-300 ease-out origin-bottom-right
        ${theme.containerBg}
        ${isOpen ? 'scale-100 opacity-100 translate-y-0' : 'scale-90 opacity-0 pointer-events-none translate-y-10'}
      `}>
          {/* Header */}
          <div className={`flex items-center justify-between px-5 py-4 rounded-t-2xl ${theme.headerBg}`}>
            <div className="flex items-center gap-3">
              <div className={`p-2 rounded-xl ${theme.iconBox}`}>
                  <Bot size={20} className={theme.iconColor} />
              </div>
              <div>
                  <h1 className={`font-semibold text-sm ${theme.headerText}`}>Next-Gen AI</h1>
                  <p className={`text-[10px] ${theme.onlineStatus}`}>Online</p>
              </div>
            </div>
            <div className="flex gap-1">
                {isSpeaking && <button onClick={stopSpeaking} className="p-2 text-rose-400 hover:bg-white/10 rounded"><StopCircle size={16}/></button>}
                <button onClick={clearHistory} className={`p-2 rounded hover:bg-white/10 ${isNeon ? 'text-[#39ff14]' : 'text-gray-400'}`}><Trash2 size={16}/></button>
                <button onClick={() => setIsOpen(!isOpen)} className={`p-2 rounded hover:bg-white/10 ${isNeon ? 'text-[#39ff14]' : 'text-gray-400'}`}><X size={18}/></button>
            </div>
          </div>

          {/* Messages */}
          <div className="flex-1 overflow-y-auto p-4 space-y-4 custom-scrollbar">
            {messages.map((msg, i) => (
              <div key={i} className={`flex ${msg.role === 'user' ? 'justify-end' : 'justify-start'}`}>
                <div className={`max-w-[85%] rounded-2xl px-4 py-3 text-sm relative group ${msg.role === 'user' ? theme.userBubble : theme.aiBubble}`}>
                    <ReactMarkdown remarkPlugins={[remarkGfm]}>{msg.text}</ReactMarkdown>
                    {msg.role === 'ai' && (
                        <button onClick={() => speak(msg.text)} className={`absolute -right-6 top-1 opacity-0 group-hover:opacity-100 transition ${isNeon ? 'text-[#39ff14]' : 'text-gray-500'}`}>
                            <Volume2 size={14}/>
                        </button>
                    )}
                </div>
              </div>
            ))}
            {isLoading && <div className={`text-xs ml-4 flex gap-2 ${isNeon ? 'text-[#39ff14]' : 'text-gray-400'}`}><Loader2 size={12} className="animate-spin"/> Thinking...</div>}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <form onSubmit={sendMessage} className={`p-4 border-t ${isNeon ? 'border-[#39ff14]/30' : 'border-white/5'}`}>
            <div className={`flex items-center gap-2 rounded-xl px-2 py-1 ${theme.inputBg}`}>
              <input type="text" value={input} onChange={(e) => setInput(e.target.value)} placeholder="Ask..." className={`flex-1 bg-transparent text-sm px-2 py-2 focus:outline-none ${isNeon ? 'text-[#39ff14] placeholder-[#39ff14]/50' : 'text-white'}`} disabled={isLoading}/>
              <button type="submit" disabled={isLoading || !input.trim()} className={`p-2 rounded-lg ${theme.sendBtn}`}><Send size={16}/></button>
            </div>
          </form>
      </div>

      {/* Floating Toggle Button (Bottom Right) */}
      <div className="fixed bottom-3 right-3 z-50 group">
        <button onClick={() => { setIsOpen(!isOpen); setIsNeon(false); }} className={`relative flex items-center justify-center w-14 h-14 rounded-full shadow-xl transition-all hover:scale-105 active:scale-95 backdrop-blur-sm ${isOpen ? 'bg-neutral-800 rotate-90' : 'bg-gradient-to-br from-blue-600 to-indigo-700'} ${isNeon ? 'border-2 border-[#39ff14] shadow-[0_0_15px_#39ff14]' : 'border border-white/20'}`}>
          {isOpen ? <X size={24} className={isNeon ? "text-[#39ff14]" : "text-gray-400"} /> : <Sparkles size={22} className="text-white" />}
        </button>
      </div>
    </>
  );
}