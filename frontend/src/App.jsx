import { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import { Send, Bot, Loader2, Volume2, StopCircle, Trash2, X, Sparkles, Languages, Globe } from 'lucide-react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import TranslatorWidget from './components/TranslatorWidget';

export default function App() {
  const urlParams = new URLSearchParams(window.location.search);
  const appMode = urlParams.get('mode') || 'chat'; // 'chat' or 'translator'

  const [isOpen, setIsOpen] = useState(false);
  const [isEmerald, setIsEmerald] = useState(false); // New "Dark Emerald" Theme
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

  // --- STRICT SIGNAL LISTENER ---
  useEffect(() => {
    const handleMessage = (event) => {
      const signal = event.data;

      // 1. Logic for TRANSLATOR Mode
      if (appMode === 'translator') {
        if (signal === 'open-translator') {
          setIsOpen(true);
          setIsEmerald(false); // Default style
        }
        if (signal === 'toggle-world-view') {
           // If you want the world button to open translator in Green mode:
           setIsOpen(prev => !prev);
           setIsEmerald(true);
        }
      }

      // 2. Logic for CHAT Mode
      if (appMode === 'chat') {
        if (signal === 'open-chat') {
          setIsOpen(true);
        }
        // If the World Button is clicked, we usually toggle the Chat
        if (signal === 'toggle-world-view') {
           setIsOpen(prev => !prev);
           setIsEmerald(true);
        }
      }
    };
    window.addEventListener('message', handleMessage);
    return () => window.removeEventListener('message', handleMessage);
  }, [appMode]);

  // Communicate with Parent (Docusaurus) to resize iframe
  useEffect(() => {
    // We send different signals based on who we are
    const prefix = appMode === 'translator' ? 'translator' : 'chat';
    const signal = isOpen ? `${prefix}-opened` : `${prefix}-closed`;
    window.parent.postMessage(signal, '*');
  }, [isOpen, appMode]);

  useEffect(() => { if (appMode === 'chat' && isOpen) scrollToBottom(); }, [messages, isOpen]);
  const scrollToBottom = () => setTimeout(() => messagesEndRef.current?.scrollIntoView({ behavior: "smooth" }), 100);

  // --- SPEECH FUNCTION ---
  const speak = (text) => { 
    window.speechSynthesis.cancel(); 
    const u = new SpeechSynthesisUtterance(text);
    const isUrduScript = /[\u0600-\u06FF]/.test(text);
    if (isUrduScript) {
        const voices = window.speechSynthesis.getVoices();
        const urduVoice = voices.find(v => v.lang.includes('ur'));
        const hindiVoice = voices.find(v => v.lang.includes('hi')); 
        if (urduVoice) { u.voice = urduVoice; u.lang = 'ur-PK'; } 
        else if (hindiVoice) { u.voice = hindiVoice; u.lang = 'hi-IN'; }
    } else { u.lang = 'en-US'; u.rate = 0.9; }
    u.onstart = () => setIsSpeaking(true); 
    u.onend = () => setIsSpeaking(false); 
    window.speechSynthesis.speak(u); 
  };
  const stopSpeaking = () => { window.speechSynthesis.cancel(); setIsSpeaking(false); };
  
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

  // --- NEW AESTHETIC: DARK EMERALD (Matte, Professional) ---
  const theme = {
    // If Emerald Mode is active: Very Dark Grey Bg + Subtle Green Borders
    container: isEmerald 
      ? 'bg-zinc-950 border border-emerald-900 shadow-2xl shadow-emerald-900/20' 
      : 'bg-gray-900/95 border border-white/10 shadow-2xl',
    
    header: isEmerald 
      ? 'bg-zinc-900 border-b border-emerald-900/50' 
      : 'bg-white/5 border-b border-white/5',
      
    textMain: isEmerald ? 'text-emerald-500' : 'text-white',
    textSub: isEmerald ? 'text-emerald-700' : 'text-green-400',
    
    // Bubbles
    userBubble: isEmerald 
      ? 'bg-emerald-900/30 border border-emerald-800 text-emerald-100' 
      : 'bg-blue-600 border-blue-500 text-white',
      
    aiBubble: isEmerald 
      ? 'bg-zinc-900 border border-zinc-800 text-gray-300' 
      : 'bg-white/5 border-white/10 text-gray-100',
      
    // Input
    inputBox: isEmerald 
      ? 'bg-zinc-900 border border-emerald-900/50 text-emerald-100 placeholder-emerald-800' 
      : 'bg-white/5 border border-white/10 text-white',
      
    sendBtn: isEmerald 
      ? 'bg-emerald-800 hover:bg-emerald-700 text-white' 
      : 'bg-blue-600 text-white',
  };

  // ==========================================
  // RENDER: TRANSLATOR MODE
  // ==========================================
  if (appMode === 'translator') {
    // If closed, return nothing
    if (!isOpen) return null;
    return (
      <div className={`backdrop-blur-md rounded-xl w-full h-full overflow-hidden animate-in fade-in slide-in-from-top-5 flex flex-col ${theme.container}`}>
         <div className={`flex justify-between items-center px-4 py-3 ${theme.header}`}>
             <h3 className={`font-medium flex items-center gap-2 text-sm ${theme.textMain}`}>
                <Globe size={16} /> Translator
             </h3>
             <button onClick={() => setIsOpen(false)} className="text-gray-500 hover:text-white transition"><X size={18}/></button>
         </div>
         {/* Translator Widget Content */}
         <div className="p-4 flex-1 overflow-y-auto">
            <TranslatorWidget isEmerald={isEmerald} />
         </div>
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
        backdrop-blur-md rounded-xl
        transition-all duration-300 ease-out origin-bottom-right
        ${theme.container}
        ${isOpen ? 'scale-100 opacity-100 translate-y-0' : 'scale-90 opacity-0 pointer-events-none translate-y-10'}
      `}>
          {/* Header */}
          <div className={`flex items-center justify-between px-5 py-4 rounded-t-xl ${theme.header}`}>
            <div className="flex items-center gap-3">
              <div className={`p-2 rounded-lg ${isEmerald ? 'bg-emerald-900/20' : 'bg-gradient-to-tr from-blue-600 to-indigo-600'}`}>
                  <Bot size={20} className={theme.textMain} />
              </div>
              <div>
                  <h1 className={`font-semibold text-sm ${isEmerald ? 'text-gray-200' : 'text-white'}`}>Next-Gen AI</h1>
                  <p className={`text-[10px] ${theme.textSub}`}>Online</p>
              </div>
            </div>
            <div className="flex gap-2">
                {isSpeaking && <button onClick={stopSpeaking} className="text-rose-400 hover:text-rose-300"><StopCircle size={18}/></button>}
                <button onClick={() => { setMessages([{ role: 'ai', text: "Reset.", sources: [] }]); stopSpeaking(); }} className="text-gray-500 hover:text-white"><Trash2 size={18}/></button>
                <button onClick={() => setIsOpen(!isOpen)} className="text-gray-500 hover:text-white"><X size={18}/></button>
            </div>
          </div>

          {/* Messages */}
          <div className="flex-1 overflow-y-auto p-4 space-y-4 custom-scrollbar">
            {messages.map((msg, i) => (
              <div key={i} className={`flex ${msg.role === 'user' ? 'justify-end' : 'justify-start'}`}>
                <div className={`max-w-[85%] rounded-2xl px-4 py-3 text-sm relative group ${msg.role === 'user' ? theme.userBubble : theme.aiBubble}`}>
                    <ReactMarkdown remarkPlugins={[remarkGfm]}>{msg.text}</ReactMarkdown>
                    {msg.role === 'ai' && <button onClick={() => speak(msg.text)} className="absolute -right-6 top-1 opacity-0 group-hover:opacity-100 text-gray-400 hover:text-white"><Volume2 size={14}/></button>}
                </div>
              </div>
            ))}
            {isLoading && <div className="text-xs ml-4 text-gray-500 flex gap-2"><Loader2 size={12} className="animate-spin"/> Thinking...</div>}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <form onSubmit={sendMessage} className={`p-4 border-t ${isEmerald ? 'border-emerald-900/30' : 'border-white/5'}`}>
            <div className={`flex items-center gap-2 rounded-lg px-2 py-1 ${theme.inputBox}`}>
              <input type="text" value={input} onChange={(e) => setInput(e.target.value)} placeholder="Ask a question..." className="flex-1 bg-transparent text-sm px-2 py-3 focus:outline-none" disabled={isLoading}/>
              <button type="submit" disabled={isLoading || !input.trim()} className={`p-2 rounded-md transition ${theme.sendBtn}`}><Send size={16}/></button>
            </div>
          </form>
      </div>
      
      {/* Standard Toggle Button (Bottom Right) - Only shows for Chat Mode */}
      {appMode === 'chat' && (
        <div className="fixed bottom-3 right-3 z-50">
            <button onClick={() => { setIsOpen(!isOpen); setIsEmerald(false); }} className={`flex items-center justify-center w-14 h-14 rounded-full shadow-xl hover:scale-105 transition ${isEmerald ? 'bg-zinc-800 border border-emerald-500/50' : 'bg-blue-600'}`}>
            {isOpen ? <X size={24} className="text-gray-300" /> : <Sparkles size={24} className="text-white" />}
            </button>
        </div>
      )}
    </>
  );
}