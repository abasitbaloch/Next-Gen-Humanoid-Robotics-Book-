import { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import { Send, Bot, Loader2, BookOpen, Volume2, StopCircle, Trash2, X, Sparkles } from 'lucide-react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';

export default function App() {
  const [isOpen, setIsOpen] = useState(false);
  const [isSpeaking, setIsSpeaking] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [input, setInput] = useState('');
  const messagesEndRef = useRef(null);

  // --- MEMORY: Load saved messages ---
  const [messages, setMessages] = useState(() => {
    const saved = localStorage.getItem('robotics_chat_history');
    return saved ? JSON.parse(saved) : [{ 
      role: 'ai', 
      text: "Greetings. I am the **Next-Gen Robotics AI**. How can I assist you with the curriculum today?", 
      sources: [] 
    }];
  });

  useEffect(() => {
    localStorage.setItem('robotics_chat_history', JSON.stringify(messages));
    if (isOpen) scrollToBottom();
  }, [messages, isOpen]);

  const scrollToBottom = () => {
    setTimeout(() => {
      messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
    }, 100);
  };

  // --- VOICE LOGIC ---
  const speak = (text) => {
    window.speechSynthesis.cancel();
    const u = new SpeechSynthesisUtterance(text);
    const voices = window.speechSynthesis.getVoices();
    // Try to find a premium sounding voice
    const preferred = voices.find(v => v.name.includes("Google US English") || v.name.includes("Samantha"));
    if (preferred) u.voice = preferred;
    u.pitch = 0.9; // Slightly deeper, more robotic/professional
    u.rate = 1.0;
    u.onstart = () => setIsSpeaking(true);
    u.onend = () => setIsSpeaking(false);
    window.speechSynthesis.speak(u);
  };

  const stopSpeaking = () => {
    window.speechSynthesis.cancel();
    setIsSpeaking(false);
  };

  const clearHistory = () => {
    if (window.confirm("Clear conversation history?")) {
      setMessages([{ role: 'ai', text: "Conversation reset.", sources: [] }]);
      stopSpeaking();
    }
  };

  // --- TOGGLE LOGIC (Simple Open/Close) ---
  const toggleChat = () => {
    const newState = !isOpen;
    setIsOpen(newState);
    // Send signal to parent to resize the iframe
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
      setMessages(prev => [...prev, { role: 'ai', text: "⚠️ I am unable to connect to the neural network.", sources: [] }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {/* No Background Div here! 
         This ensures the widget looks like it's floating over the book 
      */}

      <div className="fixed bottom-0 right-0 p-6 z-50 font-sans antialiased flex flex-col items-end">
        
        {/* 1. THE CHAT WINDOW */}
        <div className={`
          mb-4 w-[90vw] md:w-[380px] h-[600px] max-h-[75vh]
          flex flex-col overflow-hidden
          rounded-2xl border border-white/10 shadow-2xl shadow-black/50
          bg-black/80 backdrop-blur-xl 
          transition-all duration-300 ease-[cubic-bezier(0.23,1,0.32,1)] origin-bottom-right
          ${isOpen ? 'scale-100 opacity-100 translate-y-0' : 'scale-90 opacity-0 translate-y-10 pointer-events-none absolute'}
        `}>
          
          {/* Header */}
          <div className="flex items-center justify-between px-5 py-4 bg-white/5 border-b border-white/5 backdrop-blur-sm">
            <div className="flex items-center gap-3">
              <div className="relative">
                <div className="absolute inset-0 bg-blue-500 blur-md opacity-20 rounded-full"></div>
                <div className="relative p-2 bg-gradient-to-tr from-blue-600 to-indigo-600 rounded-xl shadow-lg border border-white/10">
                  <Bot size={20} className="text-white" />
                </div>
              </div>
              <div>
                <h1 className="font-semibold text-sm text-white tracking-wide">Next-Gen AI</h1>
                <div className="flex items-center gap-1.5">
                  <span className="w-1.5 h-1.5 bg-green-500 rounded-full animate-pulse"></span>
                  <p className="text-[10px] text-gray-400 font-medium uppercase tracking-wider">Online</p>
                </div>
              </div>
            </div>
            
            {/* Header Controls */}
            <div className="flex gap-1">
               {isSpeaking && (
                <button onClick={stopSpeaking} className="p-2 text-rose-400 hover:bg-white/10 rounded-lg transition-colors">
                  <StopCircle size={16} />
                </button>
              )}
              <button onClick={clearHistory} className="p-2 text-gray-400 hover:text-white hover:bg-white/10 rounded-lg transition-colors">
                <Trash2 size={16} />
              </button>
              <button onClick={toggleChat} className="p-2 text-gray-400 hover:text-white hover:bg-white/10 rounded-lg transition-colors">
                <X size={18} />
              </button>
            </div>
          </div>

          {/* Messages Area */}
          <div className="flex-1 overflow-y-auto p-5 space-y-5 custom-scrollbar scroll-smooth">
            {messages.map((msg, index) => (
              <div key={index} className={`flex ${msg.role === 'user' ? 'justify-end' : 'justify-start'}`}>
                
                {/* Message Bubble */}
                <div className={`
                  max-w-[85%] rounded-2xl px-4 py-3 text-sm leading-relaxed shadow-sm relative group
                  ${msg.role === 'user' 
                    ? 'bg-blue-600 text-white rounded-br-sm' 
                    : 'bg-white/10 text-gray-100 rounded-bl-sm border border-white/5 backdrop-blur-md'
                  }
                `}>
                    <div className="prose prose-invert prose-sm max-w-none prose-p:my-0 prose-a:text-blue-300">
                       <ReactMarkdown remarkPlugins={[remarkGfm]}>{msg.text}</ReactMarkdown>
                    </div>

                    {/* AI Tools (Speak) */}
                    {msg.role === 'ai' && (
                      <button 
                        onClick={() => speak(msg.text)}
                        className="absolute -right-8 top-1 p-1.5 text-gray-500 hover:text-blue-400 opacity-0 group-hover:opacity-100 transition-all transform hover:scale-110"
                        title="Read Aloud"
                      >
                        <Volume2 size={14} />
                      </button>
                    )}

                    {/* Sources (Premium Look) */}
                    {msg.sources && msg.sources.length > 0 && (
                      <div className="mt-3 pt-3 border-t border-white/10">
                        <p className="text-[10px] font-bold text-gray-500 mb-2 flex items-center gap-1.5 uppercase tracking-wider">
                          <BookOpen size={10} /> Reference Sources
                        </p>
                        <div className="flex flex-wrap gap-2">
                          {msg.sources.map((s, i) => (
                            <a key={i} href={s.url} target="_blank" className="flex items-center gap-1 text-[10px] bg-black/40 px-2 py-1 rounded-md text-blue-300 hover:text-white hover:bg-blue-600/50 transition-all border border-white/5">
                              <span className="opacity-50">{i+1}.</span> {s.text.substring(0, 15)}...
                            </a>
                          ))}
                        </div>
                      </div>
                    )}
                </div>
              </div>
            ))}
            
            {/* Loading Indicator */}
            {isLoading && (
              <div className="flex items-center gap-3 text-gray-400 text-xs ml-1 animate-pulse">
                <div className="p-1.5 bg-white/10 rounded-full">
                  <Loader2 size={12} className="animate-spin" />
                </div>
                <span>Processing Neural Query...</span>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <form onSubmit={sendMessage} className="p-4 bg-black/40 backdrop-blur-md border-t border-white/5">
            <div className="relative flex items-center gap-2 bg-white/5 border border-white/10 rounded-xl px-2 py-1 focus-within:ring-2 focus-within:ring-blue-500/50 focus-within:border-blue-500/50 transition-all">
              <input
                type="text"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                placeholder="Ask something about robotics..."
                className="flex-1 bg-transparent text-white text-sm px-2 py-2.5 focus:outline-none placeholder-gray-500"
                disabled={isLoading}
              />
              <button 
                type="submit" 
                disabled={isLoading || !input.trim()}
                className="p-2 bg-gradient-to-tr from-blue-600 to-indigo-600 text-white rounded-lg hover:shadow-lg hover:shadow-blue-500/30 disabled:opacity-50 disabled:cursor-not-allowed transition-all transform active:scale-95"
              >
                <Send size={16} />
              </button>
            </div>
            <div className="text-[10px] text-center text-gray-600 mt-2">
              AI-generated content. Check sources for accuracy.
            </div>
          </form>
        </div>

        {/* 2. THE FLOATING BUTTON (Premium Orb) */}
        <button
          onClick={toggleChat}
          className={`
            group relative flex items-center justify-center w-14 h-14 rounded-full shadow-2xl 
            transition-all duration-300 hover:scale-105 active:scale-95
            ${isOpen ? 'bg-neutral-800 rotate-90' : 'bg-gradient-to-r from-blue-600 to-indigo-600'}
          `}
        >
          {/* Subtle Glow Effect */}
          <span className={`absolute inset-0 rounded-full blur-lg bg-blue-500/50 group-hover:bg-blue-400/60 transition-all ${isOpen ? 'opacity-0' : 'opacity-100'}`}></span>
          
          {/* Icon */}
          <div className="relative z-10">
             {isOpen ? <X size={24} className="text-gray-300" /> : <Sparkles size={24} className="text-white animate-pulse-slow" />}
          </div>
        </button>

      </div>
    </>
  );
}