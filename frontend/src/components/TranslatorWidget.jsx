import React, { useState } from 'react';
import axios from 'axios';
import { ArrowRight, Copy, Check, Loader2, RotateCcw, Volume2, StopCircle } from 'lucide-react';

const TranslatorWidget = () => {
  const [inputText, setInputText] = useState('');
  const [targetLang, setTargetLang] = useState('es'); 
  const [translatedText, setTranslatedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [copied, setCopied] = useState(false);
  const [isSpeaking, setIsSpeaking] = useState(false);

  const handleTranslate = async () => {
    if (!inputText.trim()) return;
    setIsLoading(true);
    window.speechSynthesis.cancel(); // Stop old audio
    setIsSpeaking(false);
    
    try {
      // Free Translation API
      const response = await axios.get(`https://api.mymemory.translated.net/get`, {
        params: { q: inputText, langpair: `en|${targetLang}` }
      });
      setTranslatedText(response.data.responseData.translatedText);
    } catch (error) {
      setTranslatedText("⚠️ Error: Could not translate.");
    } finally {
      setIsLoading(false);
    }
  };

  const handleSpeak = () => {
    if (!translatedText) return;
    if (isSpeaking) {
      window.speechSynthesis.cancel();
      setIsSpeaking(false);
      return;
    }
    const utterance = new SpeechSynthesisUtterance(translatedText);
    // Simple language mapping for accurate accents
    const langMap = { 'es': 'es-ES', 'fr': 'fr-FR', 'de': 'de-DE', 'ur': 'ur-PK', 'zh': 'zh-CN', 'ja': 'ja-JP', 'ru': 'ru-RU' };
    utterance.lang = langMap[targetLang] || 'en-US';
    utterance.onstart = () => setIsSpeaking(true);
    utterance.onend = () => setIsSpeaking(false);
    window.speechSynthesis.speak(utterance);
  };

  const handleCopy = () => {
    navigator.clipboard.writeText(translatedText);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <div className="flex flex-col gap-3 text-sm text-gray-200">
      {/* Input */}
      <div className="relative">
        <textarea
          className="w-full bg-black/40 border border-white/10 rounded-xl p-3 text-white focus:ring-1 focus:ring-blue-500 outline-none resize-none placeholder-gray-500 backdrop-blur-sm"
          placeholder="Type English text here..."
          rows="3"
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          onKeyDown={(e) => { if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); handleTranslate(); } }}
        />
        {inputText && (
          <button onClick={() => { setInputText(''); setTranslatedText(''); }} className="absolute top-2 right-2 text-gray-500 hover:text-white" title="Clear">
            <RotateCcw size={12} />
          </button>
        )}
      </div>

      {/* Controls */}
      <div className="flex gap-2 items-center">
        <div className="flex-1 flex items-center gap-2 bg-black/40 rounded-lg px-2 border border-white/10 h-10">
          <span className="text-gray-400 text-xs font-bold pl-1">EN</span>
          <ArrowRight size={12} className="text-gray-500" />
          <select className="bg-transparent text-white w-full h-full outline-none cursor-pointer border-none text-xs" value={targetLang} onChange={(e) => setTargetLang(e.target.value)}>
            <option className="bg-gray-800" value="es">Spanish</option>
            <option className="bg-gray-800" value="fr">French</option>
            <option className="bg-gray-800" value="de">German</option>
            <option className="bg-gray-800" value="ur">Urdu</option>
            <option className="bg-gray-800" value="zh">Chinese</option>
            <option className="bg-gray-800" value="ja">Japanese</option>
            <option className="bg-gray-800" value="ru">Russian</option>
          </select>
        </div>
        <button onClick={handleTranslate} disabled={isLoading || !inputText} className="bg-blue-600 hover:bg-blue-500 disabled:opacity-50 text-white px-4 h-10 rounded-lg font-medium shadow-lg">
          {isLoading ? <Loader2 size={16} className="animate-spin" /> : 'Go'}
        </button>
      </div>

      {/* Output + Speaker Button */}
      {translatedText && (
        <div className="bg-white/5 rounded-xl p-3 border border-white/10 animate-in fade-in relative group backdrop-blur-md">
          <p className="pr-12 leading-relaxed">{translatedText}</p>
          <div className="absolute top-2 right-2 flex gap-1">
            {/* SPEAK BUTTON */}
            <button onClick={handleSpeak} className={`p-1.5 rounded-md transition-all ${isSpeaking ? 'text-rose-400 bg-rose-500/10' : 'text-gray-400 hover:text-blue-400'}`} title="Read Aloud">
              {isSpeaking ? <StopCircle size={14} className="animate-pulse" /> : <Volume2 size={14} />}
            </button>
            {/* COPY BUTTON */}
            <button onClick={handleCopy} className="p-1.5 text-gray-400 hover:text-green-400" title="Copy">
              {copied ? <Check size={14} /> : <Copy size={14} />}
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default TranslatorWidget;