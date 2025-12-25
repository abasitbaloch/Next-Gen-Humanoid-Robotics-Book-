import React, { useState } from 'react';
import axios from 'axios';
import { ArrowRight, Copy, Check, Loader2, RotateCcw } from 'lucide-react';

const TranslatorWidget = () => {
  const [inputText, setInputText] = useState('');
  const [targetLang, setTargetLang] = useState('es'); // Default: Spanish
  const [translatedText, setTranslatedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [copied, setCopied] = useState(false);

  // --- THE REAL TRANSLATION LOGIC ---
  const handleTranslate = async () => {
    if (!inputText.trim()) return;

    setIsLoading(true);
    setTranslatedText(''); // Clear previous result
    
    try {
      // Using MyMemory API (Free, no key required for small usage)
      const response = await axios.get(`https://api.mymemory.translated.net/get`, {
        params: {
          q: inputText,
          langpair: `en|${targetLang}` // Translating from English (en) to Target
        }
      });

      // The API returns the data here:
      setTranslatedText(response.data.responseData.translatedText);
      
    } catch (error) {
      console.error("Translation Error:", error);
      setTranslatedText("⚠️ Error: Could not translate. Try again.");
    } finally {
      setIsLoading(false);
    }
  };

  // Helper to copy text to clipboard
  const handleCopy = () => {
    if (!translatedText) return;
    navigator.clipboard.writeText(translatedText);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <div className="flex flex-col gap-3 text-sm text-gray-200">
      
      {/* 1. Input Area */}
      <div className="relative">
        <textarea
          className="w-full bg-gray-900 border border-gray-600 rounded-lg p-3 text-white focus:ring-2 focus:ring-blue-500 focus:border-transparent outline-none resize-none placeholder-gray-500"
          placeholder="Type English text here..."
          rows="3"
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === 'Enter' && !e.shiftKey) {
              e.preventDefault();
              handleTranslate();
            }
          }}
        />
        {inputText && (
          <button 
            onClick={() => { setInputText(''); setTranslatedText(''); }}
            className="absolute top-2 right-2 text-gray-500 hover:text-white"
            title="Clear"
          >
            <RotateCcw size={12} />
          </button>
        )}
      </div>

      {/* 2. Controls (Language & Button) */}
      <div className="flex gap-2 items-center">
        <div className="flex-1 flex items-center gap-2 bg-gray-900 rounded-lg px-2 border border-gray-600">
          <span className="text-gray-400 text-xs font-bold">EN</span>
          <ArrowRight size={12} className="text-gray-500" />
          <select 
            className="bg-transparent text-white p-2 w-full outline-none cursor-pointer"
            value={targetLang} 
            onChange={(e) => setTargetLang(e.target.value)}
          >
            <option value="es">Spanish</option>
            <option value="fr">French</option>
            <option value="de">German</option>
            <option value="it">Italian</option>
            <option value="pt">Portuguese</option>
            <option value="ur">Urdu</option>
            <option value="zh">Chinese</option>
            <option value="ja">Japanese</option>
            <option value="ru">Russian</option>
          </select>
        </div>

        <button 
          onClick={handleTranslate}
          disabled={isLoading || !inputText}
          className="bg-blue-600 hover:bg-blue-500 disabled:bg-gray-700 disabled:cursor-not-allowed text-white px-4 py-2 rounded-lg font-medium transition-colors flex items-center gap-2"
        >
          {isLoading ? <Loader2 size={16} className="animate-spin" /> : 'Go'}
        </button>
      </div>

      {/* 3. Output Area */}
      {translatedText && (
        <div className="bg-gray-700/50 rounded-lg p-3 border border-gray-600 animate-in fade-in duration-300 relative group">
          <p className="pr-6">{translatedText}</p>
          
          <button 
            onClick={handleCopy}
            className="absolute top-2 right-2 text-gray-400 hover:text-green-400 transition-colors"
            title="Copy Translation"
          >
            {copied ? <Check size={14} /> : <Copy size={14} />}
          </button>
        </div>
      )}
    </div>
  );
};

export default TranslatorWidget;