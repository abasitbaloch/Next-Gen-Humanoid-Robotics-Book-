import React, { useEffect, useRef } from 'react';

// This wrapper wraps every single page in your book
export default function Root({children}) {
  // We use "refs" to grab the iframe HTML elements so we can resize them directly
  const chatFrameRef = useRef(null);
  const translatorFrameRef = useRef(null);

  useEffect(() => {
    // Listen for signals coming from inside the iframes
    const handleMessage = (event) => {
      
      // --- 1. CHATBOT SIGNALS ---
      if (event.data === 'chat-opened') {
        if (chatFrameRef.current) {
          // Make big
          chatFrameRef.current.style.width = '400px';
          chatFrameRef.current.style.height = '700px';
          chatFrameRef.current.style.borderRadius = '12px'; // Square corners
          chatFrameRef.current.style.boxShadow = '0 10px 25px rgba(0,0,0,0.5)';
        }
      }
      if (event.data === 'chat-closed') {
        if (chatFrameRef.current) {
          // Make small (Button size)
          chatFrameRef.current.style.width = '80px';
          chatFrameRef.current.style.height = '80px';
          chatFrameRef.current.style.borderRadius = '50%'; // Round button
          chatFrameRef.current.style.boxShadow = 'none';
        }
      }

      // --- 2. TRANSLATOR SIGNALS ---
      if (event.data === 'translator-opened') {
        if (translatorFrameRef.current) {
          // Make visible (Dropdown size)
          translatorFrameRef.current.style.width = '360px';
          translatorFrameRef.current.style.height = '400px';
          translatorFrameRef.current.style.opacity = '1';
          translatorFrameRef.current.style.pointerEvents = 'auto'; // Enable clicking
        }
      }
      if (event.data === 'translator-closed') {
        if (translatorFrameRef.current) {
          // Hide completely
          translatorFrameRef.current.style.width = '0px';
          translatorFrameRef.current.style.height = '0px';
          translatorFrameRef.current.style.opacity = '0';
          translatorFrameRef.current.style.pointerEvents = 'none'; // Click-through
        }
      }
    };

    window.addEventListener('message', handleMessage);
    return () => window.removeEventListener('message', handleMessage);
  }, []);

  return (
    <>
      {/* The actual book content */}
      {children}
      
      {/* ===============================================
          IFRAME 1: CHATBOT (Bottom Right)
          Note: We add "?mode=chat" to the URL
         =============================================== */}
      <iframe
        ref={chatFrameRef}
        src="https://next-gen-humanoid-robotics-book-cha.vercel.app/?mode=chat" 
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          // Default State: Small Button
          width: '80px',
          height: '80px',
          border: 'none',
          zIndex: 9999,
          borderRadius: '50%', // Start round
          transition: 'all 0.3s cubic-bezier(0.25, 0.8, 0.25, 1)',
          background: 'transparent'
        }}
        title="AI Chatbot"
      />

      {/* ===============================================
          IFRAME 2: TRANSLATOR (Top Right)
          Note: We add "?mode=translator" to the URL
         =============================================== */}
      <iframe
        ref={translatorFrameRef}
        src="https://next-gen-humanoid-robotics-book-cha.vercel.app/?mode=translator"
        style={{
          position: 'fixed',
          top: '60px', // Positioned just below the Docusaurus navbar
          right: '10px',
          // Default State: Hidden
          width: '0px',
          height: '0px',
          opacity: 0,
          border: 'none',
          zIndex: 9998,
          transition: 'opacity 0.2s ease, width 0.1s ease', 
          background: 'transparent',
          pointerEvents: 'none'
        }}
        title="AI Translator"
      />
    </>
  );
}