import React, { useEffect, useState } from 'react';

// This function wraps every single page in your book
export default function Root({children}) {
  const [isOpen, setIsOpen] = useState(false);

  useEffect(() => {
    // Listen for the "open/close" signal from the robot
    const handleMessage = (event) => {
      if (event.data === 'toggle-widget') {
        setIsOpen(current => !current);
      }
    };

    window.addEventListener('message', handleMessage);
    return () => window.removeEventListener('message', handleMessage);
  }, []);

  return (
    <>
      {/* Show the actual book page */}
      {children}
      
      {/* Float the Robot on top */}
      <iframe
        // 👇👇👇 PASTE YOUR BLUE BUTTON VERCEL LINK HERE 👇👇👇
        src="https://next-gen-humanoid-robotics-book-cha.vercel.app/" 
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          // If open, make it big. If closed, make it small button.
          width: isOpen ? '380px' : '80px',
          height: isOpen ? '600px' : '80px',
          border: 'none',
          zIndex: 9999,
          borderRadius: isOpen ? '12px' : '50%',
          transition: 'all 0.3s ease',
          boxShadow: isOpen ? '0 10px 25px rgba(0,0,0,0.5)' : 'none',
          background: 'transparent'
        }}
        title="AI Assistant"
      />
    </>
  );
}