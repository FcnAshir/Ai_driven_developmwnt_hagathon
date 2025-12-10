import React, { useState, useEffect } from 'react';
import { createPortal } from 'react-dom';
import '../ChatKitWidget/ChatKitWidget.css';

const ChatKitPortal = () => {
  const [mounted, setMounted] = useState(false);
  const [isOpen, setIsOpen] = useState(false);

  useEffect(() => {
    setMounted(true);
    return () => setMounted(false);
  }, []);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  if (!mounted) {
    return null;
  }

  const chatWidget = (
    <div className="chatkit-container">
      {isOpen ? (
        <div className="chatkit-widget">
          <div className="chatkit-header">
            <h3>Book Assistant</h3>
            <button className="chatkit-close-btn" onClick={toggleChat}>
              ×
            </button>
          </div>
          <div className="chatkit-messages">
            <div className="chatkit-welcome">
              <p>Hello! I'm your book assistant. Ask me anything about the content you're reading.</p>
            </div>
          </div>
          <div className="chatkit-input-area">
            <textarea
              placeholder="Ask about the book content..."
              className="chatkit-input"
              rows="2"
              disabled
            />
            <button className="chatkit-send-btn" disabled>
              Send
            </button>
          </div>
        </div>
      ) : (
        <button className="chatkit-open-btn" onClick={toggleChat}>
          💬 Ask Book Questions
        </button>
      )}
    </div>
  );

  return createPortal(chatWidget, document.body);
};

export default ChatKitPortal;