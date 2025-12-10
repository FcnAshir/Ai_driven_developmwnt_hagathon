import React, { useState, useEffect, useRef } from 'react';
import './ChatKitWidget.css';

const ChatKitWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toLocaleTimeString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Get the current thread ID or create a new one
      let threadId = localStorage.getItem('chatkit_thread_id');
      if (!threadId) {
        threadId = 'thread_' + Date.now();
        localStorage.setItem('chatkit_thread_id', threadId);
      }

      // Call the backend API
      const response = await fetch('http://localhost:8000/chatkit', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': 'Bearer dummy-token' // In a real implementation, you'd have a real token
        },
        body: JSON.stringify({
          thread_id: threadId,
          messages: [
            ...messages.map(msg => ({
              role: msg.sender,
              content: msg.text
            })),
            { role: 'user', content: inputValue }
          ],
          context: {
            book_id: 'physical-ai-book',
            chapter_id: 'current-chapter' // This would be dynamic based on current page
          }
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      // Handle streaming response
      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let assistantMessage = '';
      let buffer = '';

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');
        buffer = lines.pop(); // Keep last incomplete line in buffer

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            try {
              const data = JSON.parse(line.slice(6)); // Remove 'data: ' prefix
              if (data.delta) {
                assistantMessage += data.delta;
              } else if (data.content && data.role === 'assistant') {
                // Complete message received
                break;
              }
            } catch (e) {
              console.error('Error parsing SSE data:', e);
            }
          }
        }
      }

      if (assistantMessage) {
        const botMessage = {
          id: Date.now() + 1,
          text: assistantMessage,
          sender: 'bot',
          timestamp: new Date().toLocaleTimeString()
        };
        setMessages(prev => [...prev, botMessage]);
      }
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error. Please try again.',
        sender: 'bot',
        timestamp: new Date().toLocaleTimeString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
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
            {messages.length === 0 ? (
              <div className="chatkit-welcome">
                <p>Hello! I'm your book assistant. Ask me anything about the content you're reading.</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`chatkit-message ${
                    message.sender === 'user' ? 'user-message' : 'bot-message'
                  }`}
                >
                  <div className="chatkit-message-text">{message.text}</div>
                  <div className="chatkit-message-timestamp">{message.timestamp}</div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="chatkit-message bot-message">
                <div className="chatkit-message-text">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <div className="chatkit-input-area">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about the book content..."
              className="chatkit-input"
              rows="2"
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || !inputValue.trim()}
              className="chatkit-send-btn"
            >
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
};

export default ChatKitWidget;