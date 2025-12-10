import React, { useState } from 'react';
import '../ChatKitWidget/ChatKitWidget.css'; // Reuse the CSS

const ChatKitFloating = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Simple static content to ensure it renders
  return (
    <div className="chatkit-container" style={{position: 'fixed', bottom: '20px', right: '20px', zIndex: 10000}}>
      {!isOpen ? (
        <button
          className="chatkit-open-btn"
          onClick={toggleChat}
          style={{
            backgroundColor: '#2563eb',
            color: 'white',
            border: 'none',
            borderRadius: '50%',
            width: '60px',
            height: '60px',
            fontSize: '24px',
            cursor: 'pointer',
            boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center'
          }}
        >
          💬
        </button>
      ) : (
        <div
          className="chatkit-widget"
          style={{
            width: '380px',
            height: '500px',
            backgroundColor: 'white',
            borderRadius: '12px',
            boxShadow: '0 10px 25px rgba(0, 0, 0, 0.2)',
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            border: '1px solid #e5e7eb',
            position: 'absolute',
            bottom: '80px',
            right: '0'
          }}
        >
          <div
            className="chatkit-header"
            style={{
              backgroundColor: '#2563eb',
              color: 'white',
              padding: '16px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <h3 style={{margin: 0, fontSize: '16px', fontWeight: '600'}}>Book Assistant</h3>
            <button
              onClick={toggleChat}
              style={{
                backgroundColor: 'transparent',
                border: 'none',
                color: 'white',
                fontSize: '24px',
                cursor: 'pointer',
                width: '30px',
                height: '30px',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                borderRadius: '50%'
              }}
            >
              ×
            </button>
          </div>
          <div
            className="chatkit-messages"
            style={{
              flex: 1,
              padding: '16px',
              overflowY: 'auto',
              display: 'flex',
              flexDirection: 'column',
              gap: '12px',
              backgroundColor: '#f9fafb'
            }}
          >
            <div
              className="chatkit-welcome"
              style={{textAlign: 'center', color: '#6b7280', fontStyle: 'italic', padding: '20px 0'}}
            >
              <p>Hello! I'm your book assistant. The full chat functionality will be available once connected to the backend.</p>
            </div>
          </div>
          <div
            className="chatkit-input-area"
            style={{
              padding: '16px',
              borderTop: '1px solid #e5e7eb',
              backgroundColor: 'white'
            }}
          >
            <textarea
              placeholder="Ask about the book content..."
              disabled
              style={{
                width: 'calc(100% - 70px)',
                padding: '12px',
                border: '1px solid #d1d5db',
                borderRadius: '8px',
                resize: 'none',
                fontFamily: 'inherit',
                fontSize: '14px',
                outline: 'none',
                marginRight: '8px',
                display: 'inline-block',
                verticalAlign: 'top'
              }}
            />
            <button
              disabled
              style={{
                backgroundColor: '#9ca3af',
                color: 'white',
                border: 'none',
                borderRadius: '8px',
                padding: '12px 16px',
                cursor: 'not-allowed',
                fontWeight: '500',
                display: 'inline-block',
                verticalAlign: 'top'
              }}
            >
              Send
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatKitFloating;