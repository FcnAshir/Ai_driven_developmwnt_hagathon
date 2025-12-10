import React from 'react';
import ChatKitFloating from '../components/ChatKitFloating/ChatKitFloating';

// This component will be loaded on all pages via swizzling
const ChatKitInject = () => {
  return <ChatKitFloating />;
};

export default ChatKitInject;