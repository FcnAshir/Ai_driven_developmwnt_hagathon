import React from 'react';
import { createRoot } from 'react-dom/client';
import ChatKitPortal from '@site/src/components/ChatKitPortal/ChatKitPortal';

export function initChatKit() {
  // Wait for the container to be available
  const interval = setInterval(() => {
    const container = document.getElementById('chatkit-container');
    if (container) {
      clearInterval(interval);

      // Create a shadow root or directly render
      const root = createRoot(container);
      root.render(<ChatKitPortal />);
    }
  }, 100);
}