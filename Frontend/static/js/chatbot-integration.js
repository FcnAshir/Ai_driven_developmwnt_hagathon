// Simple chatbot integration script that injects the chatbot into the page
document.addEventListener('DOMContentLoaded', function() {
    // Create the chatbot container
    const chatContainer = document.createElement('div');
    chatContainer.id = 'chatbot-container';
    chatContainer.style.cssText = `
        position: fixed;
        bottom: 20px;
        right: 20px;
        z-index: 10000;
        font-family: Arial, sans-serif;
    `;

    // Create the chat button
    const chatButton = document.createElement('button');
    chatButton.id = 'chatbot-button';
    chatButton.innerHTML = '💬';
    chatButton.style.cssText = `
        background-color: #2563eb;
        color: white;
        border: none;
        border-radius: 50%;
        width: 60px;
        height: 60px;
        font-size: 24px;
        cursor: pointer;
        box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
        display: flex;
        align-items: center;
        justify-content: center;
    `;

    // Create the chat window (initially hidden)
    const chatWindow = document.createElement('div');
    chatWindow.id = 'chatbot-window';
    chatWindow.style.cssText = `
        width: 380px;
        height: 500px;
        background: white;
        border-radius: 12px;
        box-shadow: 0 10px 25px rgba(0, 0, 0, 0.2);
        display: none;
        flex-direction: column;
        overflow: hidden;
        border: 1px solid #e5e7eb;
        position: absolute;
        bottom: 80px;
        right: 0;
    `;

    // Create chat header
    const chatHeader = document.createElement('div');
    chatHeader.style.cssText = `
        background: #2563eb;
        color: white;
        padding: 16px;
        display: flex;
        justify-content: space-between;
        align-items: center;
    `;
    chatHeader.innerHTML = '<h3 style="margin: 0; font-size: 16px;">Book Assistant</h3>';

    // Create close button
    const closeButton = document.createElement('button');
    closeButton.innerHTML = '×';
    closeButton.style.cssText = `
        background: none;
        border: none;
        color: white;
        font-size: 24px;
        cursor: pointer;
        width: 30px;
        height: 30px;
        display: flex;
        align-items: center;
        justify-content: center;
        border-radius: 50%;
    `;
    closeButton.onclick = function() {
        chatWindow.style.display = 'none';
        chatButton.style.display = 'flex';
    };
    chatHeader.appendChild(closeButton);

    // Create messages container
    const messagesContainer = document.createElement('div');
    messagesContainer.id = 'chatbot-messages';
    messagesContainer.style.cssText = `
        flex: 1;
        padding: 16px;
        overflow-y: auto;
        display: flex;
        flex-direction: column;
        gap: 12px;
        background-color: #f9fafb;
    `;

    // Add welcome message
    const welcomeMessage = document.createElement('div');
    welcomeMessage.style.cssText = `
        text-align: center;
        color: #6b7280;
        font-style: italic;
        padding: 20px 0;
    `;
    welcomeMessage.innerHTML = '<p>Hello! I\'m your book assistant. Ask me anything about the content you\'re reading.</p>';
    messagesContainer.appendChild(welcomeMessage);

    // Create input area
    const inputArea = document.createElement('div');
    inputArea.style.cssText = `
        padding: 16px;
        border-top: 1px solid #e5e7eb;
        background: white;
        display: flex;
        gap: 8px;
    `;

    const textArea = document.createElement('textarea');
    textArea.placeholder = 'Ask about the book content...';
    textArea.style.cssText = `
        flex: 1;
        padding: 12px;
        border: 1px solid #d1d5db;
        border-radius: 8px;
        resize: none;
        font-family: inherit;
        font-size: 14px;
        outline: none;
    `;
    textArea.rows = 2;

    const sendButton = document.createElement('button');
    sendButton.innerHTML = 'Send';
    sendButton.style.cssText = `
        background-color: #2563eb;
        color: white;
        border: none;
        border-radius: 8px;
        padding: 12px 16px;
        cursor: pointer;
        font-weight: 500;
    `;
    sendButton.onclick = function() {
        const message = textArea.value.trim();
        if (message) {
            // In a real implementation, this would connect to your backend
            // For now, we'll just show a placeholder response
            textArea.value = '';

            // Add user message to chat
            const userMsg = document.createElement('div');
            userMsg.style.cssText = `
                max-width: 80%;
                padding: 12px 16px;
                background-color: #2563eb;
                color: white;
                border-radius: 18px;
                align-self: flex-end;
                border-bottom-right-radius: 4px;
            `;
            userMsg.textContent = message;
            messagesContainer.appendChild(userMsg);

            // Connect to backend API
            const threadId = localStorage.getItem('chatkit_thread_id') || `thread_${Date.now()}`;
            localStorage.setItem('chatkit_thread_id', threadId);

            // Show typing indicator
            const typingIndicator = document.createElement('div');
            typingIndicator.id = 'typing-indicator';
            typingIndicator.style.cssText = `
                max-width: 80%;
                padding: 12px 16px;
                background-color: white;
                border: 1px solid #e5e7eb;
                border-radius: 18px;
                align-self: flex-start;
                border-bottom-left-radius: 4px;
                display: flex;
                align-items: center;
            `;
            typingIndicator.innerHTML = `
                <div style="display: flex; gap: 4px;">
                    <div style="width: 8px; height: 8px; background-color: #6b7280; border-radius: 50%; animation: typing 1.4s infinite ease-in-out;"></div>
                    <div style="width: 8px; height: 8px; background-color: #6b7280; border-radius: 50%; animation: typing 1.4s infinite ease-in-out; animation-delay: -0.32s;"></div>
                    <div style="width: 8px; height: 8px; background-color: #6b7280; border-radius: 50%; animation: typing 1.4s infinite ease-in-out; animation-delay: -0.16s;"></div>
                </div>
            `;
            messagesContainer.appendChild(typingIndicator);

            // Add CSS for typing animation
            if (!document.querySelector('#typing-style')) {
                const style = document.createElement('style');
                style.id = 'typing-style';
                style.textContent = `
                    @keyframes typing {
                        0%, 80%, 100% { transform: scale(0.8); opacity: 0.5; }
                        40% { transform: scale(1); opacity: 1; }
                    }
                `;
                document.head.appendChild(style);
            }

            // Scroll to bottom
            messagesContainer.scrollTop = messagesContainer.scrollHeight;

            // Call the backend API - using public endpoint that doesn't require authentication
            fetch('http://localhost:8000/chatkit-public', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    thread_id: threadId,
                    messages: [
                        { role: 'user', content: message }
                    ],
                    context: {
                        book_id: 'physical-ai-book',
                        chapter_id: window.location.pathname || 'current-section'
                    }
                })
            })
            .then(response => {
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }

                // Handle streaming response
                const reader = response.body.getReader();
                const decoder = new TextDecoder();
                let assistantMessage = '';
                let buffer = '';

                function readStream() {
                    reader.read().then(({ done, value }) => {
                        if (done) {
                            // Remove typing indicator and add final message
                            typingIndicator.remove();

                            if (assistantMessage) {
                                const botMsg = document.createElement('div');
                                botMsg.style.cssText = `
                                    max-width: 80%;
                                    padding: 12px 16px;
                                    background-color: white;
                                    border: 1px solid #e5e7eb;
                                    border-radius: 18px;
                                    align-self: flex-start;
                                    border-bottom-left-radius: 4px;
                                `;
                                botMsg.textContent = assistantMessage;
                                messagesContainer.appendChild(botMsg);

                                // Scroll to bottom
                                messagesContainer.scrollTop = messagesContainer.scrollHeight;
                            } else {
                                const botMsg = document.createElement('div');
                                botMsg.style.cssText = `
                                    max-width: 80%;
                                    padding: 12px 16px;
                                    background-color: white;
                                    border: 1px solid #e5e7eb;
                                    border-radius: 18px;
                                    align-self: flex-start;
                                    border-bottom-left-radius: 4px;
                                `;
                                botMsg.textContent = 'Sorry, I couldn\'t process your request. Please try again.';
                                messagesContainer.appendChild(botMsg);

                                // Scroll to bottom
                                messagesContainer.scrollTop = messagesContainer.scrollHeight;
                            }
                            return;
                        }

                        buffer += decoder.decode(value, { stream: true });
                        const lines = buffer.split('\n');
                        buffer = lines.pop(); // Keep last incomplete line in buffer

                        for (const line of lines) {
                            if (line.startsWith('data: ')) {
                                try {
                                    const data = JSON.parse(line.slice(6)); // Remove 'data: ' prefix
                                    if (data.delta) {
                                        assistantMessage += data.delta;
                                        // Update the typing indicator with partial response
                                        typingIndicator.innerHTML = `<span>${assistantMessage}</span>`;
                                    } else if (data.content && data.role === 'assistant') {
                                        // Complete message received
                                        break;
                                    }
                                } catch (e) {
                                    console.error('Error parsing SSE data:', e);
                                }
                            }
                        }

                        readStream(); // Continue reading
                    }).catch(error => {
                        console.error('Error reading stream:', error);
                        typingIndicator.remove();

                        const errorMsg = document.createElement('div');
                        errorMsg.style.cssText = `
                            max-width: 80%;
                            padding: 12px 16px;
                            background-color: white;
                            border: 1px solid #e5e7eb;
                            border-radius: 18px;
                            align-self: flex-start;
                            border-bottom-left-radius: 4px;
                        `;
                        errorMsg.textContent = 'Error connecting to the backend. Please check if the server is running.';
                        messagesContainer.appendChild(errorMsg);

                        // Scroll to bottom
                        messagesContainer.scrollTop = messagesContainer.scrollHeight;
                    });
                }

                readStream();
            })
            .catch(error => {
                console.error('Error sending message to backend:', error);
                typingIndicator.remove();

                const errorMsg = document.createElement('div');
                errorMsg.style.cssText = `
                    max-width: 80%;
                    padding: 12px 16px;
                    background-color: white;
                    border: 1px solid #e5e7eb;
                    border-radius: 18px;
                    align-self: flex-start;
                    border-bottom-left-radius: 4px;
                `;
                errorMsg.textContent = 'Error connecting to the backend. Please make sure the backend server is running on http://localhost:8000.';
                messagesContainer.appendChild(errorMsg);

                // Scroll to bottom
                messagesContainer.scrollTop = messagesContainer.scrollHeight;
            });

            // Scroll to bottom
            messagesContainer.scrollTop = messagesContainer.scrollHeight;
        }
    };

    // Allow sending with Enter key (without Shift)
    textArea.addEventListener('keydown', function(e) {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            sendButton.click();
        }
    });

    inputArea.appendChild(textArea);
    inputArea.appendChild(sendButton);

    // Assemble the chat window
    chatWindow.appendChild(chatHeader);
    chatWindow.appendChild(messagesContainer);
    chatWindow.appendChild(inputArea);

    // Add button and window to container
    chatContainer.appendChild(chatButton);
    chatContainer.appendChild(chatWindow);

    // Add container to the page
    document.body.appendChild(chatContainer);

    // Add click event to show/hide chat
    chatButton.onclick = function() {
        chatButton.style.display = 'none';
        chatWindow.style.display = 'flex';
    };
});