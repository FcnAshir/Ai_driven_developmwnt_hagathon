# Lab 1: Whisper Installation and Voice Command

## Objective

This lab will guide you through installing OpenAI's Whisper speech-to-text model and implementing a basic Python script to process an audio file and transcribe spoken commands.

## Prerequisites

- Ubuntu 22.04 LTS (or similar Linux distribution)
- Python 3.8+
- `pip` package manager

## Steps

### 1. Install Whisper

First, open a terminal and install Whisper using pip:

```bash
pip install -U openai-whisper
```

You might also need to install `ffmpeg` for audio processing:

```bash
sudo apt update
sudo apt install ffmpeg
```

### 2. Create the Python Script

Create a new file named `whisper_command.py` in `src/vla_agents/lab1/` with the following content:

```python
import whisper
import sys
import os

def transcribe_audio(audio_path):
    """
    Transcribes an audio file using the Whisper ASR model.
    """
    if not os.path.exists(audio_path):
        print(f"Error: Audio file not found at {audio_path}")
        sys.exit(1)

    print(f"Loading Whisper model (this may take a moment)...")
    # You can choose different models like 'tiny', 'base', 'small', 'medium', 'large'
    # 'base' is a good balance for accuracy and speed
    model = whisper.load_model("base")

    print(f"Transcribing audio from {audio_path}...")
    result = model.transcribe(audio_path)

    print("\n--- Transcription Result ---")
    print(result["text"])
    print("--------------------------")
    return result["text"]

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python whisper_command.py <path_to_audio_file>")
        print("Example: python whisper_command.py my_voice_command.wav")
        sys.exit(1)

    audio_file = sys.argv[1]
    transcribe_audio(audio_file)
```

### 3. Prepare an Audio File

You will need an audio file (e.g., `.wav`, `.mp3`) containing a spoken command. For example, record yourself saying "robot, move forward five meters". Save this file as `my_voice_command.wav` (or similar) in a convenient location.

### 4. Run the Script

Execute the Python script from your terminal, providing the path to your audio file:

```bash
python src/vla_agents/lab1/whisper_command.py <path_to_your_audio_file>
```

Replace `<path_to_your_audio_file>` with the actual path to your recorded command.

### Expected Output

The script will load the Whisper model, transcribe your audio, and print the detected text.

```
Loading Whisper model (this may take a moment)...
Transcribing audio from my_voice_command.wav...

--- Transcription Result ---
 Robot, move forward five meters.
--------------------------
```

## Troubleshooting

-   **`No module named 'whisper'`**: Ensure Whisper is installed (`pip install -U openai-whisper`).
-   **`ffmpeg not found`**: Install `ffmpeg` (`sudo apt install ffmpeg`).
-   **Low accuracy**: Try a larger Whisper model (e.g., `model = whisper.load_model("small")` or `"medium"`), or ensure your audio quality is good.
-   **`Error: Audio file not found`**: Double-check the path to your audio file.

## Next Steps

This lab provides the foundation for converting speech to text. In subsequent labs, we will integrate this output with LLMs for planning and ROS 2 for action execution.
