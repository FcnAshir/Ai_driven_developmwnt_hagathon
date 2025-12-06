# Lab 1: Voice Command Processing with Whisper

## Overview
This lab introduces voice command processing using OpenAI's Whisper ASR (Automatic Speech Recognition) model. You'll learn how to transcribe voice commands to text for use in VLA systems.

## Learning Objectives
- Set up and use Whisper for speech-to-text conversion
- Process audio files for command transcription
- Integrate voice input into robotics systems

## Files
- `whisper_command.py`: Implementation of Whisper-based command transcription

## Theory
Whisper is a general-purpose speech recognition model that can transcribe speech to text with high accuracy. In VLA systems, voice commands serve as the primary input modality for human-robot interaction.

## Implementation
The implementation includes:
- Loading Whisper model with appropriate size (base model recommended)
- Processing audio files for transcription
- Error handling for missing files

## Running the Demo
```bash
python whisper_command.py <path_to_audio_file>
# Example:
python whisper_command.py mock_command.wav
```

## Exercises
1. Try different Whisper model sizes (tiny, base, small) and compare performance
2. Add support for real-time microphone input
3. Implement command validation after transcription