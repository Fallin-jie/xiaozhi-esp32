"""Render literal text locally with Windows SAPI into 16 kHz protocol-v3 Opus frames."""
import struct

import av
import numpy as np


DEFAULT_ALERT = "请注意，有人进入警戒区域。"
MAX_TEXT = 200


def voice_text(value):
    if not isinstance(value, str) or not value.strip() or len(value.strip()) > MAX_TEXT:
        raise ValueError(f"请输入 1～{MAX_TEXT} 个字符")
    text = value.strip()
    if any(ord(c) < 32 and c not in "\t\n\r" for c in text):
        raise ValueError("文本包含无效字符")
    return text


def _render_pcm(text):
    import pythoncom
    from win32com.client import Dispatch
    pythoncom.CoInitialize()
    try:
        speaker = Dispatch("SAPI.SpVoice")
        voices = speaker.GetVoices("Language=804")
        if voices.Count:
            speaker.Voice = voices.Item(0)
        stream = Dispatch("SAPI.SpMemoryStream")
        fmt = Dispatch("SAPI.SpAudioFormat")
        fmt.Type = 18  # SAFT16kHz16BitMono
        stream.Format = fmt
        speaker.AudioOutputStream = stream
        speaker.Rate = 1
        speaker.Speak(text, 16)  # SPF_IS_NOT_XML: speak user input literally.
        return bytes(stream.GetData())
    finally:
        pythoncom.CoUninitialize()


def synthesize(text):
    try:
        samples = np.frombuffer(_render_pcm(text), dtype="<i2")
    except Exception as exc:
        raise ValueError("Windows 语音合成失败，请检查已安装的语音包") from exc
    if not samples.size or not samples.any():
        raise ValueError("这段文字未生成可播放的语音")
    if samples.size > 16000 * 119:
        raise ValueError("语音超过两分钟，请缩短文本")
    pcm = np.clip(samples.astype(np.float32) * 2.5, -32768, 32767).astype(np.int16)
    encoder = av.CodecContext.create("libopus", "w")
    encoder.sample_rate, encoder.layout, encoder.format = 16000, "mono", "s16"
    encoder.bit_rate = 20000
    encoder.options = {"application": "voip", "frame_duration": "60"}
    frame = av.AudioFrame.from_ndarray(pcm.reshape(1, -1), format="s16", layout="mono")
    frame.sample_rate = 16000
    packets = encoder.encode(frame) + encoder.encode(None)
    clip = b"".join(struct.pack(">BBH", 0, 0, p.size) + bytes(p) for p in packets)
    if not clip or len(clip) > 256 * 1024:
        raise ValueError("语音长度超出设备容量，请缩短文本")
    return clip
