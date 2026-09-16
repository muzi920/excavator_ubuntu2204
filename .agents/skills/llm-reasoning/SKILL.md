---
name: "llm-reasoning"
description: "Integrates Large Language Models (Qwen, Transformers) for high-level decision making. Invoke when designing prompt engineering, reasoning tasks, or multimodal LLM integration."
---

# LLM Reasoning & Decision Making

Guidelines for integrating Large Language Models (LLMs) and Vision-Language Models (VLMs) like Qwen or generic Transformers into the robotic control pipeline.

## When to Invoke
- When translating natural language instructions into JSON action playbooks.
- When using Qwen-VL or similar multimodal models to analyze the excavator's environment and make high-level digging decisions.
- When building the reasoning engine for autonomous operation and task planning.

## Best Practices
1. **Prompt Engineering**: Design strict system prompts that output structured formats (like JSON) compatible with the excavator's `ClosedLoopScriptRunner`.
2. **Local Inference**: Use frameworks like `vLLM`, `llama.cpp`, or `Ollama` for local deployment of Qwen/Transformer models to ensure low latency and data privacy.
3. **Feedback Loop**: Feed the current joint angles and visual descriptions back to the LLM to correct errors in real-time, forming a "perceive -> reason -> act -> evaluate" loop.