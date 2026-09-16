---
name: "prompt-optimizer"
description: "Best practices for prompt engineering and optimization. Invoke when refining LLM prompts, implementing few-shot learning, or debugging AI reasoning outputs."
---

# Prompt Optimizer

This skill provides strategies and templates for optimizing Large Language Model (LLM) prompts, ensuring high-quality, predictable, and structured outputs for downstream systems.

## When to Invoke
- When LLMs (like Qwen or GPT) fail to follow JSON formatting rules or hallucinate instructions.
- When applying techniques like Chain-of-Thought (CoT), Few-Shot prompting, or DSPy-like optimization.
- When generating complex system prompts for autonomous agents (e.g., instructing an LLM to generate excavator action playbooks).

## Best Practices
1. **Clear Persona & Constraints**: Always start prompts by defining a strict persona and explicitly stating what the model MUST and MUST NOT do.
2. **Few-Shot Examples**: Provide 2-3 high-quality examples of the expected input-output pairs. This is significantly more effective than long descriptive instructions.
3. **Chain-of-Thought (CoT)**: For complex reasoning (like deciding how to move an excavator arm around an obstacle), force the model to output a `<thinking>` block before emitting the final `<json>` command.
4. **Iterative Tuning**: Treat prompts like code. Use a test set of challenging scenarios to evaluate prompt changes systematically.