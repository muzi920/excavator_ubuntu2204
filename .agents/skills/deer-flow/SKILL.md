---
name: "deer-flow"
description: "Guidelines for orchestrating data pipelines and workflows using ByteDance's Deer-Flow. Invoke when building DAGs, data processing tasks, or model training pipelines."
---

# Deer-Flow (ByteDance) Workflow Orchestration

This skill provides guidelines and templates for using Deer-Flow to orchestrate complex data processing, computer vision, and machine learning pipelines.

## When to Invoke
- When designing Directed Acyclic Graphs (DAGs) for data pipelines.
- When orchestrating tasks like data collection from ROS2 bags, image preprocessing, and YOLO/ViT model training.
- When configuring task dependencies, retries, and resource allocations in Deer-Flow.

## Best Practices
1. **Modularity**: Break down complex machine learning workflows into independent Deer-Flow nodes (e.g., `DataExtractionNode`, `DataAugmentationNode`, `ModelTrainingNode`).
2. **Configuration Management**: Externalize pipeline configurations (batch sizes, epochs, data paths) into YAML or JSON files for easy tuning.
3. **Error Handling & Retries**: Always implement graceful degradation and automatic retries for nodes that depend on external resources (like pulling large datasets).