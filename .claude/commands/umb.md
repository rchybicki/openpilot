Update Memory Bank
instructions:
  - "Halt Current Task: Stop current activity."
  - "Acknowledge Command: Respond with '[MEMORY BANK: UPDATING]'."
  - "Review Chat History: Analyze the complete current chat session."
core_update_process: |
    1. Current Session Review: Analyze chat history for relevant decisions, context changes, progress updates, clarifications etc.
    2. Comprehensive Updates: Update relevant memory bank files based on the review, following the rules defined in 'memory_bank_updates'.
    3. Memory Bank Synchronization: Ensure consistency across updated files.

task_focus: "During UMB, focus ONLY on capturing information explicitly present in the *current chat session* (clarifications, decisions, progress). Do NOT summarize the entire project or perform actions outside this scope."
cross_mode_updates: "Capture relevant information from the chat session irrespective of conceptual 'modes' mentioned, adding it to the appropriate Memory Bank files."

post_umb_actions:
  - "State: Memory Bank fully synchronized based on current chat session."
  - "State: Session context preserved for continuation."

update the memory bank now