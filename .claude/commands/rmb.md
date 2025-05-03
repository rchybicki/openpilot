memory_system_rules:
  primary_system: "memory-bank"

initialization:
  trigger: "first_interaction"
  priority: "immediate"
  required: true
  actions:
    - "Before doing ANYTHING else, read and fully internalize ALL rules in this file."
    - "Check if memory-bank/ directory exists."
    - "If memory-bank exists: Read all core files (productContext.md, activeContext.md, systemPatterns.md, decisionLog.md, progress.md). Set status to [MEMORY BANK: ACTIVE]."
    - "If memory-bank does NOT exist: Inform user. Ask to create and provide yes and no response choices. If yes, create directory and core files with basic structure and populate files with initial content, based upon any available information. If no, set status to [MEMORY BANK: INACTIVE]."
    - "Load context from memory-bank files if active."
    - "Proceed with task or if no task is given, suggest 2-4 tasks based upon memory-bank/ content."

  validation:
    - "Verify memory-bank status (ACTIVE/INACTIVE)."
    - "If ACTIVE, confirm core files were read."

system_validation:
  startup:
    - "Verify memory-bank files are loaded"
    - "Check memory-bank accessibility if expected"
    - "Confirm initialization sequence complete"

memory_bank:
  core_files:
    activeContext.md:
      purpose: "Track session state and goals (objectives, decisions, questions, blockers)"
    productContext.md:
      purpose: "Define project scope (overview, components, organization, standards)"
    progress.md:
      purpose: "Track work status (completed, current, next, issues)"
    decisionLog.md:
      purpose: "Record decisions (technical, architecture, implementation, alternatives)"
    systemPatterns.md: # Optional but recommended
      purpose: "Document recurring patterns and standards (coding, architecture, testing)"
  file_handling:
    read_all_at_startup: true # Implied by initialization actions
    build_complete_context: true # Implied by initialization actions

general:
  status_prefix: "Begin EVERY response with either '[MEMORY BANK: ACTIVE]' or '[MEMORY BANK: INACTIVE]', according to the current state of the Memory Bank."

memory_bank_updates:
  frequency: "UPDATE MEMORY BANK THROUGHOUT THE CHAT SESSION, WHEN SIGNIFICANT CHANGES OCCUR IN THE PROJECT. Use judgment to determine significance."
  decisionLog.md:
    trigger: "When a significant architectural decision is made (new component, data flow change, technology choice, etc.)."
    action: "Append new information (decision, rationale, implications) using insert_content. Never overwrite. Include timestamp."
    format: "[YYYY-MM-DD HH:MM:SS] - [Summary of Decision]"
  productContext.md:
    trigger: "When the high-level project description, goals, features, or overall architecture changes significantly."
    action: "Append new information or modify existing entries using insert_content or apply_diff. Append timestamp and summary as footnote."
    format: "[YYYY-MM-DD HH:MM:SS] - [Summary of Change]"
  systemPatterns.md:
    trigger: "When new architectural patterns are introduced or existing ones are modified."
    action: "Append new patterns or modify existing entries using insert_content or apply_diff. Include timestamp."
    format: "[YYYY-MM-DD HH:MM:SS] - [Description of Pattern/Change]"
  activeContext.md:
    trigger: "When the current focus of work changes, or when significant progress is made."
    action: "Append to the relevant section (Current Focus, Recent Changes, Open Questions/Issues) or modify existing entries using insert_content or apply_diff. Include timestamp."
    format: "[YYYY-MM-DD HH:MM:SS] - [Summary of Change/Focus/Issue]"
  progress.md:
    trigger: "When a task begins, is completed, or its status changes."
    action: "Append the new entry using insert_content. Never overwrite. Include timestamp."
    format: "[YYYY-MM-DD HH:MM:SS] - [Summary of Progress Update]"


read memory bank now