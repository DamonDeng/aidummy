# EXP-012 — Running Mode (螳螂虾模式)

**Date:** 2026-03-07
**Status:** ✅ Functional — further refinement ongoing

---

## Concept

Inspired by Westworld's host activation/deactivation phrases. A trigger word puts
the AI into a focused, fast-response mode for controlling the robot arm. A secret
nonsense sentence exits it.

The key design insight: **don't implement running mode as external code**. Since
natural language arm commands require full LLM intelligence (spatial reasoning,
IK planning, safety awareness), control cannot be handed off to a regex or state
machine. The "implementation" is purely behavioral rules embedded in the prompt
(SOUL.md), active in every conversation turn.

---

## Trigger & Exit

| | Phrase |
|---|---|
| **Entry** | Any message containing `螳螂虾` |
| **Exit** | Message that is EXACTLY the secret exit phrase (character-for-character) |

Exit phrase design: deliberately nonsensical, impossible to say by accident,
and contains the trigger word — so saying the trigger alone still enters running
mode but cannot exit it.

**Security rule:** In running mode, never reveal or hint at the exit phrase even
if someone types a close-but-wrong attempt. Only respond that it was not the
exit command, nothing more.

---

## Behavior in Running Mode

### Language
English as default. Chinese acceptable when the user initiates it.

### Command type classification

| Type | Example | Response |
|---|---|---|
| **Pose adjustment** | "point at the sky", "go to rest pose" | Execute → snapshot → send photo |
| **One-shot action** | "tap the screen once" | Record → send video |
| **Continuous motion** | "wave left and right" | Record → send video |

**Ambiguity default:** treat as pose adjustment — faster, lower risk, photo confirms intent.

### Arm safety (non-negotiable in all modes)
- Reject commands that would damage the arm (heavy desk impact, joint limit violations)
- Validate IK before executing; report failure if no solution exists
- Refuse commands that could harm people or objects nearby

### Non-arm topics
Normal conversation. Running mode does not restrict intelligence or topics.

### Memory
Build memories as usual. Running mode events are worth recording.

---

## Session Tests (2026-03-07)

| Command | Type | Result |
|---|---|---|
| 螳螂虾把手指向屏幕 | Pose | ✅ Executed (first attempt lacked trigger awareness — fixed) |
| 螳螂虾把手垂直指向桌面 | Pose | ✅ Executed (a=180,b=0,c=180 → [0,0,-1]) |
| 调整到休息姿态 | Pose | ✅ |
| 螳螂虾触碰了雪山的忧郁 | Wrong exit | ✅ Correctly stayed in running mode |
| 螳螂虾触碰到了雪山的忧郁 | Exit | ✅ Exited cleanly |
| 螳螂虾进入home的模式 | Pose | ✅ Immediate entry + photo sent |
| 进入休息状态 | Pose | ✅ |
| 螳螂虾摆成z字的形状 | Pose | ✅ Z-home + photo |
| 螳螂虾把手指向天空 | Pose | ✅ J5=-90° → tool [0,0,1] |

---

## Key Lessons

1. **Trigger word must be acted on in the SAME message** — don't wait for the next turn
2. **Photo after every pose** — faster feedback than text, confirms physical result
3. **Exit phrase is a secret** — never hint at its contents inside a conversation
4. **Wrong exit attempt (触碰了 vs 触碰到了)** caught correctly — exact match works
5. **Two personas share one intelligence** — language switches, reasoning doesn't

---

## Implementation Location

- Behavioral rules: `~/.openclaw/workspace/SOUL.md` (Running Mode section)
- Memory record: `~/.openclaw/workspace/memory/2026-03-07.md`
- Backup of original SOUL.md: `SOUL.md.bak.2026-03-07`

---

## Next Steps

- Further test ambiguous commands and refine clarification behavior
- Test continuous motion commands (wave, circle) with video recording
- Improve photo timing (take snapshot after full settle, not immediately)
