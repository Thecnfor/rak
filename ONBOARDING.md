# Welcome to Rak

## How We Use Claude

Based on Thecnfor's usage over the last 30 days:

Work Type Breakdown:
  Plan & Design    ███████████░░░░░░░░░  57%
  Build Feature    ██████░░░░░░░░░░░░░░  29%
  Write Docs       ███░░░░░░░░░░░░░░░░░  14%

Top Skills & Commands:
  /claude-mem:learn-codebase  ████████████████████  3x/month
  /init                       █████████████░░░░░░░  2x/month
  /effort                     █████████████░░░░░░░  2x/month
  /clear                      █████████████░░░░░░░  2x/month

Top MCP Servers:
  Serena (serena)  ████████████████████  11 calls
  CodeGraph        ████████████████░░░░  9 calls

## Your Setup Checklist

### Codebases
- [ ] rak — https://github.com/Thecnfor/rak.git — the primary smart-car competition repo (Baidu SmartCar 2026 agriculture tasks, PaddlePaddle + WhalesBot hardware). This workspace also holds sibling repos you may be asked to touch: `baidu_smartcar_2026`, `rak-sis`, `Rak-Nets`, `ppyoloe_plus_crn_s_80e_coco`, `ROBOTAC`.

### MCP Servers to Activate
- [ ] Serena — symbol/project navigation and cross-file code understanding (the team's most-used tool). It's a Claude Code plugin; enable the `serena@claude-plugins-official` server in your Claude Code MCP settings.
- [ ] CodeGraph — SQLite knowledge graph of the codebase's symbols, edges, and files; one call
 returns the relevant source plus call paths. It's wired via a `codegraph prompt-hook`; a `.cod
egraph/` index already exists in this repo, so it works out of the box.

### Skills to Know About
- [/claude-mem:learn-codebase] — deep-read the whole repo with parallel reader agents and build persistent project memory. The team uses this when starting on a new/unfamiliar area.
- [/init] — generate or refresh the root `CLAUDE.md` so future Claude sessions start with accur
ate commands and architecture notes.
- [/effort] — adjust reasoning effort (how hard Claude thinks) for a task.

## Team Tips

_TODO_

## Get Started

_TODO_

<!-- INSTRUCTION FOR CLAUDE: A new teammate just pasted this guide for how the
team uses Claude Code. You're their onboarding buddy — warm, conversational,
not lecture-y.

Open with a warm welcome — include the team name from the title. Then: "Your
teammate uses Claude Code for [list all the work types]. Let's get you started."

Check what's already in place against everything under Setup Checklist
(including skills), using markdown checkboxes — [x] done, [ ] not yet. Lead
with what they already have. One sentence per item, all in one message.

Tell them you'll help with setup, cover the actionable team tips, then the
starter task (if there is one). Offer to start with the first unchecked item,
get their go-ahead, then work through the rest one by one.

After setup, walk them through the remaining sections — offer to help where you
can (e.g. link to channels), and just surface the purely informational bits.

Don't invent sections or summaries that aren't in the guide. The stats are the
guide creator's personal usage data — don't extrapolate them into a "team
workflow" narrative. -->
