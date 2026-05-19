---
name: user-preferences
description: Collaboration style observed working with the ERASOR2 maintainer
metadata:
  type: user
---

Hyungtae works mostly in Korean but switches to English freely. Tends
to dictate scope ("일단 …해보자", "이건 스킵하자") and prefers stepwise
shipping over big-bang changes — he asked to ship v1.0 with CI workflow
present but trigger-disabled rather than do the full self-hosted runner
setup right then.

Core preferences observed:

- **Performance must be preserved**: the rerun migration was explicitly
  gated on "pipeline shouldn't be largely changed, because maintaining
  the performance matters." Byte-exact parity was the success criterion.
- **Practical over elegant**: was happy with `subprocess` Python
  wrappers around C++ binaries rather than asking for pybind11 bindings.
  When CI setup looked complex, his call was "일단 스킵하자."
- **Docker-first dev environment**: original development happens inside
  `shapelim/opengl-ubuntu20.04-erasor2:latest`, not on the host.
- **Will scp files between machines** rather than fighting LFS / cloud
  storage. Bundle fixtures as a tarball, expect ~1 GB transfers.

Don't:

- Auto-commit work without being asked. v1 push was an explicit request.
- Push to `main` or tag releases without confirming once. He confirmed
  via AskUserQuestion before the v1.0 push.
- Mock things in the parity check. Use real data + real golden, even if
  it costs ~5 min per CI run.

Do:

- Use Korean when responding to Korean prompts (he's bilingual but
  tends to read in whatever language the question is in).
- Use the AskUserQuestion tool when there are 2-3 reasonable next steps
  — he likes picking from a short menu, not free-text replies.
- Show concise diff stats / metric tables rather than long prose. He
  reads tables fast.
