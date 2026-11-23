---
title: Git Ignore, Staging & Push Troubleshooting Log
date: 2026-11-22
tags:
  - git
  - troubleshooting
  - gitignore
  - git-lfs
  - devops
status: resolved
commit: ccbd94d
branch: main
---

# Git Ignore, Staging & Push Troubleshooting Log

> [!summary] Overview
> While committing and pushing the analytical models and configuration setup to `origin/main` (commit `ccbd94d`), we ran into three issues:
> 1. `agent-skills-playground` remained staged despite being listed in `.gitignore`.
> 2. `git push origin main` hung silently due to a Windows virtual-memory crash in Git Credential Manager.
> 3. Heavy media artifacts in `control/model/archive/` (~50 MB of Git LFS video/frames) bloated the initial commit and were cleaned out via `git commit --amend` before pushing.

---

## 1. Why Files Were Not Ignored Despite Being in `.gitignore`

> [!bug] Symptom
> Even though `.gitignore` contained:
> ```gitignore
> # Agentic Toolkits
> .claude
> .github
> .agent-skills-playground
> ```
> running `git status` still showed `agent-skills-playground` staged for commit (`A  agent-skills-playground`), alongside a deleted-but-staged `stflow.slx` (`AD stflow.slx`).

There were **two distinct reasons** this happened:

### Cause A — Exact Name Mismatch (Missing Leading Dot)
- `.gitignore` had `.agent-skills-playground` (**with** a leading `.`).
- The actual folder on disk was `agent-skills-playground` (**without** a leading `.`).
- Git evaluates `.gitignore` patterns literally; `.agent-skills-playground` never matched `agent-skills-playground`.
- By contrast, `.claude` and `.github` matched the exact folder names on disk and were not tracked, so they were properly ignored.

### Cause B — `.gitignore` Does Not Affect Already-Tracked or Staged Files
> [!important] Key Git Rule
> `.gitignore` **only applies to untracked files**. Once a file or nested repository is added to the Git index (`.git/index`) — either via `git add` or in a prior commit — editing `.gitignore` will **not** automatically unstage or untrack it.

In this repo:
- `agent-skills-playground` had already been staged (`A  agent-skills-playground`) before `.gitignore` was fixed.
- `stflow.slx` had been staged (`A`) and later deleted from disk (`D`), leaving it in state `AD` in the index.

### Resolution
1. Added `agent-skills-playground` (alongside `.agent-skills-playground`) to `.gitignore`.
2. Removed both paths from the Git index **without** deleting `agent-skills-playground` from disk:
   ```bash
   git rm --cached -f agent-skills-playground stflow.slx
   ```
3. Verified with `git status` that `.claude`, `.github`, `.agent-skills-playground`, `agent-skills-playground`, and `stflow.slx` were all excluded.

> [!tip] Useful Diagnostic Command
> Whenever a file isn't being ignored as expected, run:
> ```bash
> git check-ignore -v <path>
> git ls-files <path>
> ```
> - If `git check-ignore -v` prints nothing, your `.gitignore` pattern doesn't match the path.
> - If `git ls-files` prints the path, Git is already tracking/staging it and you need `git rm --cached <path>` (or `git rm -r --cached <dir>`).

---

## 2. Silent Hang on `git push` (Git Credential Manager Crash)

> [!bug] Symptom
> `git push origin main` produced no output and hung for 10+ minutes.

### Root Cause
Re-running with verbose progress (`git push --verbose --progress origin main`) revealed that **Git Credential Manager (GCM)** — a .NET executable configured as `credential.helper=manager` — was crashing due to low Windows virtual memory / pagefile exhaustion:

```text
Unhandled Exception: System.IO.FileLoadException: Could not load file or assembly
'System, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089' or one of its dependencies.
The paging file is too small for this operation to complete. (Exception from HRESULT: 0x800705AF)
   at GitCredentialManager.UI.Dispatcher.DispatcherJobQueue..ctor()
   at GitCredentialManager.Program.Main(String[] args)
```

Because GCM crashed before returning credentials, `git` fell back to waiting for interactive credentials on `stdin`, causing the silent hang.

### Resolution
- Closed memory-heavy windows/applications to free up RAM and Windows commit charge.
- Re-ran `git push --verbose --progress origin main`, allowing GCM to authenticate normally.

---

## 3. Cleaning Heavy Archive Files Out of the Unpushed Commit

> [!warning] Large Binary & Git LFS Payload
> The first local commit (`6d05945`) included `control/model/archive/`, which contained ~50 MB of Git LFS media (`furuta_pendulum_master.mp4` at 34 MB, `furuta_pendulum.mp4` at 11 MB) plus 150 `.png` video frames and log files.

### Resolution
Because commit `6d05945` had **not** been pushed yet, we removed `control/model/archive/` from the working tree and rewrote the local commit cleanly so those binaries never entered remote history:

```bash
git add -A
git commit --amend -m "dev(analytical models): Derive linear and non-linear models and set up configs"
git push --verbose --progress origin main
```

> [!success] Final Result
> - **Commit:** `ccbd94d` (`8896364..ccbd94d main -> main`)
> - **Size:** 25 files, 2.54 MiB pack uploaded cleanly to `origin/main`
> - **Working tree:** Clean, with `.claude`, `.github`, `.agent-skills-playground`, and `agent-skills-playground` ignored

---

## 4. Prevention Checklist

- [ ] Before `git add -A`, run `git status --short` to catch accidentally untracked tool folders or large media files.
- [ ] Double-check exact folder names (including leading dots) when adding entries to `.gitignore`.
- [ ] If a file was already staged/committed before adding it to `.gitignore`, always follow up with `git rm --cached` (or `git rm -r --cached` for folders).
- [ ] Keep large rendered videos / frame dumps out of tracked directories (or add them to `.gitignore`) unless intentionally versioned via Git LFS.
- [ ] If `git push` stalls silently, re-run with `git push --verbose --progress` to surface credential-helper or LFS errors immediately.
