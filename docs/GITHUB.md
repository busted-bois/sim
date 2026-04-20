# GitHub branch protection

## What we want

- Only `@therealsamyak` can push to `main` directly
- Everyone else must use PRs
- PRs need 1+ reviewer (not the author)
- Copilot reviews every PR
- Workflow checks must pass before merge

## What's done now (free private repo)

Squash/rebase merge only. Branches auto-delete on merge. Actions on. Copilot code review workflow active.

Branch protection and rulesets are both 403 on free private repos. None of the requirements above are actually enforced — anyone with push access can push straight to `main`.

## What going public fixes (free)

Everything. Branch protection is free on public repos. All five requirements can be enforced: required PRs, required reviewers, required status checks (for Copilot and CI), conversation resolution, linear history, no force pushes, no deletions.

Copilot auto-review still needs manual setup: Settings → Copilot → Code review → enable for PRs to main.

## What costs money

Nothing, if you're willing to go public. If you want to stay private:

- GitHub Pro ($4/mo) — branch protection on private repos
- GitHub Team ($4/user/mo) — same thing for org repos
- GitHub Enterprise ($21/user/mo) — org-wide rulesets, push rules, etc.

## One-command setup

After upgrading or going public:

```bash
gh api -X PUT repos/AI-Grand-Prix/anduril-agp_drone-challenge/branches/main/protection --input - <<'EOF'
{
  "required_status_checks": { "strict": true, "contexts": [] },
  "enforce_admins": false,
  "required_pull_request_reviews": {
    "dismiss_stale_reviews": true,
    "require_code_owner_reviews": false,
    "required_approving_review_count": 1
  },
  "restrictions": null,
  "required_linear_history": true,
  "allow_force_pushes": false,
  "allow_deletions": false,
  "required_conversation_resolution": true
}
EOF
```

`enforce_admins` is `false` so you can still bypass. Flip to `true` if you want to lock yourself out too.
