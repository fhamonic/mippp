# Working on a contributor's PR

Pushing to the contributor's branch updates their PR and runs CI (`c-cpp.yml`
triggers on `pull_request` to `main`). Nothing gets merged until someone
clicks merge, runs `gh pr merge` or enables auto-merge. A draft PR can't be
merged at all.

The commands use PR #3 (`mheyman:astra/iis-support`). For another PR, read the
values from:

```bash
gh pr view <N> --json headRepositoryOwner,headRefName,maintainerCanModify,isDraft
```

Pushing to the fork needs `maintainerCanModify: true` ("Allow edits by
maintainers" on the PR). Without it, push to `origin` and open your own draft
PR (`gh pr create --draft --base main`). A branch pushed to `origin` without a
PR doesn't start CI.

## 1. Make it a draft (once)

Click "Convert to draft" under *Reviewers* in the PR sidebar, or run:

```bash
gh api graphql -F id="$(gh pr view 3 --json id --jq .id)" \
  -f query='mutation($id:ID!){convertPullRequestToDraft(input:{pullRequestId:$id}){pullRequest{isDraft}}}'
```

CI still runs on every push to a draft.

## 2. Check out the branch (once)

```bash
git remote add mheyman git@github.com:mheyman/mippp.git
git fetch mheyman
git switch -c astra/iis-support --track mheyman/astra/iis-support
git status -sb        # ## astra/iis-support...mheyman/astra/iis-support
```

`gh pr checkout 3` should do the same thing. The named remote makes `git status`
show ahead/behind counts.

## 3. Work loop

```bash
git pull --rebase     # pick up the author's new commits first
# ... edit, build, test ...
git commit -am "..."
git pull --rebase     # again, right before pushing
git push              # updates the PR, CI starts
```

- A bare `git push` works because the local and remote branch names match
  (`push.default` is `simple`).
- Never use `--force`: it's the author's branch. If a push is rejected, run
  `git pull --rebase` and push again.
- `--rebase` replays only your unpushed commits on top of theirs.

## 4. Watch CI

```bash
gh pr checks 3        # one-shot summary
gh run watch          # pick the run and follow it live
```

## 5. Finish

```bash
gh pr ready 3         # leave draft state
gh pr merge 3         # or merge from the web UI
```

## gh 2.4.0 (Ubuntu package)

It has no `gh pr ready --undo` (hence the GraphQL call in step 1) and no
`gh pr checks --watch` (hence `gh run watch`). The gh from GitHub's own apt
repository has both.
