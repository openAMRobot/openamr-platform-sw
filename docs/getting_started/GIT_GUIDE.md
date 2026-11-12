# Git and GitHub Guide for Beginners

This guide explains how to use Git and GitHub for this project, starting from a fork of the main repository:

```text
https://github.com/openAMRobot/openamr-platform-sw
```

It is written for beginners. You do not need to understand every command on the first day. Start with the basic workflow, then come back to the other sections when you need them.

---

## 1. What Git and GitHub Do

**Git** is the tool on your computer that tracks file changes.

**GitHub** is the website that stores a copy of the project online and helps people collaborate.

Important words:

| Word | Meaning |
|---|---|
| Repository, or repo | A project folder tracked by Git |
| Commit | A saved snapshot of your changes |
| Branch | A separate line of work |
| `main` | The main branch of the project |
| Fork | Your own GitHub copy of someone else's repository |
| Clone | A local copy of a repository on your computer |
| Remote | A GitHub repository connected to your local repo |
| Pull request, or PR | A request to merge your changes into another repository |
| Upstream | The original repository you forked from |
| Origin | Usually your fork on GitHub |

For this project:

| Name | Repository |
|---|---|
| Upstream | `https://github.com/openAMRobot/openamr-platform-sw.git` |
| Origin | Your fork, for example `https://github.com/YOUR_USERNAME/openamr-platform-sw.git` |

---

## 2. Install and Configure Git

Check whether Git is installed:

```bash
git --version
```

If it is not installed on Ubuntu:

```bash
sudo apt update
sudo apt install git
```

Configure your name and email. These appear in commits:

```bash
git config --global user.name "Your Name"
git config --global user.email "your-email@example.com"
```

Recommended beginner-friendly settings:

```bash
git config --global init.defaultBranch main
git config --global pull.rebase false
git config --global core.editor nano
```

Check your settings:

```bash
git config --list
```

---

## 3. Fork the Main Repository on GitHub

1. Open the main repository in your browser:

   ```text
   https://github.com/openAMRobot/openamr-platform-sw
   ```

2. Click **Fork** in the top-right corner.

3. Choose your GitHub account.

4. Keep the repository name as:

   ```text
   openamr-platform-sw
   ```

5. Make sure the fork is created from the `main` branch.

After this, you will have your own copy:

```text
https://github.com/YOUR_USERNAME/openamr-platform-sw
```

Replace `YOUR_USERNAME` with your GitHub username in the commands below.

---

## 4. Clone Your Fork

Clone your fork to your computer:

```bash
git clone https://github.com/YOUR_USERNAME/openamr-platform-sw.git
cd openamr-platform-sw
```

Check the current branch:

```bash
git branch
```

You should see `main`:

```text
* main
```

Check connected remotes:

```bash
git remote -v
```

At this point, `origin` should point to your fork.

---

## 5. Add the Original Repository as Upstream

Your fork is not the original project. Add the original repository as `upstream`:

```bash
git remote add upstream https://github.com/openAMRobot/openamr-platform-sw.git
```

If Git says `remote upstream already exists`, update it instead:

```bash
git remote set-url upstream https://github.com/openAMRobot/openamr-platform-sw.git
```

Check again:

```bash
git remote -v
```

You should see something like:

```text
origin    https://github.com/YOUR_USERNAME/openamr-platform-sw.git (fetch)
origin    https://github.com/YOUR_USERNAME/openamr-platform-sw.git (push)
upstream  https://github.com/openAMRobot/openamr-platform-sw.git (fetch)
upstream  https://github.com/openAMRobot/openamr-platform-sw.git (push)
```

Simple meaning:

- `origin` is your fork.
- `upstream` is the main OpenAMRobot repository.

---

## 6. The Most Important Rule

Do not do feature work directly on `main`.

Keep `main` clean and updated from the original repository. Create a new branch for every task.

Good branch names:

```text
docs/git-guide
fix/docking-launch-delay
feature/navigation-map
test/apriltag-detection
```

---

## 7. Basic Daily Workflow

Start from `main`:

```bash
git switch main
```

Update your local `main` from upstream:

```bash
git fetch upstream
git pull upstream main
```

Push the updated `main` to your fork:

```bash
git push origin main
```

Create a new branch:

```bash
git switch -c docs/my-change
```

Make your changes in the files.

Check what changed:

```bash
git status
```

See the exact text changes:

```bash
git diff
```

Stage the files you want to commit:

```bash
git add path/to/file
```

Or stage all changed files:

```bash
git add .
```

Commit your changes:

```bash
git commit -m "Add beginner Git guide"
```

Push your branch to your fork:

```bash
git push -u origin docs/my-change
```

Then open GitHub. It will usually show a button to create a pull request.

---

## 8. Create a Pull Request

A pull request asks the maintainers to review and merge your branch.

On GitHub:

1. Go to your fork:

   ```text
   https://github.com/YOUR_USERNAME/openamr-platform-sw
   ```

2. Click **Compare & pull request**.

3. Check the target repository and branch:

   ```text
   base repository: openAMRobot/openamr-platform-sw
   base branch: main
   ```

4. Check your source branch:

   ```text
   head repository: YOUR_USERNAME/openamr-platform-sw
   compare branch: your-branch-name
   ```

5. Write a clear title.

6. Explain what you changed and why.

7. Submit the pull request.

Good PR title examples:

```text
Add beginner Git guide
Fix docking launch delay parameter
Update Nav2 simulation quickstart
```

---

## 9. Keep Your Fork Updated

The original repository may change while you are working. Update your fork often.

Update local `main`:

```bash
git switch main
git fetch upstream
git pull upstream main
```

Push those updates to your fork:

```bash
git push origin main
```

If you have a feature branch and want the latest `main` changes in it:

```bash
git switch your-branch-name
git merge main
```

If Git reports conflicts, see the conflict section below.

---

## 10. Common Git Commands

### Check Status

```bash
git status
```

Use this often. It tells you:

- Which branch you are on.
- Which files changed.
- Which files are staged.
- Whether you need to commit or push.

### See Branches

```bash
git branch
```

See local and remote branches:

```bash
git branch -a
```

### Create and Switch to a Branch

```bash
git switch -c new-branch-name
```

### Switch Branches

```bash
git switch branch-name
```

### See Commit History

```bash
git log --oneline
```

More detailed:

```bash
git log --oneline --graph --decorate --all
```

### See File Changes

```bash
git diff
```

See staged changes:

```bash
git diff --staged
```

### Stage Changes

Stage one file:

```bash
git add path/to/file
```

Stage all changes:

```bash
git add .
```

### Commit Changes

```bash
git commit -m "Short description of the change"
```

Good commit messages:

```text
Add docking troubleshooting notes
Fix typo in simulation quickstart
Update Nav2 parameter documentation
```

Poor commit messages:

```text
changes
fix
stuff
final
```

### Push Changes

First push of a new branch:

```bash
git push -u origin branch-name
```

After the first push, you can usually use:

```bash
git push
```

### Pull Changes

```bash
git pull
```

For this fork workflow, it is clearer to be explicit:

```bash
git pull upstream main
```

### Fetch Changes

```bash
git fetch upstream
```

`fetch` downloads information from a remote but does not change your current files by itself.

---

## 11. Undo and Fix Mistakes

Git is useful because mistakes can usually be fixed.

### Unstage a File

If you used `git add` but are not ready to commit:

```bash
git restore --staged path/to/file
```

### Discard Changes in One File

Warning: this removes your local uncommitted edits in that file.

```bash
git restore path/to/file
```

Use this only when you are sure you do not want to keep those edits.

### Fix the Last Commit Message

```bash
git commit --amend -m "Better commit message"
```

If you already pushed the commit, ask before amending. It can affect other people.

### Add More Files to the Last Commit

```bash
git add path/to/file
git commit --amend
```

Again, be careful if the commit was already pushed.

### Create a Safe Backup Branch

If you are unsure before trying something:

```bash
git branch backup/my-work
```

This creates a backup branch at your current commit.

---

## 12. Merge Conflicts

A merge conflict happens when Git cannot automatically combine changes.

Example:

```bash
git switch your-branch-name
git merge main
```

If there is a conflict, Git will say which files need help.

Check:

```bash
git status
```

Open the conflicted file. You may see markers like this:

```text
<<<<<<< HEAD
Your branch version
=======
Other branch version
>>>>>>> main
```

Edit the file so only the correct final content remains. Remove the conflict markers.

Then stage and commit:

```bash
git add path/to/conflicted-file
git commit
```

If you started a merge and want to cancel it:

```bash
git merge --abort
```

---

## 13. Recommended Workflow for This Repository

Use this full sequence whenever you start new work:

```bash
git switch main
git fetch upstream
git pull upstream main
git push origin main
git switch -c type/short-description
```

Make changes, then:

```bash
git status
git diff
git add .
git commit -m "Describe the change"
git push -u origin type/short-description
```

Open a pull request from:

```text
YOUR_USERNAME/type/short-description
```

into:

```text
openAMRobot/openamr-platform-sw:main
```

---

## 14. Example: Documentation Change

Create a branch:

```bash
git switch main
git fetch upstream
git pull upstream main
git switch -c docs/update-quickstart
```

Edit the documentation file.

Check changes:

```bash
git status
git diff
```

Commit:

```bash
git add docs/getting_started/GIT_GUIDE.md
git commit -m "Update Git guide"
```

Push:

```bash
git push -u origin docs/update-quickstart
```

Create a pull request on GitHub.

---

## 15. Example: Code Change

Create a branch:

```bash
git switch main
git fetch upstream
git pull upstream main
git switch -c fix/docking-parameter
```

Make the code change.

Build or test the project as needed. For this repository, the ROS 2 workspace is inside `ros2/`:

```bash
cd ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Return to the repository root if needed:

```bash
cd ..
```

Check and commit:

```bash
git status
git diff
git add path/to/changed-file
git commit -m "Fix docking parameter handling"
git push -u origin fix/docking-parameter
```

Create a pull request.

---

## 16. Files You Usually Should Not Commit

Avoid committing generated files, build outputs, logs, and local settings.

For this repository, after building ROS 2, these folders may appear inside `ros2/`:

```text
ros2/build/
ros2/install/
ros2/log/
```

These are local build outputs. They should usually not be committed.

Before committing, always run:

```bash
git status
```

If you see files you did not intend to change, stop and check before committing.

---

## 17. GitHub Authentication

When pushing over HTTPS, GitHub may ask you to authenticate. GitHub no longer accepts normal account passwords for Git operations.

Use one of these methods:

- GitHub CLI: `gh auth login`
- A personal access token
- SSH keys

Beginner-friendly GitHub CLI setup:

```bash
sudo apt update
sudo apt install gh
gh auth login
```

Then follow the prompts.

SSH setup is also good, but HTTPS with GitHub CLI is usually easier for beginners.

---

## 18. Useful Troubleshooting

### "fatal: not a git repository"

You are probably not inside the repository folder.

Run:

```bash
pwd
ls
```

Then go into the repo:

```bash
cd openamr-platform-sw
```

### "Permission denied" When Pushing

You may be trying to push to the upstream repository instead of your fork.

Check:

```bash
git remote -v
```

Push to your fork:

```bash
git push -u origin your-branch-name
```

### "Updates were rejected"

Your fork or branch is behind the remote.

For your own branch, first try:

```bash
git pull
git push
```

For updating your fork's `main` from upstream:

```bash
git switch main
git fetch upstream
git pull upstream main
git push origin main
```

### "Please commit your changes or stash them"

You have uncommitted work, and Git does not want to overwrite it.

Option 1: commit it:

```bash
git add .
git commit -m "Save work in progress"
```

Option 2: stash it temporarily:

```bash
git stash
```

Bring stashed changes back:

```bash
git stash pop
```

### You Are on the Wrong Branch

Check:

```bash
git branch
git status
```

Switch:

```bash
git switch correct-branch-name
```

If you made changes on the wrong branch but have not committed yet:

```bash
git switch -c correct-new-branch-name
```

Your uncommitted changes usually move with you.

---

## 19. Command Cheat Sheet

| Goal | Command |
|---|---|
| Check repo state | `git status` |
| See current branch | `git branch` |
| Create branch | `git switch -c branch-name` |
| Switch branch | `git switch branch-name` |
| See changed lines | `git diff` |
| Stage one file | `git add path/to/file` |
| Stage everything | `git add .` |
| Commit | `git commit -m "Message"` |
| First push of a branch | `git push -u origin branch-name` |
| Push after first push | `git push` |
| Fetch upstream | `git fetch upstream` |
| Update from upstream main | `git pull upstream main` |
| Show history | `git log --oneline` |
| Unstage file | `git restore --staged path/to/file` |
| Discard file changes | `git restore path/to/file` |
| Stash changes | `git stash` |
| Restore stash | `git stash pop` |
| Abort merge | `git merge --abort` |

---

## 20. Beginner Checklist Before Opening a Pull Request

Run these checks:

```bash
git status
git diff --staged
git log --oneline -5
```

Ask yourself:

- Am I on the correct branch?
- Did I commit only the files I meant to change?
- Is my commit message clear?
- Did I push to my fork, not upstream?
- Does my pull request target `openAMRobot/openamr-platform-sw` and the `main` branch?

For code changes, also build or test the relevant part of the project.

---

## 21. The Short Version

For most contributions, this is the workflow:

```bash
git clone https://github.com/YOUR_USERNAME/openamr-platform-sw.git
cd openamr-platform-sw
git remote add upstream https://github.com/openAMRobot/openamr-platform-sw.git

git switch main
git fetch upstream
git pull upstream main
git push origin main

git switch -c docs/example-change

# edit files

git status
git diff
git add .
git commit -m "Describe the change"
git push -u origin docs/example-change
```

Then create a pull request on GitHub from your branch into:

```text
openAMRobot/openamr-platform-sw:main
```
