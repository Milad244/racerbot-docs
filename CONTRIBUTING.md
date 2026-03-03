# Contributing

Contributions are welcome!

**Before making any changes, please check with the project maintainer.** This ensures your work fits the project and avoids duplicated effort.

## 1. Reporting Issues

If you find a bug, have a feature idea, or notice something unclear:

1. Go to the **Issues** tab on GitHub.
2. Click **New Issue**.
3. Use a clear **title** (e.g., `Fix missing nav_msgs dependency in node_safety`).
4. Write a short **description**:
   - What happened  
   - How to reproduce it  
   - Screenshots or error messages if available  

## 2. Forking the Repository

1. Click **Fork** on the top-right of the project page to create your own copy.
2. Clone your fork locally:
   ```bash
   git clone https://github.com/your-username/project-name.git
   cd project-name
   ```
3. Add the original repo as upstream so you can sync later:
    ```bash
    git remote add upstream https://github.com/racerbot/project-name.git
    ```
4. Verify remotes:
    ```bash
    git remote -v
    ```
    You should see origin (your fork) and upstream (original repo).

## 3. Keeping Your Fork Updated
Before starting work, make sure your fork is in sync with the original repo:
```bash
git fetch upstream
git checkout main
git merge upstream/main
git push origin main
```
This prevents conflicts when submitting your changes.

## 4. Making Changes

1. Create a new branch for your work:
    ```bash
    git checkout -b feature/<name>
    ```
    Follow this convention for naming branches:
    - Use hyphens for multi-word names (e.g., `feature/add-sim-map`)
    - `feature/<name>` : New features or enhancements
    - `bugfix/<name>` : Fixing non-urgent bugs
    - `hotfix/<name>` : Urgent production fixes
    - `release/<name>` : Preparing a release version
    - `test/<name>` : Adding or updating tests
    - `refactor/<name>` : Restructuring or improving existing code
    - `docs/<name>` : Documentation-only changes
    - `chore/<name>` : Maintenance tasks (config, dependencies, cleanup)
2. Make your changes in your new branch (code, documentation, etc.).
3. Commit your changes with a clear message:
    ```bash
    git add .
    git commit -m "Add feature X"
    ```
4. Push your branch to your fork:
    ```bash
    git push origin feature/<name>
    ```
## 5. Submitting a Pull Request (PR)
1. Go to your fork on GitHub.
2. Click Compare & Pull Request for your branch.
3. Write a concise and clear description of your changes.
4. Submit the PR.
5. Push updates to the same branch if maintainers request changes.
6. After your PR is merged, update your fork and delete the branch locally and remotely to keep things tidy.

Thank you for helping improve SFU Racerbot!
