
# leapfrog

This is the GIT push of the project software for Leapfrog as of 22nd August 2024. It is hosted on a private repostiroy belonging to Will Christian and will be regularly synced with the SERC gitlab.

## Notes

This repository will contain a version control of the associated software and branch from the main. Any long term changes will then be merged back with main after review. 

Currently the main contains two main sections: flight software for the STM32 and the ground station software, in the associated folders.

By our own convention, version 0.0 will denote the files uploaded as of 22nd August 2024.

To see Documentation, open documents/docs/html/index.html on local computer since github doesn't allow so many files uploaded ```:(```



Please adhere to at least the basic practises of git push/pull/commits/merges. See instructions below.

## Table of Contents

- [Developer Workflow](#developer-workflow)
    - [1. Create a New Branch](#create-a-new-branch)
    - [2. Make Changes and Commit](#make-changes-and-commit)
    - [3. Create a Pull Request (PR)](#create-a-pull-request-pr)
    - [4. Review and Approval](#review-and-approval)
    - [5. Merging the PR into `develop`](#merging-the-pr-into-develop)
    - [6. Clean Up](#clean-up)
- [First Timers](#how-to-git)
    - [Prerequisites](#prerequisites)
    - [Setting Up Your SSH Key](#setting-up-your-ssh-key)
    - [Cloning the Repository](#cloning-the-repository)

---

## TODO List
temporary todo task
| Task                             | Status       | Assignee        | Due Date     | Priority   |
|----------------------------------|--------------|-----------------|--------------|------------|
| Jenkins/github action for doxygen| Not Started  | Any Team Member | 2025-04-01   | Low        |
| Improve Documentation(doxygen)   | Not Started  | Any Team Member | 2025-04-01   | Mid        |



## Developer Workflow
### 1. Create a New Branch
no touch main!

Whenever a new feature, bug fix, or change needs to be made, start by creating a new branch. This ensures that the `develop` branch remains stable and can receive tested code only.

#### Commands:
1. Fetch the latest updates from the remote repository:
    ```bash
    git fetch origin
    ```

2. Switch to the `develop` branch to ensure you're branching off the latest state:
    ```bash
    git checkout develop
    ```

3. Pull the latest changes to avoid working on an outdated version:
    ```bash
    git pull origin develop
    ```

4. Create and switch to a new branch for your changes:
    ```bash
    git checkout -b feature/your-feature-name
    ```
   - *Tip:* Use descriptive branch names prefixed with the type of change, such as `feature/`, `fix/`, or `hotfix/`.


### 2. Make Changes and Commit

After switching to your new branch, you can begin making changes.

#### Commands:
1. Add the changes to be staged for commit:
    ```bash
    git add .
    ```

2. Commit the changes with a descriptive message:
    ```bash
    git commit -m "Add detailed description of the changes made"
    ```

3. Push your branch to the remote repository:
    ```bash
    git push origin feature/your-feature-name
    ```


### 3. Create a Pull Request (PR)

Once changes are made and pushed to the new branch, create a Pull Request (PR) to merge it into the `develop` branch. This process allows for code review and testing before merging.

1. Go to the GitHub repository.
2. Navigate to the "Pull Requests" tab.
3. Select "New Pull Request."
4. Set the base branch to `develop` and compare it with your feature branch.
5. Add a title and description to explain the changes.
6. Submit the PR for review.


### 4. Review and Approval

- Once the PR is submitted, the code will be reviewed by peers or maintainers.
- Address any requested changes or feedback.
- Once approved, the PR can be merged into the `develop` branch.


### 5. Merging the PR into `develop`

#### Commands:
1. Once the PR is approved, ensure your `develop` branch is up to date:
    ```bash
    git checkout develop
    git pull origin develop
    ```

2. Merge the feature branch into `develop`:
    ```bash
    git merge --no-ff feature/your-feature-name
    ```
    - *Tip:* The `--no-ff` flag ensures that the merge creates a commit, preserving the history of the branch.

3. Push the updated `develop` branch to the remote repository:
    ```bash
    git push origin develop
    ```


### 6. Clean Up

Once the feature is merged, you can delete the branch locally and remotely:

#### Commands:
1. Delete the local branch:
    ```bash
    git branch -d feature/your-feature-name
    ```

2. Delete the remote branch:
    ```bash
    git push origin --delete feature/your-feature-name
    ```

[Back to Top](#table-of-contents)

---

##  How to GIT


### Prerequisites
Before you begin, ensure you have the following installed on your machine:
- Git
- A text editor (e.g., **Visual Studio Code** recommended, Atom, etc.)

### Setting Up Your SSH Key
Only first time. Follow these steps to create and add your SSH key:

1. **Open your terminal (or Git Bash on Windows)**.
2. **Generate a new SSH key**:
   - Use the command `ssh-keygen -t rsa -b 4096 -C "your_email@example.com"`, replacing `your_email@example.com` with the email address associated with your GitHub account.

3. **When prompted, press Enter** to accept the default file location. If you want to add a passphrase for additional security, you can do that as well.

4. **Start the SSH agent**:
   - Run the command `eval "$(ssh-agent -s)"`.

5. **Add your SSH private key to the SSH agent**:
   - Use `ssh-add ~/.ssh/id_rsa`.

6. **Copy your SSH key to the clipboard**:
   - For Windows, use `clip < ~/.ssh/id_rsa.pub`.
   - For macOS, use `pbcopy < ~/.ssh/id_rsa.pub`.
   - For Linux, use `xclip -sel clip < ~/.ssh/id_rsa.pub`.

7. **Add the SSH key to your GitHub account**:
   - Go to your GitHub account.
   - Navigate to **Settings** > **SSH and GPG keys** > **New SSH key**.
   - Paste your SSH key and give it a title.
   - Click **Add SSH key**.

### Cloning the Repository
Once your SSH key is set up, you can clone the repository using the command `git clone git@github.com:hall-engine/leapfrog.git`.

Navigate into the project directory with the command `cd leapfrog`.

See [Developer Workflow](#developer-workflow) for implementing changes

[Back to Top](#table-of-contents)

## Current Team
<strong>Will Christian</strong> [william.christian@usc.edu](mailto:william.christian@usc.edu)

<strong>Varick John</strong> [vjohn@usc.edu](mailto:vjohn@usc.edu)

<strong>Howard Hall</strong> [hahall@usc.edu](mailto:hahall@usc.edu)

<strong>Briana Zeggane</strong> [zeggane@usc.edu](mailto:zeggane@usc.edu)

<strong>Trevor Gross</strong> [trevorg@usc.edu](mailto:trevorg@usc.edu)
